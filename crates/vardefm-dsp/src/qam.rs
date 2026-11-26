use num_complex::Complex32;

/// Constellations with Gray mapping and unit average power.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Modulation {
    Bpsk,
    Qpsk,
    Qam16,
    Qam64,
}

impl Modulation {
    pub const ALL: [Modulation; 4] = [Self::Bpsk, Self::Qpsk, Self::Qam16, Self::Qam64];

    pub fn bits_per_symbol(self) -> usize {
        match self {
            Self::Bpsk => 1,
            Self::Qpsk => 2,
            Self::Qam16 => 4,
            Self::Qam64 => 6,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Bpsk => "bpsk",
            Self::Qpsk => "qpsk",
            Self::Qam16 => "16qam",
            Self::Qam64 => "64qam",
        }
    }

    pub fn from_name(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|m| m.name() == s)
    }

    /// Bits carried on each axis (I and Q). BPSK uses only I.
    pub fn bits_per_axis(self) -> usize {
        match self {
            Self::Bpsk => 1,
            _ => self.bits_per_symbol() / 2,
        }
    }

    fn scale(self) -> f32 {
        match self {
            Self::Bpsk => 1.0,
            _ => {
                let l = (1usize << self.bits_per_axis()) as f32;
                1.0 / (2.0 * (l * l - 1.0) / 3.0).sqrt()
            }
        }
    }

    /// Gray-coded PAM level (unscaled odd integer) for an axis word, MSB first.
    fn pam_level(word: usize, m: usize) -> f32 {
        let mut idx = word;
        let mut shift = word >> 1;
        while shift != 0 {
            idx ^= shift;
            shift >>= 1;
        }
        2.0 * idx as f32 - ((1usize << m) as f32 - 1.0)
    }

    fn word(bits: &[u8]) -> usize {
        bits.iter().fold(0, |w, &b| (w << 1) | b as usize)
    }

    pub fn map(self, bits: &[u8]) -> Complex32 {
        debug_assert_eq!(bits.len(), self.bits_per_symbol());
        let s = self.scale();
        let m = self.bits_per_axis();
        match self {
            Self::Bpsk => Complex32::new(Self::pam_level(bits[0] as usize, 1), 0.0),
            _ => Complex32::new(
                Self::pam_level(Self::word(&bits[..m]), m) * s,
                Self::pam_level(Self::word(&bits[m..]), m) * s,
            ),
        }
    }

    /// Map a bit stream (length must be a multiple of bits_per_symbol).
    pub fn map_all(self, bits: &[u8]) -> Vec<Complex32> {
        bits.chunks_exact(self.bits_per_symbol()).map(|c| self.map(c)).collect()
    }

    /// Max-log LLRs (positive means bit 0) for one received symbol.
    /// `noise_var` is the complex noise variance (both axes together).
    pub fn demap_llr(self, y: Complex32, noise_var: f32, out: &mut Vec<f32>) {
        let n_axes = if self == Self::Bpsk { 1 } else { 2 };
        for &v in &[y.re, y.im][..n_axes] {
            self.demap_axis(v, noise_var, 0, 0, out);
        }
    }

    /// Max-log LLRs as [`Modulation::demap_llr`], given that some of the symbol's
    /// bits are already known: `known[i]` is `Some(bit == 1)` for a known bit and
    /// `None` otherwise. Only the points that agree with the known bits on the same
    /// axis count. A known bit's own LLR is constrained only on the axis's other known
    /// bits; callers do not use it.
    pub fn demap_llr_known(self, y: Complex32, noise_var: f32, known: &[Option<bool>], out: &mut Vec<f32>) {
        debug_assert_eq!(known.len(), self.bits_per_symbol());
        let m = self.bits_per_axis();
        let n_axes = if self == Self::Bpsk { 1 } else { 2 };
        for (a, &v) in [y.re, y.im][..n_axes].iter().enumerate() {
            // Known bits of this axis, as a mask and their values, MSB first.
            let (mut kmask, mut kval) = (0usize, 0usize);
            for (bit, &k) in known[a * m..(a + 1) * m].iter().enumerate() {
                if let Some(one) = k {
                    kmask |= 1 << (m - 1 - bit);
                    kval |= usize::from(one) << (m - 1 - bit);
                }
            }
            self.demap_axis(v, noise_var, kmask, kval, out);
        }
    }

    /// Max-log LLRs of the bits of one axis, at value `v`, over the axis words that
    /// agree with `kval` on the bits of `kmask` other than the bit itself.
    fn demap_axis(self, v: f32, noise_var: f32, kmask: usize, kval: usize, out: &mut Vec<f32>) {
        let m = self.bits_per_axis();
        let s = self.scale();
        let axis_var = (noise_var / 2.0).max(1e-9);
        let l = 1usize << m;
        for bit in 0..m {
            let mask = 1 << (m - 1 - bit);
            // Constrain on the other known bits only, so the bit's own LLR stays
            // finite. With nothing known (`demap_llr`) every word counts.
            let (cmask, cval) = (kmask & !mask, kval & !mask);
            let mut d0 = f32::INFINITY;
            let mut d1 = f32::INFINITY;
            for word in 0..l {
                if word & cmask != cval {
                    continue;
                }
                let d = v - Self::pam_level(word, m) * s;
                let d = d * d;
                if word & mask == 0 {
                    d0 = d0.min(d);
                } else {
                    d1 = d1.min(d);
                }
            }
            out.push((d1 - d0) / (2.0 * axis_var));
        }
    }

    pub fn hard_bits(self, y: Complex32, out: &mut Vec<u8>) {
        let mut llr = Vec::with_capacity(self.bits_per_symbol());
        self.demap_llr(y, 1.0, &mut llr);
        out.extend(llr.iter().map(|&l| (l < 0.0) as u8));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prng::Prng;

    #[test]
    fn roundtrip_and_unit_power() {
        for m in Modulation::ALL {
            let mut rng = Prng::new(7);
            let bits = rng.bits(m.bits_per_symbol() * 600);
            let syms = m.map_all(&bits);
            let p: f32 = syms.iter().map(|c| c.norm_sqr()).sum::<f32>() / syms.len() as f32;
            assert!((p - 1.0).abs() < 0.1, "{m:?} power {p}");
            let mut out = Vec::new();
            for s in &syms {
                m.hard_bits(*s, &mut out);
            }
            assert_eq!(out, bits, "{m:?}");
        }
    }

    #[test]
    fn known_bits_sharpen_the_rest_of_the_axis() {
        let m = Modulation::Qam16;
        let s = m.scale();
        // Between the two inner points of the I axis, the sign (first bit) is
        // unsure. Knowing the second bit says the point is an outer one (+-3), whose
        // distance from y differs three times as much between the signs.
        let y = Complex32::new(0.1 * s, 0.0);
        let (mut free, mut given) = (Vec::new(), Vec::new());
        m.demap_llr(y, 0.5, &mut free);
        m.demap_llr_known(y, 0.5, &[None, Some(false), None, None], &mut given);
        assert!(free[0] < 0.0 && given[0] < 0.0, "y > 0 means bit 1: {free:?} {given:?}");
        assert!((given[0] / free[0] - 3.0).abs() < 1e-3, "{free:?} {given:?}");
        // The other axis and the known bit's own LLR are left as they are.
        assert_eq!(free[1..], given[1..]);
        // Nothing known is the plain demapper.
        for m in Modulation::ALL {
            let y = Complex32::new(0.3, -0.7);
            let (mut a, mut b) = (Vec::new(), Vec::new());
            m.demap_llr(y, 0.2, &mut a);
            m.demap_llr_known(y, 0.2, &vec![None; m.bits_per_symbol()], &mut b);
            assert_eq!(a, b, "{m:?}");
        }
    }

    /// Max-log LLR of bit `b` of `m` at `y`, by brute force over every point whose
    /// bits agree with `known` (except bit `b` itself).
    fn brute_llr(m: Modulation, y: Complex32, noise_var: f32, known: &[Option<bool>], b: usize) -> f32 {
        let n = m.bits_per_symbol();
        let (mut d0, mut d1) = (f32::INFINITY, f32::INFINITY);
        for word in 0..1usize << n {
            let bits: Vec<u8> = (0..n).map(|i| (word >> (n - 1 - i)) as u8 & 1).collect();
            let agrees = (0..n).all(|i| i == b || known[i].is_none_or(|one| u8::from(one) == bits[i]));
            if !agrees {
                continue;
            }
            let d = (y - m.map(&bits)).norm_sqr();
            if bits[b] == 0 {
                d0 = d0.min(d);
            } else {
                d1 = d1.min(d);
            }
        }
        (d1 - d0) / noise_var
    }

    #[test]
    fn known_bits_match_brute_force_at_64qam() {
        // 1 and 2 of the 3 bits of an axis known, on either axis or both, against
        // the max-log LLR over every allowed point of the whole constellation.
        let m = Modulation::Qam64;
        let mut rng = Prng::new(11);
        let patterns: [[bool; 6]; 6] = [
            [true, false, false, false, false, false],
            [false, false, true, false, false, false],
            [true, true, false, false, false, false],
            [false, true, true, false, true, false],
            [false, false, false, true, false, true],
            [true, false, true, false, true, true],
        ];
        for t in 0..200 {
            let sent = rng.bits(6);
            let noise = |r: &mut Prng| (r.next_u32() as f32 / u32::MAX as f32 - 0.5) * 0.4;
            let y = m.map(&sent) + Complex32::new(noise(&mut rng), noise(&mut rng));
            let pat = &patterns[t % patterns.len()];
            let known: Vec<Option<bool>> = (0..6).map(|i| pat[i].then_some(sent[i] == 1)).collect();
            let mut got = Vec::new();
            m.demap_llr_known(y, 0.05, &known, &mut got);
            for b in (0..6).filter(|&b| !pat[b]) {
                let want = brute_llr(m, y, 0.05, &known, b);
                assert!((got[b] - want).abs() <= 1e-3 * want.abs().max(1.0), "trial {t} bit {b}: {got:?} vs {want}");
            }
        }
    }
}
