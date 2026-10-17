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
    fn bits_per_axis(self) -> usize {
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
        bits.chunks_exact(self.bits_per_symbol())
            .map(|c| self.map(c))
            .collect()
    }

    /// Max-log LLRs (positive means bit 0) for one received symbol.
    /// `noise_var` is the complex noise variance (both axes together).
    pub fn demap_llr(self, y: Complex32, noise_var: f32, out: &mut Vec<f32>) {
        let m = self.bits_per_axis();
        let s = self.scale();
        let axis_var = (noise_var / 2.0).max(1e-9);
        let axes = [y.re, y.im];
        let n_axes = if self == Self::Bpsk { 1 } else { 2 };
        let l = 1usize << m;
        for &v in &axes[..n_axes] {
            for bit in 0..m {
                let mask = 1 << (m - 1 - bit);
                let mut d0 = f32::INFINITY;
                let mut d1 = f32::INFINITY;
                for word in 0..l {
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
}
