//! Systematic IRA (irregular repeat-accumulate) LDPC codes.
//!
//! The parity-check matrix is `H = [Hu | Hp]`:
//! - `Hu` (M x K): every information bit takes part in `dv` checks, spread evenly
//!   and pseudo-randomly over the M checks (see [`Ldpc::new`] for the exact algorithm).
//! - `Hp` (M x M): a staircase. Check `i` contains parity bits `i` and `i-1`.
//!
//! Encoding is a running XOR. Codeword = information bits followed by parity bits.
//! Decoding is layered normalized min-sum.

use vardefm_dsp::prng::Prng;

const ALPHA: f32 = 0.75;
const LLR_CLAMP: f32 = 40.0;

#[derive(Clone, Debug)]
pub struct Ldpc {
    pub n: usize,
    pub k: usize,
    /// For each check: the information-bit variables it contains.
    info_vars: Vec<Vec<u32>>,
    /// Flattened adjacency of every check (information and parity variables).
    check_ptr: Vec<usize>,
    check_vars: Vec<u32>,
}

impl Ldpc {
    /// Build the (n, k) code from `seed`.
    ///
    /// Construction (normative, both ends must match):
    /// 1. `m = n - k`, `dv = max(3, ceil(3m / k))`, capped at `m`.
    /// 2. Socket list `s[e] = e mod m` for `e` in `0..k*dv`, shuffled with
    ///    [`crate::permutation`]-style Fisher-Yates using `Prng::new(seed)`.
    /// 3. Information bit `j` connects to checks `s[j*dv .. (j+1)*dv]`.
    ///    A repeated check in one column is fixed by swapping that socket with
    ///    socket `r = rng.next_u32() mod (k*dv)` (same generator, continuing)
    ///    until the column has distinct checks and the swap does not create a
    ///    repeat in the other column.
    pub fn new(n: usize, k: usize, seed: u32) -> Self {
        assert!(k > 0 && n > k);
        let m = n - k;
        let dv = (3usize).max((3 * m).div_ceil(k)).min(m);
        let e = k * dv;
        let mut rng = Prng::new(seed);
        let mut s: Vec<usize> = (0..e).map(|i| i % m).collect();
        for i in (1..e).rev() {
            let j = rng.next_u32() as usize % (i + 1);
            s.swap(i, j);
        }
        let col_has = |s: &[usize], col: usize, check: usize, skip: usize| {
            (col * dv..(col + 1) * dv).any(|i| i != skip && s[i] == check)
        };
        for col in 0..k {
            for i in col * dv..(col + 1) * dv {
                let mut tries = 0;
                while col_has(&s, col, s[i], i) && tries < 10_000 {
                    let r = rng.next_u32() as usize % e;
                    let other = r / dv;
                    if other != col && !col_has(&s, col, s[r], i) && !col_has(&s, other, s[i], r) {
                        s.swap(i, r);
                    }
                    tries += 1;
                }
            }
        }

        let mut info_vars = vec![Vec::new(); m];
        for (i, &c) in s.iter().enumerate() {
            info_vars[c].push((i / dv) as u32);
        }
        for v in &mut info_vars {
            v.sort_unstable();
        }

        let mut check_ptr = Vec::with_capacity(m + 1);
        let mut check_vars = Vec::new();
        check_ptr.push(0);
        for (c, iv) in info_vars.iter().enumerate() {
            check_vars.extend_from_slice(iv);
            check_vars.push((k + c) as u32);
            if c > 0 {
                check_vars.push((k + c - 1) as u32);
            }
            check_ptr.push(check_vars.len());
        }
        Self { n, k, info_vars, check_ptr, check_vars }
    }

    pub fn m(&self) -> usize {
        self.n - self.k
    }

    /// Encode `k` information bits into an `n`-bit codeword.
    pub fn encode(&self, info: &[u8]) -> Vec<u8> {
        assert_eq!(info.len(), self.k);
        let mut cw = Vec::with_capacity(self.n);
        cw.extend_from_slice(info);
        let mut prev = 0u8;
        for iv in &self.info_vars {
            let s = iv.iter().fold(0u8, |acc, &v| acc ^ info[v as usize]);
            prev ^= s;
            cw.push(prev);
        }
        cw
    }

    /// True if `bits` (length n) satisfies every parity check.
    pub fn check(&self, bits: &[u8]) -> bool {
        (0..self.m()).all(|c| {
            self.check_vars[self.check_ptr[c]..self.check_ptr[c + 1]].iter().fold(0u8, |acc, &v| acc ^ bits[v as usize])
                == 0
        })
    }

    /// Decode channel LLRs (positive = bit 0). Returns the `k` information bits
    /// if a valid codeword was found within `max_iter` iterations.
    pub fn decode(&self, llr: &[f32], max_iter: usize) -> Option<Vec<u8>> {
        assert_eq!(llr.len(), self.n);
        let mut total: Vec<f32> = llr.iter().map(|l| l.clamp(-LLR_CLAMP, LLR_CLAMP)).collect();
        let mut r = vec![0f32; self.check_vars.len()];
        let mut q = Vec::with_capacity(64);
        let mut hard = vec![0u8; self.n];

        for _ in 0..max_iter {
            for c in 0..self.m() {
                let (a, b) = (self.check_ptr[c], self.check_ptr[c + 1]);
                q.clear();
                let (mut min1, mut min2, mut imin) = (f32::INFINITY, f32::INFINITY, 0);
                let mut neg = false;
                for e in a..b {
                    let v = total[self.check_vars[e] as usize] - r[e];
                    let mag = v.abs();
                    if mag < min1 {
                        min2 = min1;
                        min1 = mag;
                        imin = e;
                    } else if mag < min2 {
                        min2 = mag;
                    }
                    neg ^= v < 0.0;
                    q.push(v);
                }
                for (i, e) in (a..b).enumerate() {
                    let mag = ALPHA * if e == imin { min2 } else { min1 };
                    let sign_neg = neg ^ (q[i] < 0.0);
                    let new = if sign_neg { -mag } else { mag };
                    r[e] = new;
                    total[self.check_vars[e] as usize] = q[i] + new;
                }
            }
            for (h, t) in hard.iter_mut().zip(&total) {
                *h = (*t < 0.0) as u8;
            }
            if self.check(&hard) {
                return Some(hard[..self.k].to_vec());
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// BPSK through AWGN at the given Es/N0; returns LLRs = 2y/sigma^2.
    fn awgn_bpsk(cw: &[u8], esn0_db: f32, seed: u32) -> Vec<f32> {
        let sigma = (0.5 * 10f32.powf(-esn0_db / 10.0)).sqrt();
        let mut rng = Prng::new(seed);
        let mut u = || (rng.next_u32() as f32 + 1.0) / (u32::MAX as f32 + 2.0);
        cw.iter()
            .map(|&b| {
                let n = (-2.0 * u().ln()).sqrt() * (2.0 * std::f32::consts::PI * u()).cos();
                let y = if b == 0 { 1.0 } else { -1.0 } + sigma * n;
                2.0 * y / (sigma * sigma)
            })
            .collect()
    }

    #[test]
    fn encode_satisfies_checks() {
        for (n, k) in [(2400, 1200), (2400, 1600), (2400, 1800), (2400, 2000), (400, 96)] {
            let code = Ldpc::new(n, k, 1);
            let info = Prng::new(9).bits(k);
            let cw = code.encode(&info);
            assert!(code.check(&cw), "({n},{k})");
            let llr: Vec<f32> = cw.iter().map(|&b| if b == 0 { 4.0 } else { -4.0 }).collect();
            assert_eq!(code.decode(&llr, 5).unwrap(), info);
        }
    }

    #[test]
    fn corrects_noise() {
        // Rate 1/2 BPSK at Eb/N0 = 3.5 dB (Es/N0 = 0.5 dB): FER is well under 1%.
        let code = Ldpc::new(2400, 1200, 1);
        let mut ok = 0;
        for t in 0..20 {
            let info = Prng::new(100 + t).bits(1200);
            let cw = code.encode(&info);
            let llr = awgn_bpsk(&cw, 0.5, 7 + t);
            if code.decode(&llr, 50).as_deref() == Some(&info[..]) {
                ok += 1;
            }
        }
        assert!(ok >= 19, "decoded {ok}/20");
    }
}

#[cfg(test)]
mod curve {
    use super::*;

    /// Prints frame error rate vs Eb/N0 for each payload rate (BPSK, AWGN).
    /// `cargo test --release -p vardefm-fec fer_curve -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn fer_curve() {
        for k in [1200, 1600, 1800, 2000] {
            let code = Ldpc::new(2400, k, 0x1d9c_0000 + k as u32);
            let rate = k as f32 / 2400.0;
            let mut line = format!("rate {rate:.2}:");
            for ebn0 in [0.5f32, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0] {
                let esn0 = ebn0 + 10.0 * rate.log10();
                let sigma = (0.5 * 10f32.powf(-esn0 / 10.0)).sqrt();
                let trials = 60;
                let mut fail = 0;
                for t in 0..trials {
                    let info = Prng::new(1 + t).bits(k);
                    let cw = code.encode(&info);
                    let mut rng = Prng::new(1000 + t);
                    let mut u = || (rng.next_u32() as f32 + 1.0) / (u32::MAX as f32 + 2.0);
                    let llr: Vec<f32> = cw
                        .iter()
                        .map(|&b| {
                            let n = (-2.0 * u().ln()).sqrt() * (2.0 * std::f32::consts::PI * u()).cos();
                            let y = if b == 0 { 1.0 } else { -1.0 } + sigma * n;
                            2.0 * y / (sigma * sigma)
                        })
                        .collect();
                    if code.decode(&llr, 50).as_deref() != Some(&info[..]) {
                        fail += 1;
                    }
                }
                line += &format!(" {ebn0:.1}dB:{:.2}", fail as f32 / trials as f32);
            }
            println!("{line}");
        }
    }
}
