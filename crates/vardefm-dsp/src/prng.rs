/// Small deterministic generator (xorshift32) for pilots, preambles and test data.
/// Both ends must produce identical sequences, so this must never change.
#[derive(Clone, Debug)]
pub struct Prng(u32);

impl Prng {
    pub fn new(seed: u32) -> Self {
        Self(if seed == 0 { 0x9e37_79b9 } else { seed })
    }

    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }

    pub fn bit(&mut self) -> u8 {
        (self.next_u32() >> 31) as u8
    }

    /// +1.0 or -1.0
    pub fn bpsk(&mut self) -> f32 {
        if self.bit() == 0 {
            1.0
        } else {
            -1.0
        }
    }

    pub fn bits(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.bit()).collect()
    }
}
