//! Error correction for VardeFM: IRA LDPC codes, CRCs, scrambling and interleaving.

pub mod crc;
pub mod ldpc;

pub use ldpc::Ldpc;
use vardefm_dsp::prng::Prng;

/// Bytes to bits, MSB first.
pub fn bytes_to_bits(bytes: &[u8]) -> Vec<u8> {
    bytes.iter().flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1)).collect()
}

/// Bits (MSB first) to bytes. The length must be a multiple of 8.
pub fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.as_chunks::<8>().0.iter().map(|c| c.iter().fold(0u8, |acc, &b| (acc << 1) | (b & 1))).collect()
}

/// Whitening sequence: XORed onto data bits so long runs of equal bytes do not
/// produce a peaky OFDM signal.
pub fn scramble_bits(seed: u32, n: usize) -> Vec<u8> {
    Prng::new(seed).bits(n)
}

/// Pseudo-random permutation of 0..n (Fisher-Yates driven by xorshift32).
/// Bit `j` of the input goes to position `perm[j]`.
pub fn permutation(seed: u32, n: usize) -> Vec<usize> {
    let mut rng = Prng::new(seed);
    let mut p: Vec<usize> = (0..n).collect();
    for i in (1..n).rev() {
        let j = rng.next_u32() as usize % (i + 1);
        p.swap(i, j);
    }
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bits_roundtrip() {
        let b = vec![0x00, 0xff, 0xa5, 0x3c];
        assert_eq!(bits_to_bytes(&bytes_to_bits(&b)), b);
        assert_eq!(&bytes_to_bits(&[0x80])[..2], &[1, 0]);
    }

    #[test]
    fn permutation_is_a_permutation() {
        let mut p = permutation(5, 1000);
        p.sort();
        assert_eq!(p, (0..1000).collect::<Vec<_>>());
    }
}
