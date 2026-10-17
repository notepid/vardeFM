//! VardeFM bursts.
//!
//! A burst is one transmission: `[leader][preamble A][preamble B][header][payload]`.
//! - The header is 96 bits (80 bits of fields + CRC-16), LDPC-coded to 400 bits and
//!   sent as BPSK. That is 10 OFDM symbols in narrow mode and 8 in wide mode.
//! - The payload is `ncw` codewords of 2400 bits at the speed level named in the
//!   header. Each codeword carries `k/8 - 4` data bytes plus a CRC-32.

mod header;
mod receiver;

pub use header::{Header, Kind, FLAG_POLL};
pub use receiver::{Receiver, RxBurst};
pub use vardefm_dsp::OfdmParams;

use vardefm_dsp::{Complex32, Modulation, Ofdm};
use vardefm_fec::{bytes_to_bits, crc, permutation, scramble_bits, Ldpc};

/// Bits per payload codeword.
pub const CW_BITS: usize = 2400;
/// Header bits before coding (80 bits of fields + CRC-16).
pub const HDR_INFO_BITS: usize = 96;
/// Header bits after coding.
pub const HDR_CODE_BITS: usize = 400;
/// Most codewords in one burst (the ack bitmap has 24 bits).
pub const MAX_NCW: usize = 24;
/// Payload airtime limit per burst. Keeps clock drift inside the cyclic-prefix slack.
pub const MAX_PAYLOAD_SECONDS: f32 = 4.0;
const DECODE_ITERS: usize = 50;

const SEED_CODE: u32 = 0x1d9c_0000;
const SEED_HDR_CODE: u32 = 0x1d9c_1000;
const SEED_SCRAMBLE: u32 = 0x5c4a_0000;
const SEED_HDR_SCRAMBLE: u32 = 0x5c4a_ffff;
const SEED_INTERLEAVE: u32 = 0x17e4_0001;
const SEED_HDR_INTERLEAVE: u32 = 0x17e4_0002;

/// One speed level: constellation and LDPC information length (codeword = 2400 bits).
#[derive(Clone, Copy, Debug)]
pub struct Level {
    pub modulation: Modulation,
    pub k: usize,
    pub name: &'static str,
}

pub const LEVELS: [Level; 7] = [
    Level { modulation: Modulation::Bpsk, k: 1200, name: "BPSK 1/2" },
    Level { modulation: Modulation::Qpsk, k: 1200, name: "QPSK 1/2" },
    Level { modulation: Modulation::Qpsk, k: 1800, name: "QPSK 3/4" },
    Level { modulation: Modulation::Qam16, k: 1200, name: "16QAM 1/2" },
    Level { modulation: Modulation::Qam16, k: 1800, name: "16QAM 3/4" },
    Level { modulation: Modulation::Qam64, k: 1600, name: "64QAM 2/3" },
    Level { modulation: Modulation::Qam64, k: 2000, name: "64QAM 5/6" },
];

/// A burst to transmit.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Burst {
    pub header: Header,
    /// `header.ncw` codewords, each exactly `payload_bytes(header.level)` long.
    pub payload: Vec<Vec<u8>>,
}

/// Modulator/coder for one mode. Shared (read-only) by the transmitter and receiver.
pub struct Modem {
    pub ofdm: Ofdm,
    /// Leader length in seconds (must exceed the radio's key-up time).
    pub txdelay: f32,
    hdr_code: Ldpc,
    codes: Vec<Ldpc>,
    hdr_perm: Vec<usize>,
    hdr_syms: usize,
}

impl Modem {
    pub fn new(params: OfdmParams, txdelay: f32) -> Self {
        let ofdm = Ofdm::new(params);
        let nd = ofdm.data_carriers();
        assert_eq!(HDR_CODE_BITS % nd, 0, "header must fill whole symbols");
        for l in &LEVELS {
            assert_eq!(CW_BITS % (nd * l.modulation.bits_per_symbol()), 0);
        }
        Self {
            hdr_syms: HDR_CODE_BITS / nd,
            hdr_code: Ldpc::new(HDR_CODE_BITS, HDR_INFO_BITS, SEED_HDR_CODE),
            codes: LEVELS.iter().map(|l| Ldpc::new(CW_BITS, l.k, SEED_CODE + l.k as u32)).collect(),
            hdr_perm: permutation(SEED_HDR_INTERLEAVE, HDR_CODE_BITS),
            ofdm,
            txdelay,
        }
    }

    pub fn fs(&self) -> f32 {
        self.ofdm.p.fs
    }

    /// Data bytes per codeword at `level`.
    pub fn payload_bytes(level: u8) -> usize {
        LEVELS[level as usize].k / 8 - 4
    }

    pub fn header_symbols(&self) -> usize {
        self.hdr_syms
    }

    pub fn symbols_per_codeword(&self, level: u8) -> usize {
        CW_BITS / (self.ofdm.data_carriers() * LEVELS[level as usize].modulation.bits_per_symbol())
    }

    /// Most codewords per burst at `level`.
    pub fn max_ncw(&self, level: u8) -> usize {
        let cw_s = self.symbols_per_codeword(level) as f32 * self.ofdm.p.symbol_duration();
        ((MAX_PAYLOAD_SECONDS / cw_s) as usize).clamp(1, MAX_NCW)
    }

    fn leader_symbols(&self) -> usize {
        (self.txdelay / self.ofdm.p.symbol_duration()).ceil() as usize
    }

    /// Length in samples of a burst.
    pub fn burst_samples(&self, level: u8, ncw: usize) -> usize {
        let syms = self.leader_symbols() + 2 + self.hdr_syms + ncw * self.symbols_per_codeword(level);
        syms * self.ofdm.p.symbol_len()
    }

    pub fn burst_seconds(&self, level: u8, ncw: usize) -> f32 {
        self.burst_samples(level, ncw) as f32 / self.fs()
    }

    /// Net payload rate of a full burst at `level`, bits/s (including all overhead).
    pub fn net_rate(&self, level: u8) -> f32 {
        let ncw = self.max_ncw(level);
        (ncw * Self::payload_bytes(level) * 8) as f32 / self.burst_seconds(level, ncw)
    }

    fn header_bits(&self, h: &Header) -> Vec<u8> {
        let mut bits = bytes_to_bits(&h.pack());
        for (b, s) in bits.iter_mut().zip(scramble_bits(SEED_HDR_SCRAMBLE, HDR_INFO_BITS)) {
            *b ^= s;
        }
        let cw = self.hdr_code.encode(&bits);
        let mut slots = vec![0u8; HDR_CODE_BITS];
        for (j, &b) in cw.iter().enumerate() {
            slots[self.hdr_perm[j]] = b;
        }
        slots
    }

    fn payload_bits(&self, level: u8, payload: &[Vec<u8>]) -> Vec<u8> {
        let code = &self.codes[level as usize];
        let nbytes = Self::payload_bytes(level);
        let mut all = Vec::with_capacity(payload.len() * CW_BITS);
        for (j, data) in payload.iter().enumerate() {
            assert_eq!(data.len(), nbytes, "codeword {j} has the wrong length");
            let mut bytes = data.clone();
            bytes.extend_from_slice(&crc::crc32(data).to_be_bytes());
            let mut bits = bytes_to_bits(&bytes);
            for (b, s) in bits.iter_mut().zip(scramble_bits(SEED_SCRAMBLE + j as u32, code.k)) {
                *b ^= s;
            }
            all.extend(code.encode(&bits));
        }
        let perm = permutation(SEED_INTERLEAVE, all.len());
        let mut slots = vec![0u8; all.len()];
        for (j, &b) in all.iter().enumerate() {
            slots[perm[j]] = b;
        }
        slots
    }

    /// Audio for one burst (starts with the leader; ends with the last symbol).
    pub fn modulate(&self, burst: &Burst) -> Vec<f32> {
        let h = &burst.header;
        assert_eq!(h.ncw as usize, burst.payload.len());
        let nd = self.ofdm.data_carriers();
        let mut symbols: Vec<Vec<Complex32>> = Vec::new();
        let bpsk = Modulation::Bpsk;
        for chunk in self.header_bits(h).chunks(nd) {
            symbols.push(chunk.iter().map(|&b| bpsk.map(&[b])).collect());
        }
        if h.ncw > 0 {
            let m = LEVELS[h.level as usize].modulation;
            let bits = self.payload_bits(h.level, &burst.payload);
            for chunk in bits.chunks(nd * m.bits_per_symbol()) {
                symbols.push(m.map_all(chunk));
            }
        }
        self.ofdm.modulate_with_leader(&symbols, self.txdelay)
    }

    /// LLRs for every bit slot of `symbols`, using per-carrier noise variance.
    fn llrs(m: Modulation, symbols: &[Vec<Complex32>], noise_var: &[f32]) -> Vec<f32> {
        let mut out = Vec::with_capacity(symbols.len() * noise_var.len() * m.bits_per_symbol());
        for sym in symbols {
            for (y, &nv) in sym.iter().zip(noise_var) {
                m.demap_llr(*y, nv, &mut out);
            }
        }
        out
    }

    fn decode_header(&self, symbols: &[Vec<Complex32>], noise_var: &[f32]) -> Option<Header> {
        let slots = Self::llrs(Modulation::Bpsk, symbols, noise_var);
        let llr: Vec<f32> = (0..HDR_CODE_BITS).map(|j| slots[self.hdr_perm[j]]).collect();
        let mut bits = self.hdr_code.decode(&llr, DECODE_ITERS)?;
        for (b, s) in bits.iter_mut().zip(scramble_bits(SEED_HDR_SCRAMBLE, HDR_INFO_BITS)) {
            *b ^= s;
        }
        Header::unpack(&vardefm_fec::bits_to_bytes(&bits))
    }

    fn decode_payload(&self, h: &Header, symbols: &[Vec<Complex32>], noise_var: &[f32]) -> Vec<Option<Vec<u8>>> {
        let level = LEVELS[h.level as usize];
        let code = &self.codes[h.level as usize];
        let slots = Self::llrs(level.modulation, symbols, noise_var);
        let total = h.ncw as usize * CW_BITS;
        let perm = permutation(SEED_INTERLEAVE, total);
        (0..h.ncw as usize)
            .map(|j| {
                let llr: Vec<f32> = (0..CW_BITS).map(|i| slots[perm[j * CW_BITS + i]]).collect();
                let mut bits = code.decode(&llr, DECODE_ITERS)?;
                for (b, s) in bits.iter_mut().zip(scramble_bits(SEED_SCRAMBLE + j as u32, code.k)) {
                    *b ^= s;
                }
                let bytes = vardefm_fec::bits_to_bytes(&bits);
                let (data, tail) = bytes.split_at(bytes.len() - 4);
                (crc::crc32(data).to_be_bytes() == tail).then(|| data.to_vec())
            })
            .collect()
    }
}

#[cfg(test)]
mod tests;
