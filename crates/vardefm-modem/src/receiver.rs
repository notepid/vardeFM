//! Streaming receiver: feed audio in chunks, get decoded bursts out.

use crate::{Header, Modem, LEVELS};
use std::sync::Arc;

/// Squelch-open threshold for the BUSY indication (RMS, full scale = 1).
const BUSY_RMS: f32 = 0.01;
/// How long the channel stays busy after the last loud chunk.
const BUSY_HOLD_S: f32 = 0.15;

/// A received burst.
#[derive(Clone, Debug)]
pub struct RxBurst {
    pub header: Header,
    /// One entry per codeword: `Some(data)` if it decoded and passed its CRC.
    pub payload: Vec<Option<Vec<u8>>>,
    /// Average SNR per symbol (Es/N0) estimated from the pilots, dB.
    pub snr_db: f32,
    /// Absolute sample index (since the receiver was created) where the burst ended.
    pub end_sample: u64,
}

pub struct Receiver {
    modem: Arc<Modem>,
    buf: Vec<f32>,
    /// Absolute sample index of `buf[0]`.
    base: u64,
    /// Where to continue searching for preambles (index into `buf`).
    search: usize,
    /// A frame whose header decoded but whose payload has not fully arrived: (preamble-B index, header).
    pending: Option<(usize, Header)>,
    /// Samples since the channel was last loud.
    quiet_samples: usize,
}

impl Receiver {
    pub fn new(modem: Arc<Modem>) -> Self {
        Self { modem, buf: Vec::new(), base: 0, search: 0, pending: None, quiet_samples: usize::MAX / 2 }
    }

    /// Absolute index of the next sample to be pushed.
    pub fn position(&self) -> u64 {
        self.base + self.buf.len() as u64
    }

    /// True while a signal is heard (squelch open) or a frame is being received.
    pub fn busy(&self) -> bool {
        self.pending.is_some() || (self.quiet_samples as f32) < BUSY_HOLD_S * self.modem.fs()
    }

    /// Drop all buffered audio (e.g. when our own transmitter keys up).
    pub fn reset(&mut self) {
        self.base += self.buf.len() as u64;
        self.buf.clear();
        self.search = 0;
        self.pending = None;
    }

    pub fn push(&mut self, samples: &[f32]) -> Vec<RxBurst> {
        let ms = samples.iter().map(|x| x * x).sum::<f32>() / samples.len().max(1) as f32;
        if ms.sqrt() > BUSY_RMS {
            self.quiet_samples = 0;
        } else {
            self.quiet_samples += samples.len();
        }
        self.buf.extend_from_slice(samples);
        let mut out = Vec::new();
        while let Some(progress) = self.step(&mut out) {
            if !progress {
                break;
            }
        }
        self.trim();
        out
    }

    /// Sample index (in `buf`) needed to demodulate `nsym` symbols after preamble B at `b`.
    fn needed(&self, b: usize, nsym: usize) -> usize {
        let p = &self.modem.ofdm.p;
        b - p.ncp / 2 + (nsym + 1) * p.symbol_len() + p.nfft
    }

    /// One processing step. Returns Some(true) if something changed and we should go again.
    fn step(&mut self, out: &mut Vec<RxBurst>) -> Option<bool> {
        let m = &self.modem;
        let hdr_syms = m.header_symbols();
        if let Some((b, h)) = self.pending {
            let nsym = hdr_syms + h.ncw as usize * m.symbols_per_codeword(h.level);
            if self.buf.len() < self.needed(b, nsym) {
                return Some(false);
            }
            self.pending = None;
            let Some(frame) = m.ofdm.demodulate(&self.buf, b, nsym) else {
                self.search = b;
                return Some(true);
            };
            let payload = m.decode_payload(&h, &frame.symbols[hdr_syms..], &frame.noise_var);
            out.push(RxBurst { header: h, payload, snr_db: frame.snr_db, end_sample: self.base + frame.end as u64 });
            self.search = frame.end.min(self.buf.len());
            return Some(true);
        }

        let l = m.ofdm.p.symbol_len();
        match m.ofdm.detect(&self.buf, self.search) {
            Some(b) => {
                if self.buf.len() < self.needed(b, hdr_syms) {
                    // Come back when the header has arrived; re-detect from just before it.
                    self.search = b.saturating_sub(3 * l);
                    return Some(false);
                }
                let header = m
                    .ofdm
                    .demodulate(&self.buf, b, hdr_syms)
                    .and_then(|f| m.decode_header(&f.symbols, &f.noise_var).map(|h| (h, f)));
                match header {
                    None => self.search = b,
                    Some((h, f)) if h.ncw == 0 => {
                        out.push(RxBurst {
                            header: h,
                            payload: Vec::new(),
                            snr_db: f.snr_db,
                            end_sample: self.base + f.end as u64,
                        });
                        self.search = f.end.min(self.buf.len());
                    }
                    Some((h, _)) => {
                        debug_assert!((h.level as usize) < LEVELS.len());
                        self.pending = Some((b, h));
                    }
                }
                Some(true)
            }
            None => {
                // Nothing found. Keep enough history to catch a preamble that is still arriving.
                let keep = 3 * l + 2 * m.ofdm.p.nfft;
                self.search = self.search.max(self.buf.len().saturating_sub(keep));
                Some(false)
            }
        }
    }

    fn trim(&mut self) {
        let l = self.modem.ofdm.p.symbol_len();
        let keep_from = match self.pending {
            Some((b, _)) => b.saturating_sub(3 * l),
            None => self.search,
        };
        if keep_from > 20 * l {
            self.buf.drain(..keep_from);
            self.base += keep_from as u64;
            self.search -= keep_from;
            if let Some((b, _)) = &mut self.pending {
                *b -= keep_from;
            }
        }
    }
}
