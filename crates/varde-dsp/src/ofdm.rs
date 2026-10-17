//! Real-valued (audio) OFDM modulator and demodulator.
//!
//! Frame layout: `[preamble A][preamble B][data symbol 0] ... [data symbol n-1]`
//! - Preamble A uses only even FFT bins, so its time signal repeats every nfft/2
//!   samples. The receiver finds it with a Schmidl-Cox autocorrelation.
//! - Preamble B is a known BPSK symbol on every carrier. It gives exact timing
//!   (cross-correlation) and the initial channel estimate.
//! - Data symbols carry scattered pilots used to track gain, phase and timing drift
//!   (sound-card clocks differ by tens of ppm).

use crate::filter::Chain;
use crate::params::OfdmParams;
use crate::prng::Prng;
use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};
use std::f32::consts::SQRT_2;
use std::sync::Arc;

const PILOT_TABLE_LEN: usize = 32;
const DETECT_THRESHOLD: f64 = 0.3;
/// Pilots are averaged over this many symbols either side.
const TRACK_HALF_WINDOW: usize = 4;
const MIN_PREAMBLE_CORR: f32 = 0.3;

pub struct Ofdm {
    pub p: OfdmParams,
    fft: Arc<dyn Fft<f32>>,
    ifft: Arc<dyn Fft<f32>>,
    /// Carrier indices (0..num_carriers) that hold pilots.
    pub pilot_idx: Vec<usize>,
    /// Carrier indices that hold data.
    pub data_idx: Vec<usize>,
    pilot_table: Vec<Vec<f32>>,
    gains: Vec<f32>,
    pre_a: Vec<Complex32>,
    pre_b: Vec<Complex32>,
    /// Time-domain body (no cyclic prefix) of preamble B, unit energy.
    pre_b_body: Vec<f32>,
}

/// A demodulated frame.
pub struct Frame {
    /// Equalized data-carrier symbols, one Vec per OFDM symbol.
    pub symbols: Vec<Vec<Complex32>>,
    /// Estimated noise variance after equalization, per data carrier.
    pub noise_var: Vec<f32>,
    /// Average SNR per symbol (Es/N0) in dB.
    pub snr_db: f32,
    /// Sample index just past the end of the frame.
    pub end: usize,
}

impl Ofdm {
    pub fn new(p: OfdmParams) -> Self {
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(p.nfft);
        let ifft = planner.plan_fft_inverse(p.nfft);
        let nc = p.num_carriers;

        let (pilot_idx, data_idx): (Vec<usize>, Vec<usize>) =
            (0..nc).partition(|&k| k % p.pilot_spacing == 0 || k == nc - 1);

        let mut rng = Prng::new(0x5eed_0001);
        let pilot_table = (0..PILOT_TABLE_LEN)
            .map(|_| pilot_idx.iter().map(|_| rng.bpsk()).collect())
            .collect();

        let f0 = p.carrier_freq(0);
        let gains = (0..nc)
            .map(|k| {
                let octaves = (p.carrier_freq(k) / f0).log2();
                10f32.powf(p.tilt_db_per_octave * octaves / 20.0)
            })
            .collect();

        let mut rng = Prng::new(0x5eed_00a1);
        let pre_a = (0..nc)
            .map(|k| {
                let v = rng.bpsk();
                if (p.first_bin + k) % 2 == 0 {
                    Complex32::new(SQRT_2 * v, 0.0)
                } else {
                    Complex32::new(0.0, 0.0)
                }
            })
            .collect();
        let mut rng = Prng::new(0x5eed_00b2);
        let pre_b = (0..nc).map(|_| Complex32::new(rng.bpsk(), 0.0)).collect();

        let mut s = Self {
            p,
            fft,
            ifft,
            pilot_idx,
            data_idx,
            pilot_table,
            gains,
            pre_a,
            pre_b,
            pre_b_body: Vec::new(),
        };
        let b = s.symbol_time(&s.pre_b);
        let mut body = b[s.p.ncp..].to_vec();
        let e = body.iter().map(|x| x * x).sum::<f32>().sqrt();
        body.iter_mut().for_each(|x| *x /= e);
        s.pre_b_body = body;
        s
    }

    pub fn data_carriers(&self) -> usize {
        self.data_idx.len()
    }

    /// Length in samples of a frame with `nsym` data symbols.
    pub fn frame_len(&self, nsym: usize) -> usize {
        (nsym + 2) * self.p.symbol_len()
    }

    fn pilot_value(&self, sym: usize, n: usize) -> f32 {
        self.pilot_table[sym % PILOT_TABLE_LEN][n]
    }

    /// One OFDM symbol (cyclic prefix + body) from carrier values.
    fn symbol_time(&self, carriers: &[Complex32]) -> Vec<f32> {
        let n = self.p.nfft;
        let mut x = vec![Complex32::new(0.0, 0.0); n];
        for (k, &v) in carriers.iter().enumerate() {
            let bin = self.p.first_bin + k;
            let v = v * self.gains[k];
            x[bin] = v;
            x[n - bin] = v.conj();
        }
        self.ifft.process(&mut x);
        let scale = 1.0 / (n as f32).sqrt();
        let body: Vec<f32> = x.iter().map(|c| c.re * scale).collect();
        let mut out = Vec::with_capacity(self.p.symbol_len());
        out.extend_from_slice(&body[n - self.p.ncp..]);
        out.extend_from_slice(&body);
        out
    }

    /// Build a frame. Each entry of `data` holds `data_carriers()` symbols.
    pub fn modulate(&self, data: &[Vec<Complex32>]) -> Vec<f32> {
        self.modulate_with_leader(data, 0.0)
    }

    /// Build a frame preceded by at least `leader_s` seconds of filler symbols.
    /// The leader covers the radio's key-up time (PTT to carrier up and squelch open)
    /// and lets the receiver's audio levels settle. The filler is random QPSK, so it
    /// cannot be mistaken for preamble A.
    pub fn modulate_with_leader(&self, data: &[Vec<Complex32>], leader_s: f32) -> Vec<f32> {
        let nc = self.p.num_carriers;
        let n_leader = (leader_s / self.p.symbol_duration()).ceil() as usize;
        let mut out = Vec::with_capacity(self.frame_len(data.len()) + n_leader * self.p.symbol_len());
        let mut rng = Prng::new(0x5eed_00c3);
        for _ in 0..n_leader {
            let c: Vec<Complex32> = (0..nc)
                .map(|_| Complex32::new(rng.bpsk(), rng.bpsk()) * (1.0 / SQRT_2))
                .collect();
            out.extend(self.symbol_time(&c));
        }
        out.extend(self.symbol_time(&self.pre_a));
        out.extend(self.symbol_time(&self.pre_b));
        for (i, d) in data.iter().enumerate() {
            assert_eq!(d.len(), self.data_idx.len(), "wrong number of data symbols");
            let mut c = vec![Complex32::new(0.0, 0.0); nc];
            for (j, &k) in self.data_idx.iter().enumerate() {
                c[k] = d[j];
            }
            for (n, &k) in self.pilot_idx.iter().enumerate() {
                c[k] = Complex32::new(self.pilot_value(i, n), 0.0);
            }
            out.extend(self.symbol_time(&c));
        }
        let rms = (out.iter().map(|x| x * x).sum::<f32>() / out.len() as f32).sqrt();
        let g = self.p.tx_rms / rms.max(1e-12);
        out.iter_mut().for_each(|x| *x = (*x * g).clamp(-1.0, 1.0));
        out
    }

    /// FFT of the window starting at `pos`, returning only the carrier bins.
    fn carriers_at(&self, rx: &[f32], pos: isize) -> Option<Vec<Complex32>> {
        let n = self.p.nfft;
        if pos < 0 || pos as usize + n > rx.len() {
            return None;
        }
        let pos = pos as usize;
        let mut x: Vec<Complex32> = rx[pos..pos + n].iter().map(|&v| Complex32::new(v, 0.0)).collect();
        self.fft.process(&mut x);
        let scale = 1.0 / (n as f32).sqrt();
        Some(
            (0..self.p.num_carriers)
                .map(|k| x[self.p.first_bin + k] * scale)
                .collect(),
        )
    }

    /// Search `rx` from `from` for a frame. Returns the sample index where the
    /// body of preamble B starts.
    pub fn detect(&self, rx: &[f32], from: usize) -> Option<usize> {
        // Filtering keeps preamble A periodic (the filter is linear and time-invariant),
        // so the autocorrelation still works on the filtered copy.
        let filtered;
        let sc: &[f32] = match self.p.detect_lowpass_hz {
            Some(f) => {
                let mut lp = Chain::lowpass4(self.p.fs, f);
                filtered = rx[from..].iter().map(|&x| lp.process(x)).collect::<Vec<f32>>();
                &filtered
            }
            None => &rx[from..],
        };
        self.detect_in(rx, sc, from)
    }

    /// Schmidl-Cox search over `sc` (which starts at `rx[from]`), confirmed on `rx`.
    fn detect_in(&self, rx: &[f32], sc: &[f32], from: usize) -> Option<usize> {
        let n = self.p.nfft;
        let h = n / 2;
        let ncp = self.p.ncp;
        let l = self.p.symbol_len();
        if sc.len() < 3 * l {
            return None;
        }
        let x = |i: usize| sc[i] as f64;
        let mut p = 0f64;
        let mut r = 0f64;
        for m in 0..h {
            p += x(m) * x(m + h);
            r += x(m + h).powi(2);
        }
        let energy_floor = 1e-7 * h as f64;
        let mut plateau: Option<(usize, usize)> = None;
        let mut d = 0;
        while d + n + 2 * l < sc.len() {
            let metric = if r > energy_floor && p > 0.0 { p * p / (r * r) } else { 0.0 };
            match (&mut plateau, metric > DETECT_THRESHOLD) {
                (None, true) => plateau = Some((d, d)),
                (Some((_, end)), true) => *end = d,
                (Some((start, end)), false) => {
                    let (start, end) = (*start, *end);
                    plateau = None;
                    if end - start >= ncp / 4 {
                        let a0 = from + ((start + end) / 2).saturating_sub(ncp / 2);
                        if let Some(b) = self.fine_timing(rx, a0 + l + ncp) {
                            return Some(b);
                        }
                    }
                }
                (None, false) => {}
            }
            p += x(d + h) * x(d + n) - x(d) * x(d + h);
            r += x(d + n).powi(2) - x(d + h).powi(2);
            d += 1;
        }
        None
    }

    /// Cross-correlate with preamble B around `expected` and return the best start.
    fn fine_timing(&self, rx: &[f32], expected: usize) -> Option<usize> {
        let n = self.p.nfft;
        let span = self.p.ncp as isize;
        let mut best = (0.0f32, 0usize);
        for off in -span..=span {
            let s = expected as isize + off;
            if s < 0 || s as usize + n > rx.len() {
                continue;
            }
            let w = &rx[s as usize..s as usize + n];
            let e = w.iter().map(|x| x * x).sum::<f32>().sqrt();
            if e <= 0.0 {
                continue;
            }
            let c = w.iter().zip(&self.pre_b_body).map(|(a, b)| a * b).sum::<f32>().abs() / e;
            if c > best.0 {
                best = (c, s as usize);
            }
        }
        (best.0 >= MIN_PREAMBLE_CORR).then_some(best.1)
    }

    /// Demodulate `nsym` data symbols of a frame whose preamble B starts at `b_start`.
    pub fn demodulate(&self, rx: &[f32], b_start: usize, nsym: usize) -> Option<Frame> {
        let n = self.p.nfft;
        let l = self.p.symbol_len() as isize;
        let nc = self.p.num_carriers;
        let np = self.pilot_idx.len();
        // Centre the FFT window in the cyclic prefix. That leaves ncp/2 samples of
        // slack each way for sound-card clock drift without ever moving the window.
        let base = b_start as isize - (self.p.ncp / 2) as isize;

        let yb = self.carriers_at(rx, base)?;
        let raw: Vec<Complex32> = (0..nc).map(|k| yb[k] / self.pre_b[k]).collect();
        // Light smoothing across frequency (the audio channel response is smooth).
        // Remove the linear phase from the timing offset first, or smoothing would
        // average over a rotating phase.
        let rot: Complex32 = (1..nc).map(|k| raw[k] * raw[k - 1].conj()).sum();
        let slope = rot.arg();
        let flat: Vec<Complex32> = (0..nc)
            .map(|k| raw[k] * Complex32::from_polar(1.0, -slope * k as f32))
            .collect();
        let hb: Vec<Complex32> = (0..nc)
            .map(|k| {
                let lo = flat[k.saturating_sub(1)];
                let hi = flat[(k + 1).min(nc - 1)];
                (lo + flat[k] * 2.0 + hi) * 0.25 * Complex32::from_polar(1.0, slope * k as f32)
            })
            .collect();

        // Pass 1: FFT every symbol and compute pilot ratios against the preamble estimate.
        let mut ys = Vec::with_capacity(nsym);
        let mut cs: Vec<Vec<Complex32>> = Vec::with_capacity(nsym);
        for i in 0..nsym {
            let y = self.carriers_at(rx, base + (i as isize + 1) * l)?;
            cs.push(
                self.pilot_idx
                    .iter()
                    .enumerate()
                    .map(|(m, &k)| y[k] / (hb[k] * self.pilot_value(i, m)))
                    .collect(),
            );
            ys.push(y);
        }

        // Pass 2: average each pilot over neighbouring symbols, interpolate between
        // pilots, equalize. Each carrier only depends on nearby pilots, so noisy pilots
        // at one end of the band do not corrupt carriers at the other end.
        let mut pilot_err = vec![0f32; np];
        let mut symbols = Vec::with_capacity(nsym);
        for i in 0..nsym {
            let lo = i.saturating_sub(TRACK_HALF_WINDOW);
            let hi = (i + TRACK_HALF_WINDOW + 1).min(nsym);
            let cnt = (hi - lo) as f32;
            let avg: Vec<Complex32> = (0..np)
                .map(|m| cs[lo..hi].iter().map(|c| c[m]).sum::<Complex32>() / cnt)
                .collect();
            if cnt > 1.0 {
                let bias = cnt / (cnt - 1.0);
                for m in 0..np {
                    pilot_err[m] += (cs[i][m] / avg[m] - 1.0).norm_sqr() * bias;
                }
            }
            symbols.push(
                self.data_idx
                    .iter()
                    .map(|&k| ys[i][k] / (hb[k] * self.interp_pilots(&avg, k)))
                    .collect(),
            );
        }

        // Noise per data carrier, interpolated from the pilots. The preamble estimate
        // adds its own noise (3/8 of one symbol's, after smoothing) on top.
        let pilot_var: Vec<f32> = pilot_err
            .iter()
            .map(|e| e / nsym.max(1) as f32 * 1.375)
            .collect();
        let noise_var: Vec<f32> = self
            .data_idx
            .iter()
            .map(|&k| {
                let (m, t) = self.pilot_pos(k);
                (pilot_var[m] * (1.0 - t) + pilot_var[m + 1] * t).max(1e-6)
            })
            .collect();
        let mean_var = noise_var.iter().sum::<f32>() / noise_var.len() as f32;

        Some(Frame {
            symbols,
            noise_var,
            snr_db: -10.0 * mean_var.log10(),
            end: (base + (nsym as isize + 1) * l + n as isize) as usize,
        })
    }

    /// Index of the pilot at or below carrier `k` and the fractional position towards the next one.
    fn pilot_pos(&self, k: usize) -> (usize, f32) {
        let hi = self.pilot_idx.partition_point(|&p| p < k).clamp(1, self.pilot_idx.len() - 1);
        let (k0, k1) = (self.pilot_idx[hi - 1], self.pilot_idx[hi]);
        (hi - 1, (k as f32 - k0 as f32) / (k1 - k0) as f32)
    }

    /// Interpolate pilot values at carrier `k`: linear in magnitude and in phase.
    fn interp_pilots(&self, pilots: &[Complex32], k: usize) -> Complex32 {
        let (m, t) = self.pilot_pos(k);
        let (a, b) = (pilots[m], pilots[m + 1]);
        let mag = a.norm() * (1.0 - t) + b.norm() * t;
        let dphi = (b * a.conj()).arg();
        Complex32::from_polar(mag, a.arg() + dphi * t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::qam::Modulation;

    fn roundtrip(p: OfdmParams, m: Modulation, lead: usize) {
        let ofdm = Ofdm::new(p);
        let nsym = 20;
        let mut rng = Prng::new(42);
        let bits = rng.bits(nsym * ofdm.data_carriers() * m.bits_per_symbol());
        let syms = m.map_all(&bits);
        let data: Vec<Vec<Complex32>> = syms.chunks(ofdm.data_carriers()).map(|c| c.to_vec()).collect();
        let mut rx = vec![0.0; lead];
        rx.extend(ofdm.modulate(&data));
        rx.extend(vec![0.0; 2000]);

        let b = ofdm.detect(&rx, 0).expect("frame not detected");
        let frame = ofdm.demodulate(&rx, b, nsym).expect("demod failed");
        let mut out = Vec::new();
        for s in frame.symbols.iter().flatten() {
            m.hard_bits(*s, &mut out);
        }
        assert_eq!(out, bits);
        assert!(frame.snr_db > 30.0, "snr {}", frame.snr_db);
    }

    #[test]
    fn clean_roundtrip_narrow() {
        roundtrip(OfdmParams::narrow(), Modulation::Qam64, 3333);
    }

    #[test]
    fn clean_roundtrip_wide() {
        roundtrip(OfdmParams::wide(), Modulation::Qam64, 1234);
    }
}
