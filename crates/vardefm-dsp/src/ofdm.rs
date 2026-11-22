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
/// A symbol whose pilot noise exceeds this many times the typical level gets its own
/// noise scale. Normal symbols almost never get there (the per-symbol estimate averages
/// 18-24 real noise samples).
const SCALE_THRESHOLD: f32 = 2.0;
/// TX low-pass cutoff relative to the top carrier frequency.
const TX_LOWPASS_FACTOR: f32 = 1.15;
/// Clip-and-filter passes before the final clamp.
const CLIP_FILTER_PASSES: usize = 2;

/// How [`Ofdm::detect`] finds a burst.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Detector {
    /// Schmidl-Cox autocorrelation of preamble A, confirmed by preamble B.
    #[default]
    SchmidlCox,
}

impl Detector {
    pub const ALL: [Detector; 1] = [Self::SchmidlCox];

    pub fn name(self) -> &'static str {
        match self {
            Self::SchmidlCox => "sc",
        }
    }
}

impl std::fmt::Display for Detector {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

impl std::str::FromStr for Detector {
    type Err = String;

    /// A detector by its [`Detector::name`].
    fn from_str(s: &str) -> Result<Self, String> {
        Self::ALL.into_iter().find(|d| d.name() == s).ok_or_else(|| {
            let names: Vec<&str> = Self::ALL.iter().map(|d| d.name()).collect();
            format!("unknown detector {s:?} (one of {})", names.join(", "))
        })
    }
}

/// What a search from `from` found, and where the next one must start.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Detection {
    /// Where the body of preamble B starts, if a frame was found.
    pub at: Option<usize>,
    /// Where a caller searches on from: after a found frame whose header fails, or
    /// once more audio has arrived. Schmidl-Cox resumes at its detection, which can
    /// lie past the end of the plateau it scanned. `from <= resume <= max(from,
    /// rx.len())`, with `resume == from` when `from` is at or past the end, and
    /// `resume > from` when a frame was found. A caller that drops old audio keeps
    /// [`Ofdm::detect_history`] samples before `resume`: the next search warms its
    /// filters on them, and on cold filters a CTCSS tone rings at the search's start.
    pub resume: usize,
}

/// One OFDM symbol (cyclic prefix + body) from carrier values, with the TX tilt `gains`.
fn symbol_time(p: &OfdmParams, gains: &[f32], ifft: &dyn Fft<f32>, carriers: &[Complex32]) -> Vec<f32> {
    let n = p.nfft;
    let mut x = vec![Complex32::new(0.0, 0.0); n];
    for (k, &v) in carriers.iter().enumerate() {
        let bin = p.first_bin + k;
        let v = v * gains[k];
        x[bin] = v;
        x[n - bin] = v.conj();
    }
    ifft.process(&mut x);
    let scale = 1.0 / (n as f32).sqrt();
    let body: Vec<f32> = x.iter().map(|c| c.re * scale).collect();
    let mut out = Vec::with_capacity(p.symbol_len());
    out.extend_from_slice(&body[n - p.ncp..]);
    out.extend_from_slice(&body);
    out
}

pub struct Ofdm {
    pub p: OfdmParams,
    /// Burst detector used by [`Ofdm::detect`].
    detector: Detector,
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
    /// [`pre_b_template`] of preamble B's body at the mode's high-pass corner.
    pre_b_hp: Vec<f32>,
}

/// A demodulated frame.
pub struct Frame {
    /// Equalized data-carrier symbols, one Vec per OFDM symbol.
    pub symbols: Vec<Vec<Complex32>>,
    /// Estimated noise variance after equalization, per data carrier.
    pub noise_var: Vec<f32>,
    /// Noise scale per OFDM symbol (at least 1). The noise variance of symbol `i`,
    /// data carrier `j` is `noise_var[j] * symbol_noise_scale[i]`; it is large for a
    /// symbol hit by an FM click, a squelch pop or a short fade.
    pub symbol_noise_scale: Vec<f32>,
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
        let pilot_table = (0..PILOT_TABLE_LEN).map(|_| pilot_idx.iter().map(|_| rng.bpsk()).collect()).collect();

        let f0 = p.carrier_freq(0);
        let gains: Vec<f32> = (0..nc)
            .map(|k| {
                let octaves = (p.carrier_freq(k) / f0).log2();
                10f32.powf(p.tilt_db_per_octave * octaves / 20.0)
            })
            .collect();

        let mut rng = Prng::new(0x5eed_00a1);
        let pre_a: Vec<Complex32> = (0..nc)
            .map(|k| {
                let v = rng.bpsk();
                if (p.first_bin + k).is_multiple_of(2) {
                    Complex32::new(SQRT_2 * v, 0.0)
                } else {
                    Complex32::new(0.0, 0.0)
                }
            })
            .collect();
        let mut rng = Prng::new(0x5eed_00b2);
        let pre_b: Vec<Complex32> = (0..nc).map(|_| Complex32::new(rng.bpsk(), 0.0)).collect();

        let sym_b = symbol_time(&p, &gains, ifft.as_ref(), &pre_b);
        // Time-domain body (no cyclic prefix) of preamble B, unit energy.
        let mut pre_b_body = sym_b[p.ncp..].to_vec();
        let e = pre_b_body.iter().map(|x| x * x).sum::<f32>().sqrt();
        pre_b_body.iter_mut().for_each(|x| *x /= e);
        let pre_b_hp = pre_b_template(&pre_b_body, p.fs, p.detect_band_hz.0);
        Self {
            p,
            detector: Detector::default(),
            fft,
            ifft,
            pilot_idx,
            data_idx,
            pilot_table,
            gains,
            pre_a,
            pre_b,
            pre_b_hp,
        }
    }

    /// This modem with [`Ofdm::detect`] finding bursts with `detector`.
    pub fn with_detector(mut self, detector: Detector) -> Self {
        self.detector = detector;
        self
    }

    /// The detector [`Ofdm::detect`] uses.
    pub fn detector(&self) -> Detector {
        self.detector
    }

    /// Leader symbols sent for a leader of at least `leader_s` seconds.
    pub fn leader_symbols(&self, leader_s: f32) -> usize {
        (leader_s / self.p.symbol_duration()).ceil() as usize
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
        symbol_time(&self.p, &self.gains, self.ifft.as_ref(), carriers)
    }

    /// Build a frame. Each entry of `data` holds `data_carriers()` symbols.
    pub fn modulate(&self, data: &[Vec<Complex32>]) -> Vec<f32> {
        self.modulate_with_leader(data, 0.0)
    }

    /// Build a frame preceded by at least `leader_s` seconds of filler symbols, at the
    /// mode's reference level `p.tx_rms`.
    /// The leader covers the radio's key-up time (PTT to carrier up and squelch open)
    /// and lets the receiver's audio levels settle. The filler is random QPSK, so it
    /// cannot be mistaken for preamble A.
    pub fn modulate_with_leader(&self, data: &[Vec<Complex32>], leader_s: f32) -> Vec<f32> {
        self.modulate_with_leader_rms(data, leader_s, self.p.tx_rms)
    }

    /// Like [`Ofdm::modulate_with_leader`], at RMS level `rms` (1.0 = full scale).
    ///
    /// The frame is scaled to `rms` before clipping, so the level is exact unless peaks
    /// are clipped. Clipping splatters far outside the carriers, and in wide mode the
    /// audio goes straight to the radio's VCO, so the output is shaped:
    /// 1. A low-pass just above the top carrier removes the sidelobes of the symbol edges.
    /// 2. Clip and filter: what clipping at full scale would remove is low-passed and
    ///    subtracted. The correction is filtered forwards and backwards (zero phase), so it
    ///    stays aligned with the peaks it cuts; each pass leaves less overshoot.
    /// 3. A final clamp to ±1.0 catches what overshoot is left.
    ///
    /// The low-pass is linear and time-invariant and the corrections only touch peaks, so
    /// preamble A stays periodic enough for detection.
    pub fn modulate_with_leader_rms(&self, data: &[Vec<Complex32>], leader_s: f32, rms: f32) -> Vec<f32> {
        let mut out = self.frame_samples(data, leader_s);
        let now = (out.iter().map(|x| x * x).sum::<f32>() / out.len() as f32).sqrt();
        let g = rms / now.max(1e-12);
        out.iter_mut().for_each(|x| *x *= g);
        let lowpass = || Chain::lowpass4(self.p.fs, self.tx_lowpass_hz());
        lowpass().process_buf(&mut out);
        for _ in 0..CLIP_FILTER_PASSES {
            let mut e: Vec<f32> = out.iter().map(|x| x.clamp(-1.0, 1.0) - x).collect();
            if e.iter().all(|&v| v == 0.0) {
                break;
            }
            lowpass().process_buf(&mut e);
            e.reverse();
            lowpass().process_buf(&mut e);
            e.reverse();
            out.iter_mut().zip(&e).for_each(|(x, e)| *x += e);
        }
        out.iter_mut().for_each(|x| *x = x.clamp(-1.0, 1.0));
        out
    }

    /// Unscaled samples of a frame with its leader.
    fn frame_samples(&self, data: &[Vec<Complex32>], leader_s: f32) -> Vec<f32> {
        let nc = self.p.num_carriers;
        let n_leader = self.leader_symbols(leader_s);
        let mut out = Vec::with_capacity(self.frame_len(data.len()) + n_leader * self.p.symbol_len());
        let mut rng = Prng::new(0x5eed_00c3);
        for _ in 0..n_leader {
            let c: Vec<Complex32> = (0..nc).map(|_| Complex32::new(rng.bpsk(), rng.bpsk()) * (1.0 / SQRT_2)).collect();
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
        out
    }

    /// Cutoff of the TX splatter filter.
    pub fn tx_lowpass_hz(&self) -> f32 {
        self.p.carrier_freq(self.p.num_carriers - 1) * TX_LOWPASS_FACTOR
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
        Some((0..self.p.num_carriers).map(|k| x[self.p.first_bin + k] * scale).collect())
    }

    /// Search `rx` from `from` for a frame with the modem's detector. See [`Detection`].
    pub fn detect(&self, rx: &[f32], from: usize) -> Detection {
        self.detect_with(self.detector, rx, from)
    }

    /// Like [`Ofdm::detect`], with the given detector.
    pub fn detect_with(&self, detector: Detector, rx: &[f32], from: usize) -> Detection {
        if from >= rx.len() {
            return Detection { at: None, resume: from };
        }
        let found = match detector {
            Detector::SchmidlCox => {
                let at = self.detect_sc(rx, from);
                // Nothing found: search again over the end that could hold a preamble
                // still arriving, preambles A and B and margin for the fine timing
                // around them. (Not the filter warm-up, `detect_history`, which the
                // search reads before its start by itself.)
                let rescan = 3 * self.p.symbol_len() + 2 * self.p.nfft;
                Detection { at, resume: at.unwrap_or(from.max(rx.len().saturating_sub(rescan))) }
            }
        };
        // Checked in every build: a caller that searches on from `resume` after a
        // detection would loop forever if it did not move on, so fail instead.
        assert!(found.resume >= from && found.resume <= rx.len(), "{detector}: {found:?} from {from}");
        assert!(found.at.is_none() || found.resume > from, "{detector}: {found:?} from {from}");
        found
    }

    /// Schmidl-Cox search, the original detector.
    fn detect_sc(&self, rx: &[f32], from: usize) -> Option<usize> {
        self.detect_chunked(rx, from, 8 * self.p.symbol_len())
    }

    /// Samples before `from` that [`detect`](Self::detect) reads to settle its filters.
    /// A caller that drops old audio should keep this much before where it searches next.
    pub fn detect_history(&self) -> usize {
        detect_warmup(self.p.fs, self.p.detect_band_hz.0)
    }

    /// [`detect`](Self::detect), filtering `chunk` samples past what the search needs
    /// each time it runs out (fewer, larger steps; the result does not depend on it).
    fn detect_chunked(&self, rx: &[f32], from: usize, chunk: usize) -> Option<usize> {
        // Filtering keeps preamble A periodic (the filter is linear and time-invariant),
        // so the autocorrelation still works on the filtered copy. See
        // `OfdmParams::detect_band_hz` for the band.
        let (lo, hi) = self.p.detect_band_hz;
        let mut band = Band::new(rx, from, self.p.fs, lo, hi);
        self.detect_in(&mut band, &self.pre_b_hp, from, chunk)
    }

    /// Schmidl-Cox search over the band-passed copy of `rx[from..]` in `band`,
    /// confirmed on its high-passed copy against preamble B's `template`.
    ///
    /// The filters run only as far as the search reads (plus `chunk`), so a search that
    /// stops early (at a burst) costs no more than the audio it has looked at. Each
    /// read is preceded by a fill to the sample it needs.
    fn detect_in(&self, band: &mut Band, template: &[f32], from: usize, chunk: usize) -> Option<usize> {
        let n = self.p.nfft;
        let h = n / 2;
        let ncp = self.p.ncp;
        let l = self.p.symbol_len();
        let len = band.len();
        if len < 3 * l {
            return None;
        }
        band.fill_to(n);
        let mut p = 0f64;
        let mut r = 0f64;
        {
            let x = |i: usize| band.sc()[i] as f64;
            for m in 0..h {
                p += x(m) * x(m + h);
                r += x(m + h).powi(2);
            }
        }
        let energy_floor = 1e-7 * h as f64;
        let mut plateau: Option<(usize, usize)> = None;
        let mut d = 0;
        while d + n + 2 * l < len {
            // The update below reads up to `d + n`.
            if band.filled() <= d + n {
                band.fill_to(d + n + 1 + chunk);
            }
            let metric = if r > energy_floor && p > 0.0 { p * p / (r * r) } else { 0.0 };
            match (&mut plateau, metric > DETECT_THRESHOLD) {
                (None, true) => plateau = Some((d, d)),
                (Some((_, end)), true) => *end = d,
                (Some((start, end)), false) => {
                    let (start, end) = (*start, *end);
                    plateau = None;
                    if end - start >= ncp / 4 {
                        let a0 = from + ((start + end) / 2).saturating_sub(ncp / 2);
                        let expected = a0 - from + l + ncp;
                        // `fine_timing` reads up to `expected + ncp + n`.
                        band.fill_to(expected + ncp + n);
                        if let Some(b) = self.fine_timing(band.hp(), template, expected) {
                            return Some(from + b);
                        }
                    }
                }
                (None, false) => {}
            }
            let x = |i: usize| band.sc()[i] as f64;
            p += x(d + h) * x(d + n) - x(d) * x(d + h);
            r += x(d + n).powi(2) - x(d + h).powi(2);
            d += 1;
        }
        None
    }

    /// Cross-correlate `x` with preamble B's `template` around `expected` and return
    /// the best start.
    ///
    /// Detection passes the high-passed audio and the template filtered the same way,
    /// so the timing carries no filter delay. On unfiltered audio a sub-tone would leak
    /// into the correlation with preamble B's lowest carriers (281 Hz in wide) and
    /// swell the energy it is normalised by, and the confirmation would fail.
    fn fine_timing(&self, x: &[f32], template: &[f32], expected: usize) -> Option<usize> {
        let n = self.p.nfft;
        let span = self.p.ncp as isize;
        let mut best = (0.0f32, 0usize);
        for off in -span..=span {
            let s = expected as isize + off;
            if s < 0 || s as usize + n > x.len() {
                continue;
            }
            let w = &x[s as usize..s as usize + n];
            let e = w.iter().map(|x| x * x).sum::<f32>().sqrt();
            if e <= 0.0 {
                continue;
            }
            let c = w.iter().zip(template).map(|(a, b)| a * b).sum::<f32>().abs() / e;
            if c > best.0 {
                best = (c, s as usize);
            }
        }
        (best.0 >= MIN_PREAMBLE_CORR).then_some(best.1)
    }

    /// Demodulate `nsym` data symbols of a frame whose preamble B starts at `b_start`.
    /// Returns `None` if `nsym` is 0 or the frame runs past the end of `rx`.
    pub fn demodulate(&self, rx: &[f32], b_start: usize, nsym: usize) -> Option<Frame> {
        self.demodulate_known(rx, b_start, nsym, &[])
    }

    /// Like [`demodulate`](Self::demodulate), but the data-carrier values of the first
    /// `known.len()` symbols are known (for example a header that has already been
    /// decoded). Those symbols train the channel estimate together with preamble B.
    pub fn demodulate_known(&self, rx: &[f32], b_start: usize, nsym: usize, known: &[Vec<Complex32>]) -> Option<Frame> {
        if nsym == 0 {
            return None;
        }
        let n = self.p.nfft;
        let l = self.p.symbol_len() as isize;
        let nc = self.p.num_carriers;
        // Centre the FFT window in the cyclic prefix. That leaves ncp/2 samples of
        // slack each way for sound-card clock drift without ever moving the window.
        let base = b_start as isize - (self.p.ncp / 2) as isize;

        let yb = self.carriers_at(rx, base)?;
        let ys: Vec<Vec<Complex32>> =
            (0..nsym).map(|i| self.carriers_at(rx, base + (i as isize + 1) * l)).collect::<Option<_>>()?;

        // Channel estimate: preamble B averaged with the known symbols. Clock drift moves
        // the timing by at most a sample or so over the header; the pilot tracking below
        // takes that out, so the symbols are simply averaged. (Removing each symbol's own
        // phase slope instead costs up to 2 dB at low SNR: the slope estimate is noisy and
        // its error grows across the band.)
        let nk = known.len().min(nsym);
        let mut obs: Vec<Vec<Complex32>> = Vec::with_capacity(nk + 1);
        obs.push((0..nc).map(|k| yb[k] / self.pre_b[k]).collect());
        for (i, d) in known[..nk].iter().enumerate() {
            assert_eq!(d.len(), self.data_idx.len(), "wrong number of known symbols");
            let mut o = vec![Complex32::new(0.0, 0.0); nc];
            for (j, &k) in self.data_idx.iter().enumerate() {
                o[k] = ys[i][k] / d[j];
            }
            for (m, &k) in self.pilot_idx.iter().enumerate() {
                o[k] = ys[i][k] * self.pilot_value(i, m);
            }
            obs.push(o);
        }
        // A header symbol hit by an FM click can still decode (the header code is rate
        // 0.24) and so be offered as training. At full weight it would corrupt the
        // estimate for every carrier of the burst, which the per-symbol noise scale
        // below cannot undo. So each training symbol is weighted by the inverse of its
        // noise, and `ntrain` is the effective number of symbols averaged.
        let weights = training_weights(&obs);
        let ntrain: f32 = weights.iter().sum();
        let acc: Vec<Complex32> =
            (0..nc).map(|k| obs.iter().zip(&weights).map(|(o, &w)| o[k] * w).sum::<Complex32>() / ntrain).collect();
        // Light [1 2 1]/4 smoothing across frequency (the audio channel response is
        // smooth). It still pays with the header averaged in: dropping it costs about
        // 0.5 dB in wide mode. Remove the linear phase from the timing offset first, or
        // smoothing would average over a rotating phase.
        let (slope, flat) = remove_slope(&acc);
        let hb: Vec<Complex32> = (0..nc)
            .map(|k| {
                let lo = flat[k.saturating_sub(1)];
                let hi = flat[(k + 1).min(nc - 1)];
                (lo + flat[k] * 2.0 + hi) * 0.25 * Complex32::from_polar(1.0, slope * k as f32)
            })
            .collect();
        // Noise the estimate adds to every equalized symbol, relative to one symbol's:
        // the kernel passes 3/8 of the noise of the `ntrain` averaged symbols.
        let est_noise = 0.375 / ntrain;

        // Pilot ratios against the channel estimate.
        let cs: Vec<Vec<Complex32>> = ys
            .iter()
            .enumerate()
            .map(|(i, y)| {
                self.pilot_idx.iter().enumerate().map(|(m, &k)| y[k] / (hb[k] * self.pilot_value(i, m))).collect()
            })
            .collect();

        // Average each pilot over neighbouring symbols, weighting each symbol by the
        // inverse of its noise. The first pass (equal weights) finds symbols hit by an
        // FM click or a fade; the second keeps them out of their neighbours' averages.
        let (_, dev) = track_pilots(&cs, &vec![1.0; nsym]);
        let (scale, _) = noise_model(&dev);
        let (avg, dev) = track_pilots(&cs, &scale);
        let (scale, pilot_var) = noise_model(&dev);

        // Interpolate between pilots and equalize. Each carrier only depends on nearby
        // pilots, so noisy pilots at one end of the band do not corrupt carriers at the
        // other end.
        let symbols: Vec<Vec<Complex32>> = (0..nsym)
            .map(|i| self.data_idx.iter().map(|&k| ys[i][k] / (hb[k] * self.interp_pilots(&avg[i], k))).collect())
            .collect();

        // Noise per data carrier, interpolated from the pilots, plus the noise of the
        // channel estimate.
        let raw_var: Vec<f32> = self
            .data_idx
            .iter()
            .map(|&k| {
                let (m, t) = self.pilot_pos(k);
                (pilot_var[m] * (1.0 - t) + pilot_var[m + 1] * t) * (1.0 + est_noise)
            })
            .collect();
        let noise_var: Vec<f32> = raw_var.iter().map(|v| v.max(1e-6)).collect();
        // The reported SNR averages the noise over all symbols, click-hit ones included,
        // as it did before there was a per-symbol scale. Scale the unclamped variance:
        // on a nearly noiseless frame the floor would otherwise be multiplied too.
        let mean_var = raw_var.iter().sum::<f32>() / raw_var.len() as f32;
        let mean_scale = scale.iter().sum::<f32>() / nsym.max(1) as f32;

        Some(Frame {
            symbols,
            noise_var,
            symbol_noise_scale: scale,
            snr_db: -10.0 * (mean_var * mean_scale.max(1.0)).max(1e-6).log10(),
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

/// The body of preamble B (`body`) as it comes out of the high-pass at `lo` Hz in
/// steady state (preamble B is cyclic, so a few periods settle the filter), scaled to
/// the energy of the unfiltered body. Correlating high-passed audio against it gives
/// the same timing as unfiltered audio against the body: both carry the same delay.
fn pre_b_template(body: &[f32], fs: f32, lo: f32) -> Vec<f32> {
    let n = body.len();
    let mut f = Chain::highpass8(fs, lo);
    let periods = 4;
    let y: Vec<f32> = body.iter().cycle().take(periods * n).map(|&x| f.process(x)).collect();
    let mut t = y[(periods - 1) * n..].to_vec();
    let norm = |v: &[f32]| v.iter().map(|x| x * x).sum::<f32>().sqrt();
    let g = norm(body) / norm(&t).max(1e-12);
    t.iter_mut().for_each(|x| *x *= g);
    t
}

/// The detection band of `src`, filtered on demand: `hp` is the high-passed copy and
/// `lp` (when the band has an upper edge) the high- then low-passed copy, each
/// covering `src[..filled]`. Filtering sample by sample from the start gives the same
/// values as filtering all of `src` at once. A NaN goes in as 0 and a huge or infinite
/// sample clamped (see [`sanitize`]), so neither can leave the filters' state NaN and
/// hide every burst after it.
///
/// The filters are run over up to [`detect_warmup`] samples before `src` first, so a
/// search does not start on their start-up transient: in a strong tone, the high-pass
/// would otherwise ring at its corner over the first milliseconds of every search.
struct Band<'a> {
    src: &'a [f32],
    hp_f: Chain,
    lp_f: Option<Chain>,
    hp: Vec<f32>,
    lp: Vec<f32>,
}

impl<'a> Band<'a> {
    /// The band of `rx[from..]`, its filters warmed up on the audio before `from`.
    fn new(rx: &'a [f32], from: usize, fs: f32, lo: f32, hi: Option<f32>) -> Self {
        let from = from.min(rx.len());
        let mut hp_f = Chain::highpass8(fs, lo);
        let mut lp_f = hi.map(|hi| Chain::lowpass4(fs, hi));
        for &x in &rx[from.saturating_sub(detect_warmup(fs, lo))..from] {
            let y = hp_f.process(sanitize(x));
            if let Some(lp) = &mut lp_f {
                lp.process(y);
            }
        }
        Self { src: &rx[from..], hp_f, lp_f, hp: Vec::new(), lp: Vec::new() }
    }

    fn len(&self) -> usize {
        self.src.len()
    }

    fn filled(&self) -> usize {
        self.hp.len()
    }

    /// Filter on to `end` samples, or to the end of `src`.
    fn fill_to(&mut self, end: usize) {
        let end = end.min(self.src.len());
        for &x in &self.src[self.hp.len().min(end)..end] {
            let y = self.hp_f.process(sanitize(x));
            self.hp.push(y);
            if let Some(lp) = &mut self.lp_f {
                self.lp.push(lp.process(y));
            }
        }
    }

    /// The high-passed copy, for the preamble B confirmation.
    fn hp(&self) -> &[f32] {
        &self.hp
    }

    /// The copy the Schmidl-Cox search runs on: band-passed when the band has an upper
    /// edge, else high-passed.
    fn sc(&self) -> &[f32] {
        if self.lp_f.is_some() {
            &self.lp
        } else {
            &self.hp
        }
    }
}

/// Samples of history [`Band`] warms its filters on: about ten time constants of the
/// high-pass's slowest-decaying section (Q 2.56, time constant Q / (pi lo)), so its
/// start-up transient has decayed by about 85 dB when the search starts.
fn detect_warmup(fs: f32, lo: f32) -> usize {
    (8.0 * fs / lo).ceil() as usize
}

/// A sample as detection's filters take it: a NaN as 0, anything else clamped to
/// [`MAX_SAMPLE`](crate::MAX_SAMPLE). The streaming receiver has already done that; a
/// recording read whole (`decode_all`, a corrupt float WAV) has not.
fn sanitize(x: f32) -> f32 {
    if x.is_nan() {
        0.0
    } else {
        x.clamp(-crate::MAX_SAMPLE, crate::MAX_SAMPLE)
    }
}

/// Split the linear phase (the timing offset) off a per-carrier channel estimate.
/// Returns the slope in radians per carrier and the estimate without it.
fn remove_slope(raw: &[Complex32]) -> (f32, Vec<Complex32>) {
    let rot: Complex32 = raw.windows(2).map(|w| w[1] * w[0].conj()).sum();
    let slope = rot.arg();
    (slope, raw.iter().enumerate().map(|(k, &h)| h * Complex32::from_polar(1.0, -slope * k as f32)).collect())
}

/// Average each pilot over up to `TRACK_HALF_WINDOW` symbols either side, weighting
/// symbol `i` by `1 / scale[i]`. Returns the averages and each pilot's squared
/// deviation from its average, corrected for the pilot's own share of that average.
fn track_pilots(cs: &[Vec<Complex32>], scale: &[f32]) -> (Vec<Vec<Complex32>>, Vec<Vec<f32>>) {
    let nsym = cs.len();
    let np = cs.first().map_or(0, |c| c.len());
    let mut avgs = Vec::with_capacity(nsym);
    let mut devs = Vec::with_capacity(nsym);
    for i in 0..nsym {
        let lo = i.saturating_sub(TRACK_HALF_WINDOW);
        let hi = (i + TRACK_HALF_WINDOW + 1).min(nsym);
        let wsum: f32 = scale[lo..hi].iter().map(|s| 1.0 / s).sum();
        let avg: Vec<Complex32> =
            (0..np).map(|m| (lo..hi).map(|j| cs[j][m] / scale[j]).sum::<Complex32>() / wsum).collect();
        // With weights matched to the noise, the deviation's variance is the symbol's
        // own noise times (1 - its share of the average).
        let share = 1.0 / scale[i] / wsum;
        let dev = if share < 0.999 {
            let bias = 1.0 / (1.0 - share);
            (0..np).map(|m| (cs[i][m] / avg[m] - 1.0).norm_sqr() * bias).collect()
        } else {
            vec![0.0; np]
        };
        avgs.push(avg);
        devs.push(dev);
    }
    (avgs, devs)
}

/// Weight per channel observation (one per training symbol): 1 for a symbol whose
/// deviation from the plain average is at most `SCALE_THRESHOLD` times the typical
/// level, otherwise typical / own, so that a symbol hit by a click adds almost nothing.
/// Fewer than three observations cannot show an outlier and all get weight 1.
fn training_weights(obs: &[Vec<Complex32>]) -> Vec<f32> {
    let n = obs.len();
    if n < 3 {
        return vec![1.0; n];
    }
    let nc = obs[0].len();
    let mean: Vec<Complex32> = (0..nc).map(|k| obs.iter().map(|o| o[k]).sum::<Complex32>() / n as f32).collect();
    let e: Vec<f32> = obs.iter().map(|o| o.iter().zip(&mean).map(|(x, h)| (x - h).norm_sqr()).sum()).collect();
    let reference = robust_reference(&e);
    e.iter().map(|x| x / reference).map(|r| if r > SCALE_THRESHOLD { 1.0 / r } else { 1.0 }).collect()
}

/// Typical level of `e`: the mean over the values not far above the median, so that
/// the outliers (clicks) do not raise it.
fn robust_reference(e: &[f32]) -> f32 {
    let mut sorted = e.to_vec();
    sorted.sort_by(f32::total_cmp);
    let median = sorted[e.len() / 2];
    let (sum, cnt) = e.iter().filter(|&&x| x <= 3.0 * median).fold((0.0, 0), |(s, c), &x| (s + x, c + 1));
    (sum / cnt.max(1) as f32).max(1e-12)
}

/// Fit the pilot deviations to a separable noise model: variance = per-pilot variance
/// × per-symbol scale. Returns (scale per symbol, at least 1; variance per pilot).
///
/// A symbol's scale is 1 unless its noise is more than `SCALE_THRESHOLD` times the
/// typical level: with 9-12 pilots per symbol the estimate is noisy, and the point is
/// to catch symbols hit by a click, not to fine-tune the rest. (Scaling every symbol by
/// `max(1, e/mean)` cost a little at the 64QAM levels.)
fn noise_model(dev: &[Vec<f32>]) -> (Vec<f32>, Vec<f32>) {
    let nsym = dev.len();
    let np = dev.first().map_or(0, |d| d.len());
    let per_pilot = |scale: &[f32]| -> Vec<f32> {
        (0..np).map(|m| dev.iter().zip(scale).map(|(d, s)| d[m] / s).sum::<f32>() / nsym.max(1) as f32).collect()
    };
    let ones = vec![1.0; nsym];
    let var = per_pilot(&ones);
    if nsym < 3 {
        return (ones, var);
    }
    // Noise energy per symbol, each pilot relative to its own average.
    let e: Vec<f32> =
        dev.iter().map(|d| d.iter().zip(&var).map(|(x, v)| x / v.max(1e-12)).sum::<f32>() / np as f32).collect();
    let reference = robust_reference(&e);
    let scale: Vec<f32> = e.iter().map(|x| x / reference).map(|r| if r > SCALE_THRESHOLD { r } else { 1.0 }).collect();
    let var = per_pilot(&scale);
    (scale, var)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::qam::Modulation;
    use crate::CTCSS_HZ;

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

        let b = ofdm.detect(&rx, 0).at.expect("frame not detected");
        let frame = ofdm.demodulate(&rx, b, nsym).expect("demod failed");
        let mut out = Vec::new();
        for s in frame.symbols.iter().flatten() {
            m.hard_bits(*s, &mut out);
        }
        assert_eq!(out, bits);
        assert!(frame.snr_db > 30.0, "snr {}", frame.snr_db);
        // No symbols is not a frame (the noise model would have no pilots to fit).
        assert!(ofdm.demodulate(&rx, b, 0).is_none());
    }

    #[test]
    fn clean_roundtrip_narrow() {
        roundtrip(OfdmParams::narrow(), Modulation::Qam64, 3333);
    }

    #[test]
    fn clean_roundtrip_wide() {
        roundtrip(OfdmParams::wide(), Modulation::Qam64, 1234);
    }

    /// Every EIA CTCSS tone at 1500 Hz deviation (twice the usual), on the mode's
    /// channel deviation (2.5 kHz narrow, 3 kHz wide), at two phases: detection finds
    /// the frame where it finds it without a tone (#12).
    #[test]
    fn ctcss_tones_do_not_hide_a_frame() {
        let mut missed = Vec::new();
        for (p, deviation_hz) in [(OfdmParams::narrow(), 2500.0), (OfdmParams::wide(), 3000.0)] {
            let ofdm = Ofdm::new(p);
            let mut rx = vec![0.0; 1000];
            rx.extend(ofdm.modulate_with_leader(&random_frame(&ofdm, Modulation::Qpsk, 4, 1), 0.15));
            rx.extend(vec![0.0; 1000]);
            let truth = ofdm.detect(&rx, 0).at.expect("frame not detected without a tone");
            let a = 1500.0 / deviation_hz;
            for f in CTCSS_HZ {
                for ph in [0.0, 1.9] {
                    let w = 2.0 * std::f32::consts::PI * f / ofdm.p.fs;
                    let x: Vec<f32> = rx.iter().enumerate().map(|(i, v)| v + a * (w * i as f32 + ph).sin()).collect();
                    let got = ofdm.detect(&x, 0).at;
                    // As `detector_eval` counts a hit: within a quarter cyclic prefix.
                    if !got.is_some_and(|b| b.abs_diff(truth) <= ofdm.p.ncp / 4) {
                        missed.push(format!("{} {f} Hz phase {ph}: {got:?}, want {truth}", ofdm.p.name));
                    }
                }
            }
        }
        assert!(missed.is_empty(), "{missed:#?}");
    }

    /// The receiver starts a new search on every push, so preamble A can land in the
    /// first milliseconds of a search. In a strong CTCSS tone, a search that starts
    /// over the 25 ms before preamble A finds the frame exactly where a search from the
    /// start of the recording does. Starts within two cyclic prefixes of preamble A are
    /// left out: a search that starts within about 1.5 ms of preamble A can miss it, with
    /// or without a tone. Streaming a 20 dB burst in pushes of 256 to 4096 samples, the
    /// receiver's search that found it started at least 17 ms (wide) or 35 ms (narrow)
    /// before preamble A.
    #[test]
    fn a_search_starting_just_before_a_frame_finds_it_in_a_tone() {
        let mut missed = Vec::new();
        for (p, deviation_hz) in [(OfdmParams::narrow(), 2500.0), (OfdmParams::wide(), 3000.0)] {
            let ofdm = Ofdm::new(p);
            let (l, ncp) = (ofdm.p.symbol_len(), ofdm.p.ncp);
            let mut rx = vec![0.0; 4000];
            rx.extend(ofdm.modulate_with_leader(&random_frame(&ofdm, Modulation::Qpsk, 4, 1), 0.15));
            rx.extend(vec![0.0; 1000]);
            let truth = ofdm.detect(&rx, 0).at.expect("frame not detected without a tone");
            let a_start = truth - ncp - l;
            let a = 1500.0 / deviation_hz;
            // Starts over the 25 ms before preamble A, every half millisecond.
            let step = (ofdm.p.fs / 2000.0) as usize;
            let span = (ofdm.p.fs * 0.025) as usize;
            for f in CTCSS_HZ {
                for ph in [0.0, 1.9] {
                    let w = 2.0 * std::f32::consts::PI * f / ofdm.p.fs;
                    let x: Vec<f32> = rx.iter().enumerate().map(|(i, v)| v + a * (w * i as f32 + ph).sin()).collect();
                    let settled = ofdm.detect(&x, 0).at;
                    assert!(settled.is_some_and(|b| b.abs_diff(truth) <= ncp / 4), "{} {f} Hz", ofdm.p.name);
                    for from in (a_start - span..=a_start - 2 * ncp).step_by(step) {
                        let got = ofdm.detect(&x, from).at;
                        if got != settled {
                            missed.push(format!(
                                "{} {f} Hz phase {ph} from A - {}: {got:?}, want {settled:?} (no tone {truth})",
                                ofdm.p.name,
                                a_start - from
                            ));
                        }
                    }
                }
            }
        }
        assert!(missed.is_empty(), "{} misses: {missed:#?}", missed.len());
    }

    /// A search's filters start settled: the band of `rx[from..]` is the band of all of
    /// `rx` from `from` on, to within 1e-4 of a strong tone's amplitude, so a search does
    /// not start on the high-pass ringing at its corner.
    #[test]
    fn detection_filters_start_settled() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let (lo, hi) = ofdm.p.detect_band_hz;
            let mut rng = Prng::new(11);
            for f in [CTCSS_HZ[0], CTCSS_HZ[20]] {
                let a = 0.6;
                let w = 2.0 * std::f32::consts::PI * f / ofdm.p.fs;
                let rx: Vec<f32> = (0..20_000).map(|i| a * (w * i as f32 + 0.7).sin() + 0.01 * rng.bpsk()).collect();
                let mut all = Band::new(&rx, 0, ofdm.p.fs, lo, hi);
                all.fill_to(rx.len());
                for from in [ofdm.detect_history(), 5000, 12_345] {
                    let mut band = Band::new(&rx, from, ofdm.p.fs, lo, hi);
                    band.fill_to(rx.len());
                    for (name, got, want) in
                        [("hp", band.hp(), &all.hp()[from..]), ("sc", band.sc(), &all.sc()[from..])]
                    {
                        let err = got.iter().zip(want).map(|(g, w)| (g - w).abs()).fold(0.0, f32::max);
                        assert!(err < 1e-4 * a, "{} {f} Hz from {from}, {name}: off by {err}", ofdm.p.name);
                    }
                }
            }
        }
    }

    /// `detect` filters only as far as its search reads. On a recording with several
    /// bursts, a CTCSS tone and noise, it finds what a search over the whole recording
    /// filtered up front finds, from the start, from just past each burst and from
    /// starts spread over the recording, with the default fill step and with none (each
    /// fill exactly to the sample read next).
    #[test]
    fn lazy_detection_filtering_matches_filtering_everything() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let (lo, hi) = ofdm.p.detect_band_hz;
            let mut rng = Prng::new(7);
            let mut rx = vec![0.0; 1500];
            for seed in 0..3 {
                rx.extend(ofdm.modulate_with_leader(&random_frame(&ofdm, Modulation::Qpsk, 4, seed), 0.1));
                rx.extend(vec![0.0; 3000 + 700 * seed as usize]);
            }
            let w = 2.0 * std::f32::consts::PI * 100.0 / ofdm.p.fs;
            let rx: Vec<f32> =
                rx.iter().enumerate().map(|(i, v)| v + 0.3 * (w * i as f32).sin() + 0.01 * rng.bpsk()).collect();
            let eager = |from: usize| {
                let mut band = Band::new(&rx, from, ofdm.p.fs, lo, hi);
                band.fill_to(rx.len());
                ofdm.detect_in(&mut band, &ofdm.pre_b_hp, from, 0)
            };
            let lazy = |from: usize| {
                let got = ofdm.detect(&rx, from).at;
                assert_eq!(ofdm.detect_chunked(&rx, from, 0), got, "{} from {from}, no fill step", ofdm.p.name);
                got
            };
            let mut found = 0;
            let mut from = 0;
            while let Some(b) = eager(from) {
                assert_eq!(lazy(from), Some(b), "{} from {from}", ofdm.p.name);
                found += 1;
                from = b + 1;
            }
            assert_eq!(lazy(from), None, "{} from {from}", ofdm.p.name);
            assert_eq!(found, 3, "{}", ofdm.p.name);
            for from in (0..rx.len()).step_by(997) {
                assert_eq!(lazy(from), eager(from), "{} from {from}", ofdm.p.name);
            }
        }
    }

    /// A NaN, an infinite or a huge finite sample before a frame, or a long run of exact silence,
    /// does not leave detection's filters stuck: the frame after it is still found
    /// where it is found without them, and silence alone finds nothing.
    #[test]
    fn detection_recovers_from_nan_inf_and_silence() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let tol = ofdm.p.ncp / 4;
            let silence = vec![0.0; 10 * ofdm.p.fs as usize];
            assert_eq!(ofdm.detect(&silence, 0).at, None, "{}", ofdm.p.name);
            let mut rx = silence.clone();
            rx.extend(ofdm.modulate_with_leader(&random_frame(&ofdm, Modulation::Qpsk, 4, 3), 0.1));
            rx.extend(vec![0.0; 2000]);
            let truth = ofdm.detect(&rx, 0).at.expect("frame not detected");
            for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 3e38, -3e38] {
                let mut x = rx.clone();
                x[1000..1010].fill(bad);
                let got = ofdm.detect(&x, 0).at;
                assert!(got.is_some_and(|b| b.abs_diff(truth) <= tol), "{} {bad}: {got:?}, want {truth}", ofdm.p.name);
            }
        }
    }

    /// Where the body of preamble B starts in a frame with a leader of `leader_s`,
    /// after `lead` samples of silence.
    fn known_start(ofdm: &Ofdm, lead: usize, leader_s: f32) -> usize {
        lead + (ofdm.leader_symbols(leader_s) + 1) * ofdm.p.symbol_len() + ofdm.p.ncp
    }

    #[test]
    fn every_detector_finds_the_known_start() {
        // Within a quarter of the cyclic prefix, as the detector evaluation counts a hit;
        // the TX low-pass delays the waveform by a few samples.
        for p in OfdmParams::all() {
            let ofdm = Ofdm::new(p);
            let data = random_frame(&ofdm, Modulation::Qpsk, 20, 3);
            for (lead, sign) in [(0, 1.0), (777, 1.0), (4321, -1.0)] {
                let mut rx = vec![0.0; lead];
                rx.extend(ofdm.modulate_with_leader(&data, 0.1).iter().map(|v| v * sign));
                rx.extend(vec![0.0; 2000]);
                let want = known_start(&ofdm, lead, 0.1);
                for d in Detector::ALL {
                    let got = ofdm.detect_with(d, &rx, 0).at.unwrap_or_else(|| panic!("{d} found nothing"));
                    let name = ofdm.p.name;
                    assert!(got.abs_diff(want) <= ofdm.p.ncp / 4, "{name} {d}: lead {lead}: {got}, want {want}");
                }
            }
        }
    }

    #[test]
    fn every_detector_finds_the_burst_resuming_as_told() {
        // A streaming receiver calls detect on a growing buffer and, while nothing is
        // found, searches again from `resume`. A detection must still be the one the
        // whole recording gives.
        for p in OfdmParams::all() {
            let ofdm = Ofdm::new(p);
            let data = random_frame(&ofdm, Modulation::Qpsk, 20, 4);
            let mut rx = vec![0.0; 1000];
            rx.extend(ofdm.modulate_with_leader(&data, 0.1));
            for d in Detector::ALL {
                let full = ofdm.detect_with(d, &rx, 0).at.unwrap_or_else(|| panic!("{d} found nothing"));
                for step in [1, 97, 480] {
                    let mut search = 0;
                    let got = (step..rx.len()).step_by(step).find_map(|end| {
                        let found = ofdm.detect_with(d, &rx[..end], search);
                        search = found.resume;
                        found.at
                    });
                    assert_eq!(got, Some(full), "{} {d}: chunks of {step}", ofdm.p.name);
                }
            }
        }
    }

    #[test]
    fn every_detector_finds_a_burst_once_when_resuming() {
        // Searching on from `resume` after each detection, as after a header that
        // fails, must not find the same burst again.
        for p in OfdmParams::all() {
            let ofdm = Ofdm::new(p);
            let data = random_frame(&ofdm, Modulation::Qpsk, 20, 6);
            let mut rx = vec![0.0; 5000];
            rx.extend(ofdm.modulate_with_leader(&data, 0.1));
            rx.extend(vec![0.0; 5000]);
            for d in Detector::ALL {
                let mut found = Vec::new();
                let mut from = 0;
                loop {
                    let det = ofdm.detect_with(d, &rx, from);
                    let Some(b) = det.at else { break };
                    found.push(b);
                    from = det.resume;
                }
                assert_eq!(found.len(), 1, "{} {d}: found {found:?}", ofdm.p.name);
            }
        }
    }

    #[test]
    fn every_detector_keeps_its_resume_contract() {
        // `from <= resume <= len`, and a detection moves the search on, wherever the
        // buffer ends and the search starts: in silence, in noise, in a burst.
        for p in OfdmParams::all() {
            let ofdm = Ofdm::new(p);
            let l = ofdm.p.symbol_len();
            let data = random_frame(&ofdm, Modulation::Qpsk, 4, 8);
            let mut rx = vec![0.0; 3 * l];
            let mut rng = Prng::new(9);
            rx.extend((0..5 * l).map(|_| 0.3 * rng.uniform()));
            rx.extend(ofdm.modulate_with_leader(&data, 0.1));
            for d in Detector::ALL {
                for end in (0..=rx.len()).step_by(l / 3 + 1) {
                    for from in (0..=end + 1).step_by(l / 2 + 1) {
                        let det = ofdm.detect_with(d, &rx[..end], from);
                        let name = ofdm.p.name;
                        assert!(det.resume >= from, "{name} {d}: {det:?} from {from} in {end}");
                        assert!(from > end || det.resume <= end, "{name} {d}: {det:?} from {from} in {end}");
                        assert!(det.at.is_none() || det.resume > from, "{name} {d}: {det:?} from {from} in {end}");
                    }
                }
            }
        }
    }

    #[test]
    fn every_detector_ignores_polarity() {
        // Some radios invert the audio. Exactly the same sample either way.
        for p in OfdmParams::all() {
            let ofdm = Ofdm::new(p);
            let data = random_frame(&ofdm, Modulation::Qpsk, 20, 3);
            let mut rx = vec![0.0; 777];
            rx.extend(ofdm.modulate_with_leader(&data, 0.1));
            let mut rng = Prng::new(11);
            rx.iter_mut().for_each(|v| *v += 0.05 * rng.uniform());
            let inverted: Vec<f32> = rx.iter().map(|v| -v).collect();
            for d in Detector::ALL {
                let got = ofdm.detect_with(d, &rx, 0);
                assert!(got.at.is_some(), "{} {d}: nothing found", ofdm.p.name);
                assert_eq!(ofdm.detect_with(d, &inverted, 0), got, "{} {d}", ofdm.p.name);
            }
        }
    }

    #[test]
    fn every_detector_returns_none_past_the_end() {
        for p in OfdmParams::all() {
            let ofdm = Ofdm::new(p);
            let rx = vec![0.0; 1000];
            for d in Detector::ALL {
                for from in [rx.len(), rx.len() + 1] {
                    let det = ofdm.detect_with(d, &rx, from);
                    assert_eq!(det, Detection { at: None, resume: from }, "{} {d}", ofdm.p.name);
                }
            }
        }
    }

    #[test]
    fn detector_list_is_complete() {
        // A variant missing from `ALL` could not be chosen, benchmarked or evaluated.
        // A new variant needs an arm here, and its index only fits if `ALL` grew.
        let index = |d: Detector| match d {
            Detector::SchmidlCox => 0,
        };
        for (i, d) in Detector::ALL.into_iter().enumerate() {
            assert_eq!(index(d), i, "{d}");
            assert_eq!(d.name().parse::<Detector>(), Ok(d));
        }
        let max = Detector::ALL.into_iter().map(index).max();
        assert_eq!(max, Some(Detector::ALL.len() - 1));
    }

    fn random_frame(ofdm: &Ofdm, m: Modulation, nsym: usize, seed: u32) -> Vec<Vec<Complex32>> {
        let mut rng = Prng::new(seed);
        let bits = rng.bits(nsym * ofdm.data_carriers() * m.bits_per_symbol());
        m.map_all(&bits).chunks(ofdm.data_carriers()).map(|c| c.to_vec()).collect()
    }

    fn rms(x: &[f32]) -> f32 {
        (x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32).sqrt()
    }

    /// Power in each [lo, hi) Hz band, from a Hann-windowed Welch average.
    fn band_power(x: &[f32], fs: f32, bands: &[(f32, f32)]) -> Vec<f32> {
        let n = 2048;
        let fft = FftPlanner::new().plan_fft_forward(n);
        let win: Vec<f32> =
            (0..n).map(|i| 0.5 - 0.5 * (2.0 * std::f32::consts::PI * i as f32 / n as f32).cos()).collect();
        let mut psd = vec![0f32; n / 2];
        for blk in x.windows(n).step_by(n / 2) {
            let mut b: Vec<Complex32> = blk.iter().zip(&win).map(|(&v, &w)| Complex32::new(v * w, 0.0)).collect();
            fft.process(&mut b);
            psd.iter_mut().zip(&b).for_each(|(p, c)| *p += c.norm_sqr());
        }
        let hz = fs / n as f32;
        bands
            .iter()
            .map(|&(lo, hi)| {
                psd.iter().enumerate().filter(|(i, _)| (lo..hi).contains(&(*i as f32 * hz))).map(|(_, p)| p).sum()
            })
            .collect()
    }

    #[test]
    fn frame_rms_matches_request() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let data = random_frame(&ofdm, Modulation::Qam16, 40, 7);
            // Levels with (almost) no clipping: only the filter can move the level.
            for want in [0.05, 0.1, 0.2] {
                let got = rms(&ofdm.modulate_with_leader_rms(&data, 0.1, want));
                assert!((got / want - 1.0).abs() < 0.03, "{} rms {got} want {want}", ofdm.p.name);
            }
            assert_eq!(ofdm.modulate(&data), ofdm.modulate_with_leader_rms(&data, 0.0, ofdm.p.tx_rms));
        }
    }

    #[test]
    fn clipping_splatter_is_filtered() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let fs = ofdm.p.fs;
            let top = ofdm.p.carrier_freq(ofdm.p.num_carriers - 1);
            let bands = [(ofdm.p.carrier_freq(0), top), (1.3 * top, fs / 2.0)];
            let data = random_frame(&ofdm, Modulation::Qam64, 60, 9);
            // Louder than the loudest level the modem uses (0.5).
            let level = 0.55;
            let tx = ofdm.modulate_with_leader_rms(&data, 0.1, level);
            assert!(tx.iter().all(|v| v.abs() <= 1.0));
            let pw = band_power(&tx, fs, &bands);
            let oob_db = 10.0 * (pw[1] / pw[0]).log10();
            // The same frame scaled and clipped without shaping, for comparison.
            let mut raw = ofdm.frame_samples(&data, 0.1);
            let g = level / rms(&raw);
            raw.iter_mut().for_each(|v| *v = (*v * g).clamp(-1.0, 1.0));
            let clipped = raw.iter().filter(|v| v.abs() >= 1.0).count() as f32 / raw.len() as f32;
            assert!(clipped > 0.04, "{}: only {clipped} of samples clipped", ofdm.p.name);
            let pr = band_power(&raw, fs, &bands);
            let raw_db = 10.0 * (pr[1] / pr[0]).log10();
            println!("{}: out-of-band {oob_db:.1} dB (clipped only: {raw_db:.1} dB)", ofdm.p.name);
            assert!(oob_db < -30.0, "{}: out-of-band power {oob_db:.1} dB", ofdm.p.name);
        }
    }

    #[test]
    fn clipped_frame_still_decodes() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let m = Modulation::Qpsk;
            let mut rng = Prng::new(5);
            let bits = rng.bits(20 * ofdm.data_carriers() * m.bits_per_symbol());
            let data: Vec<Vec<Complex32>> = m.map_all(&bits).chunks(ofdm.data_carriers()).map(|c| c.to_vec()).collect();
            let mut rx = vec![0.0; 777];
            rx.extend(ofdm.modulate_with_leader_rms(&data, 0.1, 0.5));
            rx.extend(vec![0.0; 2000]);
            let b = ofdm.detect(&rx, 0).at.expect("frame not detected");
            let frame = ofdm.demodulate(&rx, b, data.len()).expect("demod failed");
            let mut out = Vec::new();
            for s in frame.symbols.iter().flatten() {
                m.hard_bits(*s, &mut out);
            }
            assert_eq!(out, bits, "{}", ofdm.p.name);
            println!("{}: SNR at rms 0.5 without noise {:.1} dB", ofdm.p.name, frame.snr_db);
        }
    }
}
