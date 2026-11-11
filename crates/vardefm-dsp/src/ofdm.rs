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
/// Normalized correlation with preambles A and B that marks a burst for the matched
/// filter. In ten minutes each of FM channel noise and keyed tones the largest value
/// was 0.31, in white noise 0.17 ([`Ofdm::mf_peak`], reported by `detector_eval` in
/// vardefm-modem with `NOISE_SECONDS=600`). Bursts at a CNR where none decodes
/// still cross it, as the evaluation's `hit` counts show for the simulator's channel.
pub const MF_THRESHOLD: f32 = 0.4;
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
    /// Normalized cross-correlation with the known waveform of preambles A and B.
    MatchedFilter,
}

impl Detector {
    pub const ALL: [Detector; 2] = [Self::SchmidlCox, Self::MatchedFilter];

    pub fn name(self) -> &'static str {
        match self {
            Self::SchmidlCox => "sc",
            Self::MatchedFilter => "mf",
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

/// Matched filter for preambles A and B, evaluated by FFT (overlap-save).
struct MatchedFilter {
    /// Template length: preambles A and B with their cyclic prefixes.
    len: usize,
    /// Square root of the template energy.
    norm: f32,
    fft: Arc<dyn Fft<f32>>,
    ifft: Arc<dyn Fft<f32>>,
    /// Conjugate spectrum of the zero-padded template, scaled by 1/FFT size.
    spec: Vec<Complex32>,
}

impl MatchedFilter {
    fn new(tpl: Vec<f32>) -> Self {
        let len = tpl.len();
        // At least twice the template, so each block yields more outputs than it costs.
        let n = (2 * len).next_power_of_two();
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(n);
        let ifft = planner.plan_fft_inverse(n);
        let norm = tpl.iter().map(|x| x * x).sum::<f32>().sqrt();
        let mut spec: Vec<Complex32> = tpl.into_iter().map(|v| Complex32::new(v, 0.0)).collect();
        spec.resize(n, Complex32::new(0.0, 0.0));
        fft.process(&mut spec);
        spec.iter_mut().for_each(|c| *c = c.conj() / n as f32);
        Self { len, norm, fft, ifft, spec }
    }
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
    pub detector: Detector,
    mf: MatchedFilter,
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

        let mut tpl = symbol_time(&p, &gains, ifft.as_ref(), &pre_a);
        tpl.extend(symbol_time(&p, &gains, ifft.as_ref(), &pre_b));
        let mf = MatchedFilter::new(tpl);
        let mut s = Self {
            p,
            detector: Detector::default(),
            mf,
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
        let n_leader = (leader_s / self.p.symbol_duration()).ceil() as usize;
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

    /// Search `rx` from `from` for a frame with [`Ofdm::detector`]. Returns the sample
    /// index where the body of preamble B starts.
    pub fn detect(&self, rx: &[f32], from: usize) -> Option<usize> {
        self.detect_with(self.detector, rx, from)
    }

    /// Like [`Ofdm::detect`], with the given detector.
    pub fn detect_with(&self, detector: Detector, rx: &[f32], from: usize) -> Option<usize> {
        match detector {
            Detector::SchmidlCox => self.detect_sc(rx, from),
            Detector::MatchedFilter => self.detect_mf(rx, from),
        }
    }

    /// How many samples before the end of `rx` a streaming caller must search again
    /// from after [`Ofdm::detect`] returned `None`, so as not to lose a frame that has
    /// only partly arrived.
    pub fn detect_history(&self) -> usize {
        let l = self.p.symbol_len();
        match self.detector {
            // Preambles A and B, and margin for the fine timing around them.
            Detector::SchmidlCox => 3 * l + 2 * self.p.nfft,
            // The template, then the one-symbol search window after its first crossing.
            Detector::MatchedFilter => self.mf.len + l,
        }
    }

    /// Matched-filter search: normalized cross-correlation of `rx` with the waveform of
    /// preambles A and B. The FM audio channel has no frequency offset and a smooth
    /// response, so the known waveform correlates coherently. Unlike the Schmidl-Cox
    /// metric it multiplies noise by a clean template, never by more noise, so it
    /// keeps working at a lower signal-to-noise ratio.
    ///
    /// The first position above `MF_THRESHOLD` opens a search window one symbol long
    /// (preamble A repeats every nfft/2 samples, so a side peak can cross first); the
    /// largest correlation in it is the frame. If the window does not fit in `rx`,
    /// returns `None` so that a streaming caller searches again with more audio.
    ///
    /// The correlation is computed in FFT blocks that start at `from`, so a later call
    /// with another `from` rounds it differently. That can only matter for a value
    /// within float rounding of `MF_THRESHOLD`, or for two peaks in the window within
    /// float rounding of each other, where the other one may win.
    ///
    /// A finite sample so large that the f32 FFT overflows (beyond about 1e35, far
    /// outside any sound card's range) blanks the block it falls in.
    pub fn detect_mf(&self, rx: &[f32], from: usize) -> Option<usize> {
        let l = self.p.symbol_len();
        let mut peak: Option<(usize, f32, usize)> = None;
        let mut found = None;
        self.mf_scan(rx, from, |d, v| {
            match &mut peak {
                None if v > MF_THRESHOLD => peak = Some((d, v, d)),
                Some((first, best, at)) => {
                    if v > *best {
                        (*best, *at) = (v, d);
                    }
                    if d - *first >= l {
                        // The template starts with preamble A's cyclic prefix.
                        found = Some(from + *at + l + self.p.ncp);
                        return false;
                    }
                }
                None => {}
            }
            true
        });
        found
    }

    /// The largest matched-filter correlation anywhere in `rx`, 0 to 1. On audio
    /// without a burst, its distance below `MF_THRESHOLD` is the detector's margin
    /// against false detections.
    pub fn mf_peak(&self, rx: &[f32]) -> f32 {
        let mut peak = 0f32;
        self.mf_scan(rx, 0, |_, v| {
            peak = peak.max(v);
            true
        });
        peak
    }

    /// The normalized correlation of the preamble template with `rx` at every window
    /// start `from + d`, in order, passed to `visit(d, value)` until it returns false.
    fn mf_scan(&self, rx: &[f32], from: usize, mut visit: impl FnMut(usize, f32) -> bool) {
        let mf = &self.mf;
        let t = mf.len;
        let n = mf.spec.len();
        let hop = n - t + 1;
        // Correlation windows start at from + d for d in 0..count.
        let Some(count) = rx.len().checked_sub(from).and_then(|r| (r + 1).checked_sub(t)) else {
            return;
        };
        let x = &rx[from..];
        // A NaN or infinity would spoil a whole FFT block and the running energy, so
        // it counts as silence.
        let at = |i: usize| x.get(i).copied().filter(|v| v.is_finite()).unwrap_or(0.0);
        let sq = |i: usize| (at(i) as f64).powi(2);
        // Windows quieter than -90 dBFS RMS, exact silence among them, score 0 rather
        // than whatever rounding leaves in the running energy.
        let floor = 1e-9 * t as f64;
        let mut buf = vec![Complex32::new(0.0, 0.0); n];
        let mut start = 0;
        while start < count {
            for (i, c) in buf.iter_mut().enumerate() {
                *c = Complex32::new(at(start + i), 0.0);
            }
            mf.fft.process(&mut buf);
            buf.iter_mut().zip(&mf.spec).for_each(|(c, s)| *c *= s);
            mf.ifft.process(&mut buf);
            // Window energy, exact at each block start and then updated sample by sample.
            let mut energy: f64 = (start..start + t).map(sq).sum();
            for (j, c) in buf.iter().take(hop.min(count - start)).enumerate() {
                let d = start + j;
                if j > 0 {
                    energy += sq(d + t - 1) - sq(d - 1);
                }
                let v = if energy > floor { c.re.abs() / (mf.norm * energy.sqrt() as f32) } else { 0.0 };
                if !visit(d, v) {
                    return;
                }
            }
            start += hop;
        }
    }

    /// Schmidl-Cox search, the original detector.
    pub fn detect_sc(&self, rx: &[f32], from: usize) -> Option<usize> {
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

    /// Where the body of preamble B starts in `lead` samples of silence followed by
    /// `data` with a leader of `leader_s`.
    fn known_start(ofdm: &Ofdm, lead: usize, leader_s: f32) -> usize {
        let n_leader = (leader_s / ofdm.p.symbol_duration()).ceil() as usize;
        lead + (n_leader + 1) * ofdm.p.symbol_len() + ofdm.p.ncp
    }

    #[test]
    fn every_detector_finds_the_known_start() {
        // Within a quarter of the cyclic prefix, as the detector evaluation counts a hit;
        // the TX low-pass delays the waveform by a few samples.
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let mut ofdm = Ofdm::new(p);
            let data = random_frame(&ofdm, Modulation::Qpsk, 20, 3);
            for (lead, sign) in [(0, 1.0), (777, 1.0), (4321, -1.0)] {
                let mut rx = vec![0.0; lead];
                rx.extend(ofdm.modulate_with_leader(&data, 0.1).iter().map(|v| v * sign));
                rx.extend(vec![0.0; 2000]);
                let want = known_start(&ofdm, lead, 0.1);
                for d in Detector::ALL {
                    ofdm.detector = d;
                    let got = ofdm.detect(&rx, 0).unwrap_or_else(|| panic!("{d} found nothing"));
                    let name = ofdm.p.name;
                    assert!(got.abs_diff(want) <= ofdm.p.ncp / 4, "{name} {d}: lead {lead}: {got}, want {want}");
                }
            }
        }
    }

    #[test]
    fn every_detector_finds_the_burst_with_a_receivers_history() {
        // A streaming receiver calls detect on a growing buffer. After `None` it keeps
        // only `detect_history` samples to search again; a detection must still be the
        // one the whole recording gives.
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let mut ofdm = Ofdm::new(p);
            let data = random_frame(&ofdm, Modulation::Qpsk, 20, 4);
            let mut rx = vec![0.0; 1000];
            rx.extend(ofdm.modulate_with_leader(&data, 0.1));
            for d in Detector::ALL {
                ofdm.detector = d;
                let full = ofdm.detect(&rx, 0).unwrap_or_else(|| panic!("{d} found nothing"));
                for step in [1, 97, 480] {
                    let mut search = 0;
                    let got = (step..rx.len()).step_by(step).find_map(|end| {
                        let b = ofdm.detect(&rx[..end], search);
                        if b.is_none() {
                            search = search.max(end.saturating_sub(ofdm.detect_history()));
                        }
                        b
                    });
                    assert_eq!(got, Some(full), "{} {d}: chunks of {step}", ofdm.p.name);
                }
            }
        }
    }

    #[test]
    fn matched_filter_waits_for_the_whole_search_window() {
        // Whatever the buffer holds and wherever the search starts, a detection must be
        // the one the whole recording gives.
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let data = random_frame(&ofdm, Modulation::Qpsk, 20, 4);
            let mut rx = vec![0.0; 1000];
            rx.extend(ofdm.modulate_with_leader(&data, 0.1));
            let full = ofdm.detect_mf(&rx, 0).expect("mf found nothing");
            for from in [0, 333, 1000] {
                let mut found = 0;
                for end in (from..rx.len()).step_by(97) {
                    if let Some(b) = ofdm.detect_mf(&rx[..end], from) {
                        assert_eq!(b, full, "{}: from {from}, buffer of {end} samples", ofdm.p.name);
                        found += 1;
                    }
                }
                assert!(found > 0, "{}: from {from}: no buffer gave a detection", ofdm.p.name);
            }
        }
    }

    #[test]
    fn detector_list_is_complete() {
        // A variant missing from `ALL` could not be chosen, benchmarked or evaluated.
        // This match stops compiling when a variant is added, as a reminder.
        let count = |d: Detector| match d {
            Detector::SchmidlCox | Detector::MatchedFilter => 2,
        };
        assert_eq!(Detector::ALL.len(), count(Detector::default()));
        for d in Detector::ALL {
            assert_eq!(d.name().parse::<Detector>(), Ok(d));
        }
    }

    #[test]
    fn matched_filter_ignores_silence_noise_and_tones() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let n = 2 * p.fs as usize;
            let mut rng = Prng::new(99);
            let noise: Vec<f32> = (0..n).map(|_| 0.3 * rng.uniform()).collect();
            let tone: Vec<f32> =
                (0..n).map(|i| 0.3 * (2.0 * std::f32::consts::PI * 1500.0 * i as f32 / p.fs).sin()).collect();
            let ofdm = Ofdm::new(p);
            for (name, x) in [("silence", vec![0.0; n]), ("noise", noise.clone()), ("1500 Hz", tone.clone())] {
                assert!(ofdm.detect_mf(&x, 0).is_none(), "{}: false frame in {name}", ofdm.p.name);
            }
        }
    }

    #[test]
    fn matched_filter_recovers_from_bad_samples_silence_and_long_noise() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let fs = p.fs as usize;
            let ofdm = Ofdm::new(p);
            let data = random_frame(&ofdm, Modulation::Qpsk, 20, 5);
            // A weak burst, to check that loud audio before it leaves no trace.
            let burst: Vec<f32> = ofdm.modulate_with_leader(&data, 0.1).iter().map(|v| 0.01 * v).collect();
            let mut alone = vec![0.0; 1000];
            alone.extend(&burst);
            let at = ofdm.detect_mf(&alone, 0).expect("mf found nothing");

            // Two minutes of full-scale noise with NaN and infinities in it, then a
            // minute of exact silence, then the burst.
            let mut rng = Prng::new(7);
            let mut rx: Vec<f32> = (0..120 * fs).map(|_| rng.uniform()).collect();
            for (i, v) in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, f32::NAN].into_iter().enumerate() {
                rx[(i + 1) * 25 * fs] = v;
            }
            rx.extend(vec![0.0; 60 * fs]);
            let offset = rx.len();
            rx.extend(&alone);
            let name = ofdm.p.name;
            assert_eq!(ofdm.detect_mf(&rx, 0), Some(offset + at), "{name}: burst after noise and silence");
            assert!(ofdm.detect_mf(&rx[..offset], 0).is_none(), "{name}: detection in the noise and silence");
            assert!(ofdm.mf_peak(&rx[..offset]) < MF_THRESHOLD, "{name}: peak in the noise and silence");

            // Bad samples count as silence, so those right around the preambles cost
            // nothing. The first sample after the template is always in the FFT block
            // that finds the burst.
            let (l, ncp) = (ofdm.p.symbol_len(), ofdm.p.ncp);
            let tpl_start = at - l - ncp;
            let mut rx = alone.clone();
            rx[tpl_start - 1] = f32::INFINITY;
            rx[tpl_start + 2 * l] = f32::NAN;
            assert_eq!(ofdm.detect_mf(&rx, 0), Some(at), "{name}: bad samples before the burst");
        }
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
            let b = ofdm.detect(&rx, 0).expect("frame not detected");
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
