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
/// Order of the header's channel fit. Order 2 already matches a genie that trains
/// on the known header in the channel simulator; 4 leaves room for band edges
/// steeper than its 4th-order audio filters, at under 0.1 dB.
const HEADER_FIT_ORDER: usize = 4;
/// TX low-pass cutoff relative to the top carrier frequency.
const TX_LOWPASS_FACTOR: f32 = 1.15;
/// Clip-and-filter passes before the final clamp.
const CLIP_FILTER_PASSES: usize = 2;

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
                if (p.first_bin + k).is_multiple_of(2) {
                    Complex32::new(SQRT_2 * v, 0.0)
                } else {
                    Complex32::new(0.0, 0.0)
                }
            })
            .collect();
        let mut rng = Prng::new(0x5eed_00b2);
        let pre_b = (0..nc).map(|_| Complex32::new(rng.bpsk(), 0.0)).collect();

        let mut s =
            Self { p, fft, ifft, pilot_idx, data_idx, pilot_table, gains, pre_a, pre_b, pre_b_body: Vec::new() };
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
    /// Returns `None` if `nsym` is 0 or the frame runs past the end of `rx`.
    pub fn demodulate(&self, rx: &[f32], b_start: usize, nsym: usize) -> Option<Frame> {
        self.demodulate_known(rx, b_start, nsym, &[])
    }

    /// Like [`demodulate`](Self::demodulate), for a burst's header: nothing is known
    /// yet but preamble B and the pilots, and the channel estimate is a low-order fit
    /// across carriers to both (see `estimate_fit`). Its small bias at the band edges
    /// is far below the header's noise, but would limit dense constellations, so
    /// payload symbols use [`demodulate_known`](Self::demodulate_known).
    pub fn demodulate_header(&self, rx: &[f32], b_start: usize, nsym: usize) -> Option<Frame> {
        self.demodulate_with(rx, b_start, nsym, &[], Estimator::Fit { order: HEADER_FIT_ORDER })
    }

    /// Like [`demodulate`](Self::demodulate), but the data-carrier values of the first
    /// `known.len()` symbols are known (for example a header that has already been
    /// decoded). Those symbols train the channel estimate together with preamble B.
    pub fn demodulate_known(&self, rx: &[f32], b_start: usize, nsym: usize, known: &[Vec<Complex32>]) -> Option<Frame> {
        let known: Vec<Option<&[Complex32]>> = known.iter().take(nsym).map(|d| Some(d.as_slice())).collect();
        self.demodulate_with(rx, b_start, nsym, &known, Estimator::Smooth)
    }

    /// Demodulate with an explicit training set and estimator. `known[i]`, where
    /// present and `Some`, holds the data-carrier values of symbol `i`.
    pub fn demodulate_with(
        &self,
        rx: &[f32],
        b_start: usize,
        nsym: usize,
        known: &[Option<&[Complex32]>],
        est: Estimator,
    ) -> Option<Frame> {
        if nsym == 0 {
            return None;
        }
        let l = self.p.symbol_len() as isize;
        // Centre the FFT window in the cyclic prefix. That leaves ncp/2 samples of
        // slack each way for sound-card clock drift without ever moving the window.
        let base = b_start as isize - (self.p.ncp / 2) as isize;

        let yb = self.carriers_at(rx, base)?;
        let ys: Vec<Vec<Complex32>> =
            (0..nsym).map(|i| self.carriers_at(rx, base + (i as isize + 1) * l)).collect::<Option<_>>()?;

        let (hb, est_noise) = match est {
            Estimator::Smooth => self.estimate_smooth(&yb, &ys, known),
            Estimator::Fit { order } => self.estimate_fit(&yb, &ys, known, order),
        };
        self.equalize(ys, hb, &est_noise, base, nsym)
    }

    /// Channel estimate from preamble B and the known symbols, averaged and smoothed
    /// [1 2 1]/4 across carriers. Returns the estimate and, per carrier, its noise
    /// relative to one symbol's.
    fn estimate_smooth(
        &self,
        yb: &[Complex32],
        ys: &[Vec<Complex32>],
        known: &[Option<&[Complex32]>],
    ) -> (Vec<Complex32>, Vec<f32>) {
        let nc = self.p.num_carriers;
        // Channel estimate: preamble B averaged with the known symbols. Clock drift moves
        // the timing by at most a sample or so over the header; the pilot tracking below
        // takes that out, so the symbols are simply averaged. (Removing each symbol's own
        // phase slope instead costs up to 2 dB at low SNR: the slope estimate is noisy and
        // its error grows across the band.)
        let mut obs: Vec<Vec<Complex32>> = Vec::with_capacity(known.len() + 1);
        obs.push((0..nc).map(|k| yb[k] / self.pre_b[k]).collect());
        for (i, d) in known.iter().enumerate().take(ys.len()) {
            let Some(d) = d else { continue };
            obs.push(self.known_obs(&ys[i], i, d));
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
        (hb, vec![0.375 / ntrain; nc])
    }

    /// Channel observation of symbol `i` with known data-carrier values `d`: the
    /// received carriers divided by what was sent, pilots included.
    fn known_obs(&self, y: &[Complex32], i: usize, d: &[Complex32]) -> Vec<Complex32> {
        assert_eq!(d.len(), self.data_idx.len(), "wrong number of known symbols");
        let mut o = vec![Complex32::new(0.0, 0.0); self.p.num_carriers];
        for (j, &k) in self.data_idx.iter().enumerate() {
            o[k] = y[k] / d[j];
        }
        for (m, &k) in self.pilot_idx.iter().enumerate() {
            o[k] = y[k] * self.pilot_value(i, m);
        }
        o
    }

    /// Channel estimate as a low-order fit across carriers: Legendre polynomials up
    /// to `order` in carrier index with complex coefficients, after removing the
    /// linear phase of the timing offset. Observations: preamble B on every carrier,
    /// the known symbols on every carrier and the pilots of the other symbols.
    /// Returns the estimate and, per carrier, its noise relative to one symbol's.
    ///
    /// The audio channel's response is smooth across the band, so a few coefficients
    /// describe it, and the fit averages every observation into each of them. Before
    /// the header has decoded, that brings in the pilots of every header symbol,
    /// which an average per carrier could not use on the data carriers.
    fn estimate_fit(
        &self,
        yb: &[Complex32],
        ys: &[Vec<Complex32>],
        known: &[Option<&[Complex32]>],
        order: usize,
    ) -> (Vec<Complex32>, Vec<f32>) {
        let nc = self.p.num_carriers;
        // Per carrier: the sum of its observations and how many there are.
        let mut sum: Vec<Complex32> = (0..nc).map(|k| yb[k] / self.pre_b[k]).collect();
        let mut cnt = vec![1f32; nc];
        for (i, y) in ys.iter().enumerate() {
            match known.get(i).copied().flatten() {
                Some(d) => {
                    sum.iter_mut().zip(self.known_obs(y, i, d)).for_each(|(s, o)| *s += o);
                    cnt.iter_mut().for_each(|c| *c += 1.0);
                }
                None => {
                    for (m, &k) in self.pilot_idx.iter().enumerate() {
                        sum[k] += y[k] * self.pilot_value(i, m);
                        cnt[k] += 1.0;
                    }
                }
            }
        }
        let mean: Vec<Complex32> = sum.iter().zip(&cnt).map(|(s, c)| s / c).collect();
        // The timing slope comes from a search that sums all observations coherently.
        // The phase step between neighbouring carriers (as `remove_slope` takes it)
        // multiplies two noisy observations, and at the header's SNR its error, which
        // grows across the band, cost most of what the fit gains.
        let slope = delay_search(&mean, &cnt);
        let flat: Vec<Complex32> =
            mean.iter().enumerate().map(|(k, &h)| h * Complex32::from_polar(1.0, -slope * k as f32)).collect();
        let fit = weighted_fit(&legendre_basis(nc, order), &flat, &cnt);
        let hb = (0..nc).map(|k| fit.value[k] * Complex32::from_polar(1.0, slope * k as f32)).collect();
        (hb, fit.variance)
    }

    /// Equalize `ys` with channel estimate `hb` (whose noise per carrier, relative to
    /// one symbol's, is `est_noise`) and the pilot tracking.
    fn equalize(
        &self,
        ys: Vec<Vec<Complex32>>,
        hb: Vec<Complex32>,
        est_noise: &[f32],
        base: isize,
        nsym: usize,
    ) -> Option<Frame> {
        let n = self.p.nfft;
        let l = self.p.symbol_len() as isize;

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
                (pilot_var[m] * (1.0 - t) + pilot_var[m + 1] * t) * (1.0 + est_noise[k])
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

/// How [`Ofdm::demodulate_with`] estimates the channel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Estimator {
    /// Preamble B and the known symbols averaged, smoothed [1 2 1]/4.
    Smooth,
    /// Polynomial fit of this order across carriers, the pilots of the unknown
    /// symbols included.
    Fit { order: usize },
}

/// Phase slope per carrier (the timing offset) that best lines up `h` (carrier
/// means, `w` observations each): the peak of `|Σ w h e^{-j s k}|` on a grid 8 times
/// finer than the carriers resolve. It sums the observations coherently, so unlike
/// products of neighbours it holds up at low SNR.
fn delay_search(h: &[Complex32], w: &[f32]) -> f32 {
    let n = h.len();
    let steps = 8 * n;
    let mut best = (0f32, 0f32);
    for i in 0..steps {
        let s = -std::f32::consts::PI + 2.0 * std::f32::consts::PI * i as f32 / steps as f32;
        let rot = Complex32::from_polar(1.0, -s);
        let mut z = Complex32::new(1.0, 0.0);
        let mut acc = Complex32::new(0.0, 0.0);
        for (hk, wk) in h.iter().zip(w) {
            acc += hk * z * wk;
            z *= rot;
        }
        let m = acc.norm_sqr();
        if m > best.0 {
            best = (m, s);
        }
    }
    best.1
}

/// Legendre polynomials 0..=`order` on `n` points spread over [-1, 1]; `[p][k]`.
fn legendre_basis(n: usize, order: usize) -> Vec<Vec<f32>> {
    let x: Vec<f32> = (0..n).map(|k| 2.0 * k as f32 / (n - 1).max(1) as f32 - 1.0).collect();
    let mut b: Vec<Vec<f32>> = Vec::with_capacity(order + 1);
    for p in 0..=order {
        let row = match p {
            0 => vec![1.0; n],
            1 => x.clone(),
            _ => {
                let pf = p as f32;
                (0..n).map(|k| ((2.0 * pf - 1.0) * x[k] * b[p - 1][k] - (pf - 1.0) * b[p - 2][k]) / pf).collect()
            }
        };
        b.push(row);
    }
    b
}

/// A weighted least-squares fit: the fitted values and, per point, the variance of
/// the fitted value in units of one observation's noise.
struct Fit {
    value: Vec<Complex32>,
    variance: Vec<f32>,
}

/// Fit `y` (point `k` the mean of `w[k]` observations of equal noise) with the real
/// `basis` and complex coefficients.
fn weighted_fit(basis: &[Vec<f32>], y: &[Complex32], w: &[f32]) -> Fit {
    let p = basis.len();
    let n = y.len();
    // Normal equations G a = r, G = Σ w φ φᵀ.
    let mut g = vec![vec![0f64; p]; p];
    let mut r = vec![(0f64, 0f64); p];
    for k in 0..n {
        let wk = w[k] as f64;
        for i in 0..p {
            let bi = basis[i][k] as f64 * wk;
            for j in 0..p {
                g[i][j] += bi * basis[j][k] as f64;
            }
            r[i].0 += bi * y[k].re as f64;
            r[i].1 += bi * y[k].im as f64;
        }
    }
    let ginv = invert(g);
    let a: Vec<(f64, f64)> = (0..p)
        .map(|i| (0..p).fold((0.0, 0.0), |(re, im), j| (re + ginv[i][j] * r[j].0, im + ginv[i][j] * r[j].1)))
        .collect();
    let value = (0..n)
        .map(|k| {
            let (re, im) = (0..p)
                .fold((0.0, 0.0), |(re, im), i| (re + a[i].0 * basis[i][k] as f64, im + a[i].1 * basis[i][k] as f64));
            Complex32::new(re as f32, im as f32)
        })
        .collect();
    let variance = (0..n)
        .map(|k| {
            let mut v = 0f64;
            for i in 0..p {
                for j in 0..p {
                    v += basis[i][k] as f64 * ginv[i][j] * basis[j][k] as f64;
                }
            }
            v as f32
        })
        .collect();
    Fit { value, variance }
}

/// Inverse of a small symmetric positive definite matrix (Gauss-Jordan).
fn invert(mut a: Vec<Vec<f64>>) -> Vec<Vec<f64>> {
    let n = a.len();
    let mut inv: Vec<Vec<f64>> = (0..n).map(|i| (0..n).map(|j| f64::from(u8::from(i == j))).collect()).collect();
    for c in 0..n {
        let piv = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs())).unwrap();
        a.swap(c, piv);
        inv.swap(c, piv);
        let d = a[c][c];
        for j in 0..n {
            a[c][j] /= d;
            inv[c][j] /= d;
        }
        for i in 0..n {
            if i != c {
                let f = a[i][c];
                if f != 0.0 {
                    for j in 0..n {
                        a[i][j] -= f * a[c][j];
                        inv[i][j] -= f * inv[c][j];
                    }
                }
            }
        }
    }
    inv
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

    #[test]
    fn weighted_fit_is_exact_on_its_basis_and_counts_its_noise() {
        let n = 49;
        let basis = legendre_basis(n, 4);
        let w: Vec<f32> = (0..n).map(|k| if k % 6 == 0 { 11.0 } else { 1.0 }).collect();
        // A cubic with complex coefficients lies in the span of the basis.
        let y: Vec<Complex32> = (0..n)
            .map(|k| {
                let x = 2.0 * k as f32 / (n - 1) as f32 - 1.0;
                Complex32::new(0.5 - x + 0.3 * x * x * x, 0.2 + 0.7 * x * x)
            })
            .collect();
        let fit = weighted_fit(&basis, &y, &w);
        for (a, b) in fit.value.iter().zip(&y) {
            assert!((a - b).norm() < 1e-4, "{a} vs {b}");
        }
        // Σ w·var is the trace of the hat matrix: the number of coefficients.
        let trace: f32 = fit.variance.iter().zip(&w).map(|(v, w)| v * w).sum();
        assert!((trace - 5.0).abs() < 1e-3, "trace {trace}");
    }

    #[test]
    fn delay_search_finds_a_timing_slope() {
        let h: Vec<Complex32> =
            (0..62).map(|k| Complex32::from_polar(1.0 + 0.01 * k as f32, -0.39 * k as f32)).collect();
        let s = delay_search(&h, &vec![1.0; 62]);
        assert!((s + 0.39).abs() < 0.01, "slope {s}");
    }

    #[test]
    fn header_fit_decodes_a_clean_frame() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let m = Modulation::Bpsk;
            let data = random_frame(&ofdm, m, 10, 3);
            let mut rx = vec![0.0; 555];
            rx.extend(ofdm.modulate(&data));
            rx.extend(vec![0.0; 2000]);
            let b = ofdm.detect(&rx, 0).expect("frame not detected");
            let frame = ofdm.demodulate_header(&rx, b, data.len()).expect("demod failed");
            for (got, sent) in frame.symbols.iter().flatten().zip(data.iter().flatten()) {
                assert!((got - sent).norm() < 0.1, "{}: {got} vs {sent}", ofdm.p.name);
            }
            assert!(frame.snr_db > 25.0, "{}: snr {}", ofdm.p.name, frame.snr_db);
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
