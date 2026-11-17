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

pub struct Ofdm {
    pub p: OfdmParams,
    fft: Arc<dyn Fft<f32>>,
    ifft: Arc<dyn Fft<f32>>,
    /// Carrier indices (0..num_carriers) that hold pilots.
    pub pilot_idx: Vec<usize>,
    /// Carrier indices that hold data.
    pub data_idx: Vec<usize>,
    /// `PILOT_TABLE_LEN` rows of one value per pilot, row after row.
    pilot_table: Vec<f32>,
    gains: Vec<f32>,
    pre_a: Vec<Complex32>,
    pre_b: Vec<Complex32>,
    /// Time-domain body (no cyclic prefix) of preamble B, unit energy.
    pre_b_body: Vec<f32>,
}

/// A demodulated frame.
pub struct Frame {
    /// Equalized data-carrier symbols: `data_carriers()` values per OFDM symbol,
    /// symbol after symbol. [`Frame::symbol`] returns one OFDM symbol.
    ///
    /// The row layout comes from the other fields: `symbols.len()` must stay
    /// `symbol_noise_scale.len() * noise_var.len()`.
    pub symbols: Vec<Complex32>,
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

impl Frame {
    /// Number of OFDM symbols.
    pub fn nsym(&self) -> usize {
        self.symbol_noise_scale.len()
    }

    /// The equalized data carriers of OFDM symbol `i`.
    pub fn symbol(&self, i: usize) -> &[Complex32] {
        let nd = self.noise_var.len();
        debug_assert_eq!(self.symbols.len(), self.nsym() * nd, "Frame fields disagree on the layout");
        &self.symbols[i * nd..(i + 1) * nd]
    }
}

/// Work buffers for one in-place FFT, reused across the symbols of a frame.
struct FftBufs {
    x: Vec<Complex32>,
    scratch: Vec<Complex32>,
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
        let mut pilot_table = Vec::with_capacity(PILOT_TABLE_LEN * pilot_idx.len());
        for _ in 0..PILOT_TABLE_LEN {
            pilot_table.extend(pilot_idx.iter().map(|_| rng.bpsk()));
        }

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
        let mut b = Vec::with_capacity(s.p.symbol_len());
        s.symbol_time(&s.pre_b, &mut s.fft_bufs(), &mut b);
        let mut body = b.split_off(s.p.ncp);
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
        self.pilot_table[(sym % PILOT_TABLE_LEN) * self.pilot_idx.len() + n]
    }

    fn fft_bufs(&self) -> FftBufs {
        let zero = Complex32::new(0.0, 0.0);
        let scratch = self.fft.get_inplace_scratch_len().max(self.ifft.get_inplace_scratch_len());
        FftBufs { x: vec![zero; self.p.nfft], scratch: vec![zero; scratch] }
    }

    /// Append one OFDM symbol (cyclic prefix + body) from carrier values to `out`.
    fn symbol_time(&self, carriers: &[Complex32], bufs: &mut FftBufs, out: &mut Vec<f32>) {
        let n = self.p.nfft;
        let x = &mut bufs.x;
        x.fill(Complex32::new(0.0, 0.0));
        for (k, &v) in carriers.iter().enumerate() {
            let bin = self.p.first_bin + k;
            let v = v * self.gains[k];
            x[bin] = v;
            x[n - bin] = v.conj();
        }
        self.ifft.process_with_scratch(x, &mut bufs.scratch);
        let scale = 1.0 / (n as f32).sqrt();
        out.extend(x[n - self.p.ncp..].iter().map(|c| c.re * scale));
        out.extend(x.iter().map(|c| c.re * scale));
    }

    /// Build a frame. `data` holds `data_carriers()` values per OFDM symbol, symbol
    /// after symbol.
    pub fn modulate(&self, data: &[Complex32]) -> Vec<f32> {
        self.modulate_with_leader(data, 0.0)
    }

    /// Build a frame preceded by at least `leader_s` seconds of filler symbols, at the
    /// mode's reference level `p.tx_rms`.
    /// The leader covers the radio's key-up time (PTT to carrier up and squelch open)
    /// and lets the receiver's audio levels settle. The filler is random QPSK, so it
    /// cannot be mistaken for preamble A.
    pub fn modulate_with_leader(&self, data: &[Complex32], leader_s: f32) -> Vec<f32> {
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
    pub fn modulate_with_leader_rms(&self, data: &[Complex32], leader_s: f32, rms: f32) -> Vec<f32> {
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
    fn frame_samples(&self, data: &[Complex32], leader_s: f32) -> Vec<f32> {
        let nc = self.p.num_carriers;
        let nd = self.data_idx.len();
        assert!(data.len().is_multiple_of(nd), "wrong number of data symbols");
        let n_leader = (leader_s / self.p.symbol_duration()).ceil() as usize;
        let mut out = Vec::with_capacity(self.frame_len(data.len() / nd) + n_leader * self.p.symbol_len());
        let mut bufs = self.fft_bufs();
        // Every carrier is written before each use: data and pilots cover them all.
        let mut c = vec![Complex32::new(0.0, 0.0); nc];
        let mut rng = Prng::new(0x5eed_00c3);
        for _ in 0..n_leader {
            c.iter_mut().for_each(|v| *v = Complex32::new(rng.bpsk(), rng.bpsk()) * (1.0 / SQRT_2));
            self.symbol_time(&c, &mut bufs, &mut out);
        }
        self.symbol_time(&self.pre_a, &mut bufs, &mut out);
        self.symbol_time(&self.pre_b, &mut bufs, &mut out);
        for (i, d) in data.chunks_exact(nd).enumerate() {
            for (j, &k) in self.data_idx.iter().enumerate() {
                c[k] = d[j];
            }
            for (n, &k) in self.pilot_idx.iter().enumerate() {
                c[k] = Complex32::new(self.pilot_value(i, n), 0.0);
            }
            self.symbol_time(&c, &mut bufs, &mut out);
        }
        out
    }

    /// Cutoff of the TX splatter filter.
    pub fn tx_lowpass_hz(&self) -> f32 {
        self.p.carrier_freq(self.p.num_carriers - 1) * TX_LOWPASS_FACTOR
    }

    /// FFT of the window starting at `pos`; writes only the carrier bins to `out`
    /// (`num_carriers` long). `None` if the window is outside `rx`.
    fn carriers_at(&self, rx: &[f32], pos: isize, bufs: &mut FftBufs, out: &mut [Complex32]) -> Option<()> {
        let n = self.p.nfft;
        if pos < 0 || pos as usize + n > rx.len() {
            return None;
        }
        debug_assert_eq!(out.len(), self.p.num_carriers);
        let pos = pos as usize;
        let x = &mut bufs.x;
        x.iter_mut().zip(&rx[pos..pos + n]).for_each(|(c, &v)| *c = Complex32::new(v, 0.0));
        self.fft.process_with_scratch(x, &mut bufs.scratch);
        let scale = 1.0 / (n as f32).sqrt();
        out.iter_mut().zip(&x[self.p.first_bin..]).for_each(|(o, &c)| *o = c * scale);
        Some(())
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

    /// Like [`demodulate`](Self::demodulate), but the data-carrier values of the first
    /// symbols are known (for example a header that has already been decoded): `known`
    /// holds `data_carriers()` values per symbol, symbol after symbol. Those symbols
    /// train the channel estimate together with preamble B.
    pub fn demodulate_known(&self, rx: &[f32], b_start: usize, nsym: usize, known: &[Complex32]) -> Option<Frame> {
        if nsym == 0 {
            return None;
        }
        let n = self.p.nfft;
        let l = self.p.symbol_len() as isize;
        let nc = self.p.num_carriers;
        let nd = self.data_idx.len();
        let np = self.pilot_idx.len();
        assert!(known.len().is_multiple_of(nd), "wrong number of known symbols");
        // Centre the FFT window in the cyclic prefix. That leaves ncp/2 samples of
        // slack each way for sound-card clock drift without ever moving the window.
        let base = b_start as isize - (self.p.ncp / 2) as isize;

        // Carrier values of preamble B and of each symbol (`nc` per symbol).
        let zero = Complex32::new(0.0, 0.0);
        let mut bufs = self.fft_bufs();
        let mut yb = vec![zero; nc];
        self.carriers_at(rx, base, &mut bufs, &mut yb)?;
        let mut ys = vec![zero; nsym * nc];
        for (i, y) in ys.chunks_exact_mut(nc).enumerate() {
            self.carriers_at(rx, base + (i as isize + 1) * l, &mut bufs, y)?;
        }

        // Channel estimate: preamble B averaged with the known symbols. Clock drift moves
        // the timing by at most a sample or so over the header; the pilot tracking below
        // takes that out, so the symbols are simply averaged. (Removing each symbol's own
        // phase slope instead costs up to 2 dB at low SNR: the slope estimate is noisy and
        // its error grows across the band.)
        // One channel observation (`nc` values) per training symbol.
        let nk = (known.len() / nd).min(nsym);
        let mut obs = vec![zero; (nk + 1) * nc];
        obs[..nc].iter_mut().zip(yb.iter().zip(&self.pre_b)).for_each(|(o, (y, b))| *o = y / b);
        for (i, ((o, y), d)) in
            obs[nc..].chunks_exact_mut(nc).zip(ys.chunks_exact(nc)).zip(known.chunks_exact(nd)).enumerate()
        {
            for (j, &k) in self.data_idx.iter().enumerate() {
                o[k] = y[k] / d[j];
            }
            for (m, &k) in self.pilot_idx.iter().enumerate() {
                o[k] = y[k] * self.pilot_value(i, m);
            }
        }
        // A header symbol hit by an FM click can still decode (the header code is rate
        // 0.24) and so be offered as training. At full weight it would corrupt the
        // estimate for every carrier of the burst, which the per-symbol noise scale
        // below cannot undo. So each training symbol is weighted by the inverse of its
        // noise, and `ntrain` is the effective number of symbols averaged.
        let weights = training_weights(&obs, nc);
        let ntrain: f32 = weights.iter().sum();
        let acc: Vec<Complex32> = (0..nc)
            .map(|k| obs.chunks_exact(nc).zip(&weights).map(|(o, &w)| o[k] * w).sum::<Complex32>() / ntrain)
            .collect();
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

        // Pilot ratios against the channel estimate (`np` per symbol).
        let mut cs = Vec::with_capacity(nsym * np);
        for (i, y) in ys.chunks_exact(nc).enumerate() {
            cs.extend(self.pilot_idx.iter().enumerate().map(|(m, &k)| y[k] / (hb[k] * self.pilot_value(i, m))));
        }

        // Average each pilot over neighbouring symbols, weighting each symbol by the
        // inverse of its noise. The first pass (equal weights) finds symbols hit by an
        // FM click or a fade; the second keeps them out of their neighbours' averages.
        let (_, dev) = track_pilots(&cs, np, &vec![1.0; nsym]);
        let (scale, _) = noise_model(&dev, np);
        let (avg, dev) = track_pilots(&cs, np, &scale);
        let (scale, pilot_var) = noise_model(&dev, np);

        // Interpolate between pilots and equalize. Each carrier only depends on nearby
        // pilots, so noisy pilots at one end of the band do not corrupt carriers at the
        // other end.
        let mut symbols = Vec::with_capacity(nsym * nd);
        for (y, a) in ys.chunks_exact(nc).zip(avg.chunks_exact(np)) {
            symbols.extend(self.data_idx.iter().map(|&k| y[k] / (hb[k] * self.interp_pilots(a, k))));
        }

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
/// `cs` holds `np` pilots per symbol and `scale` one value per symbol; both results are
/// laid out like `cs`.
fn track_pilots(cs: &[Complex32], np: usize, scale: &[f32]) -> (Vec<Complex32>, Vec<f32>) {
    let nsym = scale.len();
    debug_assert_eq!(cs.len(), nsym * np);
    let mut avgs = Vec::with_capacity(nsym * np);
    let mut devs = Vec::with_capacity(nsym * np);
    for i in 0..nsym {
        let lo = i.saturating_sub(TRACK_HALF_WINDOW);
        let hi = (i + TRACK_HALF_WINDOW + 1).min(nsym);
        let wsum: f32 = scale[lo..hi].iter().map(|s| 1.0 / s).sum();
        let a0 = avgs.len();
        avgs.extend((0..np).map(|m| (lo..hi).map(|j| cs[j * np + m] / scale[j]).sum::<Complex32>() / wsum));
        let avg = &avgs[a0..];
        let c = &cs[i * np..(i + 1) * np];
        // With weights matched to the noise, the deviation's variance is the symbol's
        // own noise times (1 - its share of the average).
        let share = 1.0 / scale[i] / wsum;
        if share < 0.999 {
            let bias = 1.0 / (1.0 - share);
            devs.extend(c.iter().zip(avg).map(|(c, a)| (c / a - 1.0).norm_sqr() * bias));
        } else {
            devs.resize(devs.len() + np, 0.0);
        }
    }
    (avgs, devs)
}

/// Weight per channel observation (one per training symbol, `nc` values each, symbol
/// after symbol): 1 for a symbol whose deviation from the plain average is at most
/// `SCALE_THRESHOLD` times the typical level, otherwise typical / own, so that a symbol
/// hit by a click adds almost nothing. Fewer than three observations cannot show an
/// outlier and all get weight 1.
fn training_weights(obs: &[Complex32], nc: usize) -> Vec<f32> {
    debug_assert!(obs.len().is_multiple_of(nc));
    let n = obs.len() / nc;
    if n < 3 {
        return vec![1.0; n];
    }
    let mean: Vec<Complex32> =
        (0..nc).map(|k| obs.chunks_exact(nc).map(|o| o[k]).sum::<Complex32>() / n as f32).collect();
    let e: Vec<f32> =
        obs.chunks_exact(nc).map(|o| o.iter().zip(&mean).map(|(x, h)| (x - h).norm_sqr()).sum()).collect();
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
///
/// `dev` holds `np` pilots per symbol, symbol after symbol.
fn noise_model(dev: &[f32], np: usize) -> (Vec<f32>, Vec<f32>) {
    debug_assert!(dev.len().is_multiple_of(np));
    let nsym = dev.len() / np;
    let per_pilot = |scale: &[f32]| -> Vec<f32> {
        (0..np)
            .map(|m| dev.chunks_exact(np).zip(scale).map(|(d, s)| d[m] / s).sum::<f32>() / nsym.max(1) as f32)
            .collect()
    };
    let ones = vec![1.0; nsym];
    let var = per_pilot(&ones);
    if nsym < 3 {
        return (ones, var);
    }
    // Noise energy per symbol, each pilot relative to its own average.
    let e: Vec<f32> = dev
        .chunks_exact(np)
        .map(|d| d.iter().zip(&var).map(|(x, v)| x / v.max(1e-12)).sum::<f32>() / np as f32)
        .collect();
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
        let data = m.map_all(&bits);
        let mut rx = vec![0.0; lead];
        rx.extend(ofdm.modulate(&data));
        rx.extend(vec![0.0; 2000]);

        let b = ofdm.detect(&rx, 0).expect("frame not detected");
        let frame = ofdm.demodulate(&rx, b, nsym).expect("demod failed");
        assert_eq!(frame.nsym(), nsym);
        // Each row of `symbol` decides to the row that was sent in that symbol.
        let nd = ofdm.data_carriers();
        for (i, sent) in data.chunks_exact(nd).enumerate() {
            let (mut got, mut want) = (Vec::new(), Vec::new());
            frame.symbol(i).iter().for_each(|s| m.hard_bits(*s, &mut got));
            sent.iter().for_each(|s| m.hard_bits(*s, &mut want));
            assert_eq!(got, want, "symbol {i}");
        }
        let mut out = Vec::new();
        for s in &frame.symbols {
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

    fn random_frame(ofdm: &Ofdm, m: Modulation, nsym: usize, seed: u32) -> Vec<Complex32> {
        let mut rng = Prng::new(seed);
        let bits = rng.bits(nsym * ofdm.data_carriers() * m.bits_per_symbol());
        m.map_all(&bits)
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
            let nsym = 20;
            let bits = rng.bits(nsym * ofdm.data_carriers() * m.bits_per_symbol());
            let data = m.map_all(&bits);
            let mut rx = vec![0.0; 777];
            rx.extend(ofdm.modulate_with_leader_rms(&data, 0.1, 0.5));
            rx.extend(vec![0.0; 2000]);
            let b = ofdm.detect(&rx, 0).expect("frame not detected");
            let frame = ofdm.demodulate(&rx, b, nsym).expect("demod failed");
            let mut out = Vec::new();
            for s in &frame.symbols {
                m.hard_bits(*s, &mut out);
            }
            assert_eq!(out, bits, "{}", ofdm.p.name);
            println!("{}: SNR at rms 0.5 without noise {:.1} dB", ofdm.p.name, frame.snr_db);
        }
    }
}
