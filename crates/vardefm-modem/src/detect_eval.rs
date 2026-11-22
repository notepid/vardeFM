//! Side-by-side evaluation of burst detectors through the FM channel simulator.
//!
//! Every detector sees exactly the same received audio, so differences are paired
//! and can be tested with McNemar's exact test instead of comparing two noisy rates.
//! For each CNR it reports, per detector:
//! - `burst`, the measure that counts: the whole burst (header and every codeword)
//!   decoded by the streaming `Receiver`, fed the recording in 10 ms chunks as a sound
//!   card delivers it. Where a search starts can change what Schmidl-Cox finds, so an
//!   offline search over the whole recording would not show what the receiver does;
//! - `hit`: the detector's first detection in the whole recording, searched from its
//!   start, lies within ±ncp/4 samples of the true position: the timing at which
//!   noiseless copies of the burst demodulate their header cleanest, which no detector
//!   defines (see `link_position`). A detector that always picks another point inside
//!   the cyclic prefix can decode every burst and still score few hits, so read `hit`
//!   as a diagnostic only. Where a detector's noiseless detection is that far off, the
//!   run says so.
//!
//! `genie` decodes from the true position: a known-timing reference, not a strict
//! bound. Another valid position inside the cyclic prefix puts the FFT windows on
//! other noise samples and can decode a recording that fails at the true position,
//! so a detector may occasionally beat it. The gap between `genie` and a detector
//! approximates what detection costs at that CNR.
//!
//! A separate pass counts detections and decoded headers per minute on audio that
//! holds no burst (see [`NOISE_CASES`]: a carrier at -10 dB CNR, which sounds like an
//! open squelch, a weak unmodulated carrier, white noise, keyed tones, and the modem's
//! own data symbols without their preambles).
//! A detection there costs a header decode; a decoded header there is a false burst.
//! The streaming receiver's CPU load on that audio, fed in 10 ms chunks like a sound
//! card, is reported per detector too.
//!
//! A CTCSS pass counts decoded bursts with a sub-tone on the carrier: every EIA tone at
//! 375, 750 and 1500 Hz deviation (the tables `busy_eval` uses), at each CTCSS CNR, on each
//! receive path that can hear one ([`ctcss_cases`]), after a row without a tone. The
//! simulated radio adds the tone at its modulator, after the deviation limiter. Each
//! cell has a genie column too: where it decodes and a detector does not, detection
//! lost the burst; where neither does, the tone broke demodulation. A tone once made
//! Schmidl-Cox miss every burst for many tones (#12). At level 1 and 20 dB CNR nothing
//! is lost now. At the knee, detection keeps its no-tone share of the genie's bursts.
//! The tone still breaks demodulation: at the quieter, denser levels 5 and 6, and at
//! the knee on narrow de-emphasised audio at 1500 Hz deviation.
//!
//! `cargo test --release -p vardefm-modem detector_eval -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `TRIALS` per CNR (100), `LEVEL` (1), `NCW` (1),
//! `CNRS` (dB list, default per mode), `NOISE_SECONDS` per noise case (60),
//! `NOISE_SEED` (1, up to 2^32 - 1, for other noise in that pass), `CSV` (file to
//! append the burst results to). Three more change the bursts' path, not the noise
//! cases: `CHANNEL`, the radio audio response (matched, tx-pre-only or rx-de-only,
//! see [`CHANNELS`]), `TX_TILT`, dB/octave the sender tilts its audio by beyond
//! what the receiver expects (0), and `CTCSS`, the deviation in Hz of a CTCSS tone
//! the sending radio adds at its modulator, after the deviation limiter (0 = none;
//! wide mode only, as the narrow receive audio path filters sub-tones out).
//!
//! The separate CTCSS pass takes `CTCSS_TRIALS` per tone and deviation (10; 0 skips
//! the pass; trial `t` has the same noise in every row, so the rows are paired and
//! each rests on that many noise draws) and `CTCSS_CNRS` (dB list; 5,20 narrow and
//! 7,20 wide: the knee and well above it).

use super::*;
use crate::busy_eval::CTCSS_DEVIATION_HZ;
use std::f32::consts::PI;
use std::io::Write;
use std::sync::Arc;
use std::time::Instant;
use vardefm_dsp::ofdm::MF_THRESHOLD;
use vardefm_dsp::prng::Prng;
use vardefm_dsp::CTCSS_HZ;
use vardefm_sim::{transmit, Ctcss, FmChannelConfig};

/// The detector every other one is compared with in the paired test. Every detector
/// in [`Detector::ALL`] is evaluated.
const REFERENCE: Detector = Detector::SchmidlCox;

/// Leader length used by the evaluation, seconds (the `vardefm modem` default).
const TXDELAY: f32 = 0.15;

pub(crate) fn modem(mode: &str) -> Arc<Modem> {
    eval_modem(mode, 0.0, Detector::default())
}

/// A modem whose TX tilt is `tilt` dB/octave more than the mode's and whose receiver
/// uses `detector`.
fn eval_modem(mode: &str, tilt: f32, detector: Detector) -> Arc<Modem> {
    let mut p = OfdmParams::by_name(mode).unwrap();
    p.tilt_db_per_octave += tilt;
    let mut m = Modem::new(p, TXDELAY).with_detector(detector);
    // Trials already run on every core.
    m.decode_threads = 1;
    Arc::new(m)
}

/// Radio audio responses for `CHANNEL`: the simulator's own, where the receiving
/// radio's de-emphasis undoes the sending radio's pre-emphasis, then a sender with
/// pre-emphasis heard by a receiver without de-emphasis (about +6 dB/octave across
/// the band), and the reverse (about -6 dB/octave). The two mismatches use the
/// narrow channel's 750 us in either mode.
const CHANNELS: [&str; 3] = ["matched", "tx-pre-only", "rx-de-only"];

/// The FM channel of `mode` with response `channel` from [`CHANNELS`].
fn channel(mode: &str, channel: &str) -> FmChannelConfig {
    let base = FmChannelConfig::by_name(mode).unwrap();
    let us = FmChannelConfig::narrow().tx_emphasis_us;
    match channel {
        "matched" => base,
        "tx-pre-only" => FmChannelConfig { tx_emphasis_us: us, rx_emphasis_us: None, ..base },
        "rx-de-only" => FmChannelConfig { tx_emphasis_us: None, rx_emphasis_us: us, ..base },
        _ => panic!("CHANNEL: unknown channel {channel:?} (one of {})", CHANNELS.join(", ")),
    }
}

/// What a trial sends through and receives with: the receiving modem, whose detectors
/// are evaluated, the sending modem, which may tilt its audio differently, the
/// channel between them, without noise, and the deviation of a CTCSS tone the sending
/// radio adds at its modulator, after the deviation limiter (0 for none).
struct Link {
    rx: Arc<Modem>,
    /// The receiving modem once per detector of [`Detector::ALL`], in that order.
    receivers: Vec<Arc<Modem>>,
    tx: Arc<Modem>,
    chan: FmChannelConfig,
    ctcss_deviation_hz: f32,
}

impl Link {
    fn new(mode: &str, chan: &str, tx_tilt: f32) -> Self {
        let tx = eval_modem(mode, tx_tilt, Detector::default());
        let receivers = Detector::ALL.iter().map(|&d| eval_modem(mode, 0.0, d)).collect();
        Self { rx: modem(mode), receivers, tx, chan: channel(mode, chan), ctcss_deviation_hz: 0.0 }
    }

    fn matched(mode: &str) -> Self {
        Self::new(mode, "matched", 0.0)
    }

    /// The link with a CTCSS tone of `deviation_hz`. Only where the receive audio
    /// path passes sub-tones, as busy_eval models it: wide mode.
    fn with_ctcss(self, deviation_hz: f32) -> Self {
        let passes = self.chan.rx_band.0 < CTCSS_HZ[0];
        assert!(deviation_hz == 0.0 || passes, "CTCSS: the {} receive audio path filters it out", self.rx.ofdm.p.name);
        Self { ctcss_deviation_hz: deviation_hz, ..self }
    }

    /// The channel of trial `t` at `cnr` dB: its own noise, and the trial's CTCSS
    /// tone if the link has one, the standard tones in turn, each with its own phase.
    fn trial_channel(&self, cnr: f32, t: u32) -> FmChannelConfig {
        let ctcss = (self.ctcss_deviation_hz > 0.0).then(|| Ctcss {
            hz: CTCSS_HZ[t as usize % CTCSS_HZ.len()],
            deviation_hz: self.ctcss_deviation_hz,
            phase: (2.0 * std::f64::consts::PI * (t as f64 * 0.618).fract()) as f32,
        });
        FmChannelConfig { cnr_db: Some(cnr), seed: channel_seed(cnr, t), ctcss, ..self.chan.clone() }
    }
}

pub(crate) fn burst(level: u8, ncw: usize, seed: u32) -> Burst {
    let mut rng = Prng::new(seed);
    let n = Modem::payload_bytes(level);
    Burst {
        header: Header { kind: Kind::Data, session: 0x42, level, ncw: ncw as u8, ..Default::default() },
        payload: (0..ncw).map(|_| (0..n).map(|_| rng.next_u32() as u8).collect()).collect(),
    }
}

/// The header of the burst whose preamble B starts at `b`, if it decodes.
pub(crate) fn header_at(m: &Modem, rx: &[f32], b: usize) -> Option<Header> {
    m.ofdm.demodulate(rx, b, m.header_symbols()).and_then(|f| m.decode_header(&f))
}

/// Whether the burst at `b`, whose header `h` has decoded, is the one sent: header
/// equal and every codeword decoded and equal, with the header as channel training
/// as in `Receiver::step`.
fn burst_ok(m: &Modem, rx: &[f32], b: usize, h: &Header, sent: &Burst) -> bool {
    if *h != sent.header {
        return false;
    }
    if h.ncw == 0 {
        return true;
    }
    let nsym = m.header_symbols() + h.ncw as usize * m.symbols_per_codeword(h.level);
    let Some(frame) = m.ofdm.demodulate_known(rx, b, nsym, &m.header_carriers(h)) else {
        return false;
    };
    m.decode_payload(h, &frame).into_iter().zip(&sent.payload).all(|(got, want)| got.as_ref() == Some(want))
}

/// Whether the streaming `Receiver` on `m` makes `sent` of recording `rx`: its
/// first burst has the header sent and every codeword decoded as sent. The audio
/// comes in 10 ms chunks, as a sound card delivers it, then a second of silence so
/// that a burst at the very end completes.
fn received_ok(m: &Arc<Modem>, rx: &[f32], sent: &Burst) -> bool {
    let mut receiver = Receiver::new(m.clone());
    let chunk = (m.fs() / 100.0) as usize;
    let mut got: Vec<RxBurst> = rx.chunks(chunk).flat_map(|c| receiver.push(c)).collect();
    got.extend(receiver.push(&vec![0.0; m.fs() as usize]));
    got.first().is_some_and(|g| {
        g.header == sent.header && g.payload.iter().zip(&sent.payload).all(|(p, s)| p.as_ref() == Some(s))
    })
}

/// Outcome of one trial: (hit, burst) per detector of [`Detector::ALL`], then the
/// genie's burst.
struct Trial {
    det: Vec<(bool, bool)>,
    genie: bool,
}

/// Payload seed of trial `t`: every trial sends its own data, the same at every CNR.
pub(crate) fn payload_seed(t: u32) -> u32 {
    1 + t
}

/// Channel seed of trial `t` at `cnr` dB: every trial and every CNR point gets its
/// own noise, so the points of a curve do not share noise realisations. All
/// detectors still see the same recording within a trial, which keeps them paired.
fn channel_seed(cnr: f32, t: u32) -> u64 {
    (((cnr * 100.0).round() as i64 as u64) << 32) | t as u64
}

/// The audio the receiver gets when trial `t` sends `sent` over the `mode` channel
/// at `cnr` dB.
pub(crate) fn through_channel(m: &Modem, mode: &str, cnr: f32, sent: &Burst, t: u32) -> Vec<f32> {
    let cfg =
        FmChannelConfig { cnr_db: Some(cnr), seed: channel_seed(cnr, t), ..FmChannelConfig::by_name(mode).unwrap() };
    transmit(&cfg, &m.modulate(sent))
}

/// How a CTCSS tone on the carrier reaches the modem's receive audio.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum CtcssTap {
    /// Through the channel: the radio's tone encoder adds it at the modulator, and the
    /// receive chain (de-emphasis, audio band) treats it as it treats the burst.
    Channel,
    /// Added flat at its deviation to the received audio while the carrier is up and
    /// the squelch open, as at a discriminator tap that bypasses de-emphasis and the
    /// audio filter. The burst still takes the channel's receive chain.
    Discriminator,
}

/// A receive path for the CTCSS pass.
pub(crate) struct CtcssCase {
    pub name: &'static str,
    pub tap: CtcssTap,
    /// Low edge of the receive audio band in place of the channel's, Hz.
    pub rx_low_hz: Option<f32>,
}

/// The receive paths that can hear a CTCSS tone, per mode. Wide mode's data port is
/// flat down to 20 Hz. Narrow mode's mic/speaker channel filters the sub-tone out at
/// 300 Hz, so it is heard only on a radio whose audio output passes it: either
/// processed audio, de-emphasised (which lifts the tone by 10-13 dB relative to its
/// deviation), or a discriminator tap.
pub(crate) fn ctcss_cases(mode: &str) -> &'static [CtcssCase] {
    const WIDE: &[CtcssCase] = &[CtcssCase { name: "data port", tap: CtcssTap::Channel, rx_low_hz: None }];
    const NARROW: &[CtcssCase] = &[
        CtcssCase { name: "de-emphasised audio", tap: CtcssTap::Channel, rx_low_hz: Some(20.0) },
        CtcssCase { name: "discriminator", tap: CtcssTap::Discriminator, rx_low_hz: None },
    ];
    if mode == "narrow" {
        NARROW
    } else {
        WIDE
    }
}

/// Channel `cfg` with the receive band of case `c`.
pub(crate) fn ctcss_channel(cfg: &FmChannelConfig, c: &CtcssCase) -> FmChannelConfig {
    FmChannelConfig { rx_band: (c.rx_low_hz.unwrap_or(cfg.rx_band.0), cfg.rx_band.1), ..cfg.clone() }
}

/// What the receiver gets when `sent` goes over channel `cfg` (already set up for the
/// case, see [`ctcss_channel`]) with a CTCSS tone of `f` Hz at `dev` Hz deviation,
/// starting at phase `ph`, reaching the receiver through `tap`.
pub(crate) fn with_ctcss(
    m: &Modem,
    cfg: &FmChannelConfig,
    tap: CtcssTap,
    sent: &Burst,
    f: f32,
    dev: f32,
    ph: f32,
) -> Vec<f32> {
    let tx = m.modulate(sent);
    let toneless = (tap == CtcssTap::Discriminator).then(|| transmit(cfg, &tx));
    with_ctcss_on(cfg, tap, &tx, toneless.as_deref(), f, dev, ph)
}

/// [`with_ctcss`] for the modulated burst `tx`. A discriminator tap adds the tone to
/// `toneless`, what the channel gives without one, so a caller trying several tones on
/// one trial runs the channel once; the channel tap ignores it.
fn with_ctcss_on(
    cfg: &FmChannelConfig,
    tap: CtcssTap,
    tx: &[f32],
    toneless: Option<&[f32]>,
    f: f32,
    dev: f32,
    ph: f32,
) -> Vec<f32> {
    match tap {
        CtcssTap::Channel => {
            let ctcss = (dev > 0.0).then_some(Ctcss { hz: f, deviation_hz: dev, phase: ph });
            transmit(&FmChannelConfig { ctcss, ..cfg.clone() }, tx)
        }
        CtcssTap::Discriminator => {
            let mut rx = toneless.expect("a discriminator tap needs the toneless channel output").to_vec();
            assert_eq!(cfg.clock_offset_ppm, 0.0, "the tone span assumes no clock offset");
            // The tone starts at phase `ph` at carrier up, as `transmit` starts it, and
            // is heard while the carrier is up and the squelch open.
            let lead = (cfg.lead_silence_s * cfg.fs) as usize;
            let open = lead + (cfg.keyup_delay_s * cfg.fs) as usize..(lead + tx.len()).min(rx.len());
            let a = (dev / cfg.deviation_hz) as f64;
            let w = 2.0 * std::f64::consts::PI * f as f64 / cfg.fs as f64;
            for i in open {
                rx[i] += (a * (w * (i - lead) as f64 + ph as f64).sin()) as f32;
            }
            rx
        }
    }
}

fn trial(link: &Link, cnr: f32, level: u8, ncw: usize, truth: usize, t: u32) -> Trial {
    let m = &*link.rx;
    let sent = burst(level, ncw, payload_seed(t));
    let rx = transmit(&link.trial_channel(cnr, t), &link.tx.modulate(&sent));
    let tol = m.ofdm.p.ncp / 4;
    let det = Detector::ALL
        .iter()
        .zip(&link.receivers)
        .map(|(&d, r)| {
            let first = m.ofdm.detect_with(d, &rx, 0).at;
            (first.is_some_and(|b| b.abs_diff(truth) <= tol), received_ok(r, &rx, &sent))
        })
        .collect();
    let genie = header_at(m, &rx, truth).is_some_and(|h| burst_ok(m, &rx, truth, &h, &sent));
    Trial { det, genie }
}

/// Within this many dB of the best header SNR, a timing counts as among the best in
/// [`best_timing`].
const BEST_TIMING_MARGIN_DB: f32 = 0.5;

/// Noiseless copies, each with its own payload, that [`best_timing`] averages over.
/// Where the link clips, the error depends on the payload; the average smooths it.
const BEST_TIMING_COPIES: u32 = 4;

/// Squared error and energy of the header symbols `known` demodulated from `rx` at
/// `b`, against the values sent. Unlike `Frame::snr_db` it needs no decisions, so it
/// holds on a link that distorts the burst.
fn header_error(m: &Modem, rx: &[f32], b: usize, known: &[Vec<Complex32>]) -> Option<(f32, f32)> {
    let f = m.ofdm.demodulate(rx, b, known.len())?;
    let pairs = || f.symbols.iter().flatten().zip(known.iter().flatten());
    Some((pairs().map(|(y, x)| (y - x).norm_sqr()).sum(), pairs().map(|(_, x)| x.norm_sqr()).sum()))
}

/// The timing at which `m` demodulates the header `h` of the bursts in `rxs` cleanest,
/// over all of them, searched within a cyclic prefix either side of `nominal`: the
/// middle of the timings within [`BEST_TIMING_MARGIN_DB`] of the best. `None` if
/// none demodulates.
fn best_timing(m: &Modem, rxs: &[Vec<f32>], nominal: usize, h: &Header) -> Option<usize> {
    let ncp = m.ofdm.p.ncp;
    let known = m.header_carriers(h);
    let snr: Vec<(usize, f32)> = (nominal.saturating_sub(ncp)..=nominal + ncp)
        .filter_map(|b| {
            let (err, sig) = rxs
                .iter()
                .map(|rx| header_error(m, rx, b, &known))
                .collect::<Option<Vec<(f32, f32)>>>()?
                .into_iter()
                .fold((0.0, 0.0), |(e, s), (de, ds)| (e + de, s + ds));
            Some((b, 10.0 * (sig / f32::max(err, f32::MIN_POSITIVE)).log10()))
        })
        .collect();
    let best = snr.iter().map(|s| s.1).fold(f32::NEG_INFINITY, f32::max);
    let near: Vec<usize> = snr.iter().filter(|s| s.1 >= best - BEST_TIMING_MARGIN_DB).map(|s| s.0).collect();
    near.get(near.len() / 2).copied()
}

/// Where preamble B of a `level` burst of `ncw` codewords lands after the channel,
/// and where each detector of [`Detector::ALL`] finds it on a noiseless copy. The
/// channel's delays depend neither on its noise nor on the payload (the burst layout
/// is fixed by level and size), so noiseless copies give it for every trial.
///
/// No detector defines the position, so neither `hit` nor `genie` favours one. It is
/// the timing at which noiseless copies, without the CTCSS tone (the tone does not
/// move the burst), demodulate their header cleanest ([`best_timing`]), searched
/// around where preamble B starts in the sent audio. On a link whose response differs
/// from the sender's that is the best point for decoding, whatever point a detector's
/// fine timing prefers.
/// `Err` when the noiseless copies do not demodulate there.
fn link_position(link: &Link, level: u8, ncw: usize) -> Result<(usize, Vec<Option<usize>>), String> {
    let clean = FmChannelConfig { cnr_db: None, ..link.chan.clone() };
    let sent: Vec<Burst> = (0..BEST_TIMING_COPIES).map(|t| burst(level, ncw, payload_seed(t))).collect();
    let rxs: Vec<Vec<f32>> = sent.iter().map(|b| transmit(&clean, &link.tx.modulate(b))).collect();
    let m = &*link.rx;
    let ofdm = &m.ofdm;
    let lead = (clean.lead_silence_s * clean.fs) as usize;
    let nominal = lead + (link.tx.leader_symbols() + 1) * ofdm.p.symbol_len() + ofdm.p.ncp;
    let Some(truth) = best_timing(m, &rxs, nominal, &sent[0].header) else {
        return Err("the noiseless burst does not demodulate near where it was sent".into());
    };
    let found = Detector::ALL.iter().map(|&d| ofdm.detect_with(d, &rxs[0], 0).at).collect();
    Ok((truth, found))
}

/// What [`link_position`]'s detections say about each detector: a line for each one
/// that finds the noiseless burst further than `tol` samples from `truth`, or not at
/// all.
fn far_detections(truth: usize, found: &[Option<usize>], tol: usize) -> Vec<String> {
    Detector::ALL
        .iter()
        .zip(found)
        .filter(|(_, at)| !at.is_some_and(|b| b.abs_diff(truth) <= tol))
        .map(|(d, at)| format!("{d} finds the noiseless burst at {at:?}, the best timing is {truth}"))
        .collect()
}

/// [`link_position`] where every detector must find the noiseless burst inside the
/// part of the cyclic prefix that leaves room for the channel (within ncp/2 of the
/// best timing), as in the CI tests.
fn strict_link_position(link: &Link, level: u8, ncw: usize) -> usize {
    let (truth, found) = link_position(link, level, ncw).unwrap_or_else(|e| panic!("{e}"));
    let far = far_detections(truth, &found, link.rx.ofdm.p.ncp / 2);
    assert!(far.is_empty(), "{}", far.join("; "));
    truth
}

/// Where preamble B of a `level` burst of `ncw` codewords lands after the `mode`
/// channel, sent and received by `m`.
pub(crate) fn true_position(m: &Modem, mode: &str, level: u8, ncw: usize) -> usize {
    let clean = FmChannelConfig { cnr_db: None, ..FmChannelConfig::by_name(mode).unwrap() };
    let tx = m.modulate(&burst(level, ncw, payload_seed(0)));
    m.ofdm.detect(&transmit(&clean, &tx), 0).at.expect("burst not found on a clean channel")
}

/// `f(t)` for every trial `t` in `0..trials`, spread over every core, in trial
/// order. Each trial seeds itself, so the result does not depend on the threads.
pub(crate) fn on_every_core<R: Send>(trials: u32, f: impl Fn(u32) -> R + Sync) -> Vec<R> {
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()) as u32;
    let f = &f;
    let mut out: Vec<(u32, R)> = std::thread::scope(|s| {
        let workers: Vec<_> = (0..threads)
            .map(|th| s.spawn(move || (th..trials).step_by(threads as usize).map(|t| (t, f(t))).collect::<Vec<_>>()))
            .collect();
        workers.into_iter().flat_map(|w| w.join().unwrap()).collect()
    });
    out.sort_by_key(|(t, _)| *t);
    out.into_iter().map(|(_, r)| r).collect()
}

/// Run `trials` trials of a `level` burst of `ncw` codewords on every core, with
/// preamble B at `truth`.
fn run_trials_at(link: &Link, truth: usize, cnr: f32, level: u8, ncw: usize, trials: u32) -> Vec<Trial> {
    on_every_core(trials, |t| trial(link, cnr, level, ncw, truth, t))
}

/// [`run_trials_at`] where every detector must find the noiseless burst.
fn run_trials(link: &Link, cnr: f32, level: u8, ncw: usize, trials: u32) -> Vec<Trial> {
    run_trials_at(link, strict_link_position(link, level, ncw), cnr, level, ncw, trials)
}

/// Two-sided exact McNemar p-value for `b` and `c` discordant pairs.
fn mcnemar(b: u32, c: u32) -> f64 {
    let n = b + c;
    if n == 0 {
        return 1.0;
    }
    // Binomial(n, 1/2) terms in log space: 0.5^n alone underflows from n = 1075.
    let mut ln_term = n as f64 * 0.5f64.ln();
    let mut tail = 0.0;
    for i in 0..=b.min(c) {
        tail += ln_term.exp();
        ln_term += ((n - i) as f64 / (i + 1) as f64).ln();
    }
    (2.0 * tail).min(1.0)
}

/// Environment variable `name` parsed as `T`, or `default` when unset. A value that
/// does not parse, or is not Unicode, stops the run: falling back would quietly run
/// something else.
pub(crate) fn env_or<T: std::str::FromStr>(name: &str, default: T) -> T {
    match env_var(name) {
        Some(v) => v.trim().parse().unwrap_or_else(|_| panic!("{name}={v:?} does not parse")),
        None => default,
    }
}

/// Environment variable `name`, or `None` when unset. A value that is not Unicode
/// stops the run rather than reading as unset.
fn env_var(name: &str) -> Option<String> {
    match std::env::var(name) {
        Ok(v) => Some(v),
        Err(std::env::VarError::NotPresent) => None,
        Err(std::env::VarError::NotUnicode(v)) => panic!("{name}={v:?} is not Unicode"),
    }
}

/// Environment variable `name` as a comma-separated list of `T`, or `default` when
/// unset. As with [`env_or`], an item that does not parse stops the run.
pub(crate) fn env_list<T: std::str::FromStr>(name: &str, default: &str) -> Vec<T>
where
    T::Err: std::fmt::Display,
{
    let v = env_var(name).unwrap_or_else(|| default.to_string());
    v.split(',').map(|s| s.trim().parse().unwrap_or_else(|e| panic!("{name}: {s:?}: {e}"))).collect()
}

/// `MODES` as a list, `default` when unset, each checked to be a mode up front so
/// that a typo does not fail after the first mode has run.
pub(crate) fn env_modes(default: &str) -> Vec<String> {
    let modes: Vec<String> = env_list("MODES", default);
    for mode in &modes {
        assert!(OfdmParams::by_name(mode).is_some(), "MODES: unknown mode {mode:?} (narrow or wide)");
    }
    modes
}

/// Environment variable `name` as a switch: unset, `0` or `false` is off, `1` or
/// `true` is on. Anything else stops the run, as with [`env_or`].
pub(crate) fn env_flag(name: &str) -> bool {
    match env_var(name).as_deref().map(str::trim) {
        None | Some("0" | "false") => false,
        Some("1" | "true") => true,
        Some(v) => panic!("{name}={v:?}: use 1 or 0"),
    }
}

/// The kinds of audio without a burst in the false-detection pass, for [`noise_case`].
/// The noise case the streaming receiver's CPU load is measured on: a weak carrier
/// keeps the detectors searching without a burst to find.
const LOAD_CASE: &str = "weak carrier, 5 dB CNR";
const NOISE_CASES: [&str; 5] =
    ["carrier at -10 dB CNR", "weak carrier, 5 dB CNR", "white noise", "keyed tones", "data without preambles"];

/// Tone sets for the "keyed tones" case, Hz. 750 + 1500 Hz repeat every nfft/2
/// samples in both modes, like preamble A; then a roger beep and DTMF pairs.
const KEYED_TONES: [&[u64]; 6] = [&[750, 1500], &[1000], &[697, 1209], &[770, 1336], &[852, 1477], &[941, 1633]];
/// On and off time of each keyed tone, seconds.
const KEYED_TONE_S: f32 = 0.3;

/// Audio without a burst, `seconds` long at `fs` Hz: case `case` of [`NOISE_CASES`].
/// Every case, mode and `seed` gets its own noise, so their minutes are independent.
///
/// The keyed tones stop after each 0.3 s: a tone that never stops cannot end a
/// Schmidl-Cox plateau, so it would never be evaluated as a detection. Each burst of
/// tone picks its set and starting phase at random, so the audio does not repeat.
///
/// The data case is what the receiver scans after a header that fails, or after it
/// misses a preamble: level 1 bursts of the longest size, leader and data, with the
/// preambles cut out, through the channel at 30 dB CNR.
fn noise_case(mode: &str, case: usize, seconds: f32, fs: f32, seed: u32) -> Vec<f32> {
    let n = (seconds * fs) as usize;
    assert!(n > 0, "{seconds} s of noise is less than one sample");
    let mode_index = OfdmParams::all().iter().position(|p| p.name == mode).expect("unknown mode");
    let (seed, prng_seed) = noise_seeds(seed, case, mode_index);
    // No silence around the carrier, so the audio is exactly `seconds` of noise.
    let base = FmChannelConfig {
        keyup_delay_s: 0.0,
        lead_silence_s: 0.0,
        trail_silence_s: 0.0,
        seed,
        ..FmChannelConfig::by_name(mode).unwrap()
    };
    let mut rng = Prng::new(prng_seed);
    match case {
        0 => transmit(&FmChannelConfig { cnr_db: Some(-10.0), ..base }, &vec![0.0; n]),
        1 => transmit(&FmChannelConfig { cnr_db: Some(5.0), ..base }, &vec![0.0; n]),
        2 => (0..n).map(|_| 0.3 * rng.uniform()).collect(),
        3 => {
            let rate = fs as u64;
            let slot = (KEYED_TONE_S * fs) as usize;
            let tones: Vec<(usize, u64)> = (0..n / slot + 1)
                .map(|_| (rng.next_u32() as usize % KEYED_TONES.len(), rng.next_u32() as u64 % rate))
                .collect();
            (0..n)
                .map(|i| {
                    let (k, on) = (i / slot, (i % slot) as u64);
                    if k % 2 == 1 {
                        return 0.0;
                    }
                    let (set, start) = tones[k];
                    // Phase from an integer index, exact however long the audio is.
                    KEYED_TONES[set]
                        .iter()
                        .map(|&f| 0.15 * (2.0 * PI * (((on + start) * f) % rate) as f32 / rate as f32).sin())
                        .sum()
                })
                .collect()
        }
        4 => {
            let m = modem(mode);
            let l = m.ofdm.p.symbol_len();
            let leader = m.ofdm.leader_symbols(TXDELAY) * l;
            let ncw = m.max_ncw(1);
            let mut audio = Vec::with_capacity(n + m.burst_samples(1, ncw));
            while audio.len() < n {
                let tx = m.modulate(&burst(1, ncw, rng.next_u32()));
                audio.extend(&tx[..leader]);
                audio.extend(&tx[leader + 2 * l..]);
            }
            audio.truncate(n);
            transmit(&FmChannelConfig { cnr_db: Some(30.0), ..base }, &audio)
        }
        _ => unreachable!("noise case {case}"),
    }
}

/// The channel seed and the generator seed of noise case `case` in mode `mode_index`
/// for `NOISE_SEED` `seed`: distinct for every seed, case and mode. The generator
/// takes 32 bits, so the channel seed's high half is folded into it, not cut off.
fn noise_seeds(seed: u32, case: usize, mode_index: usize) -> (u64, u32) {
    assert!(case < 10 && mode_index < 10, "noise case {case}, mode {mode_index}: seeds would collide");
    let channel = u64::from(seed) * 100 + case as u64 * 10 + mode_index as u64;
    (channel, (channel ^ (channel >> 32)) as u32)
}

/// The CTCSS pass for one receive path: decoded bursts per detector and for the genie
/// (decoding from the true position), out of `trials`, for every tone at every
/// deviation, after a row without a tone. Where the genie decodes and a detector does
/// not, detection lost the burst; where neither does, the tone broke demodulation.
fn ctcss_pass(m: &Modem, mode: &str, case: &CtcssCase, cnr: f32, level: u8, ncw: usize, trials: u32) {
    let base = ctcss_channel(&FmChannelConfig::by_name(mode).unwrap(), case);
    let clean = FmChannelConfig { cnr_db: None, ..base.clone() };
    // The true position as in `link_position`: the best header timing over noiseless
    // copies, which no detector defines.
    let copies: Vec<Burst> = (0..BEST_TIMING_COPIES).map(|t| burst(level, ncw, payload_seed(t))).collect();
    let rxs: Vec<Vec<f32>> = copies.iter().map(|b| transmit(&clean, &m.modulate(b))).collect();
    let lead = (clean.lead_silence_s * clean.fs) as usize;
    let nominal = lead + (m.leader_symbols() + 1) * m.ofdm.p.symbol_len() + m.ofdm.p.ncp;
    let truth = best_timing(m, &rxs, nominal, &copies[0].header)
        .unwrap_or_else(|| panic!("{mode}, {}: the noiseless burst does not demodulate", case.name));
    // A receiver per detector, as in the main sweep.
    let receivers: Vec<Arc<Modem>> = Detector::ALL.iter().map(|&d| eval_modem(mode, 0.0, d)).collect();
    let names: Vec<&str> = Detector::ALL.iter().map(|d| d.name()).chain(["genie"]).collect();
    println!(
        "{mode}, {}: decoded bursts out of {trials} with a CTCSS tone, CNR {cnr} dB, at {CTCSS_DEVIATION_HZ:?} Hz \
         deviation",
        case.name
    );
    // The cells: no tone, then every tone at every deviation.
    let cells: Vec<(f32, f32)> = std::iter::once((0.0, 0.0))
        .chain(CTCSS_HZ.iter().flat_map(|&f| CTCSS_DEVIATION_HZ.iter().map(move |&dev| (f, dev))))
        .collect();
    // Per trial, per cell, per detector then the genie. A trial modulates its burst
    // once, and on a discriminator tap runs the channel once for all its cells.
    let res = on_every_core(trials, |t| {
        let sent = burst(level, ncw, payload_seed(t));
        let cfg = FmChannelConfig { cnr_db: Some(cnr), seed: channel_seed(cnr, t), ..base.clone() };
        let tx = m.modulate(&sent);
        let toneless = transmit(&cfg, &tx);
        cells
            .iter()
            .map(|&(f, dev)| {
                let rx = if dev == 0.0 {
                    toneless.clone()
                } else {
                    with_ctcss_on(&cfg, case.tap, &tx, Some(&toneless), f, dev, 0.7 * t as f32)
                };
                let genie = header_at(m, &rx, truth).is_some_and(|h| burst_ok(m, &rx, truth, &h, &sent));
                receivers.iter().map(|r| received_ok(r, &rx, &sent)).chain([genie]).collect::<Vec<bool>>()
            })
            .collect::<Vec<_>>()
    });
    // Per detector then the genie, per deviation, for the cells `first..` of one row.
    let row = |first: usize, devs: usize| -> Vec<Vec<u32>> {
        (0..names.len())
            .map(|d| (first..first + devs).map(|c| res.iter().filter(|r| r[c][d]).count() as u32).collect())
            .collect()
    };
    let print = |label: String, counts: &[Vec<u32>]| {
        let mut line = format!("  {label:9}");
        for (name, c) in names.iter().zip(counts) {
            line += &format!(" | {name}");
            c.iter().for_each(|ok| line += &format!(" {ok:4}"));
        }
        println!("{line}");
    };
    print("no tone".into(), &row(0, 1));
    let mut total = vec![vec![0u32; CTCSS_DEVIATION_HZ.len()]; names.len()];
    for (i, f) in CTCSS_HZ.into_iter().enumerate() {
        let counts = row(1 + i * CTCSS_DEVIATION_HZ.len(), CTCSS_DEVIATION_HZ.len());
        for (t, c) in total.iter_mut().zip(&counts) {
            t.iter_mut().zip(c).for_each(|(t, c)| *t += c);
        }
        print(format!("{f:6.1} Hz"), &counts);
    }
    print("all".into(), &total);
}

/// Detections and decoded headers in `rx`, scanning on after each detection.
fn false_detections(m: &Modem, d: Detector, rx: &[f32]) -> (u32, u32) {
    let (mut dets, mut headers) = (0, 0);
    let mut found = m.ofdm.detect_with(d, rx, 0);
    while let Some(b) = found.at {
        dets += 1;
        if header_at(m, rx, b).is_some() {
            headers += 1;
        }
        found = m.ofdm.detect_with(d, rx, found.resume);
    }
    (dets, headers)
}

/// Processing time over audio duration of a `Receiver` using `detector` on `audio`,
/// fed in 10 ms chunks. The best of three runs, so that a cold cache or a busy core
/// does not set the figure.
fn receiver_load(mode: &str, detector: Detector, audio: &[f32]) -> f64 {
    let m = eval_modem(mode, 0.0, detector);
    let fs = m.fs() as f64;
    let chunk = (fs / 100.0) as usize;
    (0..3)
        .map(|_| {
            let mut rx = Receiver::new(m.clone());
            let t = Instant::now();
            for c in audio.chunks(chunk) {
                rx.push(c);
            }
            t.elapsed().as_secs_f64() / (audio.len() as f64 / fs)
        })
        .fold(f64::INFINITY, f64::min)
}

#[test]
#[ignore]
fn detector_eval() {
    let modes = env_modes("narrow,wide");
    let trials: u32 = env_or("TRIALS", 100);
    let level: u8 = env_or("LEVEL", 1);
    let ncw: usize = env_or("NCW", 1);
    let noise_s: f32 = env_or("NOISE_SECONDS", 60.0);
    let noise_seed: u32 = env_or("NOISE_SEED", 1);
    let ctcss: f32 = env_or("CTCSS", 0.0);
    let chan: String = env_or("CHANNEL", "matched".to_string());
    let chan = chan.trim();
    let tx_tilt: f32 = env_or("TX_TILT", 0.0);
    assert!(CHANNELS.contains(&chan), "CHANNEL: unknown channel {chan:?} (one of {})", CHANNELS.join(", "));
    assert!(tx_tilt.is_finite(), "TX_TILT={tx_tilt}: must be a finite dB/octave");
    assert!(ctcss.is_finite() && ctcss >= 0.0, "CTCSS={ctcss}: must be a deviation in Hz, 0 for none");
    let ctcss_trials: u32 = env_or("CTCSS_TRIALS", 10);
    assert!((level as usize) < LEVELS.len(), "LEVEL {level}: levels are 0 to {}", LEVELS.len() - 1);
    assert!(trials > 0, "TRIALS must be at least 1");
    assert!(noise_s > 0.0 && noise_s.is_finite(), "NOISE_SECONDS={noise_s}: must be a finite number more than 0");
    let modes: Vec<&str> = modes.iter().map(String::as_str).collect();
    // Checked up front, so that a typo or a bad NCW does not fail after the first mode
    // has run.
    for mode in &modes {
        let link = Link::new(mode, chan, tx_tilt).with_ctcss(ctcss);
        // A longer burst would modulate, but its header would not unpack (MAX_NCW) or
        // the modem would never send it (MAX_PAYLOAD_SECONDS): every trial would fail.
        let max = link.rx.max_ncw(level);
        assert!(ncw <= max, "NCW {ncw}: a level {level} burst in {mode} mode holds at most {max} codewords");
    }
    let cnr_list = |mode: &str| -> Vec<f32> {
        let default = match mode {
            "narrow" => "3,4,5,6,7,8,10",
            "wide" => "5,6,7,8,9,10,12",
            _ => panic!("no default CNRS for mode {mode:?}: set CNRS"),
        };
        let cnrs: Vec<f32> = env_list("CNRS", default);
        for c in &cnrs {
            // NaN or infinite noise gives an all-NaN recording, which would be written as data.
            assert!(c.is_finite(), "CNRS: {c} is not a finite number of dB");
        }
        cnrs
    };
    // At the sensitivity knee, where a tone that cost a dB or two would show, and well
    // above it.
    let ctcss_cnr_list = |mode: &str| -> Vec<f32> {
        let default = if mode == "narrow" { "5,20" } else { "7,20" };
        let cnrs: Vec<f32> = env_list("CTCSS_CNRS", default);
        for c in &cnrs {
            assert!(c.is_finite(), "CTCSS_CNRS: {c} is not a finite number of dB");
        }
        cnrs
    };
    for mode in &modes {
        cnr_list(mode);
        ctcss_cnr_list(mode);
    }
    const CSV_HEADER: &str = "mode,channel,tx_tilt,ctcss_hz,level,ncw,cnr_db,trials,detector,hits,bursts";
    // Any path the OS accepts, Unicode or not.
    let mut csv = std::env::var_os("CSV").map(|p| {
        let mut f = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&p)
            .unwrap_or_else(|e| panic!("CSV: cannot open {p:?}: {e}"));
        // A pipe or a terminal is never read (that would wait for input) and gets no
        // header, rather than one per run. A regular file gets the header when empty
        // (new, or from mktemp, say), and must already have it otherwise: appending rows
        // to a file with other columns would mix them up unnoticed.
        let meta = f.metadata().unwrap_or_else(|e| panic!("CSV: cannot read {p:?}: {e}"));
        if meta.is_file() {
            if meta.len() == 0 {
                writeln!(f, "{CSV_HEADER}").unwrap();
            } else {
                let old = std::fs::read(&p).unwrap_or_else(|e| panic!("CSV: cannot read {p:?}: {e}"));
                let head = old.split(|&b| b == b'\n').next().unwrap_or_default();
                let head = String::from_utf8_lossy(head);
                assert_eq!(head.trim_end_matches('\r'), CSV_HEADER, "CSV: {p:?} has other columns; use a new file");
            }
        }
        f
    });
    let reference = Detector::ALL.iter().position(|&d| d == REFERENCE).expect("reference detector not in ALL");
    println!(
        "level {level} ({}), {ncw} codeword(s), {trials} trials per point, txdelay {TXDELAY} s, \
         channel {chan}, TX tilt {tx_tilt:+} dB/octave, CTCSS {ctcss} Hz, noise seed {noise_seed}",
        LEVELS[level as usize].name
    );
    let mut skipped = Vec::new();
    for &mode in &modes {
        let link = Link::new(mode, chan, tx_tilt).with_ctcss(ctcss);
        let m = &*link.rx;
        println!("\n{mode}: hits and decoded bursts out of {trials}; paired test against '{REFERENCE}'");
        // A link that loses the noiseless burst skips its bursts, not the whole run.
        let truth = match link_position(&link, level, ncw) {
            Ok((truth, found)) => {
                for w in far_detections(truth, &found, m.ofdm.p.ncp / 4) {
                    println!("  warning: {w}; its hits mean little");
                }
                Some(truth)
            }
            Err(e) => {
                println!("  skipped: {e}");
                // A row per point, so that a sweep over links shows the gap.
                if let Some(f) = &mut csv {
                    for cnr in cnr_list(mode) {
                        writeln!(f, "{mode},{chan},{tx_tilt},{ctcss},{level},{ncw},{cnr},{trials},skipped,,").unwrap();
                    }
                }
                skipped.push(mode);
                None
            }
        };
        if let Some(truth) = truth {
            for cnr in cnr_list(mode) {
                let res = run_trials_at(&link, truth, cnr, level, ncw, trials);
                let genie = res.iter().filter(|r| r.genie).count();
                let mut line = format!("  CNR {cnr:5.1} dB  genie {genie:4}");
                for (d, name) in Detector::ALL.iter().enumerate() {
                    let hits = res.iter().filter(|r| r.det[d].0).count();
                    let bursts = res.iter().filter(|r| r.det[d].1).count();
                    line += &format!(" | {name} hit {hits:4} burst {bursts:4}");
                    if d != reference {
                        let better = res.iter().filter(|r| r.det[d].1 && !r.det[reference].1).count() as u32;
                        let worse = res.iter().filter(|r| !r.det[d].1 && r.det[reference].1).count() as u32;
                        line += &format!(" (+{better}/-{worse}, p {:.3})", mcnemar(better, worse));
                    }
                    if let Some(f) = &mut csv {
                        writeln!(
                            f,
                            "{mode},{chan},{tx_tilt},{ctcss},{level},{ncw},{cnr},{trials},{name},{hits},{bursts}"
                        )
                        .unwrap();
                    }
                }
                if let Some(f) = &mut csv {
                    writeln!(f, "{mode},{chan},{tx_tilt},{ctcss},{level},{ncw},{cnr},{trials},genie,,{genie}").unwrap();
                }
                println!("{line}");
            }
        }
        if ctcss_trials > 0 {
            for cnr in ctcss_cnr_list(mode) {
                for case in ctcss_cases(mode) {
                    ctcss_pass(m, mode, case, cnr, level, ncw, ctcss_trials);
                }
            }
        }
        println!("{mode}: per minute of audio without a burst, detections / decoded headers");
        let mut load_audio = None;
        for (c, case) in NOISE_CASES.iter().enumerate() {
            let audio = noise_case(mode, c, noise_s, m.fs(), noise_seed);
            let mut line = format!("  {case:26}");
            let per_min = 60.0 / (audio.len() as f32 / m.fs());
            for d in Detector::ALL {
                let (n, h) = false_detections(m, d, &audio);
                line += &format!(" | {d} {:7.2} / {:5.2}", n as f32 * per_min, h as f32 * per_min);
            }
            line += &format!(" | mf peak {:.3} (threshold {MF_THRESHOLD})", m.ofdm.mf_peak(&audio));
            println!("{line}");
            if *case == LOAD_CASE {
                load_audio = Some(audio);
            }
        }
        let load_audio = load_audio.unwrap_or_else(|| panic!("LOAD_CASE {LOAD_CASE:?} is not a noise case"));
        let mut line = format!("{mode}: streaming receiver CPU on '{LOAD_CASE}', 10 ms chunks:");
        for d in Detector::ALL {
            line += &format!(" {d} {:.2} %", 100.0 * receiver_load(mode, d, &load_audio));
        }
        println!("{line}");
    }
    // The rest of the run is still worth having, but it must not pass unnoticed.
    assert!(skipped.is_empty(), "no bursts measured in {skipped:?}: see 'skipped' above");
}

#[test]
fn noise_seeds_stay_distinct_past_32_bits() {
    // The default seed gives the generator what it always did.
    assert_eq!(noise_seeds(1, 3, 1), (131, 131));
    // Seeds whose channel seed passes 2^32 no longer lose its high half.
    let seeds: Vec<u32> = [u32::MAX, u32::MAX - 1, 42_949_673, 42_949_674]
        .into_iter()
        .flat_map(|s| (0..5).map(move |c| noise_seeds(s, c, 0).1))
        .collect();
    let mut unique = seeds.clone();
    unique.sort();
    unique.dedup();
    assert_eq!(unique.len(), seeds.len(), "{seeds:?}");
}

#[test]
fn on_every_core_keeps_trial_order() {
    assert_eq!(on_every_core(37, |t| t), (0..37).collect::<Vec<_>>());
    assert!(on_every_core(0, |t| t).is_empty());
}

/// A CI guard on the matched filter's threshold, on noise the evaluation's noise cases
/// do not use: FM channel noise with other seeds and a fresh white-noise sequence, in
/// every mode, so a new mode must be calibrated. Setting `MF_MARGIN` prints the peaks.
#[test]
fn matched_filter_stays_below_threshold_on_unseen_noise() {
    let seconds = 5.0;
    for mode in OfdmParams::all().map(|p| p.name) {
        let m = modem(mode);
        let n = (seconds * m.fs()) as usize;
        let base = FmChannelConfig {
            keyup_delay_s: 0.0,
            lead_silence_s: 0.0,
            trail_silence_s: 0.0,
            ..FmChannelConfig::by_name(mode).unwrap()
        };
        let mut rng = Prng::new(0x0dd_5eed);
        let cases = [
            (
                "carrier at -10 dB",
                transmit(&FmChannelConfig { cnr_db: Some(-10.0), seed: 4242, ..base.clone() }, &vec![0.0; n]),
            ),
            ("carrier at 5 dB", transmit(&FmChannelConfig { cnr_db: Some(5.0), seed: 4243, ..base }, &vec![0.0; n])),
            ("white noise", (0..n).map(|_| 0.3 * rng.uniform()).collect()),
        ];
        for (name, audio) in cases {
            let peak = m.ofdm.mf_peak(&audio);
            if std::env::var_os("MF_MARGIN").is_some() {
                println!("{mode} {name}: mf peak {peak:.3}");
            }
            assert!(peak < MF_THRESHOLD, "{mode} {name}: mf peak {peak} reaches the threshold");
        }
    }
}

/// The harness itself: on a strong signal every detector and the genie decode every
/// burst, and no detector raises a false header on a carrier at -10 dB CNR. `hit` is
/// a diagnostic, so it is not required here.
#[test]
fn harness_sees_strong_bursts_and_no_false_headers() {
    for mode in OfdmParams::all().map(|p| p.name) {
        let link = Link::matched(mode);
        let m = &*link.rx;
        // The channel's delays must not depend on the payload, or noiseless copies
        // could not give the position for every trial.
        strict_link_position(&link, 1, 1);
        let (_, found) = link_position(&link, 1, 1).unwrap();
        let reference = Detector::ALL.iter().position(|&d| d == REFERENCE).unwrap();
        let clean = FmChannelConfig { cnr_db: None, ..link.chan.clone() };
        for t in 1..4 {
            let tx = m.modulate(&burst(1, 1, payload_seed(t)));
            let at = m.ofdm.detect_with(REFERENCE, &transmit(&clean, &tx), 0).at;
            assert_eq!(at, found[reference], "{mode}: payload {t} moves the burst");
        }
        for r in run_trials(&link, 25.0, 1, 1, 4) {
            assert!(r.genie, "{mode}: genie failed on a strong signal");
            assert!(r.det.iter().all(|&(_, ok)| ok), "{mode}: a detector missed a strong burst");
        }
        let audio = noise_case(mode, 0, 2.0, m.fs(), 1);
        for d in Detector::ALL {
            assert_eq!(false_detections(m, d, &audio).1, 0, "{mode}: {d} decoded a header from noise");
        }
    }
}

/// Every radio response, and a TX tilt of ±6 dB/octave on the matched response, still
/// carries a strong burst to the genie and to every detector. The genie check makes a
/// detector failure here a failure of detection, not of a channel nothing could
/// decode. Both at once can be too much: in wide mode, pre-emphasis and +6 dB/octave
/// clip the top carriers.
#[test]
fn mismatched_links_still_carry_strong_bursts() {
    let cases = CHANNELS.iter().map(|&c| (c, 0.0)).chain([("matched", -6.0), ("matched", 6.0)]);
    for (chan, tilt) in cases {
        for mode in OfdmParams::all().map(|p| p.name) {
            for r in run_trials(&Link::new(mode, chan, tilt), 25.0, 1, 1, 2) {
                assert!(r.genie, "{mode} {chan} tilt {tilt}: genie failed on a strong signal");
                assert!(r.det.iter().all(|&(_, ok)| ok), "{mode} {chan} tilt {tilt}: a detector missed a strong burst");
            }
        }
    }
}

/// The CTCSS path: the tone reaches the receiver at the deviation sent, and a
/// wide-mode link with a tone still carries strong bursts to the genie and to every
/// detector. A stronger tone costs Schmidl-Cox bursts at any CNR, so this keeps to
/// 375 Hz.
#[test]
fn ctcss_tone_reaches_the_receiver_and_strong_bursts_get_through() {
    let deviation_hz = 375.0;
    let link = Link::matched("wide").with_ctcss(deviation_hz);
    for t in 0..2 {
        let cfg = link.trial_channel(25.0, t);
        let tone = cfg.ctcss.clone().expect("the link adds a tone");
        let tx = link.tx.modulate(&burst(1, 1, payload_seed(t)));
        let rx = transmit(&cfg, &tx);
        // While the squelch is open: from key-up to the end of the burst.
        let (lead, keyup) = ((cfg.lead_silence_s * cfg.fs) as usize, (cfg.keyup_delay_s * cfg.fs) as usize);
        let heard = &rx[lead + keyup..lead + tx.len()];
        // The received amplitude at the tone's frequency.
        let w = 2.0 * std::f64::consts::PI * tone.hz as f64 / cfg.fs as f64;
        let (c, s) = heard.iter().enumerate().fold((0.0, 0.0), |(c, s), (i, &x)| {
            let ph = w * i as f64;
            (c + x as f64 * ph.cos(), s + x as f64 * ph.sin())
        });
        let amp = 2.0 * f64::hypot(c, s) / heard.len() as f64;
        let want = (deviation_hz / cfg.deviation_hz) as f64;
        assert!((amp / want - 1.0).abs() < 0.25, "{} Hz tone received at {amp:.3}, sent at {want:.3}", tone.hz);
    }
    for r in run_trials(&link, 25.0, 1, 1, 2) {
        assert!(r.genie, "genie failed on a strong signal with CTCSS");
        assert!(r.det.iter().all(|&(_, ok)| ok), "a detector missed a strong burst with CTCSS");
    }
}

/// What the matched filter offers, guarded in CI. Where bursts start to decode on a
/// matched link, it decodes all but at most one of what the genie decodes. That is
/// where it beats Schmidl-Cox: in `detector_eval` (level 1, 200 trials) narrow mode at
/// 5 dB gives the genie 200, the matched filter 200 and Schmidl-Cox 173; wide mode at
/// 7 dB is close (196, 196, 193).
///
/// On a wide link that de-emphasises only at the receiver, Schmidl-Cox has been ahead
/// since #61 widened its search for preamble B: at 7 dB the genie 195, Schmidl-Cox
/// 183, the matched filter 171. In this test's 40 trials that is 36
/// against 32, so the matched filter may trail Schmidl-Cox by up to 6. A better
/// detector passes; a worse threshold or template fails.
#[test]
fn matched_filter_holds_its_sensitivity_where_bursts_start_to_decode() {
    let mf = Detector::ALL.iter().position(|&d| d == Detector::MatchedFilter).unwrap();
    let reference = Detector::ALL.iter().position(|&d| d == REFERENCE).unwrap();
    let count = |res: &[Trial], i: usize| res.iter().filter(|r| r.det[i].1).count();
    for (mode, cnr) in [("narrow", 5.0), ("wide", 7.0)] {
        let res = run_trials(&Link::matched(mode), cnr, 1, 1, 40);
        let genie = res.iter().filter(|r| r.genie).count();
        assert!(genie >= 30, "{mode} {cnr} dB: genie decodes only {genie} of 40; pick another point");
        let got = count(&res, mf);
        assert!(got + 1 >= genie, "{mode} {cnr} dB: matched filter {got}, genie {genie}");
    }
    let res = run_trials(&Link::new("wide", "rx-de-only", 0.0), 7.0, 1, 1, 40);
    let (got, sc) = (count(&res, mf), count(&res, reference));
    assert!(got + 6 >= sc, "wide rx-de-only 7 dB: matched filter {got}, {REFERENCE} {sc}");
}

#[test]
fn mcnemar_matches_known_values() {
    assert_eq!(mcnemar(0, 0), 1.0);
    assert!((mcnemar(0, 5) - 0.0625).abs() < 1e-12);
    assert!((mcnemar(2, 8) - 0.109375).abs() < 1e-12);
    assert_eq!(mcnemar(4, 4), 1.0);
    // Past the point where 0.5^n underflows.
    assert_eq!(mcnemar(1100, 1100), 1.0);
    // z = 4.26 with continuity correction; the exact value is about 2e-5.
    let p = mcnemar(1000, 1200);
    assert!(p > 1e-6 && p < 1e-4, "p {p}");
}
