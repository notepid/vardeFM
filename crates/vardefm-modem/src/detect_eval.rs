//! Side-by-side evaluation of burst detectors through the FM channel simulator.
//!
//! Every detector sees exactly the same received audio, so differences are paired
//! and can be tested with McNemar's exact test instead of comparing two noisy rates.
//! For each CNR it reports, per detector:
//! - `burst`, the measure that counts: the whole burst (header and every codeword)
//!   decoded by an offline equivalent of `Receiver::step`. It tries the header at each
//!   detection in turn, searches on after one that fails, and keeps the first header
//!   that decodes. A test checks it against the streaming `Receiver` searching with
//!   the same detector, for every detector here;
//! - `hit`: the detector's first detection, whatever its header does, lies within
//!   ±ncp/4 samples of the true position, which is where Schmidl-Cox's preamble-B fine
//!   timing lands on a noiseless copy. A detector that always picks another point
//!   inside the cyclic prefix can decode every burst and still score few hits, so read
//!   `hit` as a diagnostic only.
//!
//! `genie` decodes from the true position: a known-timing reference, not a strict
//! bound. Another valid position inside the cyclic prefix puts the FFT windows on
//! other noise samples and can decode a recording that fails at the true position,
//! so a detector may occasionally beat it. The gap between `genie` and a detector
//! approximates what detection costs at that CNR.
//!
//! The true position comes from the reference detector, not from the channel: the
//! simulator's filters are IIR, so there is no single delay to add to the burst
//! layout, and any point in the cyclic prefix that avoids the channel's tails is as
//! valid. Each mode prints how many samples the true position lies after the sent
//! boundary of preamble B, the channel delay (the lag at which the sent and received
//! audio correlate best), and the difference. At the time of writing the true position
//! is 31 samples after the boundary in narrow mode and the channel delay is 11, so a
//! detector that lands on the boundary plus the channel delay is 20 samples from the
//! true position, inside the ±32 of `hit` but not by much. In wide mode the figures are
//! 9 and 6, 3 apart, against ±16. Both `hit` and `genie` share that bias.
//!
//! A separate pass counts detections and decoded headers per minute on audio that
//! holds no burst (a carrier at -10 dB CNR, which sounds like an open squelch, a
//! weak unmodulated carrier, white noise, and keyed tones: 750 + 1500 Hz, a roger
//! beep and DTMF pairs, each 0.3 s on and 0.3 s off).
//! A detection there costs a header decode; a decoded header there is a false burst.
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
//! `CTCSS_TRIALS` per tone and deviation (10; 0 skips the pass; trial `t` has the same
//! noise in every row, so the rows are paired and each rests on that many noise draws),
//! `CTCSS_CNRS` (dB list; 5,20 narrow and 7,20 wide: the knee and well above it),
//! `CSV` (file to append the burst results to).

use super::*;
use crate::busy_eval::CTCSS_DEVIATION_HZ;
use std::collections::HashMap;
use std::f32::consts::PI;
use std::io::Write;
use std::sync::Arc;
use vardefm_dsp::prng::Prng;
use vardefm_dsp::CTCSS_HZ;
use vardefm_sim::{transmit, Ctcss, FmChannelConfig};

pub(crate) use crate::receiver::DetectFn;

/// Detectors under evaluation. The first one is the reference for the paired tests.
pub(crate) const DETECTORS: &[(&str, DetectFn)] = &[("sc", |m, rx, from| m.ofdm.detect(rx, from))];

/// Leader length used by the evaluation, seconds (the `vardefm modem` default).
const TXDELAY: f32 = 0.15;

pub(crate) fn modem(mode: &str) -> Arc<Modem> {
    let mut m = Modem::new(OfdmParams::by_name(mode).unwrap(), TXDELAY);
    // Trials already run on every core.
    m.decode_threads = 1;
    Arc::new(m)
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

/// What decoding one recording at a preamble-B position gives, by position: `None`
/// when the header fails, else whether the burst came out as sent. The genie and the
/// detectors often land on the same position, and a decode is the costly part of a
/// trial, so each position is decoded once per recording.
type Decodes = HashMap<usize, Option<bool>>;

/// [`header_at`], then [`burst_ok`] if the header decodes, remembered in `cache`.
fn decode_at(m: &Modem, rx: &[f32], b: usize, sent: &Burst, cache: &mut Decodes) -> Option<bool> {
    *cache.entry(b).or_insert_with(|| header_at(m, rx, b).map(|h| burst_ok(m, rx, b, &h, sent)))
}

/// What the receiver makes of one recording with a given detector.
struct Outcome {
    /// Where the detector first pointed, header decoded or not.
    first: Option<usize>,
    /// Whether the burst came out as sent.
    ok: bool,
}

/// Offline equivalent of `Receiver::step` with detector `f`: decode the header at
/// each detection in turn, searching on from a detection whose header fails, and
/// judge the burst of the first header that decodes. The test
/// `offline_receive_matches_the_streaming_receiver` checks it against `Receiver`.
/// `cache` holds the decodes of `rx` made so far.
fn receive(m: &Modem, f: DetectFn, rx: &[f32], sent: &Burst, cache: &mut Decodes) -> Outcome {
    let first = advance(m, f, rx, 0);
    let mut next = first;
    while let Some(b) = next {
        if let Some(ok) = decode_at(m, rx, b, sent, cache) {
            return Outcome { first, ok };
        }
        // As `Receiver::step`: search on from the failed detection itself.
        next = advance(m, f, rx, b);
    }
    Outcome { first, ok: false }
}

/// `f` searching `rx` from `from`, which must return more than `from` ([`DetectFn`]):
/// searching on from a detection that returned `from` again would hang the
/// evaluation, so stop instead. The first search, from 0, is checked as well; a
/// real detection is never at 0, because preamble B's body follows preamble A.
fn advance(m: &Modem, f: DetectFn, rx: &[f32], from: usize) -> Option<usize> {
    let next = f(m, rx, from);
    if let Some(n) = next {
        assert!(n > from, "detector returned {n} when searching from {from}: it would loop forever");
    }
    next
}

/// Outcome of one trial: (hit, burst) per detector, then the genie's burst.
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

/// The `level` burst of `ncw` codewords that trial `t` sends, and the audio the
/// receiver gets from the `mode` channel at `cnr` dB.
pub(crate) fn received(m: &Modem, mode: &str, cnr: f32, level: u8, ncw: usize, t: u32) -> (Burst, Vec<f32>) {
    let sent = burst(level, ncw, payload_seed(t));
    let rx = through_channel(m, mode, cnr, &sent, t);
    (sent, rx)
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

fn trial(m: &Modem, mode: &str, cnr: f32, level: u8, ncw: usize, truth: usize, t: u32) -> Trial {
    let (sent, rx) = received(m, mode, cnr, level, ncw, t);
    let tol = m.ofdm.p.ncp / 4;
    let mut cache = Decodes::new();
    let det = DETECTORS
        .iter()
        .map(|(_, f)| {
            let o = receive(m, *f, &rx, &sent, &mut cache);
            (o.first.is_some_and(|b| b.abs_diff(truth) <= tol), o.ok)
        })
        .collect();
    let genie = decode_at(m, &rx, truth, &sent, &mut cache) == Some(true);
    Trial { det, genie }
}

/// Where preamble B of a `level` burst of `ncw` codewords lands after the channel.
/// The channel's delays depend neither on its noise nor on the payload (the burst
/// layout is fixed by level and size), so one noiseless copy gives it for every trial.
pub(crate) fn true_position(m: &Modem, mode: &str, level: u8, ncw: usize) -> usize {
    clean_position(m, &clean_copy(m, mode, level, ncw, 0).1)
}

/// The true position in `rx`, a noiseless copy from [`clean_copy`]: where the
/// reference detector finds preamble B.
fn clean_position(m: &Modem, rx: &[f32]) -> usize {
    m.ofdm.detect(rx, 0).expect("burst not found on a clean channel")
}

/// The `level` burst of `ncw` codewords with payload `t` as modulated, and as a
/// noiseless `mode` channel delivers it.
fn clean_copy(m: &Modem, mode: &str, level: u8, ncw: usize, t: u32) -> (Vec<f32>, Vec<f32>) {
    let clean = FmChannelConfig { cnr_db: None, ..FmChannelConfig::by_name(mode).unwrap() };
    let tx = m.modulate(&burst(level, ncw, payload_seed(t)));
    let rx = transmit(&clean, &tx);
    (tx, rx)
}

/// Where the true position lies against the channel, from one noiseless copy: (the
/// true position as [`true_position`] finds it, samples after the sent boundary of
/// preamble B's body, channel delay in samples).
///
/// The boundary is counted back from the end of the modulated burst, so the leader
/// and preambles come from the modulator itself: preamble B's body ends where the
/// header and codeword symbols start. The channel delay is the lag up to one cyclic
/// prefix at which the received copy correlates best with what was sent. Neither
/// depends on any detector. The correlation is signed: the simulated channel does
/// not invert the audio. A delay of 0 or a whole cyclic prefix is the edge of the
/// search, not a peak.
fn position_check(m: &Modem, mode: &str, level: u8, ncw: usize) -> (usize, i64, usize) {
    let cfg = FmChannelConfig::by_name(mode).unwrap();
    let p = &m.ofdm.p;
    let (tx, rx) = clean_copy(m, mode, level, ncw, 0);
    let truth = clean_position(m, &rx);
    let lead = (cfg.lead_silence_s * cfg.fs) as usize;
    let data = m.header_symbols() + ncw * m.symbols_per_codeword(level);
    let boundary = lead + tx.len() - data * p.symbol_len() - p.nfft;
    let rx = &rx[lead..];
    let corr = |d: usize| -> f64 { tx.iter().zip(&rx[d..]).map(|(&a, &b)| a as f64 * b as f64).sum() };
    let delay = (0..=p.ncp).map(|d| (corr(d), d)).max_by(|a, b| a.0.total_cmp(&b.0)).unwrap().1;
    (truth, truth as i64 - boundary as i64, delay)
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
/// preamble B at `truth` ([`true_position`]).
fn run_trials(m: &Modem, mode: &str, cnr: f32, level: u8, ncw: usize, truth: usize, trials: u32) -> Vec<Trial> {
    on_every_core(trials, |t| trial(m, mode, cnr, level, ncw, truth, t))
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
const NOISE_CASES: [&str; 4] = ["carrier at -10 dB CNR", "weak carrier, 5 dB CNR", "white noise", "keyed tones"];

/// Tone sets for the "keyed tones" case, Hz. 750 + 1500 Hz repeat every nfft/2
/// samples in both modes, like preamble A; then a roger beep and DTMF pairs.
const KEYED_TONES: [&[u64]; 6] = [&[750, 1500], &[1000], &[697, 1209], &[770, 1336], &[852, 1477], &[941, 1633]];
/// On and off time of each keyed tone, seconds.
const KEYED_TONE_S: f32 = 0.3;

/// Audio without a burst, `seconds` long at `fs` Hz: case `case` of [`NOISE_CASES`].
///
/// The keyed tones stop after each 0.3 s: a tone that never stops cannot end a
/// Schmidl-Cox plateau, so it would never be evaluated as a detection.
fn noise_case(mode: &str, case: usize, seconds: f32, fs: f32) -> Vec<f32> {
    let n = (seconds * fs) as usize;
    assert!(n > 0, "{seconds} s of noise is less than one sample");
    // No silence around the carrier, so the audio is exactly `seconds` of noise.
    let base = FmChannelConfig {
        keyup_delay_s: 0.0,
        lead_silence_s: 0.0,
        trail_silence_s: 0.0,
        ..FmChannelConfig::by_name(mode).unwrap()
    };
    match case {
        0 => transmit(&FmChannelConfig { cnr_db: Some(-10.0), ..base }, &vec![0.0; n]),
        1 => transmit(&FmChannelConfig { cnr_db: Some(5.0), ..base }, &vec![0.0; n]),
        2 => {
            let mut rng = Prng::new(99);
            (0..n).map(|_| 0.3 * rng.uniform()).collect()
        }
        3 => {
            let rate = fs as u64;
            let slot = (KEYED_TONE_S * fs) as usize;
            (0..n)
                .map(|i| {
                    let (k, on) = (i / slot, i % slot);
                    if k % 2 == 1 {
                        return 0.0;
                    }
                    // Phase from an integer index, exact however long the audio is.
                    KEYED_TONES[k / 2 % KEYED_TONES.len()]
                        .iter()
                        .map(|&f| 0.15 * (2.0 * PI * ((on as u64 * f) % rate) as f32 / rate as f32).sin())
                        .sum()
                })
                .collect()
        }
        _ => unreachable!("noise case {case}"),
    }
}

/// The CTCSS pass for one receive path: decoded bursts per detector and for the genie
/// (decoding from the true position), out of `trials`, for every tone at every
/// deviation, after a row without a tone. Where the genie decodes and a detector does
/// not, detection lost the burst; where neither does, the tone broke demodulation.
fn ctcss_pass(m: &Modem, mode: &str, case: &CtcssCase, cnr: f32, level: u8, ncw: usize, trials: u32) {
    let base = ctcss_channel(&FmChannelConfig::by_name(mode).unwrap(), case);
    let clean = FmChannelConfig { cnr_db: None, ..base.clone() };
    let truth = clean_position(m, &transmit(&clean, &m.modulate(&burst(level, ncw, payload_seed(0)))));
    let names: Vec<&str> = DETECTORS.iter().map(|(n, _)| *n).chain(["genie"]).collect();
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
                let mut cache = Decodes::new();
                let genie = decode_at(m, &rx, truth, &sent, &mut cache) == Some(true);
                DETECTORS
                    .iter()
                    .map(|(_, d)| receive(m, *d, &rx, &sent, &mut cache).ok)
                    .chain([genie])
                    .collect::<Vec<bool>>()
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
///
/// It searches on from each detection itself, as `receive` and `Receiver::step` do
/// after a header that fails, so a detector is scored here on the searches it makes
/// in the receiver. After a header that decodes the receiver skips the frame it
/// announces; this pass searches on from the detection instead, so that every
/// detection in the audio is counted.
fn false_detections(m: &Modem, f: DetectFn, rx: &[f32]) -> (u32, u32) {
    let (mut dets, mut headers) = (0, 0);
    let mut next = advance(m, f, rx, 0);
    while let Some(b) = next {
        dets += 1;
        if header_at(m, rx, b).is_some() {
            headers += 1;
        }
        next = advance(m, f, rx, b);
    }
    (dets, headers)
}

#[test]
#[ignore]
fn detector_eval() {
    let modes = env_modes("narrow,wide");
    let trials: u32 = env_or("TRIALS", 100);
    let level: u8 = env_or("LEVEL", 1);
    let ncw: usize = env_or("NCW", 1);
    let noise_s: f32 = env_or("NOISE_SECONDS", 60.0);
    let ctcss_trials: u32 = env_or("CTCSS_TRIALS", 10);
    assert!((level as usize) < LEVELS.len(), "LEVEL {level}: levels are 0 to {}", LEVELS.len() - 1);
    assert!(trials > 0, "TRIALS must be at least 1");
    assert!(noise_s > 0.0 && noise_s.is_finite(), "NOISE_SECONDS={noise_s}: must be a finite number more than 0");
    let modes: Vec<&str> = modes.iter().map(String::as_str).collect();
    // Checked up front, so that a bad NCW does not fail after the first mode has run.
    let modems: Vec<Arc<Modem>> = modes
        .iter()
        .map(|mode| {
            let m = modem(mode);
            // A longer burst would modulate, but its header would not unpack (MAX_NCW) or
            // the modem would never send it (MAX_PAYLOAD_SECONDS): every trial would fail.
            let max = m.max_ncw(level);
            assert!(ncw <= max, "NCW {ncw}: a level {level} burst in {mode} mode holds at most {max} codewords");
            m
        })
        .collect();
    let cnr_list = |mode: &str| -> Vec<f32> {
        let default = if mode == "narrow" { "3,4,5,6,7,8,10" } else { "5,6,7,8,9,10,12" };
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
    // Any path the OS accepts, Unicode or not.
    let mut csv = std::env::var_os("CSV").map(|p| {
        let mut f = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&p)
            .unwrap_or_else(|e| panic!("CSV: cannot open {p:?}: {e}"));
        // An empty file (from mktemp, say) needs the header as much as a new one. A pipe
        // or a terminal always reads as empty, so it gets none rather than one per run.
        let meta = f.metadata().unwrap_or_else(|e| panic!("CSV: cannot read {p:?}: {e}"));
        if meta.is_file() && meta.len() == 0 {
            writeln!(f, "mode,level,ncw,cnr_db,trials,detector,hits,bursts").unwrap();
        }
        f
    });
    let names: Vec<&str> = DETECTORS.iter().map(|(n, _)| *n).collect();
    println!(
        "level {level} ({}), {ncw} codeword(s), {trials} trials per point, txdelay {TXDELAY} s",
        LEVELS[level as usize].name
    );
    for (&mode, m) in modes.iter().zip(modems) {
        println!("\n{mode}: hits and decoded bursts out of {trials}; paired test against '{}'", names[0]);
        // The true position is the reference's own; show where it lies against the channel.
        let (truth, after, delay) = position_check(&m, mode, level, ncw);
        println!(
            "  true position: sample {truth}, {after} after the sent boundary; channel delay {delay}{}, \
             so {} after the boundary plus the delay; hit within ±{}",
            if delay == 0 || delay == m.ofdm.p.ncp { " (edge of the search, not a peak)" } else { "" },
            after - delay as i64,
            m.ofdm.p.ncp / 4
        );
        for cnr in cnr_list(mode) {
            let res = run_trials(&m, mode, cnr, level, ncw, truth, trials);
            let genie = res.iter().filter(|r| r.genie).count();
            let mut line = format!("  CNR {cnr:5.1} dB  genie {genie:4}");
            for (d, name) in names.iter().enumerate() {
                let hits = res.iter().filter(|r| r.det[d].0).count();
                let bursts = res.iter().filter(|r| r.det[d].1).count();
                line += &format!(" | {name} hit {hits:4} burst {bursts:4}");
                if d > 0 {
                    let better = res.iter().filter(|r| r.det[d].1 && !r.det[0].1).count() as u32;
                    let worse = res.iter().filter(|r| !r.det[d].1 && r.det[0].1).count() as u32;
                    line += &format!(" (+{better}/-{worse}, p {:.3})", mcnemar(better, worse));
                }
                if let Some(f) = &mut csv {
                    writeln!(f, "{mode},{level},{ncw},{cnr},{trials},{name},{hits},{bursts}").unwrap();
                }
            }
            if let Some(f) = &mut csv {
                writeln!(f, "{mode},{level},{ncw},{cnr},{trials},genie,,{genie}").unwrap();
            }
            println!("{line}");
        }
        if ctcss_trials > 0 {
            for cnr in ctcss_cnr_list(mode) {
                for case in ctcss_cases(mode) {
                    ctcss_pass(&m, mode, case, cnr, level, ncw, ctcss_trials);
                }
            }
        }
        println!("{mode}: per minute of audio without a burst, detections / decoded headers");
        for (c, case) in NOISE_CASES.iter().enumerate() {
            let audio = noise_case(mode, c, noise_s, m.fs());
            let mut line = format!("  {case:26}");
            let per_min = 60.0 / (audio.len() as f32 / m.fs());
            for (name, f) in DETECTORS {
                let (d, h) = false_detections(&m, *f, &audio);
                line += &format!(" | {name} {:7.2} / {:5.2}", d as f32 * per_min, h as f32 * per_min);
            }
            println!("{line}");
        }
    }
}

#[test]
fn on_every_core_keeps_trial_order() {
    assert_eq!(on_every_core(37, |t| t), (0..37).collect::<Vec<_>>());
    assert!(on_every_core(0, |t| t).is_empty());
}

/// The harness itself: on a strong signal every detector and the genie decode every
/// burst, and no detector raises a false header on a carrier at -10 dB CNR. `hit` is
/// a diagnostic, so it is not required here.
#[test]
fn harness_sees_strong_bursts_and_no_false_headers() {
    for mode in ["narrow", "wide"] {
        let m = modem(mode);
        let (truth, after, delay) = position_check(&m, mode, 1, 1);
        // The true position must not depend on the payload, or one copy could not give it.
        // `position_check` finds it as `true_position` does: `clean_position` on payload 0.
        for t in 1..4 {
            let pos = clean_position(&m, &clean_copy(&m, mode, 1, 1, t).1);
            assert_eq!(pos, truth, "{mode}: payload {t} moves the burst");
        }
        // The printed cross-check must stay meaningful: the true position lies within a
        // cyclic prefix of the sent boundary plus the channel delay.
        let ncp = m.ofdm.p.ncp as i64;
        assert!(delay > 0 && (delay as i64) < ncp, "{mode}: channel delay {delay} is the edge of the 0..={ncp} search");
        assert!(
            (after - delay as i64).abs() <= ncp,
            "{mode}: true position {after} after the sent boundary, channel delay {delay}: more than {ncp} apart"
        );
        for r in run_trials(&m, mode, 25.0, 1, 1, truth, 4) {
            assert!(r.genie, "{mode}: genie failed on a strong signal");
            assert!(r.det.iter().all(|&(_, ok)| ok), "{mode}: a detector missed a strong burst");
        }
        let audio = noise_case(mode, 0, 2.0, m.fs());
        for (name, f) in DETECTORS {
            assert_eq!(false_detections(&m, *f, &audio).1, 0, "{mode}: {name} decoded a header from noise");
        }
    }
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

/// `receive` must judge each recording the way the streaming `Receiver` does when
/// it is fed the same audio in 10 ms chunks, including the recordings where the
/// detector or the header fails. It runs for every entry of `DETECTORS`, with the
/// receiver searching through the same detector (`Receiver::detect`): `receive`
/// gives a detector the whole recording, the receiver a growing buffer it trims, and
/// a detector that depends on how much audio it is given could score well here and
/// differently in the receiver. Only the burst outcome is compared; `hit` and the
/// false-detection pass are offline only.
///
/// Each seed runs at three CNRs per mode. At the lowest even genie decodes nothing
/// and at the highest every detector so far decodes almost everything, so both
/// outcomes occur whatever the detector is, and `STREAM_SEEDS=1` works.
/// 12 seeds per mode by default.
#[test]
fn offline_receive_matches_the_streaming_receiver() {
    let seeds: u64 = env_or("STREAM_SEEDS", 12);
    for (mode, cnrs) in [("narrow", [3.0, 5.0, 8.0]), ("wide", [5.0, 7.0, 10.0])] {
        let m = modem(mode);
        let sent = burst(1, 1, 7);
        let tx = m.modulate(&sent);
        let (mut total, mut ok) = (0, vec![0; DETECTORS.len()]);
        for seed in 0..seeds {
            for cnr in cnrs {
                let cfg =
                    FmChannelConfig { cnr_db: Some(cnr), seed: 1000 + seed, ..FmChannelConfig::by_name(mode).unwrap() };
                let mut rx = transmit(&cfg, &tx);
                rx.extend(vec![0.0; m.fs() as usize]);
                // Shared by the detectors, as in a trial: they all decode the same recording.
                let mut cache = Decodes::new();
                for (d, (name, f)) in DETECTORS.iter().enumerate() {
                    // Offline first: a detector that does not move forward on the whole
                    // recording fails there; one that stalls only on the receiver's
                    // buffer fails in `Receiver::step`.
                    let offline = receive(&m, *f, &rx, &sent, &mut cache).ok;
                    let mut receiver = Receiver::new(m.clone());
                    receiver.detect = *f;
                    // 10 ms chunks, as a sound card delivers them.
                    let chunk = (m.fs() / 100.0) as usize;
                    let got: Vec<RxBurst> = rx.chunks(chunk).flat_map(|c| receiver.push(c)).collect();
                    let streamed = got.first().is_some_and(|g| {
                        g.header == sent.header
                            && g.payload.iter().zip(&sent.payload).all(|(p, s)| p.as_ref() == Some(s))
                    });
                    assert_eq!(
                        offline, streamed,
                        "{mode} {name} {cnr} dB seed {seed}: offline {offline}, streaming {streamed}"
                    );
                    ok[d] += streamed as u32;
                }
                total += 1;
            }
        }
        // Both outcomes must occur, or the comparison proves little.
        for ((name, _), ok) in DETECTORS.iter().zip(ok) {
            assert!(ok > 0 && ok < total, "{mode} {name}: {ok} of {total} bursts decoded");
        }
    }
}
