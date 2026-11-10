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
//! `cargo test --release -p vardefm-modem detector_eval -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `TRIALS` per CNR (100), `LEVEL` (1), `NCW` (1),
//! `CNRS` (dB list, default per mode), `NOISE_SECONDS` per noise case (60),
//! `CSV` (file to append the burst results to).

use super::*;
use std::collections::HashMap;
use std::f32::consts::PI;
use std::io::Write;
use std::sync::Arc;
use vardefm_dsp::prng::Prng;
use vardefm_sim::{transmit, FmChannelConfig};

pub(crate) use crate::receiver::DetectFn;

/// Detectors under evaluation. The first one is the reference for the paired tests.
pub(crate) const DETECTORS: &[(&str, DetectFn)] = &[("sc", |m, rx, from| m.ofdm.detect(rx, from))];

/// Leader length used by the evaluation, seconds (the `vardefm modem` default).
const TXDELAY: f32 = 0.15;

fn modem(mode: &str) -> Arc<Modem> {
    let mut m = Modem::new(OfdmParams::by_name(mode).unwrap(), TXDELAY);
    // Trials already run on every core.
    m.decode_threads = 1;
    Arc::new(m)
}

fn burst(level: u8, ncw: usize, seed: u32) -> Burst {
    let mut rng = Prng::new(seed);
    let n = Modem::payload_bytes(level);
    Burst {
        header: Header { kind: Kind::Data, session: 0x42, level, ncw: ncw as u8, ..Default::default() },
        payload: (0..ncw).map(|_| (0..n).map(|_| rng.next_u32() as u8).collect()).collect(),
    }
}

/// The header of the burst whose preamble B starts at `b`, if it decodes.
fn header_at(m: &Modem, rx: &[f32], b: usize) -> Option<Header> {
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
    let first = f(m, rx, 0);
    let mut next = first;
    while let Some(b) = next {
        if let Some(ok) = decode_at(m, rx, b, sent, cache) {
            return Outcome { first, ok };
        }
        // As `Receiver::step`: search on from the failed detection itself. That
        // needs a detector that reports a position past the start of what it
        // matched; one that returns `b` again would hang the evaluation, so stop.
        next = f(m, rx, b);
        if let Some(n) = next {
            assert!(n > b, "detector returned {n} when searching from {b}: it would loop forever");
        }
    }
    Outcome { first, ok: false }
}

/// Outcome of one trial: (hit, burst) per detector, then the genie's burst.
struct Trial {
    det: Vec<(bool, bool)>,
    genie: bool,
}

/// Payload seed of trial `t`: every trial sends its own data, the same at every CNR.
fn payload_seed(t: u32) -> u32 {
    1 + t
}

/// Channel seed of trial `t` at `cnr` dB: every trial and every CNR point gets its
/// own noise, so the points of a curve do not share noise realisations. All
/// detectors still see the same recording within a trial, which keeps them paired.
fn channel_seed(cnr: f32, t: u32) -> u64 {
    (((cnr * 100.0).round() as i64 as u64) << 32) | t as u64
}

fn trial(m: &Modem, mode: &str, cnr: f32, level: u8, ncw: usize, truth: usize, t: u32) -> Trial {
    let sent = burst(level, ncw, payload_seed(t));
    let cfg =
        FmChannelConfig { cnr_db: Some(cnr), seed: channel_seed(cnr, t), ..FmChannelConfig::by_name(mode).unwrap() };
    let rx = transmit(&cfg, &m.modulate(&sent));
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
fn true_position(m: &Modem, mode: &str, level: u8, ncw: usize) -> usize {
    let clean = FmChannelConfig { cnr_db: None, ..FmChannelConfig::by_name(mode).unwrap() };
    let tx = m.modulate(&burst(level, ncw, payload_seed(0)));
    m.ofdm.detect(&transmit(&clean, &tx), 0).expect("burst not found on a clean channel")
}

/// Where the body of preamble B starts in the channel's output by the burst layout
/// alone: lead silence, leader, preamble A and preamble B's cyclic prefix.
fn layout_position(m: &Modem, mode: &str) -> usize {
    let cfg = FmChannelConfig::by_name(mode).unwrap();
    let p = &m.ofdm.p;
    (cfg.lead_silence_s * cfg.fs) as usize + (m.leader_symbols() + 1) * p.symbol_len() + p.ncp
}

/// The channel's delay, samples: the lag up to one cyclic prefix at which a noiseless
/// copy of the burst correlates best with what was sent. Independent of any detector.
fn channel_delay(m: &Modem, mode: &str, level: u8, ncw: usize) -> usize {
    let clean = FmChannelConfig { cnr_db: None, ..FmChannelConfig::by_name(mode).unwrap() };
    let tx = m.modulate(&burst(level, ncw, payload_seed(0)));
    let rx = transmit(&clean, &tx);
    let rx = &rx[(clean.lead_silence_s * clean.fs) as usize..];
    let corr = |d: usize| -> f64 { tx.iter().zip(&rx[d..]).map(|(&a, &b)| a as f64 * b as f64).sum() };
    (0..=m.ofdm.p.ncp).max_by(|&a, &b| corr(a).total_cmp(&corr(b))).unwrap()
}

/// Run `trials` trials of a `level` burst of `ncw` codewords on every core, with
/// preamble B at `truth` ([`true_position`]).
fn run_trials(m: &Modem, mode: &str, cnr: f32, level: u8, ncw: usize, truth: usize, trials: u32) -> Vec<Trial> {
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()) as u32;
    let mut out: Vec<(u32, Trial)> = std::thread::scope(|s| {
        let workers: Vec<_> = (0..threads)
            .map(|th| {
                s.spawn(move || {
                    (th..trials)
                        .step_by(threads as usize)
                        .map(|t| (t, trial(m, mode, cnr, level, ncw, truth, t)))
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        workers.into_iter().flat_map(|w| w.join().unwrap()).collect()
    });
    out.sort_by_key(|(t, _)| *t);
    out.into_iter().map(|(_, r)| r).collect()
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
fn env_or<T: std::str::FromStr>(name: &str, default: T) -> T {
    match std::env::var(name) {
        Ok(v) => v.trim().parse().unwrap_or_else(|_| panic!("{name}={v:?} does not parse")),
        Err(std::env::VarError::NotPresent) => default,
        Err(std::env::VarError::NotUnicode(v)) => panic!("{name}={v:?} is not Unicode"),
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
            (0..n).map(|_| (rng.next_u32() as f32 / u32::MAX as f32 - 0.5) * 0.6).collect()
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

/// Detections and decoded headers in `rx`, scanning on after each detection.
///
/// It searches on from each detection itself, as `receive` and `Receiver::step` do
/// after a header that fails, so a detector is scored here on the searches it makes
/// in the receiver. After a header that decodes the receiver skips the frame it
/// announces; this pass searches on from the detection instead, so that every
/// detection in the audio is counted.
fn false_detections(m: &Modem, f: DetectFn, rx: &[f32]) -> (u32, u32) {
    let (mut dets, mut headers) = (0, 0);
    let mut next = f(m, rx, 0);
    while let Some(b) = next {
        dets += 1;
        if header_at(m, rx, b).is_some() {
            headers += 1;
        }
        next = f(m, rx, b);
        if let Some(n) = next {
            assert!(n > b, "detector returned {n} when searching from {b}: it would loop forever");
        }
    }
    (dets, headers)
}

#[test]
#[ignore]
fn detector_eval() {
    let modes: String = env_or("MODES", "narrow,wide".to_string());
    let trials: u32 = env_or("TRIALS", 100);
    let level: u8 = env_or("LEVEL", 1);
    let ncw: usize = env_or("NCW", 1);
    let noise_s: f32 = env_or("NOISE_SECONDS", 60.0);
    assert!((level as usize) < LEVELS.len(), "LEVEL {level}: levels are 0 to {}", LEVELS.len() - 1);
    assert!(trials > 0, "TRIALS must be at least 1");
    assert!(noise_s > 0.0 && noise_s.is_finite(), "NOISE_SECONDS={noise_s}: must be a finite number more than 0");
    // Checked up front, so that a typo does not fail after the first mode has run.
    let modes: Vec<&str> = modes.split(',').map(str::trim).collect();
    for mode in &modes {
        assert!(OfdmParams::by_name(mode).is_some(), "MODES: unknown mode {mode:?} (narrow or wide)");
        // A longer burst would modulate, but its header would not unpack (MAX_NCW) or
        // the modem would never send it (MAX_PAYLOAD_SECONDS): every trial would fail.
        let max = modem(mode).max_ncw(level);
        assert!(ncw <= max, "NCW {ncw}: a level {level} burst in {mode} mode holds at most {max} codewords");
    }
    let cnr_list = |mode: &str| -> Vec<f32> {
        let default = if mode == "narrow" { "3,4,5,6,7,8,10" } else { "5,6,7,8,9,10,12" };
        let cnrs: String = env_or("CNRS", default.to_string());
        let cnr = |c: &str| -> f32 {
            let v: f32 = c.trim().parse().unwrap_or_else(|_| panic!("CNRS: {c:?} is not a number"));
            // NaN or infinite noise gives an all-NaN recording, which would be written as data.
            assert!(v.is_finite(), "CNRS: {c:?} is not a finite number of dB");
            v
        };
        cnrs.split(',').map(cnr).collect()
    };
    for mode in &modes {
        cnr_list(mode);
    }
    // Any path the OS accepts, Unicode or not.
    let mut csv = std::env::var_os("CSV").map(|p| {
        let mut f = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&p)
            .unwrap_or_else(|e| panic!("CSV: cannot open {p:?}: {e}"));
        // An empty file (from mktemp, say) needs the header as much as a new one.
        if f.metadata().unwrap_or_else(|e| panic!("CSV: cannot read {p:?}: {e}")).len() == 0 {
            writeln!(f, "mode,level,ncw,cnr_db,trials,detector,hits,bursts").unwrap();
        }
        f
    });
    let names: Vec<&str> = DETECTORS.iter().map(|(n, _)| *n).collect();
    println!(
        "level {level} ({}), {ncw} codeword(s), {trials} trials per point, txdelay {TXDELAY} s",
        LEVELS[level as usize].name
    );
    for &mode in &modes {
        let m = modem(mode);
        println!("\n{mode}: hits and decoded bursts out of {trials}; paired test against '{}'", names[0]);
        // The true position is the reference's own; show where it lies against the channel.
        let truth = true_position(&m, mode, level, ncw);
        let after = truth as i64 - layout_position(&m, mode) as i64;
        let delay = channel_delay(&m, mode, level, ncw) as i64;
        println!(
            "  true position: sample {truth}, {after} after the sent boundary; channel delay {delay}, \
             so {} after the boundary plus the delay; hit within ±{}",
            after - delay,
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

/// The harness itself: on a strong signal every detector and the genie decode every
/// burst, and no detector raises a false header on a carrier at -10 dB CNR. `hit` is
/// a diagnostic, so it is not required here.
#[test]
fn harness_sees_strong_bursts_and_no_false_headers() {
    for mode in ["narrow", "wide"] {
        let m = modem(mode);
        // The true position must not depend on the payload, or one copy could not give it.
        let truth = true_position(&m, mode, 1, 1);
        let clean = FmChannelConfig { cnr_db: None, ..FmChannelConfig::by_name(mode).unwrap() };
        for t in 1..4 {
            let tx = m.modulate(&burst(1, 1, payload_seed(t)));
            assert_eq!(m.ofdm.detect(&transmit(&clean, &tx), 0), Some(truth), "{mode}: payload {t} moves the burst");
        }
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
                for (d, (name, f)) in DETECTORS.iter().enumerate() {
                    // Offline first: its progress check stops a detector that does not
                    // move forward, which would hang the streaming receiver.
                    let offline = receive(&m, *f, &rx, &sent, &mut Decodes::new()).ok;
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
