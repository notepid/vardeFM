//! The receiver's SNR estimate per mode and speed level, against a recorded baseline.
//!
//! The link picks its speed level by comparing the SNR the receiver reports on a
//! burst (`RxBurst::snr_db`), normalized to each level's drive by
//! `Modem::predict_snr_db`, against `SNR_THRESHOLDS_DB` in vardefm-link. The
//! decoding gates (`level_eval`) leave that estimate out, so a change that biases it
//! by a dB would pass them while the link picks levels that fail, or levels too low.
//!
//! Per mode and level, this measures at CNR points around the level's 10 % codeword
//! error point (`perf/levels.toml`), `SNR_OFFSETS_DB` from it:
//! - `snr_db`: the median SNR the receiver reports on the level's bursts, from the
//!   header and codewords demodulated with the sent header as training, as
//!   `Receiver::step` does (`level_eval::Case::frame`);
//! - `predicted_db`: the median of `Modem::predict_snr_db` from the SNR of a level-0
//!   burst over the same channel to the level, which covers the per-level drive
//!   normalisation. `snr_prediction_matches_measurement` holds it near `snr_db`.
//!
//! Medians, not means: at narrow level 0's point the mean over 312 bursts moved by
//! 0.27 dB between two samples, 3.4 times its standard error, so the per-burst
//! estimate is far from normal there. The median is less sensitive to its tails.
//!
//! What the gate does not cover:
//! - Bursts without codewords (acks, polls): `Receiver::step` measures their SNR on
//!   the header alone.
//! - Which bursts are measured: the gate measures every burst from its true position,
//!   while the receiver reports only bursts it detected and whose header decoded.
//!   Near level 0's point at narrow mode's level-0 drive the header fails more often
//!   than the codewords do, so there the link sees fewer and better bursts. Level 0's
//!   threshold is never used.
//! - Long bursts and clock drift, as in `level_eval`.
//!
//! `perf/snr.toml` holds the points. `snr_gate` measures `snr_db` and `predicted_db`
//! at `GATE_OFFSETS_DB` on fresh bursts and fails when either has moved from the
//! recorded value by more than `SNR_GATE_MARGIN_DB`. Each point is recorded and gated
//! over enough bursts that the noise of both medians stays inside that margin at 3
//! sigma (`bursts_for`): several hundred at the low levels, where the per-burst
//! estimate scatters by about a dB. The link's thresholds are checked
//! against the recorded points by `thresholds_lie_above_recorded_snr` in vardefm-link.
//!
//! `cargo test --release -p vardefm-modem snr_gate -- --ignored --nocapture`
//!
//! `snr_sweep` measures the points again; with `WRITE=1` it records them in
//! `perf/snr.toml`. The points sit at the CNRs of `perf/levels.toml`, so record that
//! first when both change. Commit the code first, so the recorded commit is the one
//! measured.
//!
//! `WRITE=1 cargo test --release -p vardefm-modem snr_sweep -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `LEVELS` (0,1,..,6), `TRIALS` (bursts per
//! point at least: 100 for the gate, 200 for the sweep), `WRITE` (sweep only).

use super::*;
use crate::detect_eval::{env_flag, env_list, env_modes, env_or, modem, on_every_core, through_channel};
use crate::level_eval::{self, trial_burst, Case, Target, ALL_MODES, GATE_BURSTS, GATE_FIRST_TRIAL};

/// CNRs the sweep records, relative to a level's 10 % codeword error point, dB. They
/// span the CNR at which the level reaches 1 % CER, where the link's thresholds sit.
const SNR_OFFSETS_DB: [f32; 4] = [-0.5, 0.0, 0.5, 1.0];
/// CNRs the gate measures, of `SNR_OFFSETS_DB`. They bracket every level's 1 % CER
/// point (+0.1 to +0.45 dB in the recorded grids).
const GATE_OFFSETS_DB: [f32; 2] = [0.0, 0.5];
/// How far a measured median may lie from the recorded one, dB. The same unit as the
/// link's thresholds; a quarter dB is the margin the decoding gate allows a CNR point.
const SNR_GATE_MARGIN_DB: f32 = 0.25;
/// How far `predicted_db` may lie from `snr_db` at a recorded point, dB. Without
/// this check a bias in `predict_snr_db` present when the points were recorded would
/// be recorded with them and never caught.
const PREDICT_TOLERANCE_DB: f32 = 0.5;
/// Bursts per point of the sweep at least. A point whose per-burst SNR scatters more
/// gets more (`bursts_for`).
const RECORD_BURSTS: u32 = 200;
/// Most bursts per point: a scatter that needs more is a change to look at.
const MAX_BURSTS: u32 = 2000;
/// The level whose SNR is normalized to every level for `predicted_db`.
const SOURCE_LEVEL: u8 = 0;
const ALL_LEVELS: &str = "0,1,2,3,4,5,6";

/// The recorded points, `perf/snr.toml`.
const SNR_BASELINE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/perf/snr.toml");

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct SnrBaseline {
    source: SnrSource,
    point: Vec<SnrPoint>,
}

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct SnrSource {
    what: String,
    commit: String,
    command: String,
    /// Bursts per point at least, `TRIALS`.
    min_bursts: u32,
    /// The commit `perf/levels.toml` names, whose CNR points these are measured at.
    levels_commit: String,
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct SnrPoint {
    mode: String,
    level: u8,
    name: String,
    /// The level's drive, `Modem::tx_rms`.
    drive: f32,
    /// The level's 10 % codeword error point in `perf/levels.toml`, dB.
    cnr_db: f32,
    /// Where the point is measured, relative to `cnr_db`, dB.
    offset_db: f32,
    /// Median SNR the receiver reports on the level's bursts, dB.
    snr_db: f32,
    /// Median SNR predicted for the level from a level-0 burst's, dB.
    predicted_db: f32,
    /// Scatter of one burst's SNR, dB: 1.4826 times the median absolute deviation,
    /// the standard deviation for a normal distribution.
    sigma_db: f32,
    /// ... and of one burst's predicted SNR, dB.
    predicted_sigma_db: f32,
    /// Bursts the medians are taken over.
    bursts: u32,
}

fn load() -> SnrBaseline {
    let text = std::fs::read_to_string(SNR_BASELINE).unwrap_or_else(|e| panic!("{SNR_BASELINE}: {e}"));
    toml::from_str(&text).unwrap_or_else(|e| panic!("{SNR_BASELINE}: {e}"))
}

impl SnrBaseline {
    fn point(&self, mode: &str, level: u8, offset: f32) -> &SnrPoint {
        let p = self.point.iter().find(|p| p.mode == mode && p.level == level && (p.offset_db - offset).abs() < 1e-4);
        p.unwrap_or_else(|| panic!("{SNR_BASELINE} has no {mode} L{level} at {offset:+} dB"))
    }
}

/// Medians and scatters over the bursts of one point.
struct Stats {
    snr_db: f32,
    predicted_db: f32,
    sigma_db: f32,
    predicted_sigma_db: f32,
}

/// Median of `x`, and its scatter: 1.4826 times the median absolute deviation.
fn median_sigma(x: &[f32]) -> (f32, f32) {
    let median = |v: &mut Vec<f32>| {
        v.sort_by(f32::total_cmp);
        let n = v.len();
        if n % 2 == 1 {
            v[n / 2]
        } else {
            0.5 * (v[n / 2 - 1] + v[n / 2])
        }
    };
    let m = median(&mut x.to_vec());
    let mad = median(&mut x.iter().map(|v| (v - m).abs()).collect());
    (m, 1.4826 * mad)
}

/// The SNR the receiver reports on trial `t` of `case` at `cnr` dB.
fn snr(case: &Case, cnr: f32, t: u32) -> f32 {
    let (_, frame) = case.frame(cnr, t);
    frame.unwrap_or_else(|| panic!("L{} at {cnr} dB, trial {t}: no frame at the true position", case.level)).snr_db
}

/// `Stats` of `bursts` trials from `first` at `cnr` dB: the SNR on `level`'s bursts,
/// and the SNR predicted for its level from `source`'s bursts. On every core.
fn measure(level: &Case, source: &Case, cnr: f32, first: u32, bursts: u32) -> Stats {
    let pairs = on_every_core(bursts, |t| {
        let s = snr(level, cnr, first + t);
        let from = if source.level == level.level { s } else { snr(source, cnr, first + t) };
        (s, level.m.predict_snr_db(from, source.level, level.level))
    });
    let (s, p): (Vec<f32>, Vec<f32>) = pairs.into_iter().unzip();
    let ((snr_db, sigma_db), (predicted_db, predicted_sigma_db)) = (median_sigma(&s), median_sigma(&p));
    Stats { snr_db, predicted_db, sigma_db, predicted_sigma_db }
}

/// Bursts for a median of a per-burst scatter `sigma` dB to stay within
/// `SNR_GATE_MARGIN_DB` of another median over as many bursts at 3 sigma. A median
/// over n bursts has a variance of about pi/2 sigma^2 / n, so the difference of two
/// has pi sigma^2 / n: `3 sigma sqrt(pi / n) <= margin`. The sweep records over this
/// many and the gate measures over as many. At least `min`.
fn bursts_for(sigma: f32, min: u32) -> u32 {
    let k = (3.0 * sigma / SNR_GATE_MARGIN_DB).powi(2);
    let n = ((std::f32::consts::PI * k).ceil() as u32).max(min);
    assert!(n <= MAX_BURSTS, "a per-burst SNR scatter of {sigma:.2} dB needs {n} bursts per point");
    n
}

/// Rounded to 0.001 dB, for the recorded file.
fn round(x: f32) -> f32 {
    (x * 1000.0).round() / 1000.0
}

/// Measures every mode, level and offset; with `WRITE=1`, records them in
/// `perf/snr.toml`. A point is measured over `TRIALS` bursts, then again over more
/// when its scatter needs more for the gate's margin (`bursts_for`).
#[test]
#[ignore]
fn snr_sweep() {
    let modes = env_modes(ALL_MODES);
    let levels: Vec<u8> = env_list("LEVELS", ALL_LEVELS);
    let bursts = env_or("TRIALS", RECORD_BURSTS);
    assert!(bursts > 0, "TRIALS must be at least 1");
    let write = env_flag("WRITE");
    let commit = level_eval::commit();
    assert!(
        !write || (std::env::var("MODES").is_err() && std::env::var("LEVELS").is_err()),
        "WRITE=1 records every mode and level: unset MODES and LEVELS"
    );
    let base = level_eval::load();
    let mut points = Vec::new();
    for mode in &modes {
        let m = modem(mode);
        let source = Case::new(&m, mode, Target::Level(SOURCE_LEVEL));
        for &l in &levels {
            let target = Target::Level(l);
            let at = base.point(mode, target);
            let case = Case::new(&m, mode, target);
            for offset in SNR_OFFSETS_DB {
                let cnr = at.cnr_db + offset;
                let mut n = bursts;
                let mut s = measure(&case, &source, cnr, 0, n);
                let need = bursts_for(s.sigma_db.max(s.predicted_sigma_db), bursts);
                if need > n {
                    n = need;
                    s = measure(&case, &source, cnr, 0, n);
                }
                println!(
                    "{} at {:.2}{offset:+.2} dB: SNR {:.2} dB (sigma {:.2}), predicted from L{SOURCE_LEVEL} {:.2} dB \
                     (sigma {:.2}), {:+.2} dB; {n} bursts",
                    target.label(mode),
                    at.cnr_db,
                    s.snr_db,
                    s.sigma_db,
                    s.predicted_db,
                    s.predicted_sigma_db,
                    s.predicted_db - s.snr_db,
                );
                points.push(SnrPoint {
                    mode: mode.clone(),
                    level: l,
                    name: target.name().into(),
                    drive: m.tx_rms(l),
                    cnr_db: at.cnr_db,
                    offset_db: offset,
                    snr_db: round(s.snr_db),
                    predicted_db: round(s.predicted_db),
                    sigma_db: round(s.sigma_db),
                    predicted_sigma_db: round(s.predicted_sigma_db),
                    bursts: n,
                });
            }
        }
    }
    if write {
        let b = SnrBaseline {
            source: SnrSource {
                what: "Median SNR the receiver reports, around each level's 10 % CER point, from the known position"
                    .into(),
                commit,
                command: format!(
                    "WRITE=1 TRIALS={bursts} cargo test --release -p vardefm-modem snr_sweep -- --ignored"
                ),
                min_bursts: bursts,
                levels_commit: base.source.commit.clone(),
            },
            point: points,
        };
        std::fs::write(SNR_BASELINE, toml::to_string(&b).unwrap()).unwrap();
        println!("wrote {SNR_BASELINE}");
    }
}

/// Fails when the median SNR the receiver reports, or the SNR predicted from a level-0
/// burst, has moved from `perf/snr.toml` by more than `SNR_GATE_MARGIN_DB`. Each point
/// is measured over as many fresh bursts as it was recorded over, or `TRIALS` if more.
#[test]
#[ignore]
fn snr_gate() {
    let modes = env_modes(ALL_MODES);
    let levels: Vec<u8> = env_list("LEVELS", ALL_LEVELS);
    let bursts = env_or("TRIALS", GATE_BURSTS);
    assert!(bursts > 0, "TRIALS must be at least 1");
    let base = load();
    println!("baseline {} ({})", base.source.commit, base.source.command);
    let mut failed = Vec::new();
    for mode in &modes {
        let m = modem(mode);
        let source = Case::new(&m, mode, Target::Level(SOURCE_LEVEL));
        for &l in &levels {
            let target = Target::Level(l);
            let case = Case::new(&m, mode, target);
            for offset in GATE_OFFSETS_DB {
                let p = base.point(mode, l, offset);
                let s = measure(&case, &source, p.cnr_db + offset, GATE_FIRST_TRIAL, p.bursts.max(bursts));
                let ok = within(p.snr_db, s.snr_db) && within(p.predicted_db, s.predicted_db);
                println!(
                    "{} at {:.2}{offset:+.2} dB: SNR {:.2} dB (recorded {:.2}), predicted {:.2} dB (recorded {:.2}) {}",
                    target.label(mode),
                    p.cnr_db,
                    s.snr_db,
                    p.snr_db,
                    s.predicted_db,
                    p.predicted_db,
                    if ok { "ok" } else { "FAIL" }
                );
                if !ok {
                    failed.push(format!("{} {offset:+}", target.label(mode)));
                }
            }
        }
    }
    assert!(failed.is_empty(), "SNR estimate moved for {failed:?}");
}

/// The gate's verdict: whether `measured` lies within `SNR_GATE_MARGIN_DB` of
/// `recorded`.
fn within(recorded: f32, measured: f32) -> bool {
    (measured - recorded).abs() <= SNR_GATE_MARGIN_DB
}

#[test]
fn snr_gate_fails_a_shifted_point() {
    for shift in [-0.5 * SNR_GATE_MARGIN_DB, 0.0, 0.5 * SNR_GATE_MARGIN_DB] {
        assert!(within(5.0, 5.0 + shift), "{shift}");
    }
    for shift in [-2.0 * SNR_GATE_MARGIN_DB, 2.0 * SNR_GATE_MARGIN_DB] {
        assert!(!within(5.0, 5.0 + shift), "{shift}");
    }
}

/// The measurement path reports what `Receiver` reports on the same audio, so the
/// gate covers the SNR the link sees.
#[test]
fn measurement_matches_the_receiver() {
    for mode in ALL_MODES.split(',') {
        let m = modem(mode);
        for l in [0, LEVELS.len() as u8 - 1] {
            let case = Case::new(&m, mode, Target::Level(l));
            let cnr = 30.0;
            for t in 0..2 {
                let sent = trial_burst(l, case.ncw, t);
                let audio = through_channel(&m, mode, cnr, &sent, t);
                let mut rx = Receiver::new(m.clone());
                let mut got: Vec<RxBurst> = audio.chunks(480).flat_map(|c| rx.push(c)).collect();
                got.extend(rx.push(&vec![0.0; 48_000]));
                assert_eq!(got.len(), 1, "{mode} L{l}: receiver got {} bursts", got.len());
                let want = snr(&case, cnr, t);
                let d = got[0].snr_db - want;
                assert!(
                    d.abs() < MATCH_TOLERANCE_DB,
                    "{mode} L{l} trial {t}: receiver {:.2} dB, gate {want:.2} dB",
                    got[0].snr_db
                );
            }
        }
    }
}

/// How far the receiver's SNR may lie from the gate's on the same audio, dB. The
/// receiver finds the burst itself, a few samples from the true position.
const MATCH_TOLERANCE_DB: f32 = 0.1;

/// The committed points cover every mode, level and offset, at the CNRs and drives
/// the levels baseline and the modem have now.
#[test]
fn snr_baseline_covers_every_mode_and_level() {
    let base = load();
    let levels = level_eval::load();
    let c = &base.source.commit;
    assert!(c != "unknown" && !c.ends_with("-dirty"), "{SNR_BASELINE} names no clean commit ({c:?}): record it again");
    assert_eq!(
        base.source.levels_commit, levels.source.commit,
        "{SNR_BASELINE} was measured at the CNRs of another perf/levels.toml: record it again"
    );
    let mut n = 0;
    for mode in ALL_MODES.split(',') {
        let m = modem(mode);
        for l in 0..LEVELS.len() as u8 {
            let at = levels.point(mode, Target::Level(l));
            for offset in SNR_OFFSETS_DB {
                let p = base.point(mode, l, offset);
                let label = format!("{SNR_BASELINE}: {mode} L{l} at {offset:+} dB");
                assert_eq!(p.name, LEVELS[l as usize].name, "{label} is named {:?}", p.name);
                assert_eq!(p.drive, m.tx_rms(l), "{label} was recorded at another drive: record it again");
                assert_eq!(p.cnr_db, at.cnr_db, "{label} was recorded at another CNR: record it again");
                n += 1;
            }
        }
    }
    assert_eq!(base.point.len(), n, "{SNR_BASELINE}: points for unknown modes, levels or offsets");
}

/// At every recorded point, the SNR predicted from a level-0 burst lies within
/// `PREDICT_TOLERANCE_DB` of the SNR measured on the level's own bursts.
#[test]
fn snr_prediction_matches_measurement() {
    let base = load();
    let bad: Vec<String> = base
        .point
        .iter()
        .filter(|p| (p.predicted_db - p.snr_db).abs() > PREDICT_TOLERANCE_DB)
        .map(|p| {
            format!(
                "{} L{} at {:+} dB: predicted {:.2}, measured {:.2}",
                p.mode, p.level, p.offset_db, p.predicted_db, p.snr_db
            )
        })
        .collect();
    assert!(bad.is_empty(), "predict_snr_db is off by more than {PREDICT_TOLERANCE_DB} dB:\n  {}", bad.join("\n  "));
}

#[test]
fn bursts_cover_both_medians() {
    // Three sigma of the difference of two medians over n bursts each fits the margin.
    for sigma in [0.2, 0.5, 1.0, 1.3] {
        let n = bursts_for(sigma, 1) as f32;
        assert!(3.0 * sigma * (std::f32::consts::PI / n).sqrt() <= SNR_GATE_MARGIN_DB, "{sigma}: {n}");
    }
    assert_eq!(bursts_for(0.1, 200), 200);
}

#[test]
fn median_sigma_ignores_outliers() {
    let (m, s) = median_sigma(&[1.0, 2.0, 3.0, 4.0, -40.0]);
    assert_eq!(m, 2.0);
    assert!((s - 1.4826).abs() < 1e-4, "{s}");
    assert_eq!(median_sigma(&[1.0, 3.0]).0, 2.0);
}

#[test]
fn gate_offsets_are_recorded() {
    assert!(GATE_OFFSETS_DB.iter().all(|o| SNR_OFFSETS_DB.contains(o)));
}
