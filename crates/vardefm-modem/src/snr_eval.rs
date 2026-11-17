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
//! - per `Source`, the SNR `Modem::predict_snr_db` gives for the level from a level-0
//!   burst over the same channel: its median, and its 95th and 99th percentiles. The
//!   link sends level-0 bursts of every `Source` kind: a data burst at level 0, a
//!   connect request (one codeword; it sets the starting level) and every
//!   acknowledgement and poll (header only), and acts on the SNR of each one alone.
//!   A burst whose prediction lies high asks the peer for a level that fails, so the
//!   upper percentiles matter as much as the median. Only predictions from level 0 are
//!   measured; the link also predicts from other levels.
//!   `snr_prediction_matches_measurement` holds the data burst's median near `snr_db`.
//!
//! Medians, not means: at narrow level 0's point the mean over 312 bursts moved by
//! 0.27 dB between two samples, 3.4 times its standard error, so the per-burst
//! estimate is far from normal there. The median is less sensitive to its tails.
//!
//! What the gate does not cover:
//! - How often the link asks for a level that fails: it acts on single bursts without
//!   smoothing, and a header-only burst's prediction can reach the target level's
//!   whole ceiling (`predict_snr_db` clamps when the source reads near its own). The
//!   gate holds the percentiles where they are; it does not judge them.
//! - Which bursts are measured: the gate measures every burst from its true position,
//!   while the receiver reports only bursts it detected and whose header decoded. At
//!   the low levels' points mostly detection misses (narrow level 1: 116 of 200
//!   detected, 197 headers decoded from the true position); the median moves by about
//!   0.05 dB. Level 0's threshold is never used.
//! - Samples beyond full scale: `Receiver::push` replaces them with the last good
//!   sample, the measurement does not. `measurement_matches_the_receiver` compares
//!   medians for this reason; single bursts at the lowest levels differ by up to a
//!   few dB.
//! - Long bursts and clock drift, as in `level_eval`.
//!
//! `perf/snr.toml` holds the points. `snr_gate` measures them at `GATE_OFFSETS_DB` on
//! fresh bursts and fails when a median has moved from the recorded value by more
//! than `SNR_GATE_MARGIN_DB`, over enough bursts that the noise of both medians would
//! stay inside that margin at 3 sigma if the per-burst estimate were normal
//! (`bursts_for`); that is an estimate, not a bound.
//!
//! The connect request's and the header-only burst's predictions are gated in share,
//! not in dB. The sweep records, over `TAIL_BURSTS` bursts, the median and 95th
//! percentile and the share of bursts at or above each; the gate measures that share
//! on as many fresh bursts and fails when it lies more than 3 sigma of the
//! difference of two binomial shares from the recorded one (`share_tolerance`). Values
//! in dB are no measure where the prediction is clamped: at wide levels 5 and 6 the
//! header-only bursts' prediction splits into a cluster at the level's ceiling and the
//! rest, so its percentiles jump between the two and its scatter asks for more bursts
//! than `MAX_BURSTS`. At wide level 6 more than 5 % sit at the ceiling, so the 95th
//! percentile is the ceiling itself and its recorded share is that cluster's. A share
//! is binomial whatever the distribution, so its noise is known. The data burst's
//! percentiles come from the median's bursts and are recorded only; so are all 99th
//! percentiles.
//!
//! The link's thresholds are checked against the recorded points by
//! `thresholds_lie_above_recorded_snr` in vardefm-link.
//!
//! `cargo test --release -p vardefm-modem snr_gate -- --ignored --nocapture`
//!
//! `snr_sweep` measures the points again; with `WRITE=1` it records them in
//! `perf/snr.toml`. The points sit at the CNRs of `perf/levels.toml`, so a new
//! `levels.toml` needs a new `snr.toml`, and `snr_baseline_covers_every_mode_and_level`
//! fails until it has one. Commit the code first, so the recorded commit is the one
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
/// The percentile whose share is gated, and the one only recorded.
const GATE_QUANTILE: f32 = 0.95;
const RECORD_QUANTILE: f32 = 0.99;
/// Bursts of each share, recorded and gated: header-only and one-codeword bursts are
/// cheap. At 5 % the share's tolerance is then 2.9 %, at 50 % 6.7 %; where the
/// per-burst scatter is a dB, a shift of a quarter dB moves the median's share by about
/// 10 %.
const TAIL_BURSTS: u32 = 1000;
/// How far the data burst's predicted median may lie from `snr_db` at a recorded
/// point, dB. Without this check a bias in `predict_snr_db` present when the points
/// were recorded would be recorded with them and never caught. The recorded
/// prediction runs up to about 0.45 dB low at wide levels 3 to 5 (the model's
/// clipping ceilings there); this leaves room for a re-recording's noise and fails a
/// bias half a dB larger.
const PREDICT_TOLERANCE_DB: f32 = 0.75;
/// Bursts per point of the sweep at least. A point whose per-burst SNR scatters more
/// gets more (`bursts_for`).
const RECORD_BURSTS: u32 = 200;
/// Most bursts per point that a scatter may ask for: one that needs more is a change
/// to look at. `TRIALS` may ask for more.
const MAX_BURSTS: u32 = 2000;
/// The level of the bursts whose SNR is normalized to every level.
const SOURCE_LEVEL: u8 = 0;
const ALL_LEVELS: &str = "0,1,2,3,4,5,6";

/// The recorded points, `perf/snr.toml`.
const SNR_BASELINE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/perf/snr.toml");

/// The level-0 bursts the link sends, whose SNR it normalizes to other levels.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Source {
    /// A data burst of up to `level_eval::CODEWORDS` codewords.
    Data,
    /// One codeword: a connect request, which sets the starting level.
    Short,
    /// Header only: an acknowledgement or poll.
    Header,
}

const SOURCES: [Source; 3] = [Source::Data, Source::Short, Source::Header];

impl Source {
    fn case<'a>(self, m: &'a Modem, mode: &'a str) -> Case<'a> {
        match self {
            Source::Data => Case::new(m, mode, Target::Level(SOURCE_LEVEL)),
            Source::Short => Case::with_ncw(m, mode, SOURCE_LEVEL, 1),
            Source::Header => Case::with_ncw(m, mode, SOURCE_LEVEL, 0),
        }
    }

    fn name(self) -> &'static str {
        match self {
            Source::Data => "data",
            Source::Short => "1 codeword",
            Source::Header => "header only",
        }
    }

    fn of(self, p: &SnrPoint) -> &Predicted {
        match self {
            Source::Data => &p.from_data,
            Source::Short => &p.from_short,
            Source::Header => &p.from_header,
        }
    }
}

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
    /// Scatter of one burst's SNR, dB: 1.4826 times the median absolute deviation,
    /// the standard deviation for a normal distribution.
    sigma_db: f32,
    /// Bursts the values are taken over.
    bursts: u32,
    /// The SNR predicted for the level from each `Source`.
    from_data: Predicted,
    from_short: Predicted,
    from_header: Predicted,
}

/// The SNR predicted for a level from the bursts of one `Source`, dB.
#[derive(serde::Serialize, serde::Deserialize, Clone, PartialEq, Debug)]
#[serde(deny_unknown_fields)]
struct Predicted {
    median_db: f32,
    /// `GATE_QUANTILE`; for the data burst over the median's bursts, for the others
    /// over `TAIL_BURSTS`.
    q95_db: f32,
    /// `RECORD_QUANTILE`.
    q99_db: f32,
    /// As `SnrPoint::sigma_db`.
    sigma_db: f32,
    /// Share of the bursts at or above `median_db` and `q95_db`; for the data burst
    /// over the median's bursts, for the others over `TAIL_BURSTS`.
    median_share: f32,
    q95_share: f32,
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

/// Median of `x`.
fn median(x: &[f32]) -> f32 {
    let mut v = x.to_vec();
    v.sort_by(f32::total_cmp);
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        0.5 * (v[n / 2 - 1] + v[n / 2])
    }
}

/// Median of `x`, and its scatter: 1.4826 times the median absolute deviation.
fn median_sigma(x: &[f32]) -> (f32, f32) {
    let m = median(x);
    let mad = median(&x.iter().map(|v| (v - m).abs()).collect::<Vec<_>>());
    (m, 1.4826 * mad)
}

/// The `q` quantile of `x` by nearest rank: the smallest value with at least a `q`
/// share of `x` at or below it.
fn quantile(x: &[f32], q: f32) -> f32 {
    let mut v = x.to_vec();
    v.sort_by(f32::total_cmp);
    let rank = (q * v.len() as f32).ceil() as usize;
    v[rank.clamp(1, v.len()) - 1]
}

/// The SNR the receiver reports on trial `t` of `case` at `cnr` dB.
fn snr(case: &Case, cnr: f32, t: u32) -> f32 {
    let (_, frame) = case.frame(cnr, t);
    frame.unwrap_or_else(|| panic!("L{} at {cnr} dB, trial {t}: no frame at the true position", case.level)).snr_db
}

/// Per-burst values of one point: the SNR on the level's bursts, and the SNR
/// predicted for its level from the level-0 data burst's.
struct Samples {
    snr: Vec<f32>,
    data: Vec<f32>,
}

/// `bursts` trials from `first` at `cnr` dB. On every core.
fn sample(level: &Case, data: &Case, cnr: f32, first: u32, bursts: u32) -> Samples {
    let rows = on_every_core(bursts, |t| {
        let s = snr(level, cnr, first + t);
        let from = if (data.level, data.ncw) == (level.level, level.ncw) { s } else { snr(data, cnr, first + t) };
        (s, level.m.predict_snr_db(from, data.level, level.level))
    });
    let (snr, data) = rows.into_iter().unzip();
    Samples { snr, data }
}

/// Median, scatter and percentiles of predictions `x`.
fn predicted(x: &[f32]) -> Predicted {
    let (median_db, sigma_db) = median_sigma(x);
    let q95_db = quantile(x, GATE_QUANTILE);
    Predicted {
        median_db,
        q95_db,
        q99_db: quantile(x, RECORD_QUANTILE),
        sigma_db,
        median_share: share_at_or_above(x, median_db),
        q95_share: share_at_or_above(x, q95_db),
    }
}

impl Samples {
    /// Bursts these samples ask for: enough for both medians to fit their margin
    /// (`bursts_for`).
    fn need(&self, min: u32) -> u32 {
        bursts_for(median_sigma(&self.snr).1.max(median_sigma(&self.data).1), min)
    }

    /// The predictions from every source in `SOURCES` order, the tailed ones from
    /// `tails`.
    fn all(&self, tails: &[Vec<f32>; 2]) -> [Predicted; 3] {
        [predicted(&self.data), predicted(&tails[0]), predicted(&tails[1])]
    }
}

/// The SNR predicted for `level`'s level from the tailed sources (`SOURCES[1..]`), over
/// `bursts` trials from `first` at `cnr` dB. On every core.
fn sample_tails(level: &Case, sources: &[Case; 3], cnr: f32, first: u32, bursts: u32) -> [Vec<f32>; 2] {
    let rows = on_every_core(bursts, |t| {
        [&sources[1], &sources[2]].map(|src| level.m.predict_snr_db(snr(src, cnr, first + t), src.level, level.level))
    });
    let mut out: [Vec<f32>; 2] = Default::default();
    for r in rows {
        for (v, x) in out.iter_mut().zip(r) {
            v.push(x);
        }
    }
    out
}

/// Share of `x` at or above `limit`.
fn share_at_or_above(x: &[f32], limit: f32) -> f32 {
    x.iter().filter(|&&v| v >= limit).count() as f32 / x.len() as f32
}

/// How far a share measured over `TAIL_BURSTS` fresh bursts may lie from `recorded`,
/// itself measured over as many: 3 sigma of the difference of two binomial shares.
fn share_tolerance(recorded: f32) -> f32 {
    3.0 * (2.0 * recorded * (1.0 - recorded) / TAIL_BURSTS as f32).sqrt()
}

/// Bursts for a median of a per-burst scatter `sigma` dB to stay within
/// `SNR_GATE_MARGIN_DB` of another median over as many bursts at 3 sigma. For a
/// normal distribution a median over n bursts has a variance of about
/// pi/2 sigma^2 / n, so the difference of two has pi sigma^2 / n:
/// `3 sigma sqrt(pi / n) <= margin`. Away from normal the variance is
/// 1 / (4 n f(m)^2), with f the density at the median, so this is an estimate. At
/// least `min`, which may exceed `MAX_BURSTS`.
fn bursts_for(sigma: f32, min: u32) -> u32 {
    let k = (3.0 * sigma / SNR_GATE_MARGIN_DB).powi(2);
    let n = (std::f32::consts::PI * k).ceil() as u32;
    assert!(n <= MAX_BURSTS, "a per-burst SNR scatter of {sigma:.2} dB needs {n} bursts per point");
    n.max(min)
}

/// Rounded to 0.001 dB, for the recorded file.
fn round(x: f32) -> f32 {
    (x * 1000.0).round() / 1000.0
}

impl Predicted {
    fn rounded(&self) -> Self {
        Predicted {
            median_db: round(self.median_db),
            q95_db: round(self.q95_db),
            q99_db: round(self.q99_db),
            sigma_db: round(self.sigma_db),
            median_share: round(self.median_share),
            q95_share: round(self.q95_share),
        }
    }
}

/// The sources' cases in `SOURCES` order.
fn source_cases<'a>(m: &'a Modem, mode: &'a str) -> [Case<'a>; 3] {
    SOURCES.map(|s| s.case(m, mode))
}

/// Measures every mode, level and offset; with `WRITE=1`, records them in
/// `perf/snr.toml`. A point is measured over `TRIALS` bursts, then again over more
/// while its scatter needs more for the gate's margin (`Samples::need`); the tails over
/// `TAIL_BURSTS`.
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
        let sources = source_cases(&m, mode);
        for &l in &levels {
            let target = Target::Level(l);
            let at = base.point(mode, target);
            let case = Case::new(&m, mode, target);
            for offset in SNR_OFFSETS_DB {
                let cnr = at.cnr_db + offset;
                // The scatter measured over more bursts can need more still: measure
                // again until the count it asks for has been measured.
                let mut n = bursts;
                let mut s = sample(&case, &sources[0], cnr, 0, n);
                loop {
                    let need = s.need(bursts);
                    if need <= n {
                        break;
                    }
                    n = need;
                    s = sample(&case, &sources[0], cnr, 0, n);
                }
                // The gate's bursts must be fresh, not a replay of these.
                assert!(n <= GATE_FIRST_TRIAL, "{n} bursts reach the gate's first trial");
                let (snr_db, sigma_db) = median_sigma(&s.snr);
                let p = s.all(&sample_tails(&case, &sources, cnr, 0, TAIL_BURSTS));
                let tails: String = SOURCES
                    .iter()
                    .zip(&p)
                    .map(|(src, p)| format!("; from {} {:.2}/{:.2}/{:.2}", src.name(), p.median_db, p.q95_db, p.q99_db))
                    .collect();
                println!(
                    "{} at {:.2}{offset:+.2} dB: SNR {snr_db:.2} dB (sigma {sigma_db:.2}){tails} (median/q95/q99); \
                     {n} bursts",
                    target.label(mode),
                    at.cnr_db,
                );
                let [from_data, from_short, from_header] = p.map(|p| p.rounded());
                points.push(SnrPoint {
                    mode: mode.clone(),
                    level: l,
                    name: target.name().into(),
                    drive: m.tx_rms(l),
                    cnr_db: at.cnr_db,
                    offset_db: offset,
                    snr_db: round(snr_db),
                    sigma_db: round(sigma_db),
                    bursts: n,
                    from_data,
                    from_short,
                    from_header,
                });
            }
        }
    }
    if write {
        let b = SnrBaseline {
            source: SnrSource {
                what: "SNR the receiver reports, and predicts from level-0 bursts, around each level's 10 % CER point"
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

/// Fails when the receiver's median SNR, or the median predicted from a level-0 data
/// burst, has moved from `perf/snr.toml` by more than `SNR_GATE_MARGIN_DB`; or when the
/// share of fresh connect requests or header-only bursts predicting at or above a
/// recorded median or 95th percentile has moved from the recorded share by more than
/// `share_tolerance`. Each point's medians are measured over as many
/// fresh bursts as they were recorded over, or `TRIALS` if more; the shares over
/// `TAIL_BURSTS`.
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
        let sources = source_cases(&m, mode);
        for &l in &levels {
            let target = Target::Level(l);
            let case = Case::new(&m, mode, target);
            for offset in GATE_OFFSETS_DB {
                let p = base.point(mode, l, offset);
                let s = sample(&case, &sources[0], p.cnr_db + offset, GATE_FIRST_TRIAL, p.bursts.max(bursts));
                let (snr_db, data_db) = (median(&s.snr), median(&s.data));
                let tails = sample_tails(&case, &sources, p.cnr_db + offset, GATE_FIRST_TRIAL, TAIL_BURSTS);
                let shares = [0, 1].map(|i| {
                    let r = SOURCES[i + 1].of(p);
                    [share_at_or_above(&tails[i], r.median_db), share_at_or_above(&tails[i], r.q95_db)]
                });
                let moved = verdict(p, snr_db, data_db, shares);
                println!(
                    "{} at {:.2}{offset:+.2} dB: SNR {snr_db:.2} dB ({:.2}), from data {data_db:.2} dB ({:.2}) \
                     (recorded in brackets); at or above recorded median/q95: from 1 codeword {:.1}/{:.1} % \
                     ({:.1}/{:.1}), header only {:.1}/{:.1} % ({:.1}/{:.1}) {}",
                    target.label(mode),
                    p.cnr_db,
                    p.snr_db,
                    p.from_data.median_db,
                    100.0 * shares[0][0],
                    100.0 * shares[0][1],
                    100.0 * p.from_short.median_share,
                    100.0 * p.from_short.q95_share,
                    100.0 * shares[1][0],
                    100.0 * shares[1][1],
                    100.0 * p.from_header.median_share,
                    100.0 * p.from_header.q95_share,
                    if moved.is_empty() { "ok".to_string() } else { format!("FAIL: {}", moved.join(", ")) }
                );
                if !moved.is_empty() {
                    failed.push(format!("{} {offset:+}", target.label(mode)));
                }
            }
        }
    }
    assert!(failed.is_empty(), "SNR estimate moved for {failed:?}");
}

/// What moved at recorded point `p`, given the measured median SNR and median
/// prediction from the data burst, and for the connect request and the header-only
/// burst the shares of fresh bursts at or above their recorded median and 95th
/// percentile; empty when nothing did.
fn verdict(p: &SnrPoint, snr_db: f32, data_db: f32, shares: [[f32; 2]; 2]) -> Vec<String> {
    let mut moved = Vec::new();
    if !within(p.snr_db, snr_db, SNR_GATE_MARGIN_DB) {
        moved.push("SNR".to_string());
    }
    if !within(p.from_data.median_db, data_db, SNR_GATE_MARGIN_DB) {
        moved.push("median from data".to_string());
    }
    for (src, [median, tail]) in SOURCES[1..].iter().zip(shares) {
        let r = src.of(p);
        if !within(r.median_share, median, share_tolerance(r.median_share)) {
            moved.push(format!("median from {}", src.name()));
        }
        if !within(r.q95_share, tail, share_tolerance(r.q95_share)) {
            moved.push(format!("tail from {}", src.name()));
        }
    }
    moved
}

/// Whether `measured` lies within `margin` of `recorded`.
fn within(recorded: f32, measured: f32, margin: f32) -> bool {
    (measured - recorded).abs() <= margin
}

/// A recorded point with every value at `x` dB.
#[cfg(test)]
fn flat_point(x: f32) -> SnrPoint {
    let p = Predicted { median_db: x, q95_db: x, q99_db: x, sigma_db: 0.5, median_share: 0.5, q95_share: 0.05 };
    SnrPoint {
        mode: "narrow".into(),
        level: 1,
        name: LEVELS[1].name.into(),
        drive: 0.3,
        cnr_db: 4.0,
        offset_db: 0.0,
        snr_db: x,
        sigma_db: 0.5,
        bursts: 200,
        from_data: p.clone(),
        from_short: p.clone(),
        from_header: p,
    }
}

/// The verdict passes a measurement within the margins and fails one where any single
/// gated value moved by twice its margin, in either direction.
#[test]
fn verdict_fails_any_moved_value() {
    let p = flat_point(5.0);
    let even = [[0.5, 0.05]; 2];
    let margins = [share_tolerance(0.5), share_tolerance(0.05)];
    assert!(verdict(&p, 5.0, 5.0, even).is_empty());
    let half = 0.5 * SNR_GATE_MARGIN_DB;
    assert!(verdict(&p, 5.0 + half, 5.0 - half, even).is_empty());
    for sign in [-1.0, 1.0] {
        let d = sign * 2.0 * SNR_GATE_MARGIN_DB;
        assert_eq!(verdict(&p, 5.0 + d, 5.0, even), ["SNR"]);
        assert_eq!(verdict(&p, 5.0, 5.0 + d, even), ["median from data"]);
        for i in 0..2 {
            for j in 0..2 {
                let mut shares = even;
                shares[i][j] += sign * 2.0 * margins[j];
                assert_eq!(verdict(&p, 5.0, 5.0, shares).len(), 1, "{:?} share {j}", SOURCES[i + 1]);
                shares[i][j] = even[i][j] + sign * 0.5 * margins[j];
                assert!(verdict(&p, 5.0, 5.0, shares).is_empty());
            }
        }
    }
}

/// The share tolerances quoted at `TAIL_BURSTS` hold, and a share counts ties: a
/// cluster at a clamp is at or above a percentile that falls on it.
#[test]
fn share_tolerances_and_ties() {
    assert!((share_tolerance(0.05) - 0.029).abs() < 0.001);
    assert!((share_tolerance(0.5) - 0.067).abs() < 0.001);
    assert_eq!(share_at_or_above(&[1.0, 2.0, 3.0, 4.0], 3.0), 0.5);
    let clamped = [1.0, 2.0, 29.0, 29.0];
    assert_eq!(share_at_or_above(&clamped, quantile(&clamped, GATE_QUANTILE)), 0.5);
}

// The gate measures bursts the sweep did not: a gate that replayed the recording
// would always see nothing move. The sweep also asserts this for a `TRIALS` above
// `MAX_BURSTS`.
const _: () = assert!(MAX_BURSTS <= GATE_FIRST_TRIAL && TAIL_BURSTS <= GATE_FIRST_TRIAL);

/// The measurement path reports what `Receiver` reports on the same audio, so the
/// gate covers the SNR the link sees: every level at its upper gated point and near
/// the ceiling at 30 dB CNR, and every source at the upper gated points of levels 0, 3
/// and 6. Bursts the receiver does not detect or whose header it does not decode
/// report nothing; of `MATCH_TRIALS` trials at least `MIN_DECODED` must, and the
/// medians over those must agree. Single bursts at the lowest levels can differ by a
/// few dB: the receiver finds the burst itself, and replaces samples beyond full scale.
#[test]
fn measurement_matches_the_receiver() {
    let levels = level_eval::load();
    let upper = GATE_OFFSETS_DB[GATE_OFFSETS_DB.len() - 1];
    for mode in ALL_MODES.split(',') {
        let m = modem(mode);
        let sources = source_cases(&m, mode);
        let gated = |l: u8| levels.point(mode, Target::Level(l)).cnr_db + upper;
        let own: Vec<Case> = (0..LEVELS.len() as u8).map(|l| Case::new(&m, mode, Target::Level(l))).collect();
        // Near the ceiling every burst decodes and agrees closely: fewer trials do.
        let mut cases: Vec<(&Case, f32, u32)> =
            own.iter().flat_map(|c| [(c, gated(c.level), MATCH_TRIALS), (c, 30.0, MIN_DECODED as u32)]).collect();
        for l in [0, 3, LEVELS.len() as u8 - 1] {
            cases.extend(sources.iter().map(|s| (s, gated(l), MATCH_TRIALS)));
        }
        for (c, cnr, trials) in cases {
            let pairs: Vec<(f32, f32)> = on_every_core(trials, |t| {
                let sent = trial_burst(c.level, c.ncw, t);
                let audio = through_channel(&m, mode, cnr, &sent, t);
                let mut rx = Receiver::new(m.clone());
                let mut got: Vec<RxBurst> = audio.chunks(480).flat_map(|x| rx.push(x)).collect();
                got.extend(rx.push(&vec![0.0; 48_000]));
                assert!(got.len() <= 1, "{mode} L{} at {cnr:.2} dB: receiver got {} bursts", c.level, got.len());
                got.first().map(|g| (g.snr_db, snr(c, cnr, t)))
            })
            .into_iter()
            .flatten()
            .collect();
            let at = format!("{mode} L{} {} codewords at {cnr:.2} dB", c.level, c.ncw);
            assert!(pairs.len() >= MIN_DECODED, "{at}: the receiver decoded {} bursts", pairs.len());
            let (rx, gate): (Vec<f32>, Vec<f32>) = pairs.into_iter().unzip();
            let (rx, gate) = (median(&rx), median(&gate));
            assert!(
                (rx - gate).abs() < MATCH_TOLERANCE_DB,
                "{at}: median {rx:.3} dB from the receiver, {gate:.3} dB from the gate"
            );
        }
    }
}

/// Trials per case of `measurement_matches_the_receiver`. At narrow level 0's point,
/// below the header's, the receiver decodes about one burst in four.
const MATCH_TRIALS: u32 = 64;
/// Bursts the receiver must decode in a case.
const MIN_DECODED: usize = 8;
/// How far the median of the receiver's SNR may lie from the gate's over the same
/// bursts, dB. Over 128 trials the largest gap was 0.10 dB, at narrow level 0's point
/// on about 30 decoded bursts, and 0.06 dB or less elsewhere.
const MATCH_TOLERANCE_DB: f32 = 0.15;

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
        "{SNR_BASELINE} was measured at the CNRs of another perf/levels.toml: run snr_sweep with WRITE=1 again"
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
                assert!(p.bursts <= GATE_FIRST_TRIAL, "{label}: its bursts overlap the gate's");
                n += 1;
            }
        }
    }
    assert_eq!(base.point.len(), n, "{SNR_BASELINE}: points for unknown modes, levels or offsets");
}

/// At every recorded point, the median SNR predicted from a level-0 data burst lies
/// within `PREDICT_TOLERANCE_DB` of the SNR measured on the level's own bursts.
#[test]
fn snr_prediction_matches_measurement() {
    let base = load();
    let bad: Vec<String> = base
        .point
        .iter()
        .filter(|p| (p.from_data.median_db - p.snr_db).abs() > PREDICT_TOLERANCE_DB)
        .map(|p| {
            format!(
                "{} L{} at {:+} dB: predicted {:.2}, measured {:.2}",
                p.mode, p.level, p.offset_db, p.from_data.median_db, p.snr_db
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
    // An explicit count above `MAX_BURSTS` is not a scatter that needs too many.
    assert_eq!(bursts_for(0.1, 2 * MAX_BURSTS), 2 * MAX_BURSTS);
}

#[test]
fn median_sigma_ignores_outliers() {
    let (m, s) = median_sigma(&[1.0, 2.0, 3.0, 4.0, -40.0]);
    assert_eq!(m, 2.0);
    assert!((s - 1.4826).abs() < 1e-4, "{s}");
    assert_eq!(median(&[1.0, 3.0]), 2.0);
}

#[test]
fn quantile_by_nearest_rank() {
    let x: Vec<f32> = (1..=100).map(|i| i as f32).collect();
    assert_eq!(quantile(&x, 0.95), 95.0);
    assert_eq!(quantile(&x, 0.99), 99.0);
    assert_eq!(quantile(&x, 0.0), 1.0);
    assert_eq!(quantile(&[3.0, 1.0, 2.0], 0.5), 2.0);
}

#[test]
fn gate_offsets_are_recorded() {
    assert!(GATE_OFFSETS_DB.iter().all(|o| SNR_OFFSETS_DB.contains(o)));
}
