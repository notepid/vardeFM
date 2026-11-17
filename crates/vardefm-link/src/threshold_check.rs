//! `SNR_THRESHOLDS_DB` against the decoding and SNR points vardefm-modem records
//! (`perf/levels.toml`, `perf/snr.toml`). Reads the files only; the modem's
//! `level_gate` and `snr_gate` check that the files still describe the modem.
//!
//! What this does not catch:
//! - A decoding change that `level_gate` lets through. It passes a 10 % point that
//!   moved by less than its 0.25 dB margin, and `snr_gate` measures at fixed CNRs, so
//!   neither notices; near the FM threshold the SNR rises by about 3 dB per dB of CNR,
//!   so such a move shifts the SNR at a level's 1 % CER point by up to about 0.8 dB,
//!   more than `MIN_THRESHOLD_SLACK_DB` leaves at narrow level 2. The thresholds and
//!   the curves are compared only as recorded, and agree again only once both files
//!   are recorded anew.
//! - Single bursts. The checks below use medians, while the link acts on every burst
//!   alone. The 95th and 99th percentiles of the SNR predicted from level-0
//!   acknowledgements and connect requests are printed against the switch points, not
//!   checked: at a level's 10 % CER point many such bursts lie above its switch point.

use super::*;

const LEVELS_TOML: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../vardefm-modem/perf/levels.toml");
const SNR_TOML: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../vardefm-modem/perf/snr.toml");

/// Codeword error rate at which a level counts as decoding reliably: the thresholds
/// were set where every burst in the `level_table` benchmark decoded.
///
/// Its crossing on a recorded grid is interpolated in log10 of the rate, or linearly
/// next to a grid point without errors. That makes it coarse there: at narrow level 2,
/// whose grid has a 0 just past the crossing, one codeword error more or less at that
/// point moves the crossing by about 0.1 dB of CNR, 0.3 dB of SNR.
const THRESHOLD_CER: f32 = 0.01;
/// The slack of a level: how far the SNR at which the link switches to it (its
/// threshold plus `SNR_MARGIN_DB`) lies above the median SNR at which the level
/// reaches `THRESHOLD_CER`, in the mode and on the curve where it lies nearest, dB.
/// Recorded: 0.9 dB (level 2) to 1.4 dB (levels 4 and 5).
///
/// At least this much, so a recording whose SNR rose would not pass while it eats
/// most of the link's 1 dB margin: about 0.4 dB of rise at level 2 fails.
const MIN_THRESHOLD_SLACK_DB: f32 = 0.5;
/// At most this much: the most recorded plus the 0.5 dB a threshold is rounded up by,
/// so a threshold may be re-rounded, but a recording whose SNR fell by more than
/// about 0.6 dB at levels 4 and 5 fails.
const MAX_THRESHOLD_SLACK_DB: f32 = 2.0;

#[derive(serde::Deserialize)]
struct Levels {
    level: Vec<LevelPoint>,
}

#[derive(serde::Deserialize)]
struct LevelPoint {
    mode: String,
    level: u8,
    grid_db: Vec<f32>,
    rate: Vec<f32>,
}

#[derive(serde::Deserialize)]
struct Snr {
    point: Vec<SnrPoint>,
}

#[derive(serde::Deserialize)]
struct SnrPoint {
    mode: String,
    level: u8,
    cnr_db: f32,
    offset_db: f32,
    snr_db: f32,
    from_data: Predicted,
    from_short: Predicted,
    from_header: Predicted,
}

#[derive(serde::Deserialize)]
struct Predicted {
    median_db: f32,
    q95_db: f32,
    q99_db: f32,
}

/// One of the recorded SNR curves of a point.
type Curve = fn(&SnrPoint) -> f32;

/// The two median curves checked: what the receiver reports on a level's own bursts,
/// and what `Modem::predict_snr_db` gives for the level from a level-0 data burst. The
/// link decides on the first for a burst at the level and on a prediction for the
/// others.
const CURVES: [(&str, Curve); 2] = [("measured", |p| p.snr_db), ("predicted from L0 data", |p| p.from_data.median_db)];

/// The tails printed: the upper percentiles of the SNR predicted from the level-0
/// bursts the link sends most, acknowledgements and polls, and the connect request.
const TAILS: [(&str, Curve); 4] = [
    ("q95 from L0 header only", |p| p.from_header.q95_db),
    ("q99 from L0 header only", |p| p.from_header.q99_db),
    ("q95 from L0 1 codeword", |p| p.from_short.q95_db),
    ("q99 from L0 1 codeword", |p| p.from_short.q99_db),
];

fn read<T: serde::de::DeserializeOwned>(path: &str) -> T {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    toml::from_str(&text).unwrap_or_else(|e| panic!("{path}: {e}"))
}

/// Where `rates` (falling with `grid`) cross `target`, interpolated in log10 of the
/// rate, or linearly next to a 0. A copy of `level_eval::crossing` in vardefm-modem,
/// which is test code there and cannot be shared: change both together.
fn crossing(grid: &[f32], rates: &[f32], target: f32) -> Option<f32> {
    let i = rates.windows(2).position(|w| w[0] >= target && w[1] < target)?;
    let (x0, x1, y0, y1) = (grid[i], grid[i + 1], rates[i], rates[i + 1]);
    let t =
        if y1 > 0.0 { (y0.log10() - target.log10()) / (y0.log10() - y1.log10()) } else { (y0 - target) / (y0 - y1) };
    Some(x0 + t * (x1 - x0))
}

/// `y` at `x` on the piecewise linear curve through `points` (increasing in x);
/// `None` outside it.
fn interpolate(points: &[(f32, f32)], x: f32) -> Option<f32> {
    let w = points.windows(2).find(|w| w[0].0 <= x && x <= w[1].0)?;
    let ((x0, y0), (x1, y1)) = (w[0], w[1]);
    Some(y0 + (x - x0) / (x1 - x0) * (y1 - y0))
}

/// The SNR on `curve` where `mode`'s `level` reaches `THRESHOLD_CER`.
fn snr_at_threshold_cer(levels: &Levels, snr: &Snr, mode: &str, level: u8, curve: Curve) -> f32 {
    let at = format!("{mode} L{level}");
    let lp = levels.level.iter().find(|p| p.mode == mode && p.level == level);
    let lp = lp.unwrap_or_else(|| panic!("{LEVELS_TOML} has no {at}"));
    let cnr = crossing(&lp.grid_db, &lp.rate, THRESHOLD_CER)
        .unwrap_or_else(|| panic!("{at}: the recorded grid does not cross {THRESHOLD_CER}"));
    let mut points: Vec<(f32, f32)> = snr
        .point
        .iter()
        .filter(|p| p.mode == mode && p.level == level)
        .map(|p| (p.cnr_db + p.offset_db, curve(p)))
        .collect();
    points.sort_by(|a, b| a.0.total_cmp(&b.0));
    interpolate(&points, cnr).unwrap_or_else(|| panic!("{at}: {SNR_TOML} does not span {cnr:.2} dB CNR: {points:?}"))
}

/// The slack of every level, on both median curves in both modes, lies between
/// `MIN_THRESHOLD_SLACK_DB` and `MAX_THRESHOLD_SLACK_DB`: the SNR at which the link
/// switches to the level, its threshold plus `SNR_MARGIN_DB`, lies at least the
/// minimum above the median SNR at which that level reaches `THRESHOLD_CER`, and at
/// most the maximum where it lies nearest. Level 0's threshold is never used. Prints
/// the tails (`TAILS`) at the same point.
///
/// The bare thresholds sit near that SNR (narrow level 2's 0.1 dB below it): they
/// were set at the highest SNR of any burst that still lost a codeword in a few
/// bursts each, and the margin is what keeps the link off a level's error slope.
#[test]
fn thresholds_lie_above_recorded_snr() {
    let levels: Levels = read(LEVELS_TOML);
    let snr: Snr = read(SNR_TOML);
    let mut bad = Vec::new();
    for level in 1..LEVELS.len() as u8 {
        let switch = SNR_THRESHOLDS_DB[level as usize] + SNR_MARGIN_DB;
        let mut slack = f32::INFINITY;
        for mode in ["narrow", "wide"] {
            for (name, curve) in CURVES {
                let s = snr_at_threshold_cer(&levels, &snr, mode, level, curve);
                println!(
                    "{mode} L{level} {name}: {:.0} % CER at {s:.2} dB SNR, link switches at {switch} dB",
                    100.0 * THRESHOLD_CER
                );
                slack = slack.min(switch - s);
            }
            let tails: Vec<String> = TAILS
                .iter()
                .map(|(name, curve)| {
                    format!("{name} {:+.2}", snr_at_threshold_cer(&levels, &snr, mode, level, *curve) - switch)
                })
                .collect();
            println!("{mode} L{level} tails against the switch point, dB: {}", tails.join(", "));
        }
        println!("L{level}: slack {slack:.2} dB");
        if !(MIN_THRESHOLD_SLACK_DB..=MAX_THRESHOLD_SLACK_DB).contains(&slack) {
            bad.push(format!(
                "L{level}: link switches at {switch} dB, {slack:.2} dB above the nearest SNR at {:.0} % CER, \
                 outside {MIN_THRESHOLD_SLACK_DB} to {MAX_THRESHOLD_SLACK_DB} dB",
                100.0 * THRESHOLD_CER
            ));
        }
    }
    assert!(bad.is_empty(), "SNR_THRESHOLDS_DB and the recorded curves disagree:\n  {}", bad.join("\n  "));
}

#[test]
fn interpolate_stays_inside() {
    let c = [(0.0, 1.0), (1.0, 3.0)];
    assert_eq!(interpolate(&c, 0.5), Some(2.0));
    assert_eq!(interpolate(&c, 1.0), Some(3.0));
    assert_eq!(interpolate(&c, 1.5), None);
    assert_eq!(interpolate(&c, -0.1), None);
}
