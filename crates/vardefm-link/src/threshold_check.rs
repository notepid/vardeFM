//! `SNR_THRESHOLDS_DB` against the decoding and SNR points vardefm-modem records
//! (`perf/levels.toml`, `perf/snr.toml`), so the thresholds and the measured curves
//! cannot drift apart. Reads the files only; the modem's `level_gate` and `snr_gate`
//! check that the files still describe the modem.

use super::*;

const LEVELS_TOML: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../vardefm-modem/perf/levels.toml");
const SNR_TOML: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../vardefm-modem/perf/snr.toml");

/// Codeword error rate at which a level counts as decoding reliably: the thresholds
/// were set where every burst in the `level_table` benchmark decoded.
const THRESHOLD_CER: f32 = 0.01;
/// How far the SNR at which the link switches to a level (its threshold plus
/// `SNR_MARGIN_DB`) may lie above the median SNR at which the level reaches
/// `THRESHOLD_CER`, in the mode where it lies nearest, dB. Recorded: 0.9 dB (level 2)
/// to 1.4 dB (levels 4 and 5). This allows the most recorded plus the 0.5 dB a
/// threshold is rounded up by, so a threshold may be re-rounded, but a recording whose
/// SNR fell by more than about half a dB at the most cautious level fails.
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
}

fn read<T: serde::de::DeserializeOwned>(path: &str) -> T {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    toml::from_str(&text).unwrap_or_else(|e| panic!("{path}: {e}"))
}

/// Where `rates` (falling with `grid`) cross `target`, interpolated in log10 of the
/// rate, or linearly next to a 0, as `level_eval::crossing` in vardefm-modem.
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

/// The median SNR the receiver reports where `mode`'s `level` reaches `THRESHOLD_CER`.
fn snr_at_threshold_cer(levels: &Levels, snr: &Snr, mode: &str, level: u8) -> f32 {
    let at = format!("{mode} L{level}");
    let lp = levels.level.iter().find(|p| p.mode == mode && p.level == level);
    let lp = lp.unwrap_or_else(|| panic!("{LEVELS_TOML} has no {at}"));
    let cnr = crossing(&lp.grid_db, &lp.rate, THRESHOLD_CER)
        .unwrap_or_else(|| panic!("{at}: the recorded grid does not cross {THRESHOLD_CER}"));
    let mut curve: Vec<(f32, f32)> = snr
        .point
        .iter()
        .filter(|p| p.mode == mode && p.level == level)
        .map(|p| (p.cnr_db + p.offset_db, p.snr_db))
        .collect();
    curve.sort_by(|a, b| a.0.total_cmp(&b.0));
    interpolate(&curve, cnr).unwrap_or_else(|| panic!("{at}: {SNR_TOML} does not span {cnr:.2} dB CNR: {curve:?}"))
}

/// The SNR at which the link switches to each level, its threshold plus
/// `SNR_MARGIN_DB`, lies at or above the median SNR at which that level reaches
/// `THRESHOLD_CER` in every mode, and no more than `MAX_THRESHOLD_SLACK_DB` above it in
/// the mode where it lies nearest. Level 0's threshold is never used.
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
            let s = snr_at_threshold_cer(&levels, &snr, mode, level);
            println!(
                "{mode} L{level}: {:.0} % CER at {s:.2} dB SNR, link switches at {switch} dB",
                100.0 * THRESHOLD_CER
            );
            if s > switch {
                bad.push(format!("{mode} L{level}: link switches at {switch} dB, below {s:.2} dB"));
            }
            slack = slack.min(switch - s);
        }
        println!("L{level}: slack {slack:.2} dB");
        if slack > MAX_THRESHOLD_SLACK_DB {
            bad.push(format!("L{level}: link switches at {switch} dB, {slack:.2} dB above the SNR in every mode"));
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
