//! How many failed codewords a second decoding pass rescues (issue #2).
//!
//! `Modem::decode_payload_passes` with more than one pass decodes the failed
//! codewords of a burst again with the coded bits of the decoded ones known: at 16QAM and 64QAM an
//! axis of a point carries bits of several codewords, so knowing some of them leaves
//! fewer possible points for the rest. This probe measures, per mode and level 3 to
//! 6, around the CNR at which the level's codeword error rate crosses 10 %
//! (`perf/levels.toml`), from the known burst position with the sent header as
//! training, as `level_eval` does. `level_eval` decodes as the receiver does, with
//! the later passes, so once the baseline has been recorded with them its points
//! lie lower than the single-pass ones this probe was first run around (recorded at
//! eb35213): the probe prints the first pass's CER at each point, so read the
//! offsets against that rather than against the point. It reports:
//! - `fail1`: codewords that fail the first pass, of all sent;
//! - `rescued`: of those, the ones a later pass decodes correctly; and how many of the bursts
//!   whose codewords all decoded only thanks to it (`saved`) of those that had a
//!   failure (`mixed` or all failed);
//! - the decode time of the first pass and the extra time of the later passes, per
//!   burst, on one thread (trials run on every core, so the times are under load).
//!
//! `cargo test --release -p vardefm-modem rescue_probe -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `LEVELS` (3,4,5,6), `TRIALS` (bursts per point,
//! 200), `NCW` (codewords per burst; default the level's longest burst), `OFFSETS`
//! (dB from the recorded point, -1,-0.5,-0.25,0,0.25,0.5), `PASSES` (most passes,
//! `MAX_PASSES`).

use super::*;
use crate::detect_eval::{
    burst, env_list, env_modes, env_or, modem, on_every_core, payload_seed, through_channel, true_position,
};
use std::time::Instant;

const BASELINE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/perf/levels.toml");

/// The recorded CNR (dB) at which the codeword error rate of `level` in `mode`
/// crosses 10 %.
fn level_point(mode: &str, level: u8) -> f32 {
    let text = std::fs::read_to_string(BASELINE).unwrap_or_else(|e| panic!("{BASELINE}: {e}"));
    let base: toml::Value = toml::from_str(&text).unwrap();
    let points = base["level"].as_array().expect("level points");
    let p = points
        .iter()
        .find(|p| p["mode"].as_str() == Some(mode) && p["level"].as_integer() == Some(level as i64))
        .unwrap_or_else(|| panic!("{BASELINE} has no {mode} level {level}"));
    p["cnr_db"].as_float().expect("cnr_db") as f32
}

/// One trial's outcome.
#[derive(Default, Clone, Copy)]
struct Tally {
    sent: u32,
    fail1: u32,
    rescued: u32,
    /// Bursts with a codeword failed after the first pass.
    bad_bursts: u32,
    /// ... of which every codeword decoded after the later passes.
    saved: u32,
    /// Bursts where the first pass decoded some codewords and not all.
    mixed: u32,
    first_s: f64,
    extra_s: f64,
    /// Codewords that failed the first pass and decoded in a later one with data
    /// that differs from what was sent (should stay 0).
    wrong: u32,
    /// Bursts that needed more than two passes.
    over2: u32,
    /// Most passes any burst made.
    max_passes: u32,
}

impl std::ops::Add for Tally {
    type Output = Tally;
    fn add(self, o: Tally) -> Tally {
        Tally {
            sent: self.sent + o.sent,
            fail1: self.fail1 + o.fail1,
            rescued: self.rescued + o.rescued,
            bad_bursts: self.bad_bursts + o.bad_bursts,
            saved: self.saved + o.saved,
            mixed: self.mixed + o.mixed,
            first_s: self.first_s + o.first_s,
            extra_s: self.extra_s + o.extra_s,
            wrong: self.wrong + o.wrong,
            over2: self.over2 + o.over2,
            max_passes: self.max_passes.max(o.max_passes),
        }
    }
}

/// Decodes trial `t` of a `level` burst of `ncw` codewords at `cnr` dB, with and
/// without the later passes.
fn trial(m: &Modem, mode: &str, level: u8, ncw: usize, truth: usize, cnr: f32, t: u32) -> Tally {
    let sent = burst(level, ncw, payload_seed(t));
    let rx = through_channel(m, mode, cnr, &sent, t);
    let training = m.header_carriers(&sent.header);
    let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
    let mut out = Tally { sent: ncw as u32, ..Tally::default() };
    let Some(frame) = m.ofdm.demodulate_known(&rx, truth, nsym, &training) else {
        out.fail1 = ncw as u32;
        out.bad_bursts = 1;
        return out;
    };
    let t0 = Instant::now();
    let once = m.decode_payload_passes(&sent.header, &frame, 1);
    let t1 = Instant::now();
    let twice = m.decode_payload_passes(&sent.header, &frame, env_or("PASSES", MAX_PASSES));
    let t2 = Instant::now();
    out.first_s = (t1 - t0).as_secs_f64();
    // The rescuing call repeats the first pass: count only what it adds.
    out.extra_s = ((t2 - t1).as_secs_f64() - out.first_s).max(0.0);
    let ok1 = once.data.iter().zip(&sent.payload).filter(|(g, w)| g.as_ref() == Some(*w)).count() as u32;
    let ok2 = twice.data.iter().zip(&sent.payload).filter(|(g, w)| g.as_ref() == Some(*w)).count() as u32;
    out.fail1 = ncw as u32 - ok1;
    // Counted against the sent data, as `fail1` is, so that a codeword that passes
    // its CRC with wrong data cannot count as rescued.
    out.rescued = ok2.saturating_sub(ok1);
    let rescued_wrong =
        |j: usize| once.data[j].is_none() && twice.data[j].as_ref().is_some_and(|d| *d != sent.payload[j]);
    out.wrong = (0..ncw).filter(|&j| rescued_wrong(j)).count() as u32;
    out.bad_bursts = u32::from(ok1 < ncw as u32);
    out.saved = u32::from(ok1 < ncw as u32 && ok2 == ncw as u32);
    out.mixed = u32::from(ok1 > 0 && ok1 < ncw as u32);
    out.max_passes = twice.passes as u32;
    out.over2 = u32::from(twice.passes > 2);
    out
}

#[test]
#[ignore]
fn rescue_probe() {
    let modes = env_modes("narrow,wide");
    let levels: Vec<u8> = env_list("LEVELS", "3,4,5,6");
    let trials = env_or("TRIALS", 200u32);
    let offsets: Vec<f32> = env_list("OFFSETS", "-1,-0.5,-0.25,0,0.25,0.5");
    for mode in &modes {
        let m = modem(mode);
        for &level in &levels {
            assert!((3..LEVELS.len() as u8).contains(&level), "levels 3 to 6 share bits on an axis");
            let ncw = env_or("NCW", m.max_ncw(level)).min(m.max_ncw(level));
            let truth = true_position(&m, mode, level, ncw);
            let point = level_point(mode, level);
            println!(
                "{mode} L{level} {} ({ncw} codewords per burst), 10 % CER at {point:.2} dB:",
                LEVELS[level as usize].name
            );
            for &off in &offsets {
                let cnr = point + off;
                let r = on_every_core(trials, |t| trial(&m, mode, level, ncw, truth, cnr, t))
                    .into_iter()
                    .fold(Tally::default(), |a, b| a + b);
                let pct = |a: u32, b: u32| if b == 0 { 0.0 } else { 100.0 * a as f32 / b as f32 };
                println!(
                    "  {off:+.2} dB ({cnr:.2}): CER {:.1} % -> {:.1} %; rescued {}/{} ({:.0} %); bursts with a failure {}, mixed {}, saved {} ({:.0} %); decode {:.1} ms/burst + {:.1} ms; passes > 2 in {} bursts, at most {}{}",
                    pct(r.fail1, r.sent),
                    pct(r.fail1 - r.rescued, r.sent),
                    r.rescued,
                    r.fail1,
                    pct(r.rescued, r.fail1),
                    r.bad_bursts,
                    r.mixed,
                    r.saved,
                    pct(r.saved, r.bad_bursts),
                    1e3 * r.first_s / trials as f64,
                    1e3 * r.extra_s / trials as f64,
                    r.over2,
                    r.max_passes,
                    if r.wrong > 0 { format!("; WRONG DATA {}", r.wrong) } else { String::new() },
                );
            }
        }
    }
}

/// Bursts `second_pass_rescues_a_codeword` tries at most.
const TEST_BURSTS: u32 = 20;

/// Near level 3's threshold, the later passes decode a codeword the first pass
/// lost, with the data that was sent, and leave alone what the first pass decoded.
/// The CNR is fixed rather than read from `perf/levels.toml`, so that recording the
/// baseline again cannot move it. It lies near narrow level 3's single-pass 10 %
/// point, where `rescue_probe` finds rescues in a good share of bursts.
#[test]
fn second_pass_rescues_a_codeword() {
    let (mode, level, cnr) = ("narrow", 3u8, 6.25);
    let m = modem(mode);
    let ncw = m.max_ncw(level);
    let truth = true_position(&m, mode, level, ncw);
    let training = |b: &Burst| m.header_carriers(&b.header);
    let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
    let rescued = (0..TEST_BURSTS).find_map(|t| {
        let sent = burst(level, ncw, payload_seed(t));
        let rx = through_channel(&m, mode, cnr, &sent, t);
        let frame = m.ofdm.demodulate_known(&rx, truth, nsym, &training(&sent))?;
        let once = m.decode_payload_passes(&sent.header, &frame, 1);
        let twice = m.decode_payload_passes(&sent.header, &frame, MAX_PASSES);
        for (j, (a, b)) in once.data.iter().zip(&twice.data).enumerate() {
            if a.is_some() {
                assert_eq!(a, b, "trial {t}: codeword {j} changed in a later pass");
            }
            if b.is_some() {
                assert_eq!(b.as_ref(), Some(&sent.payload[j]), "trial {t}: codeword {j} decoded wrong");
            }
        }
        (twice.rescued > 0).then_some((t, twice.rescued))
    });
    let (t, n) = rescued.unwrap_or_else(|| panic!("no codeword rescued in {TEST_BURSTS} bursts at {cnr} dB"));
    println!("trial {t}: {n} codewords rescued");
}
