//! How many failed codewords the later decoding passes rescue (issue #2).
//!
//! `Modem::decode_payload_passes` with more than one pass decodes the failed
//! codewords of a burst again with the coded bits of the decoded ones known: at 16QAM and 64QAM an
//! axis of a point carries bits of several codewords, so knowing some of them leaves
//! fewer possible points for the rest. This probe measures, per mode and level 3 to
//! 6, around the CNR at which the level's codeword error rate crosses 10 %
//! (`perf/levels.toml`), from the known burst position with the sent header as
//! training, as `level_eval` does. `level_eval` decodes as the receiver does, with
//! the later passes, so once the baseline has been recorded with them its points may
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
//! 200), `NCW` (codewords per burst, at most the level's longest burst; default that),
//! `OFFSETS` (dB from the recorded point, -1,-0.5,-0.25,0,0.25,0.5), `PASSES` (most
//! passes, `MAX_PASSES`).

use super::*;
use crate::detect_eval::{
    burst, env_list, env_modes, env_or, modem, on_every_core, payload_seed, through_channel, true_position,
};
use crate::level_eval::{load, Target};
use std::time::Instant;

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
    /// that differs from what was sent. A rescued codeword passed its CRC, so this
    /// counts CRC-32 false passes (about 2^-32 per decode): a zero here says nothing
    /// about the rescue itself, only that nothing is badly broken.
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

/// Decodes trial `t` of a `level` burst of `ncw` codewords at `cnr` dB, with one
/// pass and with up to `passes`.
#[allow(clippy::too_many_arguments)]
fn trial(m: &Modem, mode: &str, level: u8, ncw: usize, truth: usize, cnr: f32, passes: usize, t: u32) -> Tally {
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
    let twice = m.decode_payload_passes(&sent.header, &frame, passes);
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
    for &level in &levels {
        assert!((3..LEVELS.len() as u8).contains(&level), "LEVELS: {level}: levels 3 to 6 share bits on an axis");
    }
    let trials = env_or("TRIALS", 200u32);
    assert!(trials > 0, "TRIALS must be at least 1");
    let passes = env_or("PASSES", MAX_PASSES);
    assert!(passes > 0, "PASSES must be at least 1");
    let offsets: Vec<f32> = env_list("OFFSETS", "-1,-0.5,-0.25,0,0.25,0.5");
    let ncw_env: Option<usize> = std::env::var("NCW").is_ok().then(|| env_or("NCW", 0));
    let base = load();
    for mode in &modes {
        let m = modem(mode);
        for &level in &levels {
            let max = m.max_ncw(level);
            let ncw = ncw_env.unwrap_or(max);
            assert!((1..=max).contains(&ncw), "NCW={ncw}: {mode} level {level} takes 1 to {max} codewords");
            let truth = true_position(&m, mode, level, ncw);
            let point = base.point(mode, Target::Level(level)).cnr_db;
            println!(
                "{mode} L{level} {} ({ncw} codewords per burst), 10 % CER at {point:.2} dB:",
                LEVELS[level as usize].name
            );
            for &off in &offsets {
                let cnr = point + off;
                let r = on_every_core(trials, |t| trial(&m, mode, level, ncw, truth, cnr, passes, t))
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

/// Bursts each case of `receiver_decoding_rescues_codewords` decodes.
const TEST_BURSTS: u32 = 120;

/// Decodes `TEST_BURSTS` full bursts of `level` at `cnr` dB with one pass and as the
/// receiver does (`Modem::decode_payload`), and checks that the receiver rescues at
/// least `min_share` of the codewords the first pass lost, with the data that was sent.
fn check_rescue(mode: &str, level: u8, cnr: f32, min_share: f32) {
    let m = modem(mode);
    let ncw = m.max_ncw(level);
    let truth = true_position(&m, mode, level, ncw);
    let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
    let (fail1, rescued) = on_every_core(TEST_BURSTS, |t| {
        let sent = burst(level, ncw, payload_seed(t));
        let rx = through_channel(&m, mode, cnr, &sent, t);
        let Some(frame) = m.ofdm.demodulate_known(&rx, truth, nsym, &m.header_carriers(&sent.header)) else {
            return (0, 0);
        };
        let once = m.decode_payload_passes(&sent.header, &frame, 1).data;
        let rx_data = m.decode_payload(&sent.header, &frame);
        for (j, got) in rx_data.iter().enumerate() {
            if let Some(d) = got {
                assert_eq!(d, &sent.payload[j], "trial {t}: codeword {j} decoded wrong");
            }
        }
        let ok = |d: &[Option<Vec<u8>>]| d.iter().filter(|g| g.is_some()).count();
        (ncw - ok(&once), ok(&rx_data) - ok(&once))
    })
    .into_iter()
    .fold((0, 0), |a, b| (a.0 + b.0, a.1 + b.1));
    println!("{mode} L{level} at {cnr} dB: {rescued} of {fail1} failed codewords rescued");
    assert!(fail1 > 0, "{mode} L{level} at {cnr} dB: no codeword failed the first pass, nothing to rescue");
    assert!(
        rescued as f32 >= min_share * fail1 as f32,
        "{mode} L{level} at {cnr} dB: {rescued} of {fail1} failed codewords rescued, want at least {:.0} %",
        100.0 * min_share
    );
}

/// Near the single-pass 10 % points of narrow levels 3 (16QAM) and 5 (64QAM), the
/// receiver rescues a good share of the codewords the first pass lost: 45 and 39 %
/// of them on these bursts when written, and the bounds leave room below that. The CNRs are
/// fixed rather than read from `perf/levels.toml`, so that recording the baseline
/// again cannot move them.
#[test]
fn receiver_decoding_rescues_codewords() {
    check_rescue("narrow", 3, 6.25, 0.25);
    check_rescue("narrow", 5, 12.75, 0.2);
}
