//! Decoding on a channel with interference other than noise
//! (`vardefm_sim::interference`): per mode, scenario and speed level, the share of
//! codewords lost, the share of bursts not found, and the SNR the receiver reports,
//! all at one carrier-to-noise ratio.
//!
//! Each trial sends one full-length burst (the most codewords its level allows, as a
//! file transfer does) through the simulator and the streaming [`Receiver`], so
//! detection and the header count too. A lost burst counts all its codewords as
//! lost. Every scenario's trials have the same payloads and the same thermal noise,
//! so the rows of a mode compare like for like.
//!
//! Read the SNR beside the loss. The link picks the level from the SNR
//! (`vardefm_link::level_by_snr`), so interference that costs codewords without
//! lowering the SNR is interference the rate adaptation does not see; the
//! `adaptation` test of vardefm-tnc measures what that costs a session.
//!
//! Nothing here is gated, and the scenarios are not taken from measurements of real
//! radios: the levels and rates were chosen to span from harmless to crippling.
//!
//! `cargo test --release -p vardefm-modem interference_sweep -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `LEVELS` (0,1,..,6), `TRIALS` (bursts per
//! point, 40), `CNR` (dB in the IF bandwidth, 30), `INTERFERENCE` (scenarios
//! separated by commas, each a colon-separated list of interferers as in
//! `vardefm_sim::parse_interference`, e.g. `tone=1000@-30,pop=5@2@0:tone=50@-40`;
//! unset, [`default_scenarios`]).

use super::*;
use crate::detect_eval::{burst, env_list, env_modes, env_or, modem, on_every_core, payload_seed, through};
use std::sync::Arc;
use vardefm_sim::{parse_interference, FmChannelConfig};

/// Silence pushed after a trial's audio, so the receiver finishes the burst.
const FLUSH_SAMPLES: usize = 48_000;

/// The scenarios run when `INTERFERENCE` is unset, for `p`'s mode. Tones and spurs
/// sit on a data carrier midway between two pilots near the middle of the band,
/// where the pilots' noise estimate sees least of them, and once on a pilot.
/// Bursts and pops come as short (2 ms, a fraction of a symbol) and long (20 ms,
/// about one narrow symbol or two wide ones), rare (1 per second) and frequent (5).
fn default_scenarios(p: &OfdmParams) -> Vec<String> {
    let pilot = p.num_carriers / 2 / p.pilot_spacing * p.pilot_spacing;
    let (on_pilot, on_data) = (p.carrier_freq(pilot), p.carrier_freq(pilot + p.pilot_spacing / 2));
    let mut out = vec![String::new()];
    out.extend([-40, -30, -20].map(|db| format!("tone={on_data}@{db}")));
    out.push(format!("tone={on_pilot}@-30"));
    out.extend([30, 20, 10].map(|db| format!("spur={on_data}@{db}")));
    for kind in ["pop", "rfburst"] {
        // An RF burst 10 dB above the carrier; a pop at full scale, which within the
        // modem's band is about as strong as the signal.
        let level = if kind == "pop" { 0 } else { -10 };
        out.extend([(1, 2), (5, 2), (1, 20), (5, 20)].map(|(rate, ms)| format!("{kind}={rate}@{ms}@{level}")));
    }
    out
}

/// What came of some trials.
#[derive(Clone, Copy, Default)]
struct Tally {
    bursts: u32,
    /// Bursts whose header was not received.
    lost: u32,
    codewords: u32,
    /// Codewords not received, those of lost bursts included.
    failed: u32,
    /// Sum of the reported SNR over the bursts received, dB.
    snr_sum: f32,
}

impl Tally {
    fn add(self, o: Tally) -> Tally {
        Tally {
            bursts: self.bursts + o.bursts,
            lost: self.lost + o.lost,
            codewords: self.codewords + o.codewords,
            failed: self.failed + o.failed,
            snr_sum: self.snr_sum + o.snr_sum,
        }
    }

    /// Codewords lost, per cent.
    fn cer(&self) -> f32 {
        100.0 * self.failed as f32 / self.codewords.max(1) as f32
    }

    /// Bursts lost, per cent.
    fn lost(&self) -> f32 {
        100.0 * self.lost as f32 / self.bursts.max(1) as f32
    }

    /// Mean reported SNR of the bursts received, dB; NaN when none was.
    fn snr(&self) -> f32 {
        self.snr_sum / (self.bursts - self.lost) as f32
    }
}

/// Trial `t`: one burst of `ncw` codewords at `level` over `channel` at `cnr` dB,
/// through the streaming receiver.
fn trial(m: &Arc<Modem>, channel: &FmChannelConfig, cnr: f32, level: u8, ncw: usize, t: u32) -> Tally {
    let sent = burst(level, ncw, payload_seed(t));
    let audio = through(m, channel, cnr, &sent, t);
    let mut rx = Receiver::new(m.clone());
    let mut got = Vec::new();
    for chunk in audio.chunks(480) {
        got.extend(rx.push(chunk));
    }
    got.extend(rx.push(&vec![0.0; FLUSH_SAMPLES]));
    let all = Tally { bursts: 1, codewords: ncw as u32, ..Tally::default() };
    match got.iter().find(|b| b.header == sent.header) {
        Some(b) => {
            let failed = b.payload.iter().zip(&sent.payload).filter(|(g, w)| g.as_ref() != Some(*w)).count();
            Tally { failed: failed as u32, snr_sum: b.snr_db, ..all }
        }
        None => Tally { lost: 1, failed: ncw as u32, ..all },
    }
}

/// `trials` trials of full-length bursts at `level` on the mode's own channel with
/// the interference of `spec`.
fn point(m: &Arc<Modem>, mode: &str, spec: &str, cnr: f32, level: u8, trials: u32) -> Tally {
    let base = FmChannelConfig::by_name(mode).unwrap();
    let interference = parse_interference(spec, base.fs).unwrap_or_else(|e| panic!("INTERFERENCE: {e}"));
    let channel = FmChannelConfig { interference, ..base };
    let ncw = m.max_ncw(level);
    on_every_core(trials, |t| trial(m, &channel, cnr, level, ncw, t)).into_iter().fold(Tally::default(), Tally::add)
}

#[test]
#[ignore]
fn interference_sweep() {
    let modes = env_modes("narrow,wide");
    let levels: Vec<u8> = env_list("LEVELS", "0,1,2,3,4,5,6");
    assert!(levels.iter().all(|&l| (l as usize) < LEVELS.len()), "LEVELS: levels are 0 to {}", LEVELS.len() - 1);
    let trials: u32 = env_or("TRIALS", 40);
    let cnr: f32 = env_or("CNR", 30.0);
    let given: Option<Vec<String>> = std::env::var("INTERFERENCE").ok().map(|_| env_list("INTERFERENCE", ""));
    for mode in &modes {
        let m = modem(mode);
        let scenarios = given.clone().unwrap_or_else(|| default_scenarios(&m.ofdm.p));
        let width = scenarios.iter().map(|s| s.len()).max().unwrap_or(0).max(8);
        println!("\n{mode}, CNR {cnr} dB, {trials} full-length bursts per point.");
        println!("Per level: codewords lost % (lost bursts included) / bursts lost % / reported SNR dB");
        print!("{:<width$}", "scenario");
        for &l in &levels {
            print!(" | {:^16}", format!("L{l} x{}", m.max_ncw(l)));
        }
        println!();
        for spec in &scenarios {
            print!("{:<width$}", if spec.is_empty() { "none" } else { spec });
            for &l in &levels {
                let t = point(&m, mode, spec, cnr, l, trials);
                print!(" | {:5.1} {:4.0} {:5.1}", t.cer(), t.lost(), t.snr());
            }
            println!();
        }
    }
}

/// The harness sees interference: at a carrier-to-noise ratio where a burst decodes
/// whole, heavy RF bursts cost codewords.
#[test]
fn harness_loses_codewords_to_heavy_interference_only() {
    let m = modem("narrow");
    let clean = point(&m, "narrow", "", 30.0, 4, 3);
    assert_eq!((clean.bursts, clean.lost, clean.failed), (3, 0, 0));
    assert!(clean.codewords == 3 * m.max_ncw(4) as u32 && clean.snr() > 15.0, "SNR {}", clean.snr());
    let heavy = point(&m, "narrow", "rfburst=20@20@-20", 30.0, 4, 3);
    assert!(heavy.failed > 0, "nothing lost under RF bursts for 40 % of the time");
}

#[test]
fn default_scenarios_parse_and_sit_on_the_carriers_they_name() {
    for mode in ["narrow", "wide"] {
        let m = modem(mode);
        let p = &m.ofdm.p;
        let scenarios = default_scenarios(p);
        for s in &scenarios {
            parse_interference(s, p.fs).unwrap_or_else(|e| panic!("{mode}: {e}"));
        }
        let pilot = p.num_carriers / 2 / p.pilot_spacing * p.pilot_spacing;
        assert!(m.ofdm.pilot_idx.contains(&pilot), "{mode}: carrier {pilot} is not a pilot");
        assert!(m.ofdm.data_idx.contains(&(pilot + p.pilot_spacing / 2)), "{mode}");
    }
}
