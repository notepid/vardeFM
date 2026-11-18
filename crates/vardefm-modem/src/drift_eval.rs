//! Clock drift: the box average of the pilots alone against parametric drift removal
//! (`Ofdm::remove_drift`), on the longest bursts at the 64QAM levels.
//!
//! Both receivers see the same recording of each trial, decoded from the known burst
//! position with the sent header as channel training (as `level_eval` does). Per
//! mode, level and clock error it reports:
//! - the EVM of the last `EDGE` payload symbols, where the box average is one-sided,
//!   and of the payload symbols away from both ends, on a channel without noise (the
//!   floor is the clipping at the level's drive);
//! - the codeword error rate at the level's recorded 10 % point (`perf/levels.toml`,
//!   measured on 4-codeword bursts at 0 ppm, so a 4 s burst lands a little off 10 %).
//!
//! `cargo test --release -p vardefm-modem drift_eval -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `LEVELS` (5,6), `PPMS` (-150,-100,0,100,150),
//! `TRIALS` (bursts per point, 100), `EVM_TRIALS` (20), `NCW` (codewords per burst; 0,
//! the default, is the most a burst of the level holds).

use super::*;
use crate::detect_eval::{burst, env_list, env_modes, env_or, on_every_core, payload_seed};
use vardefm_sim::{transmit, FmChannelConfig};

/// Payload symbols at the end of a burst counted as its edge: the box average's half
/// width, over which it is one-sided.
const EDGE: usize = 4;
/// Leader length, seconds (as the other evaluations).
const TXDELAY: f32 = 0.15;
const BASELINE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/perf/levels.toml");

fn modem(mode: &str, remove_drift: bool) -> Modem {
    let mut m = Modem::new(OfdmParams::by_name(mode).unwrap(), TXDELAY);
    m.decode_threads = 1;
    m.ofdm.remove_drift = remove_drift;
    m
}

fn channel(mode: &str, cnr: Option<f32>, ppm: f32, seed: u64) -> FmChannelConfig {
    FmChannelConfig { cnr_db: cnr, clock_offset_ppm: ppm, seed, ..FmChannelConfig::by_name(mode).unwrap() }
}

/// The recorded 10 % CER point of `mode` at `level`.
fn level_point(mode: &str, level: u8) -> f32 {
    let t: toml::Value = toml::from_str(&std::fs::read_to_string(BASELINE).unwrap()).unwrap();
    t["level"]
        .as_array()
        .unwrap()
        .iter()
        .find(|p| p["mode"].as_str() == Some(mode) && p["level"].as_integer() == Some(level as i64))
        .and_then(|p| p["cnr_db"].as_float())
        .unwrap_or_else(|| panic!("no recorded point for {mode} level {level}")) as f32
}

/// What one receiver made of one burst.
#[derive(Default, Clone, Copy)]
struct Stats {
    /// Error and signal energy over the edge symbols, then over the middle ones.
    edge: (f64, f64),
    mid: (f64, f64),
    cw_errors: u32,
    cws: u32,
}

impl std::ops::AddAssign for Stats {
    fn add_assign(&mut self, o: Self) {
        self.edge = (self.edge.0 + o.edge.0, self.edge.1 + o.edge.1);
        self.mid = (self.mid.0 + o.mid.0, self.mid.1 + o.mid.1);
        self.cw_errors += o.cw_errors;
        self.cws += o.cws;
    }
}

fn evm_db((e, s): (f64, f64)) -> f64 {
    if s == 0.0 {
        f64::NAN
    } else {
        10.0 * (e / s).log10()
    }
}

/// Decode `rx` (preamble B at `b`) with `m`, against the sent `burst`.
fn decode(m: &Modem, rx: &[f32], b: usize, sent: &Burst, decode_cws: bool) -> Stats {
    let h = &sent.header;
    let hs = m.header_symbols();
    let nsym = hs + h.ncw as usize * m.symbols_per_codeword(h.level);
    let f = m.ofdm.demodulate_known(rx, b, nsym, &m.header_carriers(h)).expect("burst runs past the recording");
    let tx = m.symbols(sent);
    let mut st = Stats::default();
    for (i, (got, want)) in f.symbols.iter().zip(&tx).enumerate().skip(hs) {
        let (e, s) = got
            .iter()
            .zip(want)
            .fold((0.0, 0.0), |(e, s), (y, x)| (e + (y - x).norm_sqr() as f64, s + x.norm_sqr() as f64));
        if i + EDGE >= nsym {
            st.edge = (st.edge.0 + e, st.edge.1 + s);
        } else if i >= hs + EDGE {
            st.mid = (st.mid.0 + e, st.mid.1 + s);
        }
    }
    if decode_cws {
        let got = m.decode_payload(h, &f);
        st.cws = got.len() as u32;
        st.cw_errors = got.iter().zip(&sent.payload).filter(|(g, w)| g.as_ref() != Some(*w)).count() as u32;
    }
    st
}

/// Totals of `trials` bursts through the channel at `cnr` and `ppm`, for the box
/// average and for drift removal.
fn run(mode: &str, level: u8, ncw: usize, cnr: Option<f32>, ppm: f32, trials: u32, decode_cws: bool) -> [Stats; 2] {
    let ms = [modem(mode, false), modem(mode, true)];

    // The channel's delays do not depend on its noise or the payload.
    let clean = transmit(&channel(mode, None, ppm, 1), &ms[0].modulate(&burst(level, ncw, payload_seed(0))));
    let b = ms[0].ofdm.detect(&clean, 0).expect("burst not found on a clean channel");
    let per_trial = on_every_core(trials, |t| {
        let sent = burst(level, ncw, payload_seed(t));
        let seed = ((ppm.to_bits() as u64) << 32) | t as u64;
        let rx = transmit(&channel(mode, cnr, ppm, seed), &ms[0].modulate(&sent));
        [decode(&ms[0], &rx, b, &sent, decode_cws), decode(&ms[1], &rx, b, &sent, decode_cws)]
    });
    let mut tot = [Stats::default(); 2];
    for r in per_trial {
        tot[0] += r[0];
        tot[1] += r[1];
    }
    tot
}

#[test]
#[ignore]
fn drift_eval() {
    let trials: u32 = env_or("TRIALS", 100);
    let ncw_env: usize = env_or("NCW", 0);
    let evm_trials: u32 = env_or("EVM_TRIALS", 20);
    let levels: Vec<u8> = env_list("LEVELS", "5,6");
    let ppms: Vec<f32> = env_list("PPMS", "-150,-100,0,100,150");
    for mode in env_modes("narrow,wide") {
        for &level in &levels {
            let cnr = level_point(&mode, level);
            let max = modem(&mode, false).max_ncw(level);
            assert!(ncw_env <= max, "NCW={ncw_env}, but a {mode} level {level} burst holds at most {max}");
            let ncw = if ncw_env == 0 { max } else { ncw_env };
            println!(
                "{mode} level {level} ({}), {ncw} codewords; EVM without noise, CER at {cnr:.2} dB",
                LEVELS[level as usize].name
            );
            println!("    ppm | EVM edge box/drift dB | EVM mid box/drift dB | CER box   drift");
            for &ppm in &ppms {
                let evm = run(&mode, level, ncw, None, ppm, evm_trials, false);
                let cer = run(&mode, level, ncw, Some(cnr), ppm, trials, true);
                let rate = |s: &Stats| s.cw_errors as f64 / s.cws.max(1) as f64;
                println!(
                    "  {ppm:5.0} | {:7.2} {:7.2}       | {:7.2} {:7.2}      | {:.4} {:.4}  ({} cws)",
                    evm_db(evm[0].edge),
                    evm_db(evm[1].edge),
                    evm_db(evm[0].mid),
                    evm_db(evm[1].mid),
                    rate(&cer[0]),
                    rate(&cer[1]),
                    cer[0].cws
                );
            }
        }
    }
}
