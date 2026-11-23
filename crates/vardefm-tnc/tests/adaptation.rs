//! Does the link pick the right speed level on a channel with interference other
//! than noise (`vardefm_sim::interference`)?
//!
//! Per mode and scenario, station A sends data to B for a fixed time, once with the
//! level capped at each of 0 to 6 (`LinkConfig::max_level`). The cap at 6 leaves the
//! link's rate adaptation free. If a lower cap carries more, the adaptation asked
//! for levels the channel does not carry: the reported SNR, which it goes by, did
//! not show the interference.
//!
//! The table gives the free link's goodput over the best lower cap's
//! (`free/lower`): below 1, a lower cap carried more. The best of six caps is picked
//! after the fact, so run-to-run spread alone pulls the ratio down a little; the
//! lowest and highest run of both are printed beside it, and a ratio whose two
//! ranges overlap says nothing.
//!
//! What this does not show: a cap is a ceiling, not a fixed level, so a link that
//! picks too low a level looks the same as one that picks right. Compare with the
//! codeword loss per level from `interference_sweep` in vardefm-modem.
//!
//! Goodput is not decoding alone. Each station hears its audio interference all the
//! time, so its busy detector can hold a transmission off because of it; RF
//! interference exists only during transmissions (see `vardefm_sim::interference`).
//! Nothing is gated.
//!
//! `cargo test --release -p vardefm-tnc --test adaptation adaptation_gap -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `CNR` (dB, 30), `SECONDS` (of transfer per
//! run, simulated, 120), `SEEDS` (runs per point, 4), `INTERFERENCE` (scenarios
//! separated by commas, each a colon-separated list of interferers as in
//! `vardefm_sim::parse_interference`; unset, [`default_scenarios`]).

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;
use vardefm_dsp::prng::Prng;
use vardefm_modem::{Modem, OfdmParams, LEVELS};
use vardefm_sim::{parse_interference, Interferer};
use vardefm_tnc::simpair::{Headless, SimPairConfig};
use vardefm_tnc::station::HostOut;

/// Simulated seconds a connection may take before a run counts as not connected.
const CONNECT_LIMIT_S: f64 = 60.0;

/// The scenarios run when `INTERFERENCE` is unset: a tone on a data carrier midway
/// between two pilots near the middle of the band, and pops and RF bursts, short
/// and long. A subset of `interference_sweep`'s.
fn default_scenarios(p: &OfdmParams) -> Vec<String> {
    let pilot = p.num_carriers / 2 / p.pilot_spacing * p.pilot_spacing;
    let on_data = p.carrier_freq(pilot + p.pilot_spacing / 2);
    let mut out = vec![String::new(), format!("tone={on_data}@-30"), format!("tone={on_data}@-20")];
    out.extend(
        ["pop=5@2@0", "pop=1@20@0", "pop=5@20@0", "rfburst=5@2@-10", "rfburst=1@20@-10", "rfburst=5@20@-10"]
            .map(String::from),
    );
    out
}

/// What one session carried.
#[derive(Clone, Debug, Default, PartialEq)]
struct Run {
    /// Bits per second delivered to B's host over the transfer time; 0 when the
    /// stations did not connect.
    goodput: f64,
    /// Codewords sent at each level during the transfer.
    codewords: Vec<u64>,
    /// The session ended before the time was up.
    dropped: bool,
}

/// One session: A connects to B and sends for `seconds`, with levels up to
/// `max_level`, on the mode's own channel at `cnr` dB with `interference`.
fn run(mode: &str, cnr: f32, interference: &[Interferer], max_level: u8, seed: u64, seconds: f64) -> Run {
    let params = OfdmParams::by_name(mode).unwrap();
    let cfg = SimPairConfig {
        mode: mode.into(),
        cnr_db: Some(cnr),
        seed,
        verbose: false,
        interference: interference.to_vec(),
        max_level,
        ..SimPairConfig::default()
    };
    // More than the fastest level carries in the time, so A never runs dry.
    let fastest = Modem::new(params, cfg.txdelay_s).net_rate(LEVELS.len() as u8 - 1) as f64;
    let mut rng = Prng::new(seed as u32 + 1);
    let data: Vec<u8> = (0..(fastest * seconds / 8.0 * 1.5) as usize + 1000).map(|_| rng.next_u32() as u8).collect();
    let mut sim = Headless::new(cfg).unwrap();
    let b = sim.stations() - 1;
    for (station, line) in [(b, "MYCALL LA2BBB"), (b, "LISTEN ON"), (0, "MYCALL LA1AAA"), (0, "CONNECT LA1AAA LA2BBB")]
    {
        sim.host_line(station, line);
    }
    let said = |o: &HostOut, what: &str| matches!(o, HostOut::Line(l) if l.starts_with(what));
    let mut connected = false;
    while !connected && sim.now() < CONNECT_LIMIT_S {
        connected = sim.step().iter().any(|(i, o)| *i == 0 && said(o, "CONNECTED"));
    }
    if !connected {
        return Run { dropped: true, codewords: vec![0; LEVELS.len()], ..Run::default() };
    }
    sim.host_data(0, &data);
    let (start, before) = (sim.now(), sim.stats().codewords_by_level);
    let (mut bytes, mut dropped) = (0usize, false);
    while !dropped && sim.now() - start < seconds {
        for (i, o) in sim.step() {
            match o {
                HostOut::Data(d) if i == b => bytes += d.len(),
                o if said(&o, "DISCONNECTED") => dropped = true,
                _ => {}
            }
        }
    }
    let after = sim.stats().codewords_by_level;
    Run {
        goodput: bytes as f64 * 8.0 / seconds,
        codewords: after.iter().zip(&before).map(|(a, b)| a - b).collect(),
        dropped,
    }
}

/// `f(job)` for every job, on every core, in job order.
fn in_parallel<R: Send>(jobs: usize, f: impl Fn(usize) -> R + Sync) -> Vec<R> {
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()).min(jobs.max(1));
    let next = AtomicUsize::new(0);
    let out: Mutex<Vec<Option<R>>> = Mutex::new((0..jobs).map(|_| None).collect());
    std::thread::scope(|s| {
        for _ in 0..threads {
            s.spawn(|| loop {
                let j = next.fetch_add(1, Ordering::Relaxed);
                if j >= jobs {
                    break;
                }
                let r = f(j);
                out.lock().unwrap()[j] = Some(r);
            });
        }
    });
    out.into_inner().unwrap().into_iter().map(|r| r.expect("job ran")).collect()
}

fn env_or<T: std::str::FromStr>(name: &str, default: T) -> T {
    match std::env::var(name) {
        Ok(v) => v.trim().parse().unwrap_or_else(|_| panic!("{name}={v:?} does not parse")),
        Err(_) => default,
    }
}

fn env_list(name: &str, default: &str) -> Vec<String> {
    let v = std::env::var(name).unwrap_or_else(|_| default.to_string());
    v.split(',').map(|s| s.trim().to_string()).collect()
}

/// A scenario: its spec, for output, and its interferers.
type Scenario = (String, Vec<Interferer>);

/// Lowest and highest of `runs`' goodput, as `low-high`.
fn range(runs: &[Run]) -> String {
    let low = runs.iter().map(|r| r.goodput).fold(f64::INFINITY, f64::min);
    let high = runs.iter().map(|r| r.goodput).fold(0.0, f64::max);
    format!("{low:.0}-{high:.0}")
}

#[test]
#[ignore]
fn adaptation_gap() {
    let cnr: f32 = env_or("CNR", 30.0);
    let seconds: f64 = env_or("SECONDS", 120.0);
    let seeds: usize = env_or("SEEDS", 4);
    assert!(seeds > 0, "SEEDS must be at least 1");
    let given = std::env::var("INTERFERENCE").ok().map(|_| env_list("INTERFERENCE", ""));
    let caps = LEVELS.len();
    // Every mode and scenario is checked before anything runs.
    let plan: Vec<(String, Vec<Scenario>)> = env_list("MODES", "narrow,wide")
        .into_iter()
        .map(|mode| {
            let params = OfdmParams::by_name(&mode).unwrap_or_else(|| panic!("MODES: unknown mode {mode:?}"));
            let specs = given.clone().unwrap_or_else(|| default_scenarios(&params));
            let parse = |s: &String| parse_interference(s, params.fs).unwrap_or_else(|e| panic!("INTERFERENCE: {e}"));
            let scenarios = specs.iter().map(|s| (s.clone(), parse(s))).collect();
            (mode, scenarios)
        })
        .collect();
    for (mode, scenarios) in &plan {
        // Jobs in scenario, cap, seed order.
        let per_scenario = caps * seeds;
        let runs = in_parallel(scenarios.len() * per_scenario, |j| {
            let (s, rest) = (j / per_scenario, j % per_scenario);
            run(mode, cnr, &scenarios[s].1, (rest / seeds) as u8, (rest % seeds) as u64 + 1, seconds)
        });
        let width = scenarios.iter().map(|(s, _)| s.len()).max().unwrap_or(0).max(8);
        println!("\n{mode}, CNR {cnr} dB, {seconds} s of one-way transfer, mean of {seeds} runs.");
        println!("Goodput in bit/s with the level capped; the cap at 6 leaves the link free. * = a session dropped.");
        println!("free/lower: the free link over the best lower cap; then the lowest-highest run of each.");
        print!("{:<width$}", "scenario");
        (0..caps).for_each(|c| print!(" {:>7}", format!("<=L{c}")));
        println!(" | free/lower | {:<34} | codewords per level when free", "runs: free, best lower cap");
        for (s, (spec, _)) in scenarios.iter().enumerate() {
            let of_cap = |c: usize| &runs[s * per_scenario + c * seeds..][..seeds];
            let goodput: Vec<f64> =
                (0..caps).map(|c| of_cap(c).iter().map(|r| r.goodput).sum::<f64>() / seeds as f64).collect();
            print!("{:<width$}", if spec.is_empty() { "none" } else { spec });
            for (c, g) in goodput.iter().enumerate() {
                let mark = if of_cap(c).iter().any(|r| r.dropped) { "*" } else { "" };
                print!(" {:>7}", format!("{g:.0}{mark}"));
            }
            let free = caps - 1;
            let lower = (0..free).rev().max_by(|&x, &y| goodput[x].total_cmp(&goodput[y])).unwrap();
            let ratio =
                if goodput[lower] > 0.0 { format!("{:.2}", goodput[free] / goodput[lower]) } else { "-".into() };
            let spread = format!("{}, <=L{lower} {}", range(of_cap(free)), range(of_cap(lower)));
            let mut used = vec![0u64; caps];
            for r in of_cap(free) {
                used.iter_mut().zip(&r.codewords).for_each(|(u, c)| *u += c);
            }
            let total = used.iter().sum::<u64>().max(1);
            let mix: Vec<String> = used
                .iter()
                .enumerate()
                .filter(|(_, &n)| n > 0)
                .map(|(l, &n)| format!("L{l} {:.0}%", 100.0 * n as f64 / total as f64))
                .collect();
            println!(" | {ratio:>10} | {spread:<34} | {}", mix.join(", "));
        }
    }
}

/// A headless session carries data, repeats exactly, and obeys the level cap.
#[test]
fn headless_session_repeats_and_obeys_the_cap() {
    let free = run("narrow", 30.0, &[], 6, 1, 20.0);
    assert!(free.goodput > 1000.0 && !free.dropped, "{free:?}");
    assert_eq!(run("narrow", 30.0, &[], 6, 1, 20.0), free);
    let capped = run("narrow", 30.0, &[], 2, 1, 20.0);
    assert!(
        capped.codewords[3..].iter().all(|&n| n == 0) && capped.codewords[..3].iter().sum::<u64>() > 0,
        "{capped:?}"
    );
    assert!(capped.goodput > 0.0 && capped.goodput < free.goodput, "capped {capped:?}, free {free:?}");
    // A cap below the level a session starts at holds from the first burst.
    let lowest = run("narrow", 30.0, &[], 0, 1, 20.0);
    assert!(lowest.codewords[1..].iter().all(|&n| n == 0) && lowest.codewords[0] > 0, "{lowest:?}");
}

/// Audio interference is each station's own and continuous: a session under a tone
/// too weak to cost codewords still repeats exactly and carries data.
#[test]
fn session_under_audio_interference_repeats() {
    let tone = parse_interference("tone=1000@-40:pop=1@2@-20", 48e3).unwrap();
    let first = run("narrow", 30.0, &tone, 6, 3, 20.0);
    assert!(first.goodput > 1000.0 && !first.dropped, "{first:?}");
    assert_eq!(run("narrow", 30.0, &tone, 6, 3, 20.0), first);
}

/// Interference reaches a session: heavy RF bursts cost goodput.
#[test]
fn heavy_interference_costs_goodput() {
    let clean = run("narrow", 30.0, &[], 6, 2, 20.0);
    let hit = run("narrow", 30.0, &parse_interference("rfburst=10@20@-20", 48e3).unwrap(), 6, 2, 20.0);
    assert!(hit.goodput < 0.7 * clean.goodput, "clean {clean:?}, hit {hit:?}");
}
