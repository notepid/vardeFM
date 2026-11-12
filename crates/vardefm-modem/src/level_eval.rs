//! Decoding performance per mode and speed level, against a recorded baseline.
//!
//! Two error rates through the FM channel simulator, both from the known burst
//! position, so detection is left out (the detector evaluation, `detect_eval`,
//! covers it):
//! - per level, the codeword error rate (CER), with the sent header as channel
//!   training, as `Receiver::step` decodes the payload once the header has decoded;
//! - per mode and drive, the header error rate (HER): the header decoded from
//!   preamble B alone, as `Receiver::step` does first. The header is coded and
//!   modulated the same at every level, but `Modem::modulate` scales the whole
//!   burst to its level's drive (`Modem::tx_rms`), so the header of a high level
//!   goes out quieter. One point per mode and drive covers it, measured on
//!   one-codeword bursts of the lowest level sent at that drive.
//!
//! Why one point per drive suffices: every symbol of a burst carries the same mean
//! power before scaling, so the header's level depends on the drive and hardly on
//! the burst's length or payload. Clipping, heavy at wide mode's 0.5 drive, does
//! depend on the header codeword: with one fixed header the useful level at that
//! drive varied by 0.3 dB between levels and codeword counts. So each header trial
//! draws the fields that change from burst to burst at random (`trial_header`), and
//! averaged over those draws the effective level agrees within
//! `HEADER_LEVEL_TOLERANCE_DB` at every drive; `header_level_depends_only_on_drive`
//! checks it.
//!
//! A burst decodes only if both its header and its codewords do. At level 0's drive
//! the header's point lies above level 0's, so there the header limits a burst, and
//! its curve is far shallower than any level's: at level 1's point a few per cent of
//! headers still fail.
//!
//! What the gate does not cover:
//! - The receiver's SNR estimate, which the link's rate thresholds
//!   (`SNR_THRESHOLDS_DB`) are compared against. `level_table` reports it.
//! - Long bursts and clock drift: bursts hold up to `CODEWORDS` codewords and the
//!   channel presets have no clock offset, so pilot tracking over a long burst is
//!   not exercised.
//!
//! `perf/levels.toml` holds, per mode and drive, the CNR at which the HER crosses
//! 10 % and, per mode and level, the CNR at which the CER does. `level_gate` fails
//! when a point has moved by more than its margin (`GATE_MARGIN_DB`, `HEADER_MARGIN_DB` for the
//! shallower header curve): the rate must be below 10 % that far above the point and
//! above 10 % that far below. Each point decodes `GATE_BURSTS` bursts
//! of up to `CODEWORDS` codewords, or `HEADERS_PER_BURST` times as many headers.
//! Results do not depend on the number of threads. They may differ in the last bits
//! between platforms whose maths libraries round `ln`, `cos` or `powf` differently,
//! which moves points near the margin.
//!
//! `cargo test --release -p vardefm-modem level_gate -- --ignored --nocapture`
//!
//! `level_sweep` measures the points again; with `WRITE=1` it records them, and the
//! commit and command, in `perf/levels.toml`. Commit the code first, so the recorded
//! commit is the one measured.
//!
//! `WRITE=1 cargo test --release -p vardefm-modem level_sweep -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `LEVELS` (0,1,..,6; `h` is the header at
//! every drive), `TRIALS` (bursts per point: 100 for the gate, 200 for the sweep's
//! fine grid), `WRITE` (sweep only).

use super::*;
use crate::detect_eval::{
    burst, env_flag, env_list, env_modes, env_or, header_at, modem, on_every_core, payload_seed, received,
    through_channel, true_position,
};
use vardefm_dsp::prng::Prng;

/// Error rate at which a point is recorded and gated.
const TARGET_RATE: f32 = 0.1;
/// The gate measures this far either side of a level's recorded point, dB.
const GATE_MARGIN_DB: f32 = 0.25;
/// ... and this far for the header. Its curve falls only by about a factor of 1.5
/// per 0.25 dB around the point, so at `GATE_MARGIN_DB` the gate's two rates would sit
/// within 3 sigma of 10 %.
const HEADER_MARGIN_DB: f32 = 0.5;
/// Bursts per point of the gate. With 4 codewords per burst (2 at narrow level 0),
/// the binomial sigma at 10 % CER is 1.5 % (2.1 %). That takes codewords as
/// independent, which they are not quite: the interleaver spreads each codeword
/// over the whole burst, so a click hits all of them, and they share one channel
/// estimate. The real spread is somewhat larger; the recorded curves clear the
/// margin by far more than that.
const GATE_BURSTS: u32 = 100;
/// Bursts per point of the sweep's fine grid.
const RECORD_BURSTS: u32 = 200;
/// Bursts per point of the sweep's coarse search for the crossing.
const COARSE_BURSTS: u32 = 25;
/// Half-width of the sweep's fine grid in 0.25 dB steps: ±1 dB.
const FINE_STEPS: i32 = 4;
/// Times the sweep may move its fine grid before it gives up.
const MAX_MOVES: u32 = 4;
/// The sweep's search gives up above this CNR, dB: a level that still fails there
/// has an error floor.
const MAX_CNR_DB: i32 = 40;
/// ... and below this one, where nothing decodes.
const MIN_CNR_DB: i32 = -10;
/// The modes and targets a recorded baseline covers.
const ALL_MODES: &str = "narrow,wide";
const ALL_TARGETS: &str = "h,0,1,2,3,4,5,6";
/// Codewords per burst, or fewer where a level's burst holds fewer (2 at narrow
/// level 0).
const CODEWORDS: usize = 4;
/// Headers measured for each burst of a level. The header's shallow curve needs
/// more decisions than a level's: 1600 per gate point put sigma at 0.75 %. Header
/// trials send bursts of one codeword, so they are cheap.
const HEADERS_PER_BURST: u32 = 16;

/// Seed of the header fields that header trials draw, mixed with the trial.
const HEADER_SEED: u32 = 0x4ead_0001;

/// The recorded points, `perf/levels.toml`.
const BASELINE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/perf/levels.toml");

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct Baseline {
    source: Source,
    /// One header point per mode and drive.
    header: Vec<Point>,
    /// One codeword point per mode and level.
    level: Vec<Point>,
}

/// How a baseline was made, so a change to it can be reproduced.
#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct Source {
    what: String,
    commit: String,
    command: String,
    /// Bursts per point of the 1 dB search.
    coarse_bursts: u32,
    /// Bursts per point of the recorded grid, `TRIALS`.
    bursts: u32,
    /// `CODEWORDS`: codewords per burst at most.
    codewords: usize,
    /// `HEADERS_PER_BURST`.
    headers_per_burst: u32,
}

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct Point {
    mode: String,
    /// The level; for the header, the lowest level sent at its drive.
    level: u8,
    name: String,
    /// The level's drive, `Modem::tx_rms`.
    drive: f32,
    /// CNR at which the error rate crosses `TARGET_RATE`, dB.
    cnr_db: f32,
    /// The fine grid the crossing comes from, for review in diffs.
    grid_db: Vec<f32>,
    rate: Vec<f32>,
}

/// An item of `LEVELS`: the header at every drive, or the codewords of a level.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Spec {
    Header,
    Level(u8),
}

impl std::str::FromStr for Spec {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s == "h" {
            return Ok(Spec::Header);
        }
        let last = LEVELS.len() - 1;
        match s.parse() {
            Ok(l) if (l as usize) <= last => Ok(Spec::Level(l)),
            _ => Err(format!("levels are 0 to {last}, or h for the header")),
        }
    }
}

/// The levels that are the lowest sent at their drive: the header is measured once
/// per drive, on bursts of that level.
fn drive_levels(m: &Modem) -> Vec<u8> {
    let levels = 0..LEVELS.len() as u8;
    levels.filter(|&l| (0..l).all(|j| m.tx_rms(j) != m.tx_rms(l))).collect()
}

/// What a point measures: the header at the drive of a level, or the codewords of
/// a level.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Target {
    Header(u8),
    Level(u8),
}

impl Target {
    /// The points `specs` stand for in mode `m`, in order.
    fn all(m: &Modem, specs: &[Spec]) -> Vec<Target> {
        let one = |s: &Spec| match *s {
            Spec::Header => drive_levels(m).into_iter().map(Target::Header).collect(),
            Spec::Level(l) => vec![Target::Level(l)],
        };
        specs.iter().flat_map(one).collect()
    }

    /// The level of the bursts sent.
    fn level(self) -> u8 {
        match self {
            Target::Header(l) | Target::Level(l) => l,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Target::Header(_) => "header",
            Target::Level(l) => LEVELS[l as usize].name,
        }
    }

    /// How far either side of the recorded point the gate measures, dB.
    fn margin(self) -> f32 {
        match self {
            Target::Header(_) => HEADER_MARGIN_DB,
            Target::Level(_) => GATE_MARGIN_DB,
        }
    }

    /// `HER` or `CER`, for output.
    fn rate_name(self) -> &'static str {
        match self {
            Target::Header(_) => "HER",
            Target::Level(_) => "CER",
        }
    }

    fn label(self, mode: &str) -> String {
        match self {
            Target::Header(l) => format!("{mode} header at L{l} drive"),
            Target::Level(l) => format!("{mode} L{l} {}", self.name()),
        }
    }
}

fn load() -> Baseline {
    let text = std::fs::read_to_string(BASELINE).unwrap_or_else(|e| panic!("{BASELINE}: {e}"));
    toml::from_str(&text).unwrap_or_else(|e| panic!("{BASELINE}: {e}"))
}

impl Baseline {
    fn point(&self, mode: &str, target: Target) -> &Point {
        let list = match target {
            Target::Header(_) => &self.header,
            Target::Level(_) => &self.level,
        };
        let p = list.iter().find(|p| p.mode == mode && p.level == target.level());
        p.unwrap_or_else(|| panic!("{BASELINE} has no {}", target.label(mode)))
    }
}

/// One mode and target under test, with what all its trials share.
struct Case<'a> {
    m: &'a Modem,
    mode: &'a str,
    target: Target,
    /// The level of the bursts sent.
    level: u8,
    /// Codewords per burst: `CODEWORDS`, or fewer where the level's burst holds
    /// fewer; 1 for the header, whose bursts are kept short.
    ncw: usize,
    /// Where preamble B lands, the same in every trial.
    truth: usize,
    /// The header's carriers, the channel training of a level's codewords; every
    /// trial sends the same header. Empty for the header itself.
    training: Vec<Vec<Complex32>>,
}

impl<'a> Case<'a> {
    fn new(m: &'a Modem, mode: &'a str, target: Target) -> Self {
        let level = target.level();
        let ncw = match target {
            Target::Header(_) => 1,
            Target::Level(l) => m.max_ncw(l).min(CODEWORDS),
        };
        let truth = true_position(m, mode, level, ncw);
        let training = match target {
            // The header is decoded without training.
            Target::Header(_) => Vec::new(),
            Target::Level(_) => m.header_carriers(&burst(level, ncw, payload_seed(0)).header),
        };
        Case { m, mode, target, level, ncw, truth, training }
    }

    /// Failed decisions of trial `t` at `cnr` dB, from the true position: whether
    /// the header fails, or how many codewords do.
    fn errors(&self, cnr: f32, t: u32) -> u32 {
        let (m, ncw) = (self.m, self.ncw);
        let (sent, rx) = received(m, self.mode, cnr, self.level, ncw, t);
        if let Target::Header(_) = self.target {
            let sent = Burst { header: trial_header(self.level, ncw, t), ..sent };
            let rx = through_channel(m, self.mode, cnr, &sent, t);
            return u32::from(header_at(m, &rx, self.truth) != Some(sent.header));
        }
        let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(self.level);
        let Some(frame) = m.ofdm.demodulate_known(&rx, self.truth, nsym, &self.training) else {
            return ncw as u32;
        };
        let got = m.decode_payload(&sent.header, &frame);
        got.iter().zip(&sent.payload).filter(|(g, w)| g.as_ref() != Some(*w)).count() as u32
    }

    /// Trials and decisions for `bursts` bursts' worth: one per codeword of a level's
    /// burst, `HEADERS_PER_BURST` headers for each burst of the header.
    fn trials(&self, bursts: u32) -> (u32, usize) {
        match self.target {
            Target::Header(_) => (bursts * HEADERS_PER_BURST, (bursts * HEADERS_PER_BURST) as usize),
            Target::Level(_) => (bursts, bursts as usize * self.ncw),
        }
    }

    /// Error rate over `bursts` bursts' worth of trials at `cnr` dB, on every core.
    fn rate(&self, cnr: f32, bursts: u32) -> f32 {
        let (trials, decisions) = self.trials(bursts);
        let errors: u32 = on_every_core(trials, |t| self.errors(cnr, t)).iter().sum();
        errors as f32 / decisions as f32
    }
}

/// The header that header trial `t` sends: a data burst of `ncw` codewords at
/// `level`, with the fields that change from burst to burst in a session drawn at
/// random. Clipping depends on the header codeword, so a fixed header would measure
/// one codeword rather than the drive (see the module doc).
fn trial_header(level: u8, ncw: usize, t: u32) -> Header {
    let mut rng = Prng::new(HEADER_SEED ^ t.wrapping_mul(2));
    let mut field = |bits: u32| rng.next_u32() >> (32 - bits);
    Header {
        kind: Kind::Data,
        session: field(8) as u8,
        level,
        ncw: ncw as u8,
        req_level: (field(16) % LEVELS.len() as u32) as u8,
        burst_id: field(3) as u8,
        ack_burst: field(3) as u8,
        ack_offset: field(24),
        bitmap: field(24),
        flags: field(1) as u8 * FLAG_POLL,
    }
}

/// Where `rates` (falling with `grid`) cross `target`, by interpolation of log10 of
/// the rate between the bracketing points, or linearly when one of them is 0.
/// `None` when the grid does not bracket the target. A waterfall bends down in log
/// scale, so the chord lies below it and the crossing comes out slightly low; on
/// the recorded grids the bias is well inside the gate's margin.
fn crossing(grid: &[f32], rates: &[f32], target: f32) -> Option<f32> {
    let i = rates.windows(2).position(|w| w[0] >= target && w[1] < target)?;
    let (x0, x1, y0, y1) = (grid[i], grid[i + 1], rates[i], rates[i + 1]);
    let t =
        if y1 > 0.0 { (y0.log10() - target.log10()) / (y0.log10() - y1.log10()) } else { (y0 - target) / (y0 - y1) };
    Some(x0 + t * (x1 - x0))
}

/// The rate at `x` on a grid, interpolated in log10 between the neighbouring points
/// as in [`crossing`], or linearly next to a 0; clamped to the ends of the grid.
fn rate_at(grid: &[f32], rates: &[f32], x: f32) -> f32 {
    let Some(i) = grid.windows(2).position(|w| x <= w[1]) else {
        return rates[rates.len() - 1];
    };
    let (x0, x1, y0, y1) = (grid[i], grid[i + 1], rates[i], rates[i + 1]);
    let t = ((x - x0) / (x1 - x0)).clamp(0.0, 1.0);
    if y0 > 0.0 && y1 > 0.0 {
        10f32.powf(y0.log10() + t * (y1.log10() - y0.log10()))
    } else {
        y0 + t * (y1 - y0)
    }
}

/// The commit of the working tree, marked when tracked files have uncommitted
/// changes other than recorded baselines; untracked files do not count. "unknown"
/// without git or outside a repository.
fn commit() -> String {
    let git = |args: &[&str]| {
        std::process::Command::new("git")
            .args(args)
            .current_dir(env!("CARGO_MANIFEST_DIR"))
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
    };
    let Some(head) = git(&["rev-parse", "--short", "HEAD"]) else {
        return "unknown".into();
    };
    if git(&["status", "--porcelain", "--untracked-files=no", "--", ":/", ":(top,exclude,glob)**/perf/*.toml"])
        .is_some_and(|s| s.is_empty())
    {
        head
    } else {
        format!("{head}-dirty")
    }
}

/// Measures the 10 % point of `case`: a 1 dB search for the crossing from `start`,
/// then a 0.25 dB grid of ±1 dB around it, moved until it crosses 10 % with the
/// gate's two points on it. Returns the point and the whole dB the
/// search ended at. Warns when the curve is too flat for the gate to tell a shift of
/// the target's margin from noise.
fn sweep(case: &Case, start: i32, bursts: u32) -> (Point, i32) {
    let label = case.target.label(case.mode);
    let rate_name = case.target.rate_name();
    let mut seen = std::collections::BTreeMap::new();
    let mut at = |c: i32| *seen.entry(c).or_insert_with(|| case.rate(c as f32, COARSE_BURSTS));
    // c ends as the last whole dB at which the rate is still 10 % or more.
    let mut c = start;
    while at(c) < TARGET_RATE {
        assert!(c > MIN_CNR_DB, "{label}: {rate_name} is below 10 % even at {c} dB");
        c -= 1;
    }
    while at(c + 1) >= TARGET_RATE {
        c += 1;
        assert!(c < MAX_CNR_DB, "{label}: {rate_name} is still 10 % or more at {MAX_CNR_DB} dB");
    }
    // Whole quarter dB steps, so the recorded grid reads as written. The search
    // rests on few bursts, so its crossing can be off by more than the grid allows:
    // the grid is moved to its own crossing, or by its width when it does not cross,
    // until it carries the gate. A grid that cannot stops the run here, not after
    // the whole sweep.
    let margin = case.target.margin();
    let mut centre = c * 4 + 2;
    let mut moves = 0;
    let (grid, rates, cnr_db) = loop {
        let grid: Vec<f32> = (centre - FINE_STEPS..=centre + FINE_STEPS).map(|i| i as f32 / 4.0).collect();
        let rates: Vec<f32> = grid.iter().map(|&g| case.rate(g, bursts)).collect();
        let next = match crossing(&grid, &rates, TARGET_RATE).map(|x| (x * 100.0).round() / 100.0) {
            Some(x) if grid[0] <= x - margin && x + margin <= grid[grid.len() - 1] => break (grid, rates, x),
            Some(x) => (x * 4.0).round() as i32,
            None if rates[0] < TARGET_RATE => centre - FINE_STEPS,
            None => centre + FINE_STEPS,
        };
        assert!(
            moves < MAX_MOVES && next != centre,
            "{label}: the fine grid {grid:?} does not carry the gate: {rate_name} {rates:?}"
        );
        println!("  {label}: fine grid moved, {:.2} -> {:.2} dB", centre as f32 / 4.0, next as f32 / 4.0);
        centre = next;
        moves += 1;
    };

    let line: String = grid.iter().zip(&rates).map(|(g, r)| format!(" {g:.2}:{r:.3}")).collect();
    println!("{label}: 10 % {rate_name} at {cnr_db:.2} dB;{line}");
    // The rate the gate would see on either side, interpolated on the grid, against
    // the binomial sigma, which is somewhat optimistic (see `GATE_BURSTS`).
    let decisions = case.trials(GATE_BURSTS).1;
    let sigma = (TARGET_RATE * (1.0 - TARGET_RATE) / decisions as f32).sqrt();
    for d in [-margin, margin] {
        let r = rate_at(&grid, &rates, cnr_db + d);
        if (r - TARGET_RATE).abs() < 3.0 * sigma {
            println!("  WARNING: {rate_name} {r:.3} at {:.2} dB is within 3 sigma of 10 %", cnr_db + d);
        }
    }
    let p = Point {
        mode: case.mode.into(),
        level: case.level,
        name: case.target.name().into(),
        drive: case.m.tx_rms(case.level),
        cnr_db,
        grid_db: grid,
        rate: rates,
    };
    (p, c)
}

/// Measures the header point of every mode and drive and the codeword point of
/// every mode and level; with `WRITE=1`, records them in `perf/levels.toml`.
#[test]
#[ignore]
fn level_sweep() {
    let modes = env_modes(ALL_MODES);
    let specs: Vec<Spec> = env_list("LEVELS", ALL_TARGETS);
    let bursts = env_or("TRIALS", RECORD_BURSTS);
    assert!(bursts > 0, "TRIALS must be at least 1");
    let write = env_flag("WRITE");
    // The tree as measured, not as it is when the run ends.
    let commit = commit();
    // The file holds every point from one run; a partial run would drop the rest.
    assert!(
        !write || (std::env::var("MODES").is_err() && std::env::var("LEVELS").is_err()),
        "WRITE=1 records every mode and level: unset MODES and LEVELS"
    );
    let (mut header, mut level) = (Vec::new(), Vec::new());
    for mode in &modes {
        let m = modem(mode);
        // Crossings mostly rise with the level; start each search at the last of its
        // kind.
        let (mut header_start, mut level_start) = (0i32, 0i32);
        for target in Target::all(&m, &specs) {
            let case = Case::new(&m, mode, target);
            let (list, start) = match target {
                Target::Header(_) => (&mut header, &mut header_start),
                Target::Level(_) => (&mut level, &mut level_start),
            };
            let (p, c) = sweep(&case, *start, bursts);
            list.push(p);
            *start = c;
        }
    }
    if write {
        // A grid that cannot carry the gate stops the run instead of reaching the file.
        for (points, margin) in [(&header, HEADER_MARGIN_DB), (&level, GATE_MARGIN_DB)] {
            for p in points {
                let at = format!("{} L{} {}", p.mode, p.level, p.name);
                check_point(&at, &p.grid_db, &p.rate, p.cnr_db, TARGET_RATE, margin);
            }
        }
        let b = Baseline {
            source: Source {
                what: "CNR at 10 % header and codeword error rate, decoded from the known burst position".into(),
                commit,
                command: format!(
                    "WRITE=1 TRIALS={bursts} cargo test --release -p vardefm-modem level_sweep -- --ignored"
                ),
                coarse_bursts: COARSE_BURSTS,
                bursts,
                codewords: CODEWORDS,
                headers_per_burst: HEADERS_PER_BURST,
            },
            header,
            level,
        };
        std::fs::write(BASELINE, toml::to_string(&b).unwrap()).unwrap();
        println!("wrote {BASELINE}");
    }
}

/// Fails when a header or level point has moved from `perf/levels.toml` by more
/// than its margin.
#[test]
#[ignore]
fn level_gate() {
    let modes = env_modes(ALL_MODES);
    let specs: Vec<Spec> = env_list("LEVELS", ALL_TARGETS);
    let bursts = env_or("TRIALS", GATE_BURSTS);
    assert!(bursts > 0, "TRIALS must be at least 1");
    let base = load();
    println!("baseline {} ({})", base.source.commit, base.source.command);
    let mut failed = Vec::new();
    for mode in &modes {
        let m = modem(mode);
        for target in Target::all(&m, &specs) {
            let p = base.point(mode, target);
            let case = Case::new(&m, mode, target);
            let margin = target.margin();
            let below = case.rate(p.cnr_db - margin, bursts);
            let above = case.rate(p.cnr_db + margin, bursts);
            let ok = above < TARGET_RATE && below > TARGET_RATE;
            println!(
                "{} at {:.2} dB: {} {below:.3} at -{margin} dB, {above:.3} at +{margin} dB {}",
                target.label(mode),
                p.cnr_db,
                target.rate_name(),
                if ok { "ok" } else { "FAIL" }
            );
            if !ok {
                failed.push(target.label(mode));
            }
        }
    }
    assert!(failed.is_empty(), "10 % point moved for {failed:?}");
}

#[test]
fn crossing_interpolates_in_log() {
    let grid = [1.0, 1.5, 2.0];
    assert_eq!(crossing(&grid, &[0.5, 0.1, 0.01], 0.1), Some(1.5));
    let x = crossing(&grid, &[1.0, 0.1, 0.01], 0.0316).unwrap();
    assert!((x - 1.75).abs() < 1e-3, "{x}");
    assert_eq!(crossing(&grid, &[0.3, 0.2, 0.0], 0.1), Some(1.75));
    assert_eq!(crossing(&grid, &[0.05, 0.02, 0.0], 0.1), None);
    // A rate exactly at the target, as a measured one can be, is not yet below it.
    assert_eq!(crossing(&[1.0, 1.5, 2.0, 2.5], &[0.2, 0.1, 0.1, 0.05], 0.1), Some(2.0));
}

#[test]
fn rate_at_interpolates_in_log_and_clamps() {
    let (grid, rates) = ([1.0, 2.0, 3.0], [0.1, 0.001, 0.0]);
    assert!((rate_at(&grid, &rates, 1.5) - 0.01).abs() < 1e-5);
    assert!((rate_at(&grid, &rates, 2.5) - 0.0005).abs() < 1e-7);
    assert_eq!(rate_at(&grid, &rates, 0.0), 0.1);
    assert_eq!(rate_at(&grid, &rates, 9.0), 0.0);
}

#[test]
fn targets_parse() {
    let s: Vec<Spec> = "h, 0,6".split(',').map(|s| s.trim().parse().unwrap()).collect();
    assert_eq!(s, [Spec::Header, Spec::Level(0), Spec::Level(6)]);
    assert!("x".parse::<Spec>().is_err());
    assert!(LEVELS.len().to_string().parse::<Spec>().is_err());
}

/// How far the header's effective level may differ, at one drive, between the
/// bursts the gate measures and any other level or codeword count, dB. Well inside
/// `HEADER_MARGIN_DB`.
const HEADER_LEVEL_TOLERANCE_DB: f32 = 0.1;
/// Header trials averaged per level and codeword count by
/// `header_level_depends_only_on_drive`.
const HEADER_LEVEL_TRIALS: u32 = 32;
/// SNR at which clipping distortion is weighed against channel noise, dB. The
/// receiver's estimate on the header at the recorded header points is about -2 dB
/// (median) and at most about 1 dB (90th percentile); a higher value weighs the
/// distortion more.
const HEADER_REFERENCE_SNR_DB: f32 = 3.0;

/// What the receiver gets of the header's symbol bodies (the cyclic prefix is
/// dropped) over `trials` header trials of a `level` burst of `ncw` codewords, the
/// same bursts the gate sends: the useful level against the drive, dB, and the
/// signal-to-distortion ratio of the clipping, dB. Noiseless.
fn header_level(m: &Modem, level: u8, ncw: usize, trials: u32) -> (f32, f32) {
    let sl = m.ofdm.p.symbol_len();
    let start = (m.leader_symbols() + 2) * sl;
    let end = start + m.header_symbols() * sl;
    let bodies =
        |x: &[f32]| -> Vec<f32> { x[start..end].chunks(sl).flat_map(|s| s[m.ofdm.p.ncp..].to_vec()).collect() };
    // Linear reference: the same symbols scaled to the drive without clipping.
    let tiny = 1e-3;
    let (mut cl, mut ll, mut cc, mut n) = (0f64, 0f64, 0f64, 0usize);
    for t in 0..trials {
        let sent = Burst { header: trial_header(level, ncw, t), ..burst(level, ncw, payload_seed(t)) };
        let linear = bodies(&m.ofdm.modulate_with_leader_rms(&m.symbols(&sent), m.txdelay, tiny));
        let clipped = bodies(&m.modulate(&sent));
        let k = (m.tx_rms(level) / tiny) as f64;
        for (&c, &l) in clipped.iter().zip(&linear) {
            let (c, l) = (c as f64, l as f64 * k);
            (cl, ll, cc) = (cl + c * l, ll + l * l, cc + c * c);
        }
        n += linear.len();
    }
    // The clipped header is a times the linear one plus distortion.
    let a = cl / ll;
    let useful = a * a * ll;
    let distortion = cc - 2.0 * a * cl + useful;
    let level_db = 10.0 * (useful / n as f64).log10() - 20.0 * (m.tx_rms(level) as f64).log10();
    (level_db as f32, (10.0 * (useful / distortion).log10()) as f32)
}

/// The gate's header points, measured on one-codeword bursts of the lowest level at
/// each drive, stand for every level and codeword count at that drive: the header's
/// effective level differs by at most `HEADER_LEVEL_TOLERANCE_DB`. `modulate`
/// scales the whole burst to the drive, so the burst's length and payload could
/// move the header; and it clips, which depends on the header codeword, so the
/// gate draws the header's fields at random (`trial_header`) and this check
/// averages over the same draws. Effective level: useful level less the noise the
/// clipping adds at `HEADER_REFERENCE_SNR_DB`.
#[test]
fn header_level_depends_only_on_drive() {
    let effective = |(level_db, sdr_db): (f32, f32)| {
        let snr = 10f32.powf(HEADER_REFERENCE_SNR_DB / 10.0);
        level_db - 10.0 * (1.0 + snr / 10f32.powf(sdr_db / 10.0)).log10()
    };
    for mode in ALL_MODES.split(',') {
        let m = modem(mode);
        let drives = drive_levels(&m);
        let mut report = Vec::new();
        let mut worst = 0f32;
        for level in 0..LEVELS.len() as u8 {
            let measured = *drives.iter().find(|&&d| m.tx_rms(d) == m.tx_rms(level)).unwrap();
            let reference = effective(header_level(&m, measured, 1, HEADER_LEVEL_TRIALS));
            for ncw in [1, m.max_ncw(level)] {
                let (level_db, sdr_db) = header_level(&m, level, ncw, HEADER_LEVEL_TRIALS);
                let d = effective((level_db, sdr_db)) - reference;
                report.push(format!(
                    "L{level} ncw {ncw}: level {level_db:+.3} dB, SDR {sdr_db:.1} dB, {d:+.3} dB against L{measured} ncw 1"
                ));
                worst = worst.max(d.abs());
            }
        }
        assert!(
            worst <= HEADER_LEVEL_TOLERANCE_DB,
            "{mode}: the header's effective level differs by up to {worst:.3} dB at one drive:
  {}",
            report.join(
                "
  "
            )
        );
    }
}

/// The header is measured at every drive a mode sends, once each.
#[test]
fn header_at_every_drive() {
    for mode in ALL_MODES.split(',') {
        let m = modem(mode);
        let levels = drive_levels(&m);
        assert_eq!(levels[0], 0);
        let mut drives: Vec<f32> = (0..LEVELS.len() as u8).map(|l| m.tx_rms(l)).collect();
        drives.dedup();
        assert_eq!(levels.iter().map(|&l| m.tx_rms(l)).collect::<Vec<_>>(), drives, "{mode}");
    }
}

/// The committed baseline holds one consistent point per mode for the header and
/// per mode and level for the codewords, recorded with the settings the gate
/// measures with.
#[test]
fn baseline_covers_every_mode_and_level() {
    let base = load();
    let recorded = (base.source.codewords, base.source.headers_per_burst);
    assert_eq!(recorded, (CODEWORDS, HEADERS_PER_BURST), "{BASELINE} was recorded with other bursts: record it again");
    let specs: Vec<Spec> = ALL_TARGETS.split(',').map(|s| s.parse().unwrap()).collect();
    assert_eq!(specs.len(), LEVELS.len() + 1, "ALL_TARGETS does not list every level");
    let (mut headers, mut levels) = (0, 0);
    for mode in ALL_MODES.split(',') {
        let m = modem(mode);
        for target in Target::all(&m, &specs) {
            match target {
                Target::Header(_) => headers += 1,
                Target::Level(_) => levels += 1,
            }
            let p = base.point(mode, target);
            let at = format!("{BASELINE}: {}", target.label(mode));
            assert_eq!(p.name, target.name(), "{at} is named {:?}", p.name);
            // A changed drive table moves the points: record them again.
            assert_eq!(p.drive, m.tx_rms(target.level()), "{at} was recorded at another drive");
            check_point(&at, &p.grid_db, &p.rate, p.cnr_db, TARGET_RATE, target.margin());
        }
    }
    assert_eq!(base.header.len(), headers, "{BASELINE}: header entries for unknown modes or drives");
    assert_eq!(base.level.len(), levels, "{BASELINE}: entries for unknown modes or levels");
}

/// A recorded point is the crossing of its own grid, and the gate's two points lie
/// on that grid.
fn check_point(at: &str, grid: &[f32], rates: &[f32], point: f32, target: f32, margin: f32) {
    assert_eq!(grid.len(), rates.len(), "{at}: grid and rates differ in length");
    assert!(grid.windows(2).all(|w| w[0] < w[1]), "{at}: grid is not increasing");
    let x = crossing(grid, rates, target).unwrap_or_else(|| panic!("{at}: the grid does not cross the target"));
    assert!(((x * 100.0).round() / 100.0 - point).abs() < 1e-3, "{at}: recorded {point}, the grid crosses at {x}");
    assert!(grid[0] <= point - margin && point + margin <= grid[grid.len() - 1], "{at}: gate points off the grid");
}
