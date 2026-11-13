//! Issue 5: incremental redundancy (IR-HARQ) against today's resends, through the
//! FM channel simulator. An experiment, not a gate: nothing here is part of the
//! protocol.
//!
//! For each mode, level and CNR, real bursts of `CODEWORDS` codewords go through
//! the modem and the channel and are demodulated from the known burst position with
//! the header as training, as `level_eval` does. Each received bit slot gives a
//! reliability: its LLR times `1 - 2·(sent bit)`, positive when the hard decision is
//! right. Those of one codeword, in codeword order (the burst interleaver undone),
//! are the channel a codeword sees in one burst: clicks, channel estimate and all.
//!
//! A scheme then sends a codeword (of the rate-compatible mother code,
//! `Ldpc::with_extension`) through a sequence of such channels, one per attempt,
//! each from a different burst, with LLR = reliability × `1 - 2·bit`. That carries
//! over the bits of one burst to the bits of another codeword, which holds while the
//! interleaver spreads every codeword over the whole burst; the pairing of the bits
//! of one symbol is lost, which matters little for the same reason.
//!
//! Schemes, each charged the coded bits it puts on the air:
//! - `resend`: today. A failed codeword is sent again as a new one (2400 bits).
//! - `chase`: the same codeword again, LLRs added to the stored ones (2400 bits).
//! - `ir<E>`: increments of `E` extension bits, LLRs combined with the stored
//!   codeword, up to `EXT_BITS` in all; then the codeword starts over.
//!
//! Efficiency = information delivered / coded bits spent, relative to one codeword
//! per 2400 bits; goodput = efficiency × `Modem::net_rate(level)`. A scheme gives up
//! on a codeword after `MAX_BITS` bits. The envelope over levels is a genie's
//! level choice; the real link chooses by its SNR thresholds, with margin.
//!
//! Not modelled: header and detection failures (the same for every scheme), the
//! turns increments wait for (one per round, as for a resend today), signalling.
//!
//! `cargo test --release -p vardefm-modem ir_sweep -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `LEVELS` (0,..,6), `TRIALS` (bursts per point,
//! 48), `DEGREE` (extension row degree; default per k, `ext_degree`), `STEP` (CNR step, dB, 0.5).

use super::*;
use crate::detect_eval::{
    burst, env_list, env_modes, env_or, modem, on_every_core, payload_seed, through_channel, true_position,
};
use vardefm_dsp::prng::Prng;

/// Codewords per burst, or fewer where a level's burst holds fewer.
const CODEWORDS: usize = 4;
/// Extension bits of the mother code: rate k/4800.
const EXT_BITS: usize = 2400;
/// Variables per extension check for information length `k`: the best of 4, 6, 8
/// and 10 by the AWGN sweep `ir_curve` (vardefm-fec), over the increments.
fn ext_degree(k: usize) -> usize {
    if k <= 1600 {
        4
    } else {
        6
    }
}
const EXT_SEED: u32 = 0x1d9c_2000;
/// A scheme gives up on a codeword after this many coded bits.
const MAX_BITS: usize = 6 * CW_BITS;
/// Increment sizes measured, bits.
const INCREMENTS: [usize; 3] = [300, 600, 1200];
/// A level is measured from `BELOW_DB` under the 10 % CER point of the level below
/// (`perf/levels.toml`; level 0: under its own) to `ABOVE_DB` over its own. Below
/// that the level below carries more in every scheme measured; above it every
/// scheme delivers nearly everything, and counts as delivering everything.
const BELOW_DB: f32 = 1.0;
const ABOVE_DB: f32 = 1.5;
/// Level 0 is measured from this far under its point.
const LEVEL0_BELOW_DB: f32 = 3.0;

/// The 10 % CER points of `perf/levels.toml`, narrow then wide, levels 0 to 6.
fn level_point(mode: &str, level: u8) -> f32 {
    const NARROW: [f32; 7] = [3.04, 4.17, 5.42, 6.26, 8.07, 12.73, 16.13];
    const WIDE: [f32; 7] = [4.86, 6.15, 8.01, 10.91, 14.95, 20.61, 24.18];
    if mode == "narrow" {
        NARROW[level as usize]
    } else {
        WIDE[level as usize]
    }
}

/// Reliabilities of the codewords of trial burst `t`, in codeword bit order.
fn reliabilities(m: &Modem, mode: &str, level: u8, ncw: usize, truth: usize, cnr: f32, t: u32) -> Vec<Vec<f32>> {
    let sent = burst(level, ncw, payload_seed(t));
    let rx = through_channel(m, mode, cnr, &sent, t);
    let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
    let Some(frame) = m.ofdm.demodulate_known(&rx, truth, nsym, &m.header_carriers(&sent.header)) else {
        return vec![vec![0.0; CW_BITS]; ncw];
    };
    let slots = Modem::llrs(LEVELS[level as usize].modulation, &frame, m.header_symbols());
    let bits = m.payload_bits(level, &sent.payload);
    let perm = permutation(SEED_INTERLEAVE, ncw * CW_BITS);
    (0..ncw)
        .map(|j| {
            (0..CW_BITS)
                .map(|i| {
                    let s = perm[j * CW_BITS + i];
                    slots[s] * (1.0 - 2.0 * bits[s] as f32)
                })
                .collect()
        })
        .collect()
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Scheme {
    Resend,
    Chase,
    Ir(usize),
}

impl Scheme {
    fn name(self) -> String {
        match self {
            Scheme::Resend => "resend".into(),
            Scheme::Chase => "chase".into(),
            Scheme::Ir(e) => format!("ir{e}"),
        }
    }
}

/// The outcome of every scheme for codeword `i` of a point: (bits spent, delivered).
fn outcomes(
    schemes: &[Scheme],
    base: &Ldpc,
    mother: &Ldpc,
    pool: &[Vec<f32>],
    ncw: usize,
    i: usize,
) -> Vec<(usize, bool)> {
    let info = Prng::new(0x1f0_0000 + i as u32).bits(base.k);
    let cw = mother.encode(&info);
    let bursts = pool.len() / ncw;
    // Transmission a of codeword i (attempt or increment): a codeword of another
    // burst for each a, as long as 7·a is not a multiple of the bursts.
    let channel = |a: usize| -> &Vec<f32> { &pool[((i / ncw + a * 7) % bursts) * ncw + (i + a) % ncw] };
    let llr = |r: &[f32], bits: &[u8]| -> Vec<f32> {
        r.iter().zip(bits).map(|(&r, &b)| r * (1.0 - 2.0 * b as f32)).collect()
    };
    let ok = |got: Option<Vec<u8>>| got.as_deref() == Some(&info[..]);
    schemes
        .iter()
        .map(|&s| {
            let (mut spent, mut a) = (0, 0);
            match s {
                Scheme::Resend => {
                    while spent < MAX_BITS {
                        spent += CW_BITS;
                        let l = llr(channel(a), &cw[..CW_BITS]);
                        a += 1;
                        if ok(base.decode(&l, DECODE_ITERS)) {
                            return (spent, true);
                        }
                    }
                }
                Scheme::Chase => {
                    let mut sum = vec![0f32; CW_BITS];
                    while spent < MAX_BITS {
                        spent += CW_BITS;
                        for (s, x) in sum.iter_mut().zip(llr(channel(a), &cw[..CW_BITS])) {
                            *s += x;
                        }
                        a += 1;
                        if ok(base.decode(&sum, DECODE_ITERS)) {
                            return (spent, true);
                        }
                    }
                }
                Scheme::Ir(e) => {
                    spent += CW_BITS;
                    let mut l = llr(channel(a), &cw[..CW_BITS]);
                    a += 1;
                    if ok(base.decode(&l, DECODE_ITERS)) {
                        return (spent, true);
                    }
                    // Increments run through the extension, then round the whole
                    // mother codeword again, adding to the stored LLRs.
                    l.resize(mother.n, 0.0);
                    let mut pos = CW_BITS;
                    while spent < MAX_BITS {
                        let r = channel(a);
                        a += 1;
                        for &x in &r[..e] {
                            let j = pos % mother.n;
                            l[j] += x * (1.0 - 2.0 * cw[j] as f32);
                            pos += 1;
                        }
                        spent += e;
                        if ok(mother.decode(&l, DECODE_ITERS)) {
                            return (spent, true);
                        }
                    }
                }
            }
            (spent, false)
        })
        .collect()
}

/// Efficiency of each scheme at one point: information delivered over bits spent,
/// in codewords per 2400 bits.
fn efficiencies(
    m: &Modem,
    mode: &str,
    level: u8,
    cnr: f32,
    bursts: u32,
    schemes: &[Scheme],
    mother: &Ldpc,
) -> Vec<f32> {
    let ncw = m.max_ncw(level).min(CODEWORDS);
    let truth = true_position(m, mode, level, ncw);
    let pool: Vec<Vec<f32>> =
        on_every_core(bursts, |t| reliabilities(m, mode, level, ncw, truth, cnr, t)).into_iter().flatten().collect();
    let base = &m.codes[level as usize];
    let per_cw = on_every_core(pool.len() as u32, |i| outcomes(schemes, base, mother, &pool, ncw, i as usize));
    (0..schemes.len())
        .map(|s| {
            let (spent, got) = per_cw.iter().fold((0usize, 0usize), |(sp, g), o| (sp + o[s].0, g + o[s].1 as usize));
            (got * CW_BITS) as f32 / spent as f32
        })
        .collect()
}

/// Goodput against CNR per level and scheme, and the envelope over levels.
#[test]
#[ignore]
fn ir_sweep() {
    let modes = env_modes("narrow,wide");
    let levels: Vec<u8> = env_list("LEVELS", "0,1,2,3,4,5,6");
    let bursts: u32 = env_or("TRIALS", 48);
    let degree: usize = env_or("DEGREE", 0);
    let step: f32 = env_or("STEP", 0.5);
    let mut schemes = vec![Scheme::Resend, Scheme::Chase];
    schemes.extend(INCREMENTS.map(Scheme::Ir));
    let names: Vec<String> = schemes.iter().map(|s| s.name()).collect();
    for mode in &modes {
        let m = modem(mode);
        // From where each level's measurement starts.
        let from = |level: u8| match level {
            0 => level_point(mode, 0) - LEVEL0_BELOW_DB,
            l => level_point(mode, l - 1) - BELOW_DB,
        };
        // goodput[level][cnr index][scheme], kbit/s, on a common grid.
        let lo = levels.iter().map(|&l| from(l)).fold(f32::INFINITY, f32::min) / step;
        let hi = (level_point(mode, *levels.last().unwrap()) + ABOVE_DB) / step;
        let grid: Vec<f32> = (lo.floor() as i32..=hi.ceil() as i32).map(|i| i as f32 * step).collect();
        let mut table = vec![vec![vec![0f32; schemes.len()]; grid.len()]; levels.len()];
        // The first grid index each level was measured at.
        let mut first = vec![usize::MAX; levels.len()];
        for (li, &level) in levels.iter().enumerate() {
            let k = LEVELS[level as usize].k;
            let mother = Ldpc::with_extension(
                CW_BITS,
                k,
                SEED_CODE + k as u32,
                EXT_BITS,
                if degree > 0 { degree } else { ext_degree(k) },
                EXT_SEED + k as u32,
            );
            let rate = m.net_rate(level) / 1000.0;
            let point = level_point(mode, level);
            println!(
                "{mode} L{level} {} ({rate:.2} kbit/s, 10 % CER at {point:.2} dB): efficiency {names:?}",
                LEVELS[level as usize].name
            );
            for (ci, &cnr) in grid.iter().enumerate() {
                let eff = if cnr < from(level) - 1e-3 {
                    vec![0.0; schemes.len()]
                } else if cnr > point + ABOVE_DB {
                    vec![1.0; schemes.len()]
                } else {
                    let e = efficiencies(&m, mode, level, cnr, bursts, &schemes, &mother);
                    let line: String = e.iter().map(|x| format!(" {x:.3}")).collect();
                    println!("  {cnr:5.1} dB:{line}");
                    first[li] = first[li].min(ci);
                    e
                };
                table[li][ci] = eff.iter().map(|e| e * rate).collect();
            }
        }
        // A level that wins at the lowest CNR it was measured at may also win below
        // it, where it counts as 0: marked `*`, the range is too narrow there.
        println!("{mode} envelope over levels, kbit/s (level), genie level choice:");
        println!("  CNR dB {}", names.iter().map(|n| format!("{n:>13}")).collect::<String>());
        let mut sums = vec![0f32; schemes.len()];
        for (ci, &cnr) in grid.iter().enumerate() {
            let mut line = format!("  {cnr:6.1}");
            let mut best = vec![0f32; schemes.len()];
            for s in 0..schemes.len() {
                let (li, g) =
                    (0..levels.len())
                        .map(|li| (li, table[li][ci][s]))
                        .fold((0, 0.0), |a, b| if b.1 > a.1 { b } else { a });
                sums[s] += g;
                best[s] = g;
                let edge = if g > 0.0 && first[li] == ci && levels[li] != 0 { "*" } else { " " };
                line += &format!("  {g:6.2} (L{}){edge}", levels[li]);
            }
            for (s, name) in names.iter().enumerate().skip(1) {
                if best[0] > 0.0 {
                    line += &format!(" {name} {:+.0}%", 100.0 * (best[s] / best[0] - 1.0));
                }
            }
            println!("{line}");
        }
        let mean: String = sums.iter().map(|s| format!(" {:.2}", s / grid.len() as f32)).collect();
        println!("{mode} mean over the grid, kbit/s:{mean}");
    }
}
