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
//! over the bits of one burst to the bits of another codeword. The decoder treats
//! every codeword alike (flipping the signs of a codeword's LLRs leaves its checks'
//! signs unchanged), so this is exact for the codeword the reliabilities came from;
//! for another codeword only the pairing of the bits of one QAM symbol changes.
//! `sign_trick_matches_real_decoding` checks that a first transmission decodes
//! exactly when the modem's own decoding of the sent codeword does.
//!
//! Schemes, each charged the coded bits it puts on the air:
//! - `resend`: today. A failed codeword is sent again as a new one (2400 bits).
//! - `chase`: the same codeword again, LLRs added to the stored ones (2400 bits).
//! - `ir<E>`: increments of `E` extension bits, LLRs combined with the stored
//!   codeword, through the extension and then around the whole mother codeword.
//!
//! Efficiency = information delivered / coded bits spent, relative to one codeword
//! per 2400 bits; goodput = efficiency × `Modem::net_rate(level)`. A scheme gives up
//! on a codeword after `MAX_BITS` bits. The envelope over levels is a genie's
//! level choice; the real link chooses by its SNR thresholds, with margin.
//!
//! Not modelled: header failures, detection and frame sync (the burst position is
//! known), all the same for every scheme; the turns increments wait for (one per
//! round, as for a resend today); signalling.
//!
//! `cargo test --release -p vardefm-modem ir_sweep -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `LEVELS` (0,..,6), `TRIALS` (bursts per point,
//! 48), `DEGREE` (extension row degree; unset: per k, `ext_degree`), `STEP` (CNR
//! step, dB, 0.5).

use super::*;
use crate::detect_eval::{
    burst, env_list, env_modes, env_or, modem, on_every_core, payload_seed, through_channel, true_position,
};
use crate::level_eval::level_points;
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

/// Reliabilities of the codewords of trial burst `t`, in codeword bit order.
fn reliabilities(m: &Modem, mode: &str, level: u8, ncw: usize, truth: usize, cnr: f32, t: u32) -> Vec<Vec<f32>> {
    let sent = burst(level, ncw, payload_seed(t));
    let rx = through_channel(m, mode, cnr, &sent, t);
    let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
    // Fails only when the burst does not fit the recording, whatever the noise.
    let frame = m.ofdm.demodulate_known(&rx, truth, nsym, &m.header_carriers(&sent.header)).expect("burst fits");
    let slots = Modem::llrs(LEVELS[level as usize].modulation, &frame, m.header_symbols());
    let bits = m.payload_bits(level, &sent.payload);
    let perm = permutation(SEED_INTERLEAVE, ncw * CW_BITS);
    let reliability = |j: usize| -> Vec<f32> {
        (0..CW_BITS)
            .map(|i| {
                let s = perm[j * CW_BITS + i];
                slots[s] * (1.0 - 2.0 * bits[s] as f32)
            })
            .collect()
    };
    (0..ncw).map(reliability).collect()
}

/// The smallest of `INCREMENTS`.
const MIN_INCREMENT: usize = {
    let mut min = usize::MAX;
    let mut i = 0;
    while i < INCREMENTS.len() {
        if INCREMENTS[i] < min {
            min = INCREMENTS[i];
        }
        i += 1;
    }
    min
};
/// Most transmissions a scheme can make for one codeword: a whole codeword, then
/// the smallest increments up to `MAX_BITS`.
const MAX_TRANSMISSIONS: usize = 1 + (MAX_BITS - CW_BITS).div_ceil(MIN_INCREMENT);

/// Pool entry of transmission `a` (attempt or increment) of codeword `i`, in a pool
/// of `bursts` bursts of `ncw` codewords: 7 bursts on per transmission.
fn channel_index(i: usize, a: usize, ncw: usize, bursts: usize) -> usize {
    ((i / ncw + a * 7) % bursts) * ncw + (i + a) % ncw
}

/// Whether a pool of `bursts` bursts gives every transmission of a codeword its own
/// burst under [`channel_index`].
fn pool_is_large_enough(bursts: usize) -> bool {
    (1..MAX_TRANSMISSIONS).all(|a| (7 * a) % bursts != 0)
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

/// LLRs of `bits` sent over a channel of `reliabilities`.
fn llr(reliabilities: &[f32], bits: &[u8]) -> Vec<f32> {
    reliabilities.iter().zip(bits).map(|(&r, &b)| r * (1.0 - 2.0 * b as f32)).collect()
}

/// The information bits and mother codeword that codeword `i` of a point sends.
fn codeword(mother: &Ldpc, i: usize) -> (Vec<u8>, Vec<u8>) {
    let info = Prng::new(0x1f0_0000 + i as u32).bits(mother.k);
    let cw = mother.encode(&info);
    (info, cw)
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
    let (info, cw) = codeword(mother, i);
    let bursts = pool.len() / ncw;
    let channel = |a: usize| -> &Vec<f32> { &pool[channel_index(i, a, ncw, bursts)] };
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

/// The mother code of `level`: extension of `degree` variables per row, or
/// [`ext_degree`]'s.
fn mother_code(level: u8, degree: Option<usize>) -> Ldpc {
    let k = LEVELS[level as usize].k;
    let d = degree.unwrap_or(ext_degree(k));
    Ldpc::with_extension(CW_BITS, k, SEED_CODE + k as u32, EXT_BITS, d, EXT_SEED + k as u32)
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

/// The guard on `TRIALS` holds exactly when every transmission of every codeword
/// comes from its own burst.
#[test]
fn pool_guard_matches_distinct_bursts() {
    for ncw in [2, 4] {
        for bursts in 1..=60 {
            let distinct = (0..bursts * ncw).all(|i| {
                let mut seen: Vec<usize> =
                    (0..MAX_TRANSMISSIONS).map(|a| channel_index(i, a, ncw, bursts) / ncw).collect();
                seen.sort_unstable();
                seen.dedup();
                seen.len() == MAX_TRANSMISSIONS
            });
            assert_eq!(pool_is_large_enough(bursts), distinct, "{bursts} bursts of {ncw}");
        }
    }
}

/// A first transmission through a burst's reliabilities, applied to another
/// codeword, decodes exactly when the modem decodes the codeword actually sent.
/// This checks the interleaver mapping and signs of [`reliabilities`], and that the
/// sign trick carries a burst's channel over to other codewords.
#[test]
fn sign_trick_matches_real_decoding() {
    let mode = "narrow";
    let m = modem(mode);
    let level = 3;
    let ncw = m.max_ncw(level).min(CODEWORDS);
    let truth = true_position(&m, mode, level, ncw);
    let mother = mother_code(level, None);
    let base = &m.codes[level as usize];
    // Around the level's 10 % point, so that both outcomes can occur.
    for cnr in [5.75f32, 6.0, 6.25] {
        for t in 0..2 {
            let sent = burst(level, ncw, payload_seed(t));
            let rx = through_channel(&m, mode, cnr, &sent, t);
            let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
            let frame = m.ofdm.demodulate_known(&rx, truth, nsym, &m.header_carriers(&sent.header)).unwrap();
            let real = m.decode_payload(&sent.header, &frame);
            let rel = reliabilities(&m, mode, level, ncw, truth, cnr, t);
            for j in 0..ncw {
                let (info, cw) = codeword(&mother, 7 * j + t as usize);
                let trick = base.decode(&llr(&rel[j], &cw[..CW_BITS]), DECODE_ITERS).as_deref() == Some(&info[..]);
                assert_eq!(trick, real[j].as_ref() == Some(&sent.payload[j]), "{cnr} dB, burst {t}, codeword {j}");
            }
        }
    }
}

/// Every scheme delivers every codeword at once far above a level's point and
/// nothing far below it, so the accounting itself works.
#[test]
fn schemes_deliver_far_above_and_nothing_far_below() {
    let mode = "narrow";
    let m = modem(mode);
    let mut schemes = vec![Scheme::Resend, Scheme::Chase];
    schemes.extend(INCREMENTS.map(Scheme::Ir));
    let above = efficiencies(&m, mode, 0, 40.0, 1, &schemes, &mother_code(0, None));
    assert!(above.iter().all(|&e| e == 1.0), "at 40 dB: {above:?}");
    let below = efficiencies(&m, mode, 6, -5.0, 1, &schemes, &mother_code(6, None));
    assert!(below.iter().all(|&e| e == 0.0), "at -5 dB: {below:?}");
}

/// Goodput against CNR per level and scheme, and the envelope over levels.
#[test]
#[ignore]
fn ir_sweep() {
    let modes = env_modes("narrow,wide");
    let levels: Vec<u8> = env_list("LEVELS", "0,1,2,3,4,5,6");
    let bursts: u32 = env_or("TRIALS", 48);
    assert!(bursts > 0, "TRIALS must be at least 1");
    assert!(
        pool_is_large_enough(bursts as usize),
        "TRIALS={bursts}: a codeword's {MAX_TRANSMISSIONS} transmissions would reuse a burst; use more than \
         {MAX_TRANSMISSIONS} bursts, not a multiple of 7"
    );
    let degree: Option<usize> = std::env::var("DEGREE")
        .ok()
        .map(|v| v.trim().parse().unwrap_or_else(|_| panic!("DEGREE={v:?} does not parse")));
    assert!(degree.is_none_or(|d| d >= 1), "DEGREE must be at least 1");
    let step: f32 = env_or("STEP", 0.5);
    assert!(step.is_finite() && step > 0.0, "STEP must be a positive number of dB");
    let mut schemes = vec![Scheme::Resend, Scheme::Chase];
    schemes.extend(INCREMENTS.map(Scheme::Ir));
    let names: Vec<String> = schemes.iter().map(|s| s.name()).collect();
    for mode in &modes {
        let m = modem(mode);
        let points = level_points(mode);
        // From where each level's measurement starts.
        let from = |level: u8| match level {
            0 => points[0] - LEVEL0_BELOW_DB,
            l => points[l as usize - 1] - BELOW_DB,
        };
        // goodput[level][cnr index][scheme], kbit/s, on a common grid.
        let lo = levels.iter().map(|&l| from(l)).fold(f32::INFINITY, f32::min) / step;
        let hi = levels.iter().map(|&l| points[l as usize] + ABOVE_DB).fold(f32::NEG_INFINITY, f32::max) / step;
        let grid: Vec<f32> = (lo.floor() as i32..=hi.ceil() as i32).map(|i| i as f32 * step).collect();
        let mut table = vec![vec![vec![0f32; schemes.len()]; grid.len()]; levels.len()];
        // The first grid index each level was measured at.
        let mut first = vec![usize::MAX; levels.len()];
        for (li, &level) in levels.iter().enumerate() {
            let mother = mother_code(level, degree);
            let rate = m.net_rate(level) / 1000.0;
            let point = points[level as usize];
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
