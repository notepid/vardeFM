//! `vardefm bench`: is this computer fast enough to receive in real time?
//!
//! The receiver demodulates and decodes a burst after it has ended, and the
//! reply waits for that. So the number that matters is the *decode delay*:
//! the time between the end of a burst and its decoded result. The other
//! station waits [`REPLY_TIMEOUT_S`] for the reply to start, and the reply
//! starts [`TURNAROUND_S`] after decoding finishes.
//!
//! Each speed level is measured with its longest burst, twice:
//! - clean: a strong signal, where every codeword decodes in a few iterations;
//! - worst case: the payload is noise, so every codeword runs the decoder's
//!   full iteration count and fails.

use std::sync::Arc;
use std::time::Instant;
use vardefm_dsp::prng::Prng;
use vardefm_link::LinkConfig;
use vardefm_modem::{Burst, Header, Kind, Modem, OfdmParams, Receiver, LEVELS};
use vardefm_sim::{transmit, FmChannelConfig};

const CHUNK: usize = 480; // 10 ms, as the sound card delivers it
/// Decode delay that leaves a comfortable margin.
const OK_DELAY_S: f64 = 0.5;
/// Decode delay above which replies are likely to arrive too late.
const MAX_DELAY_S: f64 = 0.8;

struct Run {
    /// Seconds from the end of the burst to its decoded result.
    delay: f64,
    /// Processing time / audio duration.
    load: f64,
    decoded: usize,
    /// Bursts with a payload that were found (header decoded).
    found: usize,
}

fn run(modem: &Arc<Modem>, audio: &[f32]) -> Run {
    let mut rx = Receiver::new(modem.clone());
    let mut total = 0.0;
    let mut delay = 0.0;
    let mut decoded = 0;
    let mut found = 0;
    for chunk in audio.chunks(CHUNK) {
        let t = Instant::now();
        let got = rx.push(chunk);
        let dt = t.elapsed().as_secs_f64();
        total += dt;
        for b in got {
            if b.header.ncw > 0 {
                delay = dt;
                found += 1;
                decoded += b.payload.iter().filter(|p| p.is_some()).count();
            }
        }
    }
    Run { delay, load: total / (audio.len() as f64 / modem.fs() as f64), decoded, found }
}

/// Uniform noise with the given RMS (good enough for timing).
fn noise(rng: &mut Prng, rms: f32) -> f32 {
    (rng.next_u32() as f32 / u32::MAX as f32 * 2.0 - 1.0) * rms * 3f32.sqrt()
}

fn burst(modem: &Modem, level: u8, seed: u32) -> Burst {
    let ncw = modem.max_ncw(level);
    let mut rng = Prng::new(seed);
    let n = Modem::payload_bytes(level);
    Burst {
        header: Header {
            kind: Kind::Data,
            session: 1,
            level,
            ncw: ncw as u8,
            req_level: level,
            burst_id: 1,
            ack_burst: 0,
            ack_offset: 0,
            bitmap: 0,
            flags: 0,
        },
        payload: (0..ncw).map(|_| (0..n).map(|_| rng.next_u32() as u8).collect()).collect(),
    }
}

/// Through the simulated radio at a strong signal.
fn on_air(mode: &str, tx: &[f32]) -> Vec<f32> {
    let cfg = FmChannelConfig { cnr_db: Some(40.0), seed: 7, ..FmChannelConfig::by_name(mode).unwrap() };
    transmit(&cfg, tx)
}

pub fn bench(modes: &[&str], threads: Option<usize>) -> Result<(), String> {
    let cores = std::thread::available_parallelism().map_or(1, |n| n.get());
    let threads = threads.unwrap_or(cores).max(1);
    let link = LinkConfig::default();
    println!("VardeFM {} receive benchmark", env!("CARGO_PKG_VERSION"));
    println!("{cores} CPU cores, decoding on {threads} thread(s).");
    println!(
        "Decode delay = time from the end of a burst to its decoded result. The reply starts {:.1} s later, \
         and the other station waits {:.1} s for it: up to {OK_DELAY_S} s is fine, above {MAX_DELAY_S} s replies \
         may come too late.",
        link.turnaround_s, link.response_timeout_s
    );

    for &mode in modes {
        let params = OfdmParams::by_name(mode).ok_or("mode must be narrow, wide or both")?;
        let mut modem = Modem::new(params, 0.15);
        modem.decode_threads = threads;
        let modem = Arc::new(modem);

        // Listening to an empty channel (noise): the preamble search runs on every chunk.
        let mut rng = Prng::new(3);
        let empty: Vec<f32> = (0..(5.0 * modem.fs()) as usize).map(|_| noise(&mut rng, 0.1)).collect();
        let idle = run(&modem, &empty);
        // Warm up caches and the CPU clock before timing bursts.
        let _ = run(&modem, &on_air(mode, &modem.modulate(&burst(&modem, 0, 1))));

        println!();
        println!("{mode} mode (listening to an empty channel: {:.0}% CPU)", 100.0 * idle.load);
        println!("  level        burst | clean: delay   CPU | worst case: delay   CPU | verdict");
        let mut max_ok = None;
        for level in 0..LEVELS.len() as u8 {
            let b = burst(&modem, level, 100 + level as u32);
            let tx = modem.modulate(&b);
            let clean = run(&modem, &on_air(mode, &tx));

            // Worst case: keep the header, replace the payload with noise.
            let mut bad = tx.clone();
            let hdr_end = modem.burst_samples(level, 0);
            let mut rng = Prng::new(9);
            for x in &mut bad[hdr_end..] {
                *x = noise(&mut rng, 0.3);
            }
            let worst = run(&modem, &on_air(mode, &bad));

            let ncw = b.payload.len();
            let verdict = if clean.found == 0 || worst.found == 0 {
                "burst not found!"
            } else if clean.decoded < ncw {
                "decode error!"
            } else if worst.delay <= MAX_DELAY_S && clean.delay <= OK_DELAY_S {
                "ok"
            } else if clean.delay <= MAX_DELAY_S {
                "ok, slow when codewords fail"
            } else {
                "too slow"
            };
            if verdict.starts_with("ok") {
                max_ok = Some(level);
            }
            println!(
                "  {:<10} {:>5.1} s |   {:>7.0} ms {:>4.0}% |      {:>7.0} ms {:>4.0}% | {verdict}",
                LEVELS[level as usize].name,
                modem.burst_seconds(level, ncw),
                1000.0 * clean.delay,
                100.0 * clean.load,
                1000.0 * worst.delay,
                100.0 * worst.load,
            );
        }
        match max_ok {
            Some(l) if l as usize == LEVELS.len() - 1 => println!("  => {mode} mode runs at every speed level."),
            Some(l) => println!(
                "  => In {mode} mode, run the modem with --max-level {l} ({}) on this computer.",
                LEVELS[l as usize].name
            ),
            None => println!("  => This computer is too slow for {mode} mode."),
        }
    }
    Ok(())
}
