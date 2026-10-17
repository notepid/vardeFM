//! `varde` — test and measurement tool for the Varde modem.

use clap::{Parser, Subcommand};
use num_complex::Complex32;
use varde_dsp::prng::Prng;
use varde_dsp::{Modulation, Ofdm, OfdmParams};
use varde_sim::{transmit, FmChannelConfig};

#[derive(Parser)]
#[command(name = "varde", about = "Varde modem test tool")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(clap::Args, Clone)]
struct ModemArgs {
    /// narrow (mic/speaker) or wide (9600 data port)
    #[arg(long, default_value = "narrow")]
    mode: String,
    /// bpsk, qpsk, 16qam or 64qam
    #[arg(long = "mod", default_value = "qpsk")]
    modulation: String,
    /// Data symbols per frame
    #[arg(long, default_value_t = 40)]
    symbols: usize,
    /// TX spectral tilt in dB/octave (negative tilts down the high carriers)
    #[arg(long)]
    tilt: Option<f32>,
    /// TX RMS level, 0..1
    #[arg(long)]
    rms: Option<f32>,
    /// Leader before each frame, seconds (must exceed the radio's key-up time)
    #[arg(long, default_value_t = 0.1)]
    txdelay: f32,
}

impl ModemArgs {
    fn build(&self) -> (Ofdm, Modulation) {
        let mut p = OfdmParams::by_name(&self.mode).unwrap_or_else(|| die("unknown mode"));
        if let Some(t) = self.tilt {
            p.tilt_db_per_octave = t;
        }
        if let Some(r) = self.rms {
            p.tx_rms = r;
        }
        let m = Modulation::from_name(&self.modulation).unwrap_or_else(|| die("unknown modulation"));
        (Ofdm::new(p), m)
    }
}

#[derive(Subcommand)]
enum Cmd {
    /// Send frames through the FM channel simulator and measure bit errors
    Sim {
        #[command(flatten)]
        modem: ModemArgs,
        /// Carrier-to-noise ratio in dB (omit for no noise)
        #[arg(long)]
        cnr: Option<f32>,
        #[arg(long, default_value_t = 10)]
        frames: usize,
        /// Receiver sound-card clock error in ppm
        #[arg(long, default_value_t = 0.0)]
        ppm: f32,
        /// Simulated radio key-up time, seconds
        #[arg(long, default_value_t = 0.05)]
        keyup: f32,
    },
    /// BER table over CNR for every modulation
    Sweep {
        #[command(flatten)]
        modem: ModemArgs,
        #[arg(long, default_value_t = 5)]
        frames: usize,
        #[arg(long, default_value_t = 0.0)]
        ppm: f32,
        /// Simulated radio key-up time, seconds
        #[arg(long, default_value_t = 0.05)]
        keyup: f32,
    },
    /// Write test frames to a WAV file (play it into a radio)
    TxWav {
        #[command(flatten)]
        modem: ModemArgs,
        #[arg(long, default_value_t = 5)]
        frames: usize,
        #[arg(long)]
        out: String,
    },
    /// Decode test frames from a WAV file (a recording of a radio's output)
    RxWav {
        #[command(flatten)]
        modem: ModemArgs,
        #[arg(long = "in")]
        input: String,
    },
}

fn die(msg: &str) -> ! {
    eprintln!("error: {msg}");
    std::process::exit(2)
}

/// Known test payload for frame number `f`.
fn test_bits(ofdm: &Ofdm, m: Modulation, nsym: usize, f: usize) -> Vec<u8> {
    Prng::new(1000 + f as u32).bits(nsym * ofdm.data_carriers() * m.bits_per_symbol())
}

fn make_frame(ofdm: &Ofdm, m: Modulation, bits: &[u8], txdelay: f32) -> Vec<f32> {
    let syms = m.map_all(bits);
    let data: Vec<Vec<Complex32>> = syms.chunks(ofdm.data_carriers()).map(|c| c.to_vec()).collect();
    ofdm.modulate_with_leader(&data, txdelay)
}

fn count_errors(m: Modulation, frame: &varde_dsp::Frame, bits: &[u8]) -> usize {
    let mut out = Vec::with_capacity(bits.len());
    for s in frame.symbols.iter().flatten() {
        m.hard_bits(*s, &mut out);
    }
    out.iter().zip(bits).filter(|(a, b)| a != b).count()
}

struct Stats {
    bits: usize,
    errors: usize,
    lost: usize,
    snr_sum: f32,
    snr_n: usize,
}

fn run_sim(ofdm: &Ofdm, m: Modulation, nsym: usize, txdelay: f32, cfg: &FmChannelConfig, frames: usize) -> Stats {
    let mut st = Stats { bits: 0, errors: 0, lost: 0, snr_sum: 0.0, snr_n: 0 };
    for f in 0..frames {
        let bits = test_bits(ofdm, m, nsym, f);
        let tx = make_frame(ofdm, m, &bits, txdelay);
        let rx = transmit(&FmChannelConfig { seed: f as u64 + 1, ..cfg.clone() }, &tx);
        st.bits += bits.len();
        match ofdm.detect(&rx, 0).and_then(|b| ofdm.demodulate(&rx, b, nsym)) {
            Some(frame) => {
                if std::env::var("VARDE_DUMP").is_ok() {
                    let nv: Vec<String> = frame.noise_var.iter().map(|v| format!("{:.0}", -10.0 * v.log10())).collect();
                    eprintln!("frame {f}: per-carrier SNR dB: {}", nv.join(" "));
                }
                st.errors += count_errors(m, &frame, &bits);
                st.snr_sum += frame.snr_db;
                st.snr_n += 1;
            }
            None => {
                st.lost += 1;
                st.errors += bits.len() / 2;
            }
        }
    }
    st
}

fn raw_rate(ofdm: &Ofdm, m: Modulation, nsym: usize) -> f32 {
    let bits = (nsym * ofdm.data_carriers() * m.bits_per_symbol()) as f32;
    bits / (ofdm.frame_len(nsym) as f32 / ofdm.p.fs)
}

fn main() {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Sim { modem, cnr, frames, ppm, keyup } => {
            let (ofdm, m) = modem.build();
            let mut cfg = FmChannelConfig::by_name(&modem.mode).unwrap();
            cfg.cnr_db = cnr;
            cfg.clock_offset_ppm = ppm;
            cfg.keyup_delay_s = keyup;
            let st = run_sim(&ofdm, m, modem.symbols, modem.txdelay, &cfg, frames);
            println!(
                "mode={} mod={} cnr={} ppm={}  raw={:.0} bit/s",
                modem.mode,
                m.name(),
                cnr.map_or("none".into(), |c| format!("{c} dB")),
                ppm,
                raw_rate(&ofdm, m, modem.symbols)
            );
            println!(
                "frames lost {}/{}  BER {:.2e}  avg SNR {:.1} dB",
                st.lost,
                frames,
                st.errors as f64 / st.bits as f64,
                st.snr_sum / st.snr_n.max(1) as f32
            );
        }
        Cmd::Sweep { modem, frames, ppm, keyup } => {
            let (ofdm, _) = modem.build();
            let cnrs = [6.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0, 40.0];
            println!("mode={} symbols/frame={} ppm={} frames/point={}", modem.mode, modem.symbols, ppm, frames);
            print!("{:>8} {:>8}", "mod", "raw b/s");
            for c in cnrs {
                print!(" {:>8}", format!("{c}dB"));
            }
            println!();
            let mut snr_row = Vec::new();
            for m in Modulation::ALL {
                print!("{:>8} {:>8.0}", m.name(), raw_rate(&ofdm, m, modem.symbols));
                for (ci, &c) in cnrs.iter().enumerate() {
                    let mut cfg = FmChannelConfig::by_name(&modem.mode).unwrap();
                    cfg.cnr_db = Some(c);
                    cfg.clock_offset_ppm = ppm;
                    cfg.keyup_delay_s = keyup;
                    let st = run_sim(&ofdm, m, modem.symbols, modem.txdelay, &cfg, frames);
                    print!(" {:>8.1e}", st.errors as f64 / st.bits as f64);
                    if m == Modulation::Qpsk {
                        snr_row.push((ci, st.snr_sum / st.snr_n.max(1) as f32));
                    }
                }
                println!();
            }
            print!("{:>8} {:>8}", "SNR dB", "");
            for (_, s) in snr_row {
                print!(" {:>8.1}", s);
            }
            println!();
        }
        Cmd::TxWav { modem, frames, out } => {
            let (ofdm, m) = modem.build();
            let spec = hound::WavSpec {
                channels: 1,
                sample_rate: ofdm.p.fs as u32,
                bits_per_sample: 16,
                sample_format: hound::SampleFormat::Int,
            };
            let mut w = hound::WavWriter::create(&out, spec).unwrap_or_else(|e| die(&e.to_string()));
            let gap = vec![0f32; (ofdm.p.fs * 0.5) as usize];
            let mut write = |s: &[f32]| {
                for &v in s {
                    w.write_sample((v * 32767.0) as i16).unwrap();
                }
            };
            write(&gap);
            for f in 0..frames {
                write(&make_frame(&ofdm, m, &test_bits(&ofdm, m, modem.symbols, f), modem.txdelay));
                write(&gap);
            }
            println!("wrote {frames} frames to {out}");
        }
        Cmd::RxWav { modem, input } => {
            let (ofdm, m) = modem.build();
            let mut r = hound::WavReader::open(&input).unwrap_or_else(|e| die(&e.to_string()));
            let spec = r.spec();
            if spec.sample_rate != ofdm.p.fs as u32 {
                die("WAV must be 48 kHz");
            }
            let ch = spec.channels as usize;
            let samples: Vec<f32> = match spec.sample_format {
                hound::SampleFormat::Int => {
                    let max = (1i64 << (spec.bits_per_sample - 1)) as f32;
                    r.samples::<i32>().map(|s| s.unwrap() as f32 / max).collect()
                }
                hound::SampleFormat::Float => r.samples::<f32>().map(|s| s.unwrap()).collect(),
            };
            let rx: Vec<f32> = samples.iter().step_by(ch).copied().collect();
            let mut pos = 0;
            let mut f = 0;
            while let Some(b) = ofdm.detect(&rx, pos) {
                let Some(frame) = ofdm.demodulate(&rx, b, modem.symbols) else { break };
                let bits = test_bits(&ofdm, m, modem.symbols, f);
                let e = count_errors(m, &frame, &bits);
                println!(
                    "frame {f} at {:.2}s: BER {:.2e}  SNR {:.1} dB",
                    b as f32 / ofdm.p.fs,
                    e as f64 / bits.len() as f64,
                    frame.snr_db
                );
                pos = frame.end;
                f += 1;
            }
            println!("{f} frames decoded");
        }
    }
}
