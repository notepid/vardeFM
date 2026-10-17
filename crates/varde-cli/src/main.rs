//! `varde` — test and measurement tool for the Varde modem.

use clap::{Parser, Subcommand};
use varde_dsp::{Modulation, Ofdm, OfdmParams};
use varde_cli::{count_errors, decode_all, make_frame, read_wav, test_bits, write_wav};
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
            let gap = vec![0f32; (ofdm.p.fs * 0.5) as usize];
            let mut audio = gap.clone();
            for f in 0..frames {
                audio.extend(make_frame(&ofdm, m, &test_bits(&ofdm, m, modem.symbols, f), modem.txdelay));
                audio.extend_from_slice(&gap);
            }
            write_wav(&out, ofdm.p.fs as u32, &audio).unwrap_or_else(|e| die(&e.to_string()));
            println!("wrote {frames} frames to {out}");
        }
        Cmd::RxWav { modem, input } => {
            let (ofdm, m) = modem.build();
            let (fs, rx) = read_wav(&input).unwrap_or_else(|e| die(&e.to_string()));
            if fs != ofdm.p.fs as u32 {
                die("WAV must be 48 kHz");
            }
            let frames = decode_all(&ofdm, &rx, modem.symbols);
            for (f, (b, frame)) in frames.iter().enumerate() {
                let bits = test_bits(&ofdm, m, modem.symbols, f);
                let e = count_errors(m, frame, &bits);
                println!(
                    "frame {f} at {:.2}s: BER {:.2e}  SNR {:.1} dB",
                    *b as f32 / ofdm.p.fs,
                    e as f64 / bits.len() as f64,
                    frame.snr_db
                );
            }
            println!("{} frames decoded", frames.len());
        }
    }
}
