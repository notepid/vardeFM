//! A VardeFM station on a real radio: sound card + PTT + VARA TCP ports.
//!
//! Time is the input sound card's sample clock. While transmitting, received
//! audio is ignored (half duplex). PTT is keyed before a burst's audio is
//! queued, and released only after the sound card has taken the last sample
//! plus `txtail_s`. A watchdog unkeys the radio if a transmission runs far
//! longer than expected.
//!
//! [`RadioEngine`] holds that logic with the sound card output and PTT behind
//! traits, so it can be tested without hardware; [`run`] wires it to real devices.

use crate::server::HostPort;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use vardefm_audio::{open_input, open_output, AudioIn, AudioOut, Channel, RATE};
use vardefm_engine::{EngineConfig, HostOut, LEVEL_WINDOW_S};
use vardefm_modem::{Modem, OfdmParams, LEVELS};
use vardefm_ptt::PttSpec;

pub use vardefm_engine::{AudioSink, HostIn, RadioEngine};

/// Desktop radio configuration: engine settings plus sound card, PTT and TCP ports.
#[derive(Clone, Debug)]
pub struct RadioConfig {
    pub mode: String,
    pub host: String,
    /// Command port (data = +1).
    pub port: u16,
    pub audio_in: Option<String>,
    pub audio_out: Option<String>,
    pub in_channel: Channel,
    pub out_channel: Channel,
    pub ptt: PttSpec,
    /// Leader before each burst (radio key-up time plus margin).
    pub txdelay_s: f32,
    /// PTT held after the last sample leaves the sound card.
    pub txtail_s: f32,
    /// Output level multiplier (sets FM deviation together with the sound card mixer).
    pub tx_gain: f32,
    /// Save every transmitted burst as a WAV file in this folder.
    pub record: Option<PathBuf>,
    /// Highest speed level to send or request (limits the decoding load on slow computers).
    pub max_level: u8,
    pub verbose: bool,
    /// Print the receive level every [`LEVEL_WINDOW_S`] seconds while idle.
    pub show_levels: bool,
    /// Act as a digipeater for these call signs (empty: off).
    pub digi_calls: Vec<String>,
    /// Extra wait per digipeater hop on our own routed sessions.
    pub hop_margin_s: f64,
}

impl RadioConfig {
    /// Refuses values the engine cannot run with. The command line checks its own
    /// arguments; this is for a config built any other way.
    pub fn check(&self) -> io::Result<()> {
        self.engine_config().check()
    }

    /// The subset of settings the engine needs.
    pub fn engine_config(&self) -> EngineConfig {
        EngineConfig {
            mode: self.mode.clone(),
            txdelay_s: self.txdelay_s,
            txtail_s: self.txtail_s,
            tx_gain: self.tx_gain,
            max_level: self.max_level,
            show_levels: self.show_levels,
            digi_calls: self.digi_calls.clone(),
            hop_margin_s: self.hop_margin_s,
        }
    }
}

impl Default for RadioConfig {
    fn default() -> Self {
        let e = EngineConfig::default();
        Self {
            mode: e.mode,
            host: "127.0.0.1".into(),
            port: 8300,
            audio_in: None,
            audio_out: None,
            in_channel: Channel::Left,
            out_channel: Channel::Both,
            ptt: PttSpec::None,
            txdelay_s: e.txdelay_s,
            txtail_s: e.txtail_s,
            tx_gain: e.tx_gain,
            record: None,
            max_level: e.max_level,
            verbose: true,
            show_levels: e.show_levels,
            digi_calls: e.digi_calls,
            hop_margin_s: e.hop_margin_s,
        }
    }
}

/// Adapts a desktop [`vardefm_ptt::Ptt`] to the engine's trait.
struct PttAdapter(Box<dyn vardefm_ptt::Ptt>);

impl vardefm_engine::Ptt for PttAdapter {
    fn set(&mut self, on: bool) -> io::Result<()> {
        self.0.set(on)
    }
}

/// Local wrapper so we can implement the engine's [`AudioSink`] (orphan rules).
struct CardOut(AudioOut);

impl AudioSink for CardOut {
    fn play(&mut self, samples: &[f32]) -> u64 {
        self.0.play(samples)
    }
    fn consumed(&self) -> u64 {
        self.0.consumed()
    }
    fn flush(&mut self) {
        self.0.flush()
    }
}

/// Writes each transmitted burst as a WAV file, then forwards to the inner sink.
struct RecordingSink<S: AudioSink> {
    inner: S,
    dir: PathBuf,
    tx_count: u64,
}

impl<S: AudioSink> RecordingSink<S> {
    fn new(inner: S, dir: PathBuf) -> Self {
        Self { inner, dir, tx_count: 0 }
    }
}

impl<S: AudioSink> AudioSink for RecordingSink<S> {
    fn play(&mut self, samples: &[f32]) -> u64 {
        let path = self.dir.join(format!("{:04}_tx.wav", self.tx_count));
        let _ = write_wav(&path, samples);
        self.tx_count += 1;
        self.inner.play(samples)
    }
    fn consumed(&self) -> u64 {
        self.inner.consumed()
    }
    fn flush(&mut self) {
        self.inner.flush()
    }
}

/// Run until `stop` is set. Returns an error if a device cannot be opened.
pub fn run(cfg: RadioConfig, stop: Arc<AtomicBool>) -> io::Result<()> {
    // Before any device is opened.
    cfg.check()?;
    let params = OfdmParams::by_name(&cfg.mode).ok_or_else(|| io::Error::other("mode must be narrow or wide"))?;
    let modem = Arc::new(Modem::new(params, cfg.txdelay_s));
    let desktop_ptt = vardefm_ptt::open(&cfg.ptt)?;
    let ptt_desc = desktop_ptt.describe();
    let ptt = Box::new(PttAdapter(desktop_ptt));
    let input = open_input(cfg.audio_in.as_deref(), cfg.in_channel)?;
    let output = open_output(cfg.audio_out.as_deref(), cfg.out_channel)?;
    let port = HostPort::bind(&cfg.host, cfg.port, cfg.port + 1)?;
    if let Some(d) = &cfg.record {
        std::fs::create_dir_all(d)?;
    }
    println!("VardeFM {} mode", cfg.mode);
    println!("  audio in : {} ({} Hz{})", input.name, input.device_rate, resample_note(input.device_rate));
    println!("  audio out: {} ({} Hz{})", output.name, output.device_rate, resample_note(output.device_rate));
    println!("  PTT      : {ptt_desc}");
    if (cfg.max_level as usize) < LEVELS.len() - 1 {
        println!("  max level: {} ({})", cfg.max_level, LEVELS[cfg.max_level as usize].name);
    }
    println!("  host     : command {}, data {}", port.cmd_addr, port.data_addr);
    if !cfg.digi_calls.is_empty() {
        println!("  digi     : repeating for {}", cfg.digi_calls.join(", "));
    }

    if cfg.show_levels {
        println!("  levels   : every {LEVEL_WINDOW_S} s while idle; set the radio volume so bursts peak near -6 dBFS");
    }

    let seed = vardefm_engine::random_seed();
    let eng_cfg = cfg.engine_config();
    let out = CardOut(output);
    match cfg.record.clone() {
        Some(dir) => drive(
            RadioEngine::from_config(&eng_cfg, modem, ptt, RecordingSink::new(out, dir), seed)?,
            &cfg,
            &input,
            &port,
            &stop,
        ),
        None => drive(RadioEngine::from_config(&eng_cfg, modem, ptt, out, seed)?, &cfg, &input, &port, &stop),
    }
}

fn drive<S: AudioSink>(
    mut engine: RadioEngine<S>,
    cfg: &RadioConfig,
    input: &AudioIn,
    port: &HostPort,
    stop: &AtomicBool,
) -> io::Result<()> {
    while !stop.load(Ordering::Relaxed) {
        let chunk = input.rx.recv_timeout(Duration::from_millis(50)).ok();
        let host_in: Vec<HostIn> = port.inbox.try_iter().collect();
        for o in engine.step(chunk.as_deref(), host_in) {
            match o {
                HostOut::Line(l) => port.send_line(&l),
                HostOut::Data(d) => port.send_data(&d),
            }
        }
        engine.reports.clear();
        for l in engine.log.drain(..) {
            if cfg.verbose || l.contains("failed") || l.contains("watchdog") {
                println!("{l}");
            }
        }
        for l in engine.notices.drain(..) {
            println!("{l}");
        }
    }
    engine.unkey();
    println!("Stopped; PTT released.");
    Ok(())
}

fn resample_note(rate: u32) -> &'static str {
    if rate == RATE {
        ""
    } else {
        ", resampled to 48000"
    }
}

fn write_wav(path: &Path, samples: &[f32]) -> Result<(), hound::Error> {
    let spec =
        hound::WavSpec { channels: 1, sample_rate: RATE, bits_per_sample: 16, sample_format: hound::SampleFormat::Int };
    let mut w = hound::WavWriter::create(path, spec)?;
    for &v in samples {
        w.write_sample((v.clamp(-1.0, 1.0) * 32767.0) as i16)?;
    }
    w.finalize()
}
