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

use crate::server::{HostIn, HostPort};
use crate::station::{HostOut, Station};
use std::io;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use vardefm_audio::{open_input, open_output, AudioOut, Channel, RATE};
use vardefm_link::LinkConfig;
use vardefm_modem::{Modem, OfdmParams, LEVELS};
use vardefm_ptt::{Ptt, PttSpec};

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
    pub verbose: bool,
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            mode: "narrow".into(),
            host: "127.0.0.1".into(),
            port: 8300,
            audio_in: None,
            audio_out: None,
            in_channel: Channel::Left,
            out_channel: Channel::Both,
            ptt: PttSpec::None,
            txdelay_s: 0.15,
            txtail_s: 0.05,
            tx_gain: 1.0,
            record: None,
            verbose: true,
        }
    }
}

/// Where transmit audio goes.
pub trait AudioSink {
    /// Queue samples; returns the `consumed()` value at which they have all been taken.
    fn play(&mut self, samples: &[f32]) -> u64;
    /// Samples taken by the sound card so far.
    fn consumed(&self) -> u64;
    /// Drop queued samples.
    fn flush(&mut self);
}

impl AudioSink for AudioOut {
    fn play(&mut self, samples: &[f32]) -> u64 {
        AudioOut::play(self, samples)
    }
    fn consumed(&self) -> u64 {
        AudioOut::consumed(self)
    }
    fn flush(&mut self) {
        AudioOut::flush(self)
    }
}

/// A transmission in progress.
struct Tx {
    /// `consumed()` value at which the burst has left the queue.
    end: u64,
    /// When that happened (input clock), once it has.
    drained_at: Option<f64>,
    /// Watchdog: unkey no matter what after this time.
    deadline: f64,
}

/// Station + PTT + audio output, stepped once per received audio chunk.
pub struct RadioEngine<S: AudioSink> {
    pub station: Station,
    ptt: Box<dyn Ptt>,
    out: S,
    tx_gain: f32,
    /// Seconds PTT stays on after the last sample has left the queue.
    tail_s: f64,
    rx_pos: u64,
    tx: Option<Tx>,
    tx_count: u64,
    record: Option<PathBuf>,
    /// Log lines (one per burst sent or received), drained by the caller.
    pub log: Vec<String>,
}

impl<S: AudioSink> RadioEngine<S> {
    pub fn new(modem: Arc<Modem>, ptt: Box<dyn Ptt>, out: S, tx_gain: f32, txtail_s: f32, record: Option<PathBuf>, seed: u32) -> Self {
        Self {
            station: Station::new("local", modem, LinkConfig::default(), seed),
            ptt,
            out,
            tx_gain,
            tail_s: txtail_s as f64,
            rx_pos: 0,
            tx: None,
            tx_count: 0,
            record,
            log: Vec::new(),
        }
    }

    pub fn now(&self) -> f64 {
        self.rx_pos as f64 / RATE as f64
    }

    pub fn transmitting(&self) -> bool {
        self.tx.is_some()
    }

    /// Advance with one chunk of received audio (or `None` if the sound card had
    /// nothing) and the host input since the last step. Returns host output.
    pub fn step(&mut self, chunk: Option<&[f32]>, host_in: Vec<HostIn>) -> Vec<HostOut> {
        if let Some(c) = chunk {
            self.rx_pos += c.len() as u64;
        }
        let now = self.now();
        for msg in host_in {
            match msg {
                HostIn::Line(l) => self.station.host_line(now, &l),
                HostIn::Data(d) => self.station.host_data(&d),
                HostIn::CmdClosed => self.station.host_closed(now),
            }
        }
        self.station.tick(now);

        if let Some(tx) = &mut self.tx {
            if tx.drained_at.is_none() && self.out.consumed() >= tx.end {
                tx.drained_at = Some(now);
            }
            let done = tx.drained_at.is_some_and(|t| now >= t + self.tail_s);
            if done || now > tx.deadline {
                if !done {
                    self.log.push(format!("{now:8.2}s watchdog: transmission overran, unkeying"));
                    self.out.flush();
                }
                if let Err(e) = self.ptt.set(false) {
                    self.log.push(format!("{now:8.2}s PTT off failed: {e}"));
                }
                self.tx = None;
                self.station.tx_done(now);
            }
        } else {
            if let Some(c) = chunk {
                for b in self.station.rx_audio(now, c) {
                    let ok = b.payload.iter().filter(|p| p.is_some()).count();
                    self.log.push(format!(
                        "{now:8.2}s rx {:<10} cw {ok:>2}/{:<2} SNR {:5.1} dB",
                        format!("{:?}", b.header.kind),
                        b.header.ncw,
                        b.snr_db
                    ));
                }
            }
            if let Some(t) = self.station.poll_tx(now) {
                let h = t.header;
                self.log.push(format!(
                    "{now:8.2}s tx {:<10} {:<9} cw {:>2} {:>5} B (resent {:>4}) retry {} air {:.2} s",
                    format!("{:?}", h.kind),
                    if h.ncw > 0 { LEVELS[h.level as usize].name } else { "-" },
                    h.ncw,
                    t.info.bytes,
                    t.info.resent_bytes,
                    t.info.retry,
                    t.seconds
                ));
                // Key first, then queue the audio: the leader covers the radio's key-up time.
                if let Err(e) = self.ptt.set(true) {
                    self.log.push(format!("{now:8.2}s PTT on failed: {e}"));
                }
                let audio: Vec<f32> = t.audio.iter().map(|x| x * self.tx_gain).collect();
                if let Some(d) = &self.record {
                    let _ = write_wav(&d.join(format!("{:04}_tx.wav", self.tx_count)), &audio);
                }
                self.tx_count += 1;
                let end = self.out.play(&audio);
                self.tx = Some(Tx { end, drained_at: None, deadline: now + t.seconds as f64 + 3.0 });
            }
        }
        self.station.take_output()
    }

    /// Release PTT (on shutdown).
    pub fn unkey(&mut self) {
        self.out.flush();
        let _ = self.ptt.set(false);
    }
}

/// Run until `stop` is set. Returns an error if a device cannot be opened.
pub fn run(cfg: RadioConfig, stop: Arc<AtomicBool>) -> io::Result<()> {
    let params = OfdmParams::by_name(&cfg.mode).ok_or_else(|| io::Error::other("mode must be narrow or wide"))?;
    let modem = Arc::new(Modem::new(params, cfg.txdelay_s));
    let ptt = vardefm_ptt::open(&cfg.ptt)?;
    let input = open_input(cfg.audio_in.as_deref(), cfg.in_channel)?;
    let output = open_output(cfg.audio_out.as_deref(), cfg.out_channel)?;
    let port = HostPort::bind(&cfg.host, cfg.port, cfg.port + 1)?;
    if let Some(d) = &cfg.record {
        std::fs::create_dir_all(d)?;
    }
    println!("VardeFM {} mode", cfg.mode);
    println!("  audio in : {} ({} Hz{})", input.name, input.device_rate, resample_note(input.device_rate));
    println!("  audio out: {} ({} Hz{})", output.name, output.device_rate, resample_note(output.device_rate));
    println!("  PTT      : {}", ptt.describe());
    println!("  host     : command {}, data {}", port.cmd_addr, port.data_addr);

    let mut engine = RadioEngine::new(modem, ptt, output, cfg.tx_gain, cfg.txtail_s, cfg.record.clone(), std::process::id());
    while !stop.load(Ordering::Relaxed) {
        let chunk = input.rx.recv_timeout(Duration::from_millis(50)).ok();
        let host_in: Vec<HostIn> = port.inbox.try_iter().collect();
        for o in engine.step(chunk.as_deref(), host_in) {
            match o {
                HostOut::Line(l) => port.send_line(&l),
                HostOut::Data(d) => port.send_data(&d),
            }
        }
        for l in engine.log.drain(..) {
            if cfg.verbose || l.contains("failed") || l.contains("watchdog") {
                println!("{l}");
            }
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

fn write_wav(path: &std::path::Path, samples: &[f32]) -> Result<(), hound::Error> {
    let spec = hound::WavSpec { channels: 1, sample_rate: RATE, bits_per_sample: 16, sample_format: hound::SampleFormat::Int };
    let mut w = hound::WavWriter::create(path, spec)?;
    for &v in samples {
        w.write_sample((v.clamp(-1.0, 1.0) * 32767.0) as i16)?;
    }
    w.finalize()
}

#[cfg(test)]
mod tests;
