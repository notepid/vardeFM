//! Station + PTT + audio output, stepped once per received audio chunk.
//!
//! Time is the caller's sample clock. While transmitting, received audio is
//! ignored (half duplex). PTT is keyed before a burst's audio is queued, and
//! released only after the sink has taken the last sample plus `txtail_s`. A
//! watchdog unkeys the radio if a transmission runs far longer than expected.
//!
//! [`RadioEngine`] holds that logic with the sound card output and PTT behind
//! traits, so it can be driven without hardware threads of its own.

use crate::host::valid_call;
use crate::level::{meter_line, LevelMeter, Verdict};
use crate::station::{HostOut, Station};
use std::io;
use std::sync::Arc;
use vardefm_link::{valid_hop_margin, LinkConfig, TxInfo, MAX_HOP_MARGIN_S};
use vardefm_modem::{AudioLevel, Header, Modem, OfdmParams, LEVELS};

/// Sample rate the engine expects for received and transmitted audio (Hz).
pub const RATE: u32 = 48_000;

/// Input from the host application, in one ordered stream.
#[derive(Debug)]
pub enum HostIn {
    /// One command line (without the terminator).
    Line(String),
    /// Bytes written to the data port.
    Data(Vec<u8>),
    /// The command-port client went away.
    CmdClosed,
}

/// Push-to-talk. `set` may block (for example until a radio confirms it has
/// un-keyed); the sample clock only advances when the caller feeds audio or
/// advances time, so a long return is harmless. Nothing in the engine assumes
/// that `set(false)` returns immediately.
pub trait Ptt: Send {
    fn set(&mut self, on: bool) -> io::Result<()>;
}

/// Structured per-burst statistics. Also rendered as lines in [`RadioEngine::log`].
#[derive(Clone, Debug)]
pub enum Report {
    Tx { time_s: f64, repeat: bool, header: Header, info: TxInfo, air_s: f32 },
    Rx { time_s: f64, header: Header, codewords_ok: usize, snr_db: f32, level: AudioLevel },
}

impl Report {
    /// The same line the desktop CLI prints for this report.
    pub fn log_line(&self) -> String {
        match self {
            Self::Tx { time_s, repeat, header: h, info, air_s } => format!(
                "{time_s:8.2}s {} {:<10} {:<9} cw {:>2} {:>5} B (resent {:>4}) retry {} air {air_s:.2} s",
                if *repeat { "rpt" } else { "tx " },
                format!("{:?}", h.kind),
                if h.ncw > 0 { LEVELS[h.level as usize].name } else { "-" },
                h.ncw,
                info.bytes,
                info.resent_bytes,
                info.retry,
            ),
            Self::Rx { time_s, header: h, codewords_ok, snr_db, level } => format!(
                "{time_s:8.2}s rx {:<10} cw {codewords_ok:>2}/{:<2} SNR {snr_db:5.1} dB  peak {:5.1} dBFS",
                format!("{:?}", h.kind),
                h.ncw,
                level.peak_dbfs
            ),
        }
    }
}

#[derive(Clone, Debug)]
pub struct EngineConfig {
    pub mode: String,
    /// Leader before each burst (radio key-up time plus margin).
    pub txdelay_s: f32,
    /// PTT held after the last sample leaves the sink.
    pub txtail_s: f32,
    /// Output level multiplier (sets FM deviation together with the sound card mixer).
    pub tx_gain: f32,
    /// Highest speed level to send or request (limits the decoding load on slow computers).
    pub max_level: u8,
    /// Put a receive level line in `notices` every [`LEVEL_WINDOW_S`] seconds while idle.
    pub show_levels: bool,
    /// Act as a digipeater for these call signs (empty: off).
    pub digi_calls: Vec<String>,
    /// Extra wait per digipeater hop on our own routed sessions.
    pub hop_margin_s: f64,
}

impl EngineConfig {
    /// Refuses values the engine cannot run with.
    pub fn check(&self) -> io::Result<()> {
        let invalid = |what: String| Err(io::Error::new(io::ErrorKind::InvalidInput, what));
        for (name, s) in [("txdelay", self.txdelay_s), ("txtail", self.txtail_s), ("tx gain", self.tx_gain)] {
            if !(s.is_finite() && s >= 0.0) {
                return invalid(format!("{name} must be 0 or more"));
            }
        }
        if self.max_level as usize >= LEVELS.len() {
            return invalid(format!("max level must be from 0 to {}", LEVELS.len() - 1));
        }
        if !valid_hop_margin(self.hop_margin_s) {
            return invalid(format!("hop margin must be from 0 to {MAX_HOP_MARGIN_S} seconds"));
        }
        if let Some(c) = self.digi_calls.iter().find(|c| !valid_call(c)) {
            return invalid(format!("digipeater {c:?}: not a valid call sign"));
        }
        Ok(())
    }
}

/// Seconds per line of level-meter output.
pub const LEVEL_WINDOW_S: f32 = 2.0;
/// Shortest time between two level warnings.
const LEVEL_WARN_EVERY_S: f64 = 30.0;

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            mode: "narrow".into(),
            txdelay_s: 0.15,
            txtail_s: 0.05,
            tx_gain: 1.0,
            max_level: LEVELS.len() as u8 - 1,
            show_levels: false,
            digi_calls: Vec::new(),
            hop_margin_s: LinkConfig::default().hop_margin_s,
        }
    }
}

/// Where transmit audio goes.
pub trait AudioSink {
    /// Queue samples; returns the `consumed()` value at which they have all been taken.
    fn play(&mut self, samples: &[f32]) -> u64;
    /// Samples taken by the sink so far.
    fn consumed(&self) -> u64;
    /// Drop queued samples.
    fn flush(&mut self);
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
    /// Transmit level multiplier (default 1).
    pub tx_gain: f32,
    /// Seconds PTT stays on after the last sample has left the queue.
    tail_s: f64,
    rx_pos: u64,
    tx: Option<Tx>,
    /// Structured per-burst statistics, drained by the caller.
    pub reports: Vec<Report>,
    /// Log lines (one per burst sent or received), drained by the caller.
    /// Formatted from [`Self::reports`] so the two stay in lockstep.
    pub log: Vec<String>,
    /// Lines to show even when the log is quiet (level meter and warnings).
    pub notices: Vec<String>,
    /// Put a receive level line in `notices` every [`LEVEL_WINDOW_S`] seconds while idle.
    pub show_levels: bool,
    meter: LevelMeter,
    /// When the last level warning went out.
    last_level_warning: Option<f64>,
}

impl<S: AudioSink> RadioEngine<S> {
    pub fn new(modem: Arc<Modem>, link: LinkConfig, ptt: Box<dyn Ptt>, out: S, txtail_s: f32, seed: u32) -> Self {
        Self {
            station: Station::new("local", modem, link, seed),
            ptt,
            out,
            tx_gain: 1.0,
            tail_s: txtail_s as f64,
            rx_pos: 0,
            tx: None,
            reports: Vec::new(),
            log: Vec::new(),
            notices: Vec::new(),
            show_levels: false,
            meter: LevelMeter::new((LEVEL_WINDOW_S * RATE as f32) as usize),
            last_level_warning: None,
        }
    }

    fn push_report(&mut self, report: Report) {
        self.log.push(report.log_line());
        self.reports.push(report);
    }

    /// An engine set up as `cfg` asks: the link's limits and hop margin, gain, logging
    /// and the digipeater's call signs. Fails if `cfg` does not pass [`EngineConfig::check`].
    pub fn from_config(
        cfg: &EngineConfig,
        modem: Arc<Modem>,
        ptt: Box<dyn Ptt>,
        out: S,
        seed: u32,
    ) -> io::Result<Self> {
        cfg.check()?;
        let link = LinkConfig {
            max_level: cfg.max_level,
            start_level: cfg.max_level.min(1),
            hop_margin_s: cfg.hop_margin_s,
            ..LinkConfig::default()
        };
        let mut engine = Self::new(modem, link, ptt, out, cfg.txtail_s, seed);
        engine.tx_gain = cfg.tx_gain;
        engine.show_levels = cfg.show_levels;
        engine.station.set_digi(&cfg.digi_calls);
        Ok(engine)
    }

    /// Build a modem for `cfg.mode` and `cfg.txdelay_s`, then [`Self::from_config`].
    /// `decode_threads` is the number of threads that decode a burst (`None`: one
    /// per CPU core; 1: decode on the calling thread).
    pub fn open(
        cfg: &EngineConfig,
        decode_threads: Option<usize>,
        ptt: Box<dyn Ptt>,
        out: S,
        seed: u32,
    ) -> io::Result<Self> {
        // Before the modem: building its codes takes a while.
        cfg.check()?;
        let params = OfdmParams::by_name(&cfg.mode).ok_or_else(|| io::Error::other("mode must be narrow or wide"))?;
        let mut modem = Modem::new(params, cfg.txdelay_s);
        if let Some(n) = decode_threads {
            modem.decode_threads = n;
        }
        Self::from_config(cfg, Arc::new(modem), ptt, out, seed)
    }

    /// Warn about a bad receive level, at most every [`LEVEL_WARN_EVERY_S`] seconds.
    fn level_warning(&mut self, now: f64, what: &str, level: AudioLevel) {
        let verdict = Verdict::of(level);
        if !matches!(verdict, Verdict::Clipping | Verdict::Loud | Verdict::Quiet) {
            return;
        }
        if self.last_level_warning.is_some_and(|t| now - t < LEVEL_WARN_EVERY_S) {
            return;
        }
        self.last_level_warning = Some(now);
        self.notices.push(format!(
            "{now:8.2}s {what} peaks at {:.1} dBFS: {} (aim for about -6 dBFS)",
            level.peak_dbfs,
            verdict.advice()
        ));
    }

    pub fn now(&self) -> f64 {
        self.rx_pos as f64 / RATE as f64
    }

    pub fn transmitting(&self) -> bool {
        self.tx.is_some()
    }

    /// Advance with one chunk of received audio (or `None` if the sound card had
    /// nothing) and the host input since the last step. Returns host output.
    ///
    /// The sample clock advances by `chunk.len()` when a chunk is given. While
    /// transmitting the caller must keep the clock running even if the radio
    /// returns no RX audio: either keep calling `step` with silence, or use
    /// [`Self::advance`].
    pub fn step(&mut self, chunk: Option<&[f32]>, host_in: Vec<HostIn>) -> Vec<HostOut> {
        if let Some(c) = chunk {
            self.rx_pos += c.len() as u64;
        }
        self.run(chunk, host_in)
    }

    /// Advance the sample clock by `samples` without feeding audio content.
    ///
    /// Use this while keyed when the radio is not delivering RX audio (common on
    /// half-duplex network paths). The PTT tail and watchdog are measured against
    /// this clock. Host input is handled the same way as in [`Self::step`].
    pub fn advance(&mut self, samples: u64, host_in: Vec<HostIn>) -> Vec<HostOut> {
        self.rx_pos += samples;
        self.run(None, host_in)
    }

    fn run(&mut self, chunk: Option<&[f32]>, host_in: Vec<HostIn>) -> Vec<HostOut> {
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
                    let report = Report::Rx {
                        time_s: now,
                        header: b.header,
                        codewords_ok: ok,
                        snr_db: b.snr_db,
                        level: b.level,
                    };
                    self.level_warning(now, "received burst", b.level);
                    self.push_report(report);
                }
                if let Some(level) = self.meter.push(c) {
                    if self.show_levels {
                        self.notices.push(format!("{now:8.2}s {}", meter_line(level)));
                    } else if Verdict::of(level) == Verdict::Clipping {
                        // Noise or voice that clips will clip our bursts too.
                        self.level_warning(now, "receive audio", level);
                    }
                }
            }
            if let Some(t) = self.station.poll_tx(now) {
                let h = t.header;
                self.push_report(Report::Tx {
                    time_s: now,
                    repeat: t.repeat,
                    header: h,
                    info: t.info,
                    air_s: t.seconds,
                });
                self.meter.reset();
                // Key first, then queue the audio: the leader covers the radio's key-up time.
                if let Err(e) = self.ptt.set(true) {
                    self.log.push(format!("{now:8.2}s PTT on failed: {e}"));
                }
                let audio: Vec<f32> = t.audio.iter().map(|x| x * self.tx_gain).collect();
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

#[cfg(test)]
mod tests;
