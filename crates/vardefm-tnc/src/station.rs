//! One VardeFM station: link + modulator + receiver, speaking the VARA host protocol.
//!
//! The station is driven from outside: feed it host input and received audio, ask
//! it what to transmit, and collect its host output. It has no threads or clock.

use crate::host::{parse, Command};
use std::sync::Arc;
use vardefm_link::{Digipeater, Link, LinkConfig, LinkEvent, TxInfo};
use vardefm_modem::{Header, Modem, Receiver, RxBurst};

/// Output for the host application.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HostOut {
    /// A command-port line (without the CR).
    Line(String),
    /// Bytes for the data port.
    Data(Vec<u8>),
}

/// A burst that has just been keyed up.
pub struct TxStart {
    pub audio: Vec<f32>,
    pub header: Header,
    pub info: TxInfo,
    pub seconds: f32,
    /// A digipeater repeat of another station's burst.
    pub repeat: bool,
}

const IAMALIVE_S: f64 = 60.0;

pub struct Station {
    pub name: String,
    modem: Arc<Modem>,
    pub link: Link,
    digi: Option<Digipeater>,
    /// The burst on the air is a digipeater repeat.
    repeating: bool,
    rx: Receiver,
    out: Vec<HostOut>,
    transmitting: bool,
    busy_reported: bool,
    last_alive: f64,
    bandwidth: &'static str,
}

impl Station {
    pub fn new(name: &str, modem: Arc<Modem>, cfg: LinkConfig, seed: u32) -> Self {
        let bandwidth = if modem.ofdm.p.name == "wide" { "WIDE" } else { "NARROW" };
        Self {
            name: name.to_string(),
            digi: None,
            repeating: false,
            link: Link::new(modem.clone(), cfg, seed),
            rx: Receiver::new(modem.clone()),
            modem,
            out: Vec::new(),
            transmitting: false,
            busy_reported: false,
            last_alive: 0.0,
            bandwidth,
        }
    }

    pub fn modem(&self) -> &Arc<Modem> {
        &self.modem
    }

    /// Repeat the bursts of sessions routed through any of `calls` (none: off).
    /// Works without a host and alongside our own sessions.
    pub fn set_digi(&mut self, calls: &[String]) {
        self.digi = (!calls.is_empty()).then(|| Digipeater::new(calls, self.link.config().turnaround_s));
    }

    pub fn digi(&self) -> Option<&Digipeater> {
        self.digi.as_ref()
    }

    fn line(&mut self, s: impl Into<String>) {
        self.out.push(HostOut::Line(s.into()));
    }

    /// One line from the command port.
    pub fn host_line(&mut self, now: f64, line: &str) {
        let Ok(cmd) = parse(line) else {
            self.line("WRONG");
            return;
        };
        match cmd {
            Command::MyCall(calls) => {
                self.link.set_mycalls(calls);
                self.line("OK");
            }
            Command::Listen(on) => {
                self.link.set_listen(on);
                self.line("OK");
            }
            Command::Connect { source, target, via } => match self.link.connect(now, &source, &target, &via) {
                Ok(()) => self.line("OK"),
                Err(()) => self.line("WRONG"),
            },
            Command::Disconnect => {
                self.line("OK");
                self.link.disconnect(now);
            }
            Command::Abort => {
                self.line("OK");
                self.link.abort(now);
            }
            Command::Version => {
                self.line("OK");
                self.line(format!("VERSION {}", crate::VERSION));
            }
            Command::Ignored => self.line("OK"),
        }
    }

    /// Bytes from the data port (dropped when not connected).
    pub fn host_data(&mut self, data: &[u8]) {
        self.link.send_data(data);
    }

    /// The host closed the command port: drop any session.
    pub fn host_closed(&mut self, now: f64) {
        self.link.abort(now);
    }

    /// Periodic housekeeping (IAMALIVE).
    pub fn tick(&mut self, now: f64) {
        if now - self.last_alive >= IAMALIVE_S {
            self.last_alive = now;
            self.line("IAMALIVE");
        }
    }

    /// If the link wants to transmit, key up and return the audio.
    /// Call only while not transmitting.
    pub fn poll_tx(&mut self, now: f64) -> Option<TxStart> {
        debug_assert!(!self.transmitting);
        // Repeats answer a burst just heard, so they go first.
        let (burst, repeat) = match self.digi.as_mut().and_then(|d| d.poll(now)) {
            Some(b) => (b, true),
            None => (self.link.poll(now, self.rx.busy())?, false),
        };
        let audio = self.modem.modulate(&burst);
        self.transmitting = true;
        self.repeating = repeat;
        self.rx.reset();
        self.line("PTT ON");
        Some(TxStart {
            seconds: audio.len() as f32 / self.modem.fs(),
            audio,
            info: if repeat {
                TxInfo {
                    kind: burst.header.kind,
                    level: burst.header.level,
                    ncw: burst.payload.len(),
                    ..TxInfo::default()
                }
            } else {
                self.link.last_tx().clone()
            },
            header: burst.header,
            repeat,
        })
    }

    /// The burst returned by `poll_tx` has been fully transmitted.
    pub fn tx_done(&mut self, now: f64) {
        self.transmitting = false;
        self.line("PTT OFF");
        if !std::mem::take(&mut self.repeating) {
            self.link.tx_done(now);
        }
    }

    pub fn transmitting(&self) -> bool {
        self.transmitting
    }

    /// Received audio. Returns any bursts decoded (for logging).
    pub fn rx_audio(&mut self, now: f64, samples: &[f32]) -> Vec<RxBurst> {
        if self.transmitting {
            return Vec::new();
        }
        let bursts = self.rx.push(samples);
        for b in &bursts {
            self.link.on_burst(now, b);
            if let Some(d) = &mut self.digi {
                d.on_burst(now, b);
            }
        }
        let busy = self.rx.busy();
        if busy != self.busy_reported {
            self.busy_reported = busy;
            self.line(if busy { "BUSY ON" } else { "BUSY OFF" });
        }
        bursts
    }

    /// Everything for the host since the last call, in order.
    pub fn take_output(&mut self) -> Vec<HostOut> {
        for e in self.link.take_events() {
            let o = match e {
                LinkEvent::Pending => HostOut::Line("PENDING".into()),
                LinkEvent::CancelPending => HostOut::Line("CANCELPENDING".into()),
                LinkEvent::Connected { source, destination } => {
                    HostOut::Line(format!("CONNECTED {source} {destination} {}", self.bandwidth))
                }
                LinkEvent::Disconnected => HostOut::Line("DISCONNECTED".into()),
                LinkEvent::Buffer(n) => HostOut::Line(format!("BUFFER {n}")),
                LinkEvent::Data(d) => HostOut::Data(d),
            };
            self.out.push(o);
        }
        std::mem::take(&mut self.out)
    }
}
