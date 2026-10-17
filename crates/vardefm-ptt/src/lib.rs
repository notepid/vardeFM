//! Push-to-talk (PTT) control.
//!
//! | Spec | Method |
//! |---|---|
//! | `none` or `vox` | No PTT line: the radio's VOX, or the host application, keys the radio. |
//! | `cm108[:<gpio>][@<hid path>]` | GPIO pin of a CM108/CM119-family USB sound chip (default GPIO 3). |
//! | `rts:<port>` / `dtr:<port>` | Serial control line, e.g. `rts:COM5`, `dtr:/dev/ttyUSB0`. Prefix `-` to invert (`-rts:COM5`). |
//! | `rigctld[:<host>:<port>]` | Hamlib's rigctld (CAT), default `localhost:4532`. |
//!
//! The CM108 method follows Dire Wolf's `cm108.c` (WB2OSZ, GPL-2.0-or-later):
//! the supported USB IDs, the GPIO numbering and the HID output report
//! `[report id 0, 0, data, mask, 0]`.

mod cm108;

pub use cm108::{find_cm108, is_cm108_family, Cm108, HidDeviceInfo};

use std::fmt;
use std::io::{self, BufRead, BufReader, Write};
use std::net::TcpStream;
use std::str::FromStr;
use std::time::Duration;

/// A PTT line.
pub trait Ptt: Send {
    fn set(&mut self, on: bool) -> io::Result<()>;
    fn describe(&self) -> String;
}

/// Parsed `--ptt` option.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PttSpec {
    None,
    Cm108 { gpio: u8, path: Option<String> },
    Serial { port: String, line: SerialLine, invert: bool },
    Rigctld { addr: String },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SerialLine {
    Rts,
    Dtr,
}

impl FromStr for PttSpec {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, String> {
        let lower = s.to_ascii_lowercase();
        if lower == "none" || lower == "vox" {
            return Ok(Self::None);
        }
        if let Some(rest) = lower.strip_prefix("cm108") {
            // cm108[:<gpio>][@<path>]  (path keeps its original case)
            let (gpio_part, path) = match s.find('@') {
                Some(i) => (&lower[5..i], Some(s[i + 1..].to_string())),
                None => (rest, None),
            };
            let gpio = match gpio_part.strip_prefix(':') {
                Some(g) => g.parse::<u8>().map_err(|_| format!("bad CM108 GPIO number in '{s}'"))?,
                None if gpio_part.is_empty() => 3,
                None => return Err(format!("expected cm108[:<gpio>][@<path>], got '{s}'")),
            };
            if !(1..=8).contains(&gpio) {
                return Err("CM108 GPIO must be 1-8".into());
            }
            return Ok(Self::Cm108 { gpio, path });
        }
        let (invert, body) = match s.strip_prefix('-') {
            Some(b) => (true, b),
            None => (false, s),
        };
        if let Some((kind, port)) = body.split_once(':') {
            let line = match kind.to_ascii_lowercase().as_str() {
                "rts" => Some(SerialLine::Rts),
                "dtr" => Some(SerialLine::Dtr),
                _ => None,
            };
            if let Some(line) = line {
                if port.is_empty() {
                    return Err(format!("missing serial port in '{s}'"));
                }
                return Ok(Self::Serial { port: port.to_string(), line, invert });
            }
        }
        if let Some(rest) = lower.strip_prefix("rigctld") {
            let addr = match rest.strip_prefix(':') {
                Some(a) if !a.is_empty() => a.to_string(),
                None if rest.is_empty() => "localhost:4532".into(),
                _ => return Err(format!("expected rigctld[:<host>:<port>], got '{s}'")),
            };
            return Ok(Self::Rigctld { addr });
        }
        Err(format!(
            "unknown PTT '{s}'; use none, vox, cm108[:<gpio>][@<path>], rts:<port>, dtr:<port> or rigctld[:<host>:<port>]"
        ))
    }
}

impl fmt::Display for PttSpec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::None => write!(f, "none (VOX or host)"),
            Self::Cm108 { gpio, path } => match path {
                Some(p) => write!(f, "CM108 GPIO {gpio} on {p}"),
                None => write!(f, "CM108 GPIO {gpio} (auto-detect)"),
            },
            Self::Serial { port, line, invert } => {
                write!(f, "{}{:?} on {port}", if *invert { "inverted " } else { "" }, line)
            }
            Self::Rigctld { addr } => write!(f, "rigctld at {addr}"),
        }
    }
}

/// Open a PTT line. The radio is left unkeyed.
pub fn open(spec: &PttSpec) -> io::Result<Box<dyn Ptt>> {
    let mut p: Box<dyn Ptt> = match spec {
        PttSpec::None => Box::new(NoPtt),
        PttSpec::Cm108 { gpio, path } => Box::new(Cm108::open(path.as_deref(), *gpio)?),
        PttSpec::Serial { port, line, invert } => Box::new(SerialPtt::open(port, *line, *invert)?),
        PttSpec::Rigctld { addr } => Box::new(Rigctld::open(addr)?),
    };
    p.set(false)?;
    Ok(p)
}

struct NoPtt;

impl Ptt for NoPtt {
    fn set(&mut self, _on: bool) -> io::Result<()> {
        Ok(())
    }
    fn describe(&self) -> String {
        "none (VOX or host)".into()
    }
}

/// RTS or DTR on a serial port (Digirig and many other interfaces).
pub struct SerialPtt {
    port: Box<dyn serialport::SerialPort>,
    name: String,
    line: SerialLine,
    invert: bool,
}

impl SerialPtt {
    pub fn open(name: &str, line: SerialLine, invert: bool) -> io::Result<Self> {
        let mut port = serialport::new(name, 9600)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| io::Error::other(format!("cannot open serial port {name}: {e}")))?;
        // Some systems raise both lines on open; drop the one we do not use
        // (like Dire Wolf), so it cannot key a second radio input.
        let _ = match line {
            SerialLine::Rts => port.write_data_terminal_ready(false),
            SerialLine::Dtr => port.write_request_to_send(false),
        };
        Ok(Self { port, name: name.to_string(), line, invert })
    }
}

impl Ptt for SerialPtt {
    fn set(&mut self, on: bool) -> io::Result<()> {
        let level = on != self.invert;
        match self.line {
            SerialLine::Rts => self.port.write_request_to_send(level),
            SerialLine::Dtr => self.port.write_data_terminal_ready(level),
        }
        .map_err(|e| io::Error::other(format!("{}: {e}", self.name)))
    }

    fn describe(&self) -> String {
        format!("{}{:?} on {}", if self.invert { "inverted " } else { "" }, self.line, self.name)
    }
}

impl Drop for SerialPtt {
    fn drop(&mut self) {
        let _ = self.set(false);
    }
}

/// Hamlib rigctld: `T 1` / `T 0`, answered with `RPRT 0`.
pub struct Rigctld {
    stream: TcpStream,
    reader: BufReader<TcpStream>,
    addr: String,
}

impl Rigctld {
    pub fn open(addr: &str) -> io::Result<Self> {
        let stream =
            TcpStream::connect(addr).map_err(|e| io::Error::other(format!("cannot reach rigctld at {addr}: {e}")))?;
        stream.set_read_timeout(Some(Duration::from_secs(2)))?;
        stream.set_nodelay(true)?;
        let reader = BufReader::new(stream.try_clone()?);
        Ok(Self { stream, reader, addr: addr.to_string() })
    }
}

impl Ptt for Rigctld {
    fn set(&mut self, on: bool) -> io::Result<()> {
        self.stream.write_all(if on { b"T 1\n" } else { b"T 0\n" })?;
        let mut reply = String::new();
        self.reader.read_line(&mut reply)?;
        match reply.trim() {
            "RPRT 0" => Ok(()),
            r => Err(io::Error::other(format!("rigctld refused PTT: {r}"))),
        }
    }

    fn describe(&self) -> String {
        format!("rigctld at {}", self.addr)
    }
}

impl Drop for Rigctld {
    fn drop(&mut self) {
        let _ = self.set(false);
    }
}

/// Serial ports present on this computer.
pub fn serial_ports() -> Vec<String> {
    serialport::available_ports()
        .map(|v| {
            v.into_iter()
                .map(|p| match p.port_type {
                    serialport::SerialPortType::UsbPort(u) => {
                        format!("{}  USB {:04x}:{:04x} {}", p.port_name, u.vid, u.pid, u.product.unwrap_or_default())
                    }
                    _ => p.port_name,
                })
                .collect()
        })
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_specs() {
        assert_eq!("vox".parse(), Ok(PttSpec::None));
        assert_eq!("cm108".parse(), Ok(PttSpec::Cm108 { gpio: 3, path: None }));
        assert_eq!("CM108:4".parse(), Ok(PttSpec::Cm108 { gpio: 4, path: None }));
        assert_eq!(
            r"cm108:1@\\?\HID#VID_0D8C&PID_000C".parse(),
            Ok(PttSpec::Cm108 { gpio: 1, path: Some(r"\\?\HID#VID_0D8C&PID_000C".into()) })
        );
        assert_eq!("cm108@/dev/hidraw2".parse(), Ok(PttSpec::Cm108 { gpio: 3, path: Some("/dev/hidraw2".into()) }));
        assert_eq!(
            "rts:COM5".parse(),
            Ok(PttSpec::Serial { port: "COM5".into(), line: SerialLine::Rts, invert: false })
        );
        assert_eq!(
            "-dtr:/dev/ttyUSB0".parse(),
            Ok(PttSpec::Serial { port: "/dev/ttyUSB0".into(), line: SerialLine::Dtr, invert: true })
        );
        assert_eq!("rigctld".parse(), Ok(PttSpec::Rigctld { addr: "localhost:4532".into() }));
        assert_eq!("rigctld:10.0.0.5:4532".parse(), Ok(PttSpec::Rigctld { addr: "10.0.0.5:4532".into() }));
        for bad in ["cm108:9", "cm108x", "rts:", "foo", "rigctld:"] {
            assert!(bad.parse::<PttSpec>().is_err(), "{bad}");
        }
    }

    #[test]
    fn rigctld_protocol() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let server = std::thread::spawn(move || {
            let (s, _) = listener.accept().unwrap();
            let mut r = BufReader::new(s.try_clone().unwrap());
            let mut w = s;
            let mut got = Vec::new();
            for _ in 0..3 {
                let mut l = String::new();
                r.read_line(&mut l).unwrap();
                got.push(l.trim().to_string());
                w.write_all(b"RPRT 0\n").unwrap();
            }
            got
        });
        {
            let mut p = open(&PttSpec::Rigctld { addr }).unwrap();
            p.set(true).unwrap();
        } // drop unkeys
        assert_eq!(server.join().unwrap(), vec!["T 0", "T 1", "T 0"]);
    }
}
