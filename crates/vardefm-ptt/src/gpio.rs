//! PTT on a GPIO pin (Raspberry Pi header or any Linux GPIO chip), through the
//! kernel's GPIO character device (`/dev/gpiochipN`).
//!
//! `gpio:<n>` finds the line the kernel names `GPIO<n>`, which is the Pi's
//! BCM pin numbering on every model (on the Pi 5 the header sits on a
//! different chip, so finding it by name matters). If no line has that name
//! (other boards), it falls back to line `<n>` of `gpiochip0`.
//! `gpio:<chip>:<line>` names the chip and line directly.

use std::io;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GpioLine {
    /// BCM pin number (`GPIO17` = header pin 11).
    Pin(u32),
    /// A line on a named chip: `gpiochip0` or `/dev/gpiochip0`.
    ChipLine { chip: String, offset: u32 },
}

impl GpioLine {
    pub fn parse(s: &str) -> Option<Self> {
        match s.rsplit_once(':') {
            Some((chip, offset)) if !chip.is_empty() => {
                Some(Self::ChipLine { chip: chip.to_string(), offset: offset.parse().ok()? })
            }
            Some(_) => None,
            None => Some(Self::Pin(s.parse().ok()?)),
        }
    }
}

impl std::fmt::Display for GpioLine {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Pin(n) => write!(f, "GPIO{n}"),
            Self::ChipLine { chip, offset } => write!(f, "line {offset} of {chip}"),
        }
    }
}

#[cfg(target_os = "linux")]
pub use linux::GpioPtt;

#[cfg(target_os = "linux")]
mod linux {
    use super::*;
    use crate::Ptt;
    use gpiocdev::line::Value;
    use gpiocdev::request::Request;
    use std::path::PathBuf;

    pub struct GpioPtt {
        req: Request,
        label: String,
    }

    impl GpioPtt {
        pub fn open(line: &GpioLine, invert: bool) -> io::Result<Self> {
            let (chip, offset) = match line {
                GpioLine::Pin(n) => match gpiocdev::find_named_line(&format!("GPIO{n}")) {
                    Some(found) => (found.chip, found.info.offset),
                    None => (PathBuf::from("/dev/gpiochip0"), *n),
                },
                GpioLine::ChipLine { chip, offset } => {
                    let path = if chip.starts_with('/') { chip.into() } else { PathBuf::from("/dev").join(chip) };
                    (path, *offset)
                }
            };
            let label = format!("{}{line} ({}, line {offset})", if invert { "inverted " } else { "" }, chip.display());
            let mut builder = Request::builder();
            builder.on_chip(&chip).with_line(offset).with_consumer("vardefm-ptt");
            if invert {
                builder.as_active_low();
            }
            builder.as_output(Value::Inactive);
            let req = builder.request().map_err(|e| {
                io::Error::other(format!(
                    "cannot use {label}: {e} (is your user in the 'gpio' group, and is the pin free?)"
                ))
            })?;
            Ok(Self { req, label })
        }
    }

    impl Ptt for GpioPtt {
        fn set(&mut self, on: bool) -> io::Result<()> {
            let value = if on { Value::Active } else { Value::Inactive };
            self.req.set_lone_value(value).map_err(|e| io::Error::other(format!("{}: {e}", self.label)))
        }

        fn describe(&self) -> String {
            format!("GPIO {}", self.label)
        }
    }

    impl Drop for GpioPtt {
        fn drop(&mut self) {
            let _ = self.set(false);
        }
    }
}

/// Open a GPIO PTT line, unkeyed. Only available on Linux.
pub fn open(line: &GpioLine, invert: bool) -> io::Result<Box<dyn crate::Ptt>> {
    #[cfg(target_os = "linux")]
    return Ok(Box::new(GpioPtt::open(line, invert)?));
    #[cfg(not(target_os = "linux"))]
    {
        let _ = (line, invert);
        Err(io::Error::other("GPIO PTT is only available on Linux (e.g. Raspberry Pi)"))
    }
}
