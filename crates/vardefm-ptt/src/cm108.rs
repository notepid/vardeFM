//! PTT through a GPIO pin of a CM108/CM119-family USB audio chip.
//!
//! Supported devices and report format follow Dire Wolf's cm108.c (WB2OSZ,
//! GPL-2.0-or-later). To change a pin, the host sends a 4-byte HID output report
//! (5 bytes with the leading report id 0):
//!
//! ```text
//! [0x00 report id] [0x00] [data: pin levels] [mask: pins driven as outputs] [0x00]
//! ```
//! with `mask = data = 1 << (gpio - 1)` to key and `data = 0` to release.
//! GPIO 3 (chip pin 13, at the end of the package) is the usual PTT pin.

use crate::Ptt;
use hidapi::{HidApi, HidDevice};
use std::ffi::CString;
use std::io;

const CMEDIA: u16 = 0x0d8c;
const SSS: u16 = 0x0c76;
const AIOC: (u16, u16) = (0x1209, 0x7388);

/// True for USB IDs of chips with CM108-compatible GPIO.
///
/// | Chip | VID:PID | GPIOs |
/// |---|---|---|
/// | CM108, CM109, CM119 | 0d8c:0008-000f | 4 / 8 / 8 |
/// | CM108AH | 0d8c:0139 (also seen as 013c) | 1, 3, 4 |
/// | CM108B | 0d8c:0012 | 1, 3, 4 |
/// | CM119A | 0d8c:013a | 8 |
/// | CM119B | 0d8c:0013 | 8 |
/// | SSS1621 / SSS1623 | 0c76:1605, 1607, 160b | 2 |
/// | AIOC (All-In-One Cable) | 1209:7388 | emulated |
pub fn is_cm108_family(vid: u16, pid: u16) -> bool {
    match vid {
        CMEDIA => matches!(pid, 0x0008..=0x000f | 0x0012 | 0x0013 | 0x0139 | 0x013a | 0x013c),
        SSS => matches!(pid, 0x1605 | 0x1607 | 0x160b),
        _ => (vid, pid) == AIOC,
    }
}

#[derive(Clone, Debug)]
pub struct HidDeviceInfo {
    pub path: String,
    pub vid: u16,
    pub pid: u16,
    pub manufacturer: String,
    pub product: String,
    pub interface: i32,
}

/// All CM108-family HID devices present.
pub fn find_cm108() -> io::Result<Vec<HidDeviceInfo>> {
    let api = HidApi::new().map_err(io::Error::other)?;
    let mut out: Vec<HidDeviceInfo> = api
        .device_list()
        .filter(|d| is_cm108_family(d.vendor_id(), d.product_id()))
        .map(|d| HidDeviceInfo {
            path: d.path().to_string_lossy().into_owned(),
            vid: d.vendor_id(),
            pid: d.product_id(),
            manufacturer: d.manufacturer_string().unwrap_or_default().to_string(),
            product: d.product_string().unwrap_or_default().to_string(),
            interface: d.interface_number(),
        })
        .collect();
    // One physical chip can show up once per HID collection; keep one entry per path.
    out.sort_by(|a, b| a.path.cmp(&b.path));
    out.dedup_by(|a, b| a.path == b.path);
    Ok(out)
}

pub struct Cm108 {
    dev: HidDevice,
    path: String,
    gpio: u8,
}

impl Cm108 {
    /// Open `path`, or the only CM108-family device if `path` is None.
    pub fn open(path: Option<&str>, gpio: u8) -> io::Result<Self> {
        assert!((1..=8).contains(&gpio));
        let path = match path {
            Some(p) => p.to_string(),
            None => {
                let found = find_cm108()?;
                match found.len() {
                    0 => return Err(io::Error::other("no CM108/CM119-type USB sound device found")),
                    1 => found[0].path.clone(),
                    n => {
                        let list: Vec<String> = found.iter().map(|d| format!("  {}", d.path)).collect();
                        return Err(io::Error::other(format!(
                            "{n} CM108-type devices found; pick one with --ptt cm108@<path>:\n{}",
                            list.join("\n")
                        )));
                    }
                }
            }
        };
        let api = HidApi::new().map_err(io::Error::other)?;
        let c = CString::new(path.clone()).map_err(io::Error::other)?;
        let dev = api
            .open_path(&c)
            .map_err(|e| io::Error::other(format!("cannot open HID device {path}: {e}")))?;
        Ok(Self { dev, path, gpio })
    }

    /// The HID output report that sets our pin.
    pub fn report(gpio: u8, on: bool) -> [u8; 5] {
        let bit = 1u8 << (gpio - 1);
        [0, 0, if on { bit } else { 0 }, bit, 0]
    }
}

impl Ptt for Cm108 {
    fn set(&mut self, on: bool) -> io::Result<()> {
        self.dev
            .write(&Self::report(self.gpio, on))
            .map(|_| ())
            .map_err(|e| io::Error::other(format!("CM108 write to {} failed: {e}", self.path)))
    }

    fn describe(&self) -> String {
        format!("CM108 GPIO {} on {}", self.gpio, self.path)
    }
}

impl Drop for Cm108 {
    fn drop(&mut self) {
        let _ = self.set(false);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn report_bytes() {
        assert_eq!(Cm108::report(3, true), [0, 0, 0x04, 0x04, 0]);
        assert_eq!(Cm108::report(3, false), [0, 0, 0x00, 0x04, 0]);
        assert_eq!(Cm108::report(1, true), [0, 0, 0x01, 0x01, 0]);
        assert_eq!(Cm108::report(8, true), [0, 0, 0x80, 0x80, 0]);
    }

    #[test]
    fn device_ids() {
        for (v, p) in [(0x0d8c, 0x000c), (0x0d8c, 0x0008), (0x0d8c, 0x013a), (0x0d8c, 0x0012), (0x0c76, 0x1607), (0x1209, 0x7388)] {
            assert!(is_cm108_family(v, p), "{v:04x}:{p:04x}");
        }
        for (v, p) in [(0x0d8c, 0x0014), (0x08bb, 0x2904), (0x1209, 0x0001)] {
            assert!(!is_cm108_family(v, p), "{v:04x}:{p:04x}");
        }
    }
}
