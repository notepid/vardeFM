//! Receive level meter, with advice for setting the radio's volume knob.
//!
//! The radio's speaker output (and so the volume knob) sets the level the sound
//! card hears. Received bursts should peak around -6 dBFS: loud enough to stay
//! well above the sound card's own noise, with room before clipping.

use vardefm_modem::AudioLevel;

/// Peaks at or above this are taken as clipping.
const CLIP_DBFS: f32 = -1.0;
/// Peaks above this leave too little headroom.
const LOUD_DBFS: f32 = -3.0;
/// Peaks below this are too quiet.
const QUIET_DBFS: f32 = -30.0;
/// Peaks below this mean there is no audio at all.
const SILENT_DBFS: f32 = -80.0;

/// How a level compares with the target.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    Silent,
    Quiet,
    Ok,
    Loud,
    Clipping,
}

impl Verdict {
    pub fn of(level: AudioLevel) -> Self {
        let p = level.peak_dbfs;
        if p >= CLIP_DBFS {
            Self::Clipping
        } else if p > LOUD_DBFS {
            Self::Loud
        } else if p < SILENT_DBFS {
            Self::Silent
        } else if p < QUIET_DBFS {
            Self::Quiet
        } else {
            Self::Ok
        }
    }

    /// What to do about it.
    pub fn advice(self) -> &'static str {
        match self {
            Self::Silent => "no audio: squelch closed, or check the cable and the recording level",
            Self::Quiet => "quiet: turn the radio volume up",
            Self::Ok => "ok",
            Self::Loud => "loud: turn the radio volume down a little",
            Self::Clipping => "clipping: turn the radio volume down",
        }
    }
}

impl Verdict {
    /// Advice for audio measured while no burst is being received. A quiet
    /// level then usually means the squelch is closed, not that the volume is low.
    pub fn idle_advice(self) -> &'static str {
        match self {
            Self::Quiet => "quiet: squelch closed or nothing heard; set the volume while a station transmits",
            other => other.advice(),
        }
    }
}

/// Measures audio over windows of a fixed length.
pub struct LevelMeter {
    window: usize,
    sq: f64,
    peak: f32,
    n: usize,
}

impl LevelMeter {
    /// A meter that reports every `window` samples.
    pub fn new(window: usize) -> Self {
        Self { window: window.max(1), sq: 0.0, peak: 0.0, n: 0 }
    }

    /// Add samples; returns the level once a window is full.
    pub fn push(&mut self, samples: &[f32]) -> Option<AudioLevel> {
        for &x in samples {
            self.sq += (x * x) as f64;
            self.peak = self.peak.max(x.abs());
        }
        self.n += samples.len();
        if self.n < self.window {
            return None;
        }
        let level = AudioLevel {
            rms_dbfs: (10.0 * (self.sq / self.n as f64).max(1e-12).log10()) as f32,
            peak_dbfs: 20.0 * self.peak.max(1e-6).log10(),
        };
        self.reset();
        Some(level)
    }

    /// Drop the partial window (e.g. while transmitting).
    pub fn reset(&mut self) {
        (self.sq, self.peak, self.n) = (0.0, 0.0, 0);
    }
}

/// One meter line for idle audio: `level  -22.6 dBFS rms   -8.3 dBFS peak |#####   ok`.
pub fn meter_line(level: AudioLevel) -> String {
    let bar = "#".repeat(((level.rms_dbfs + 60.0).max(0.0) / 2.0) as usize);
    format!(
        "level {:6.1} dBFS rms {:6.1} dBFS peak |{bar:<30} {}",
        level.rms_dbfs,
        level.peak_dbfs,
        Verdict::of(level).idle_advice()
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn peak(p: f32) -> AudioLevel {
        AudioLevel { rms_dbfs: p - 10.0, peak_dbfs: p }
    }

    #[test]
    fn verdicts() {
        assert_eq!(Verdict::of(peak(0.0)), Verdict::Clipping);
        assert_eq!(Verdict::of(peak(-2.0)), Verdict::Loud);
        assert_eq!(Verdict::of(peak(-6.0)), Verdict::Ok);
        assert_eq!(Verdict::of(peak(-29.0)), Verdict::Ok);
        assert_eq!(Verdict::of(peak(-40.0)), Verdict::Quiet);
        assert_eq!(Verdict::of(peak(-120.0)), Verdict::Silent);
    }

    #[test]
    fn meter_reports_each_window() {
        let mut m = LevelMeter::new(100);
        assert!(m.push(&[0.5; 60]).is_none());
        let l = m.push(&[0.5; 60]).unwrap();
        assert!((l.peak_dbfs - 20.0 * 0.5f32.log10()).abs() < 0.01);
        assert!((l.rms_dbfs - l.peak_dbfs).abs() < 0.01, "a constant has rms = peak");
        assert!(m.push(&[0.0; 50]).is_none(), "the window starts over");
        let quiet = m.push(&[0.0; 50]).unwrap();
        assert_eq!(Verdict::of(quiet), Verdict::Silent);
    }

    #[test]
    fn idle_meter_does_not_ask_for_more_volume_on_a_closed_squelch() {
        // Measured on a handheld with the squelch closed.
        let closed = AudioLevel { rms_dbfs: -42.2, peak_dbfs: -41.3 };
        assert!(meter_line(closed).contains("squelch closed"), "{}", meter_line(closed));
        assert_eq!(Verdict::of(closed).advice(), "quiet: turn the radio volume up");
    }
}
