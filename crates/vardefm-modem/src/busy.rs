//! Channel-busy detection from received audio.
//!
//! The channel is busy while the audio is louder than the background. With the
//! squelch closed the background is silence, so any signal counts. With the
//! squelch open the background is the receiver's hiss, which is steady: it is
//! learned and not counted as busy, otherwise the modem would never transmit.
//!
//! The background is a low percentile of the level over the last couple of
//! seconds, so short quiet spells (a VOX hang, a carrier with no audio) do not
//! pull it down, and a steady sound is accepted as background within that time.
//!
//! Only the mode's carrier band is measured. Sound outside it (a CTCSS tone,
//! mains hum, hiss above the band) neither makes the channel busy nor raises the
//! background over a signal inside it.

use std::collections::VecDeque;
use vardefm_dsp::filter::Chain;
use vardefm_dsp::OfdmParams;

/// Busy needs at least this level (RMS, full scale = 1), whatever the background.
const MIN_RMS: f32 = 0.01;
/// Busy needs this much more power than the background (6 dB).
const ABOVE_BACKGROUND: f32 = 4.0;
/// Level measurement block, seconds.
const BLOCK_S: f32 = 0.01;
/// How much history the background is taken from, seconds.
const HISTORY_S: f32 = 2.0;
/// The background is this percentile of the block levels in the history.
const PERCENTILE: f32 = 0.2;
/// How long the channel stays busy after the last loud block, seconds.
const HOLD_S: f32 = 0.15;
/// The measured band starts at least this high. CTCSS tones reach 254.1 Hz, too
/// close under the lowest wide carrier (281 Hz) to be filtered out there; the two
/// wide carriers below this carry little of a burst's power.
const MIN_LOW_HZ: f32 = 400.0;

pub struct BusyDetector {
    /// Band-pass filter (empty: broadband).
    band: Chain,
    block: usize,
    history: usize,
    hold: usize,
    /// Sum and sum of squares of the current partial block.
    sum: f64,
    sq: f64,
    n: usize,
    /// Power (variance) of recent blocks, oldest first.
    powers: VecDeque<f32>,
    /// Samples since the last loud block.
    quiet: usize,
}

impl BusyDetector {
    /// Measures the carrier band of the mode `p`.
    pub fn new(p: &OfdmParams) -> Self {
        let lo = p.carrier_freq(0).max(MIN_LOW_HZ);
        let hi = p.carrier_freq(p.num_carriers - 1);
        // 12th order below the band and 8th order above it, -9 and -6 dB at the edges.
        // A CTCSS tone at 254.1 Hz comes out 48 dB down: at 1500 Hz deviation on the
        // wide port (-9 dBFS) that is still 17 dB under the -40 dBFS minimum. One
        // 4th-order band-pass alone would take it down only 16 dB.
        let band = Chain::bandpass4(p.fs, lo, hi).then(Chain::bandpass4(p.fs, lo, hi)).then(Chain::highpass4(p.fs, lo));
        Self::with_band(p.fs, band)
    }

    /// Measures all of the audio, as before the band was limited (for comparison).
    #[cfg(test)]
    pub(crate) fn broadband(fs: f32) -> Self {
        Self::with_band(fs, Chain::new())
    }

    fn with_band(fs: f32, band: Chain) -> Self {
        let block = (BLOCK_S * fs).round().max(1.0) as usize;
        Self {
            band,
            block,
            history: (HISTORY_S / BLOCK_S).round() as usize,
            hold: (HOLD_S * fs) as usize,
            sum: 0.0,
            sq: 0.0,
            n: 0,
            powers: VecDeque::new(),
            quiet: usize::MAX / 2,
        }
    }

    pub fn busy(&self) -> bool {
        self.quiet < self.hold
    }

    pub fn push(&mut self, samples: &[f32]) {
        for &x in samples {
            let x = self.band.process(x);
            self.sum += x as f64;
            self.sq += (x * x) as f64;
            self.n += 1;
            if self.n == self.block {
                self.end_block();
            }
        }
    }

    fn end_block(&mut self) {
        let n = self.n as f64;
        // Variance: a DC offset from the sound card is not a signal.
        let power = (self.sq / n - (self.sum / n).powi(2)).max(0.0) as f32;
        (self.sum, self.sq, self.n) = (0.0, 0.0, 0);

        let loud = power > MIN_RMS * MIN_RMS && power > ABOVE_BACKGROUND * self.background(power);
        if loud {
            self.quiet = 0;
        } else {
            self.quiet = self.quiet.saturating_add(self.block);
        }
        if self.powers.len() == self.history {
            self.powers.pop_front();
        }
        self.powers.push_back(power);
    }

    /// Background power from the history (`first` when there is none yet).
    fn background(&self, first: f32) -> f32 {
        if self.powers.is_empty() {
            return first;
        }
        let mut v: Vec<f32> = self.powers.iter().copied().collect();
        let k = ((v.len() as f32 * PERCENTILE) as usize).min(v.len() - 1);
        *v.select_nth_unstable_by(k, f32::total_cmp).1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f32 = 48000.0;

    fn narrow() -> BusyDetector {
        BusyDetector::new(&OfdmParams::narrow())
    }

    /// Deterministic white-ish noise with the given RMS.
    fn noise(seed: &mut u32, rms: f32, seconds: f32) -> Vec<f32> {
        (0..(seconds * FS) as usize)
            .map(|_| {
                *seed ^= *seed << 13;
                *seed ^= *seed >> 17;
                *seed ^= *seed << 5;
                // Uniform in [-1, 1) has RMS 1/sqrt(3).
                (*seed as f32 / u32::MAX as f32 * 2.0 - 1.0) * rms * 3f32.sqrt()
            })
            .collect()
    }

    fn tone(rms: f32, seconds: f32) -> Vec<f32> {
        tone_hz(993.0, rms, seconds)
    }

    fn tone_hz(f: f32, rms: f32, seconds: f32) -> Vec<f32> {
        let w = 2.0 * std::f32::consts::PI * f / FS;
        (0..(seconds * FS) as usize).map(|i| rms * 2f32.sqrt() * (i as f32 * w).sin()).collect()
    }

    fn add(a: &[f32], b: &[f32]) -> Vec<f32> {
        a.iter().zip(b).map(|(x, y)| x + y).collect()
    }

    #[test]
    fn closed_squelch_then_signal_is_busy() {
        let mut d = narrow();
        d.push(&vec![0.0; FS as usize]);
        assert!(!d.busy());
        d.push(&tone(0.05, 0.1));
        assert!(d.busy());
        d.push(&vec![0.0; (0.2 * FS) as usize]);
        assert!(!d.busy(), "busy ends after the hold time");
    }

    #[test]
    fn quiet_signal_below_the_minimum_is_not_busy() {
        let mut d = narrow();
        d.push(&vec![0.0; FS as usize]);
        d.push(&tone(0.005, 0.5));
        assert!(!d.busy());
    }

    #[test]
    fn dc_offset_is_not_busy() {
        // A closed squelch measured -42 dBFS rms with -41 dBFS peak: a DC offset.
        let mut d = narrow();
        d.push(&vec![0.0; FS as usize]);
        d.push(&vec![0.02; FS as usize]);
        assert!(!d.busy());
    }

    #[test]
    fn steady_open_squelch_hiss_is_not_busy() {
        // Open squelch measured -27 dBFS rms.
        let mut seed = 1;
        let mut d = narrow();
        d.push(&noise(&mut seed, 0.045, 0.5));
        assert!(!d.busy(), "hiss from the start is background");
        d.push(&noise(&mut seed, 0.045, 10.0));
        assert!(!d.busy());
    }

    #[test]
    fn hiss_returning_after_squelch_opens_is_learned() {
        let mut seed = 2;
        let mut d = narrow();
        d.push(&vec![0.0; FS as usize]);
        d.push(&noise(&mut seed, 0.045, 0.3));
        assert!(d.busy(), "new sound is busy at first");
        d.push(&noise(&mut seed, 0.045, HISTORY_S));
        assert!(!d.busy(), "steady hiss becomes background");
    }

    #[test]
    fn short_silence_does_not_lower_the_background() {
        // A VOX hang or our own transmission: silence for 0.3 s, then hiss again.
        let mut seed = 3;
        let mut d = narrow();
        d.push(&noise(&mut seed, 0.045, 3.0));
        d.push(&vec![0.0; (0.3 * FS) as usize]);
        d.push(&noise(&mut seed, 0.045, 0.2));
        assert!(!d.busy());
    }

    #[test]
    fn signal_well_above_hiss_is_busy() {
        let mut seed = 4;
        let mut d = narrow();
        d.push(&noise(&mut seed, 0.045, 3.0));
        let hiss = noise(&mut seed, 0.045, 0.3);
        d.push(&add(&hiss, &tone(0.2, 0.3)));
        assert!(d.busy());
    }

    #[test]
    fn a_non_finite_sample_does_not_make_it_deaf() {
        for bad in [f32::NAN, f32::INFINITY] {
            let mut d = narrow();
            d.push(&vec![0.0; FS as usize]);
            d.push(&[bad]);
            d.push(&vec![0.0; FS as usize]);
            d.push(&tone(0.3, 0.1));
            assert!(d.busy(), "deaf after {bad}");
        }
    }

    #[test]
    fn ctcss_tone_is_not_busy() {
        // The highest CTCSS tone at 750 Hz deviation on the wide data port.
        // It fades in over 20 ms: a tone that starts abruptly clicks across the band.
        // The click when a squelch opens on a tone reads busy for the hold time; the
        // busy_eval case "strong carrier, CTCSS only" measures it.
        let mut ctcss = tone_hz(254.1, 0.177, 0.5);
        for (i, x) in ctcss.iter_mut().take(960).enumerate() {
            *x *= i as f32 / 960.0;
        }
        let mut d = BusyDetector::new(&OfdmParams::wide());
        d.push(&vec![0.0; FS as usize]);
        for (i, block) in ctcss.chunks(480).enumerate() {
            d.push(block);
            assert!(!d.busy(), "busy after {} ms", 10 * (i + 1));
        }
    }

    #[test]
    fn sound_above_the_band_is_not_busy() {
        let mut d = BusyDetector::new(&OfdmParams::wide());
        d.push(&vec![0.0; FS as usize]);
        for (i, block) in tone_hz(8000.0, 0.1, 0.5).chunks(480).enumerate() {
            d.push(block);
            assert!(!d.busy(), "busy after {} ms", 10 * (i + 1));
        }
    }

    #[test]
    fn broadband_counts_a_ctcss_tone() {
        // What the band limit is for: measured over all of the audio the tone is new sound.
        let mut d = BusyDetector::broadband(FS);
        d.push(&vec![0.0; FS as usize]);
        d.push(&tone_hz(254.1, 0.177, 0.1));
        assert!(d.busy());
    }
}
