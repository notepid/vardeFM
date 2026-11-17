//! Power spectrum of a transmission on the air, for occupied bandwidth and
//! adjacent-channel power.

use num_complex::Complex32;
use rustfft::FftPlanner;
use std::f64::consts::PI;

/// Averaged power spectrum of complex baseband, from -fs/2 to fs/2.
pub struct Spectrum {
    /// Power per bin, centre frequency `freq(i)`; sums to the mean signal power.
    pub power: Vec<f64>,
    pub fs: f32,
}

impl Spectrum {
    /// Welch estimate over `segments` (each one stretch of carrier), `nfft` points
    /// per FFT with half overlap. The 4-term Blackman-Harris window keeps leakage
    /// about 92 dB down, so adjacent-channel power can be read down to about -80 dBc.
    pub fn welch<'a>(segments: impl IntoIterator<Item = &'a [Complex32]>, fs: f32, nfft: usize) -> Self {
        let a = [0.35875, 0.48829, 0.14128, 0.01168];
        let window: Vec<f64> = (0..nfft)
            .map(|n| {
                let x = 2.0 * PI * n as f64 / nfft as f64;
                a[0] - a[1] * x.cos() + a[2] * (2.0 * x).cos() - a[3] * (3.0 * x).cos()
            })
            .collect();
        let wpow: f64 = window.iter().map(|w| w * w).sum();
        let fft = FftPlanner::<f32>::new().plan_fft_forward(nfft);
        let mut acc = vec![0f64; nfft];
        let mut frames = 0usize;
        let mut buf = vec![Complex32::new(0.0, 0.0); nfft];
        for seg in segments {
            let mut start = 0;
            while start + nfft <= seg.len() {
                for (b, (x, w)) in buf.iter_mut().zip(seg[start..start + nfft].iter().zip(&window)) {
                    *b = x * *w as f32;
                }
                fft.process(&mut buf);
                for (a, b) in acc.iter_mut().zip(&buf) {
                    *a += b.norm_sqr() as f64;
                }
                frames += 1;
                start += nfft / 2;
            }
        }
        assert!(frames > 0, "no segment holds {nfft} samples");
        // Shift so bin 0 is -fs/2, and scale so the bins sum to the mean power.
        let scale = 1.0 / (frames as f64 * wpow * nfft as f64);
        let power = (0..nfft).map(|i| acc[(i + nfft / 2) % nfft] * scale).collect();
        Spectrum { power, fs }
    }

    pub fn bin_hz(&self) -> f32 {
        self.fs / self.power.len() as f32
    }

    /// Centre frequency of bin `i`, Hz.
    pub fn freq(&self, i: usize) -> f32 {
        (i as f32 - (self.power.len() / 2) as f32) * self.bin_hz()
    }

    pub fn total(&self) -> f64 {
        self.power.iter().sum()
    }

    /// Fraction of the power in `lo..hi` Hz, bins counted by their centre.
    pub fn fraction_in(&self, lo: f32, hi: f32) -> f64 {
        let p: f64 = (0..self.power.len()).filter(|&i| (lo..hi).contains(&self.freq(i))).map(|i| self.power[i]).sum();
        p / self.total()
    }

    /// Fraction of the power further than `half` Hz from the carrier.
    pub fn fraction_outside(&self, half: f32) -> f64 {
        1.0 - self.fraction_in(-half, half)
    }

    /// Occupied bandwidth: the band between the frequencies below and above which
    /// `(1 - fraction) / 2` of the power lies each (ITU-R SM.328), as (low, high) Hz,
    /// interpolated within a bin.
    pub fn occupied(&self, fraction: f64) -> (f32, f32) {
        let total = self.total();
        let tail = (1.0 - fraction) / 2.0 * total;
        let edge = |bins: &mut dyn Iterator<Item = usize>, sign: f32| {
            let mut sum = 0.0;
            for i in bins {
                let p = self.power[i];
                if sum + p >= tail {
                    // The tail ends `(tail - sum) / p` of the way into this bin.
                    let t = ((tail - sum) / p) as f32;
                    return self.freq(i) - sign * self.bin_hz() * (0.5 - t);
                }
                sum += p;
            }
            unreachable!("the tails hold less power than the spectrum")
        };
        let n = self.power.len();
        (edge(&mut (0..n), 1.0), edge(&mut (0..n).rev(), -1.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    const FS: f32 = 48_000.0;
    const NFFT: usize = 4096;

    /// Sum of equal tones every 10 Hz from -`half` to `half` Hz with random phases:
    /// flat power across that band and none outside.
    fn flat_band(half: f32, n: usize) -> Vec<Complex32> {
        let mut rng = StdRng::seed_from_u64(3);
        let tones: Vec<(f32, f32)> = (-(half as i32) / 10..=(half as i32) / 10)
            .map(|k| (k as f32 * 10.0, rng.gen::<f32>() * 2.0 * std::f32::consts::PI))
            .collect();
        (0..n)
            .map(|i| {
                let t = i as f64 / FS as f64;
                tones.iter().map(|&(f, ph)| Complex32::from_polar(1.0, (2.0 * PI * f as f64 * t) as f32 + ph)).sum()
            })
            .collect()
    }

    /// A flat band of known width: 99 % of it is 0.99 times the width, centred.
    /// The wider one (16 kHz) must come out too wide for a 12.5 kHz channel, so a
    /// result that fits is not an artefact of the measurement.
    #[test]
    fn occupied_bandwidth_of_a_flat_band() {
        for half in [4_000.0, 8_000.0] {
            let s = Spectrum::welch([flat_band(half, 1 << 15).as_slice()], FS, NFFT);
            let (lo, hi) = s.occupied(0.99);
            let want = 0.99 * 2.0 * half;
            assert!(((hi - lo) - want).abs() < 2.0 * s.bin_hz(), "half {half}: {lo}..{hi}, want width {want}");
            assert!((hi + lo).abs() < 2.0 * s.bin_hz(), "half {half}: not centred: {lo}..{hi}");
            let outside = s.fraction_outside(6_250.0);
            if half < 6_250.0 {
                assert!(outside < 1e-6, "half {half}: {outside} outside 6.25 kHz");
            } else {
                let want = 1.0 - 6_250.0 / half as f64;
                assert!((outside - want).abs() < 0.01, "half {half}: {outside} outside 6.25 kHz, want {want}");
            }
        }
    }

    /// Bessel function of the first kind, by its integral.
    fn bessel_j(n: i32, x: f64) -> f64 {
        let steps = 2000;
        let f = |t: f64| (n as f64 * t - x * t.sin()).cos();
        let h = PI / steps as f64;
        let inner: f64 = (1..steps).map(|k| f(k as f64 * h)).sum();
        (inner + (f(0.0) + f(PI)) / 2.0) * h / PI
    }

    /// A tone of 1 kHz at 2.5 kHz deviation puts Jn(2.5)^2 of the power on each
    /// line n kHz from the carrier; the measured power within ±(n + 1/2) kHz must
    /// match the Bessel sum, and so must a power fraction far down the skirt.
    #[test]
    fn fm_tone_matches_bessel_lines() {
        let (fm, dev) = (1_000.0f64, 2_500.0f64);
        let beta = dev / fm;
        let x: Vec<Complex32> = (0..1 << 16)
            .map(|i| {
                let t = i as f64 / FS as f64;
                Complex32::from_polar(1.0, (beta * (2.0 * PI * fm * t).sin()) as f32)
            })
            .collect();
        let s = Spectrum::welch([x.as_slice()], FS, NFFT);
        for n in 0..8 {
            let want: f64 = (-n..=n).map(|k| bessel_j(k, beta).powi(2)).sum();
            let half = (n as f32 + 0.5) * fm as f32;
            let got = s.fraction_in(-half, half);
            assert!((got - want).abs() < 1e-4 + 1e-3 * (1.0 - want), "n {n}: {got} in ±{half} Hz, want {want}");
        }
    }
}
