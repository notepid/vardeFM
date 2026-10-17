//! Basic filters: biquads, first-order emphasis networks, FIR design and resampling.

use num_complex::Complex32;
use std::f32::consts::PI;

/// Second-order IIR section (transposed direct form II).
#[derive(Clone, Debug)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}

impl Biquad {
    fn from_coeffs(b: [f32; 3], a: [f32; 3]) -> Self {
        Self {
            b0: b[0] / a[0],
            b1: b[1] / a[0],
            b2: b[2] / a[0],
            a1: a[1] / a[0],
            a2: a[2] / a[0],
            z1: 0.0,
            z2: 0.0,
        }
    }

    pub fn lowpass(fs: f32, f0: f32, q: f32) -> Self {
        let w0 = 2.0 * PI * f0 / fs;
        let (s, c) = w0.sin_cos();
        let alpha = s / (2.0 * q);
        Self::from_coeffs(
            [(1.0 - c) / 2.0, 1.0 - c, (1.0 - c) / 2.0],
            [1.0 + alpha, -2.0 * c, 1.0 - alpha],
        )
    }

    pub fn highpass(fs: f32, f0: f32, q: f32) -> Self {
        let w0 = 2.0 * PI * f0 / fs;
        let (s, c) = w0.sin_cos();
        let alpha = s / (2.0 * q);
        Self::from_coeffs(
            [(1.0 + c) / 2.0, -(1.0 + c), (1.0 + c) / 2.0],
            [1.0 + alpha, -2.0 * c, 1.0 - alpha],
        )
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }
}

/// A cascade of filter sections applied in order.
#[derive(Clone, Debug, Default)]
pub struct Chain {
    biquads: Vec<Biquad>,
    first: Vec<FirstOrder>,
}

const BUTTER4_Q: [f32; 2] = [0.541_196_1, 1.306_563];

impl Chain {
    pub fn new() -> Self {
        Self::default()
    }

    /// 4th-order Butterworth bandpass made of a highpass and a lowpass.
    pub fn bandpass4(fs: f32, lo: f32, hi: f32) -> Self {
        let mut c = Self::new();
        for q in BUTTER4_Q {
            c.biquads.push(Biquad::highpass(fs, lo, q));
            c.biquads.push(Biquad::lowpass(fs, hi, q));
        }
        c
    }

    /// 4th-order Butterworth lowpass.
    pub fn lowpass4(fs: f32, f: f32) -> Self {
        let mut c = Self::new();
        for q in BUTTER4_Q {
            c.biquads.push(Biquad::lowpass(fs, f, q));
        }
        c
    }

    pub fn push_first_order(&mut self, f: FirstOrder) {
        self.first.push(f);
    }

    #[inline]
    pub fn process(&mut self, mut x: f32) -> f32 {
        for f in &mut self.first {
            x = f.process(x);
        }
        for b in &mut self.biquads {
            x = b.process(x);
        }
        x
    }

    pub fn process_buf(&mut self, buf: &mut [f32]) {
        for x in buf {
            *x = self.process(*x);
        }
    }
}

/// First-order IIR from an analog prototype H(s) = (n0 + n1 s) / (d0 + d1 s).
#[derive(Clone, Debug)]
pub struct FirstOrder {
    b0: f32,
    b1: f32,
    a1: f32,
    z: f32,
}

impl FirstOrder {
    pub fn from_analog(fs: f32, n0: f32, n1: f32, d0: f32, d1: f32) -> Self {
        let k = 2.0 * fs;
        let a0 = d0 + d1 * k;
        Self {
            b0: (n0 + n1 * k) / a0,
            b1: (n0 - n1 * k) / a0,
            a1: (d0 - d1 * k) / a0,
            z: 0.0,
        }
    }

    /// FM pre-emphasis with time constant `tau` (e.g. 750 us), unity gain at 1 kHz.
    pub fn preemphasis(fs: f32, tau: f32) -> Self {
        let t2 = 1.0 / (2.0 * PI * 8000.0);
        let w = 2.0 * PI * 1000.0;
        let g = ((1.0 + (w * tau).powi(2)) / (1.0 + (w * t2).powi(2))).sqrt();
        Self::from_analog(fs, 1.0 / g, tau / g, 1.0, t2)
    }

    /// FM de-emphasis with time constant `tau`, unity gain at 1 kHz.
    pub fn deemphasis(fs: f32, tau: f32) -> Self {
        let w = 2.0 * PI * 1000.0;
        let g = (1.0 + (w * tau).powi(2)).sqrt();
        Self::from_analog(fs, g, 0.0, 1.0, tau)
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z;
        self.z = self.b1 * x - self.a1 * y;
        y
    }
}

/// Windowed-sinc lowpass (Blackman window), unity DC gain.
pub fn design_lowpass_fir(ntaps: usize, cutoff_hz: f32, fs: f32) -> Vec<f32> {
    let fc = cutoff_hz / fs;
    let m = (ntaps - 1) as f32;
    let mut h: Vec<f32> = (0..ntaps)
        .map(|n| {
            let x = n as f32 - m / 2.0;
            let sinc = if x == 0.0 {
                2.0 * fc
            } else {
                (2.0 * PI * fc * x).sin() / (PI * x)
            };
            let w = 0.42 - 0.5 * (2.0 * PI * n as f32 / m).cos() + 0.08 * (4.0 * PI * n as f32 / m).cos();
            sinc * w
        })
        .collect();
    let sum: f32 = h.iter().sum();
    h.iter_mut().for_each(|v| *v /= sum);
    h
}

/// Filter a complex signal with real taps (output has the same length, delayed by taps/2).
pub fn fir_complex(taps: &[f32], input: &[Complex32]) -> Vec<Complex32> {
    let n = taps.len();
    let mut out = Vec::with_capacity(input.len());
    for i in 0..input.len() {
        let mut acc = Complex32::new(0.0, 0.0);
        let start = (i + 1).saturating_sub(n);
        for (j, x) in input[start..=i].iter().rev().enumerate() {
            acc += x * taps[j];
        }
        out.push(acc);
    }
    out
}

/// Resample by `ratio` (output rate / input rate) with cubic Hermite interpolation.
/// Used to model sound-card clock differences of a few tens of ppm.
pub fn resample_cubic(input: &[f32], ratio: f64) -> Vec<f32> {
    if input.len() < 4 {
        return input.to_vec();
    }
    let n_out = ((input.len() as f64 - 3.0) * ratio).floor() as usize;
    let at = |i: isize| input[i.clamp(0, input.len() as isize - 1) as usize];
    (0..n_out)
        .map(|j| {
            let t = j as f64 / ratio;
            let i = t.floor() as isize;
            let f = (t - i as f64) as f32;
            let (y0, y1, y2, y3) = (at(i - 1), at(i), at(i + 1), at(i + 2));
            let c1 = 0.5 * (y2 - y0);
            let c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
            let c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
            ((c3 * f + c2) * f + c1) * f + y1
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone_gain(f: &mut impl FnMut(f32) -> f32, freq: f32, fs: f32) -> f32 {
        let n = 48_000;
        let mut peak = 0.0f32;
        for i in 0..n {
            let y = f((2.0 * PI * freq * i as f32 / fs).sin());
            if i > n / 2 {
                peak = peak.max(y.abs());
            }
        }
        peak
    }

    #[test]
    fn emphasis_pair_is_roughly_flat() {
        let fs = 48_000.0;
        for freq in [500.0, 1000.0, 2500.0] {
            let mut pre = FirstOrder::preemphasis(fs, 750e-6);
            let mut de = FirstOrder::deemphasis(fs, 750e-6);
            let g = tone_gain(&mut |x| de.process(pre.process(x)), freq, fs);
            assert!((g - 1.0).abs() < 0.15, "{freq} Hz gain {g}");
        }
    }

    #[test]
    fn bandpass_passes_and_rejects() {
        let fs = 48_000.0;
        let mut bp = Chain::bandpass4(fs, 300.0, 3000.0);
        assert!(tone_gain(&mut |x| bp.process(x), 1000.0, fs) > 0.9);
        let mut bp = Chain::bandpass4(fs, 300.0, 3000.0);
        assert!(tone_gain(&mut |x| bp.process(x), 8000.0, fs) < 0.05);
    }
}
