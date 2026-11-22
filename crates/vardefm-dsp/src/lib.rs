//! Signal processing for the VardeFM modem: OFDM, constellations and filters.

pub mod filter;
pub mod ofdm;
pub mod params;
pub mod prng;
pub mod qam;

pub use num_complex::Complex32;

/// A received sample beyond this is a glitch: full scale is 1. Far enough beyond it,
/// a finite sample overflows the high-Q sections of an IIR filter and leaves its state
/// NaN, so the filters do not take one.
pub const MAX_SAMPLE: f32 = 4.0;

/// The EIA CTCSS tones, Hz.
pub const CTCSS_HZ: [f32; 21] = [
    67.0, 71.9, 77.0, 82.5, 88.5, 94.8, 100.0, 107.2, 114.8, 123.0, 131.8, 141.3, 151.4, 162.2, 173.8, 186.2, 203.5,
    218.1, 233.6, 250.3, 254.1,
];
pub use ofdm::{Detection, Detector, Frame, Ofdm};
pub use params::OfdmParams;
pub use qam::Modulation;
