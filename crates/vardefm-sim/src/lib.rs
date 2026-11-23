//! FM radio channel simulator.
//!
//! Models what happens to modem audio between two FM radios:
//! TX audio filtering, pre-emphasis, deviation limiting, the FM modulator, RF noise,
//! the IF filter, the discriminator (including threshold "clicks" at low signal),
//! de-emphasis, RX audio filtering, squelch, TX key-up delay, and sound-card clock offset.
//! Interference other than thermal noise is in [`interference`].

use num_complex::Complex32;
use rand::rngs::StdRng;
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};
use std::f32::consts::PI;
use vardefm_dsp::filter::{design_lowpass_fir, fir_complex, resample_cubic, Chain, FirstOrder};

pub mod interference;
pub mod spectrum;

pub use interference::{parse_interference, AudioInterference, Interferer};

#[derive(Clone, Debug)]
pub struct FmChannelConfig {
    pub fs: f32,
    /// Peak deviation in Hz for audio amplitude 1.0.
    pub deviation_hz: f32,
    /// Pre-/de-emphasis time constant in microseconds (None for the flat data port).
    pub emphasis_us: Option<f32>,
    /// TX audio passband in Hz.
    pub tx_band: (f32, f32),
    /// RX audio passband in Hz.
    pub rx_band: (f32, f32),
    /// IF filter bandwidth in Hz (two-sided).
    pub if_bandwidth_hz: f32,
    /// Carrier-to-noise ratio in the IF bandwidth, dB. None = no noise.
    pub cnr_db: Option<f32>,
    /// Deviation limiter: audio is clipped at this level after pre-emphasis.
    pub clip_level: f32,
    /// Time from PTT until the carrier is up and the receiver unsquelches.
    pub keyup_delay_s: f32,
    /// Noise burst heard after the carrier drops.
    pub squelch_tail_s: f32,
    /// Receiver clock error relative to the transmitter, in ppm.
    pub clock_offset_ppm: f32,
    pub lead_silence_s: f32,
    pub trail_silence_s: f32,
    pub seed: u64,
    /// Interference besides the thermal noise of `cnr_db`. Empty in every preset.
    pub interference: Vec<Interferer>,
}

impl FmChannelConfig {
    /// Mic/speaker path of a typical 12.5 kHz FM voice radio.
    pub fn narrow() -> Self {
        Self {
            fs: 48_000.0,
            deviation_hz: 2_500.0,
            emphasis_us: Some(750.0),
            tx_band: (300.0, 3_000.0),
            rx_band: (300.0, 3_000.0),
            if_bandwidth_hz: 11_000.0,
            cnr_db: None,
            clip_level: 1.0,
            keyup_delay_s: 0.05,
            squelch_tail_s: 0.1,
            clock_offset_ppm: 0.0,
            lead_silence_s: 0.2,
            trail_silence_s: 0.2,
            seed: 1,
            interference: Vec::new(),
        }
    }

    /// Flat 9600-baud data port path.
    pub fn wide() -> Self {
        Self {
            emphasis_us: None,
            deviation_hz: 3_000.0,
            tx_band: (20.0, 7_000.0),
            rx_band: (20.0, 7_000.0),
            if_bandwidth_hz: 16_000.0,
            ..Self::narrow()
        }
    }

    pub fn by_name(name: &str) -> Option<Self> {
        match name {
            "narrow" => Some(Self::narrow()),
            "wide" => Some(Self::wide()),
            _ => None,
        }
    }

    /// Mic/speaker path of a radio set to 25 kHz channels. Modelled as: the radio
    /// scales the deviation after its limiter (to 5 kHz) and opens the IF filter
    /// (to 15 kHz), so the audio chain and `clip_level` stay as in [`Self::narrow`].
    /// Both are assumptions, not measured on a radio.
    pub fn mic_25k() -> Self {
        Self { deviation_hz: 5_000.0, if_bandwidth_hz: 15_000.0, ..Self::narrow() }
    }

    /// Flat data port of a radio set to 12.5 kHz channels. The deviation is the
    /// [`Self::wide`] setting halved; that is assumed, as no source says what
    /// radios do with the data port in narrow, so sweep it.
    pub fn data_12k5() -> Self {
        Self { deviation_hz: 1_500.0, if_bandwidth_hz: 11_000.0, ..Self::wide() }
    }

    /// A channel by radio path and channel spacing, independent of the waveform:
    /// one of [`CHANNELS`]. `mic-12k5` is [`Self::narrow`] and `data-25k` is
    /// [`Self::wide`].
    pub fn by_channel(name: &str) -> Option<Self> {
        CHANNELS.iter().find(|(n, _)| *n == name).map(|(_, preset)| preset())
    }

    /// The names of [`CHANNELS`], for messages.
    pub fn channel_names() -> String {
        CHANNELS.iter().map(|(n, _)| *n).collect::<Vec<_>>().join(", ")
    }

    /// A channel from `name[:dev=HZ][:if=HZ][:INTERFERER...]`: a [`CHANNELS`] preset
    /// with its peak deviation or IF bandwidth overridden, for sweeps, and with any
    /// number of interferers (as [`Interferer::from_str`], e.g. `spur=1000@20`).
    /// Rejects unknown names and keys, and an RF signal the sample rate cannot hold.
    pub fn parse_channel(spec: &str) -> Result<Self, String> {
        let mut parts = spec.split(':');
        let name = parts.next().unwrap_or_default().trim();
        let mut cfg = Self::by_channel(name)
            .ok_or_else(|| format!("unknown channel {name:?} in {spec:?} (one of {})", Self::channel_names()))?;
        let mut seen = Vec::new();
        for part in parts {
            let (key, value) = part.split_once('=').ok_or_else(|| format!("{spec:?}: {part:?} is not key=value"))?;
            let key = key.trim();
            if Interferer::KEYS.contains(&key) {
                let i: Interferer = part.parse().map_err(|e| format!("{spec:?}: {e}"))?;
                i.check(cfg.fs).map_err(|e| format!("{spec:?}: {key}: {e}"))?;
                cfg.interference.push(i);
                continue;
            }
            let field = match key {
                "dev" => &mut cfg.deviation_hz,
                "if" => &mut cfg.if_bandwidth_hz,
                k => {
                    let known = Interferer::KEYS.join(", ");
                    return Err(format!("{spec:?}: unknown key {k:?} (dev, if, {known})"));
                }
            };
            if seen.contains(&key) {
                return Err(format!("{spec:?}: {key} is given twice"));
            }
            seen.push(key);
            let hz: f32 = value.trim().parse().map_err(|_| format!("{spec:?}: {value:?} is not a number"))?;
            if !(hz.is_finite() && hz > 0.0) {
                return Err(format!("{spec:?}: {key} must be above 0 Hz"));
            }
            *field = hz;
        }
        // The complex baseband spans fs; keep Carson's bandwidth and the IF filter
        // below half of it, so aliased sidebands stay far from the receiver.
        let carson = 2.0 * (cfg.deviation_hz + cfg.tx_band.1);
        if carson >= cfg.fs / 2.0 || cfg.if_bandwidth_hz >= cfg.fs / 2.0 {
            return Err(format!("{spec:?}: Carson bandwidth {carson} Hz or IF too wide for fs {} Hz", cfg.fs));
        }
        Ok(cfg)
    }

    /// Carrier-to-noise density in dB-Hz for a carrier-to-noise ratio of `cnr_db`
    /// in this channel's IF bandwidth: the same received signal strength gives the
    /// same C/N0 whatever the IF.
    pub fn cn0_db(&self, cnr_db: f32) -> f32 {
        cnr_db + 10.0 * self.if_bandwidth_hz.log10()
    }

    /// Standard deviation of each of the I and Q noise samples that [`receive`]
    /// adds for `cnr_db`: complex noise of N0 = 1 / (CNR · IF) per Hz across fs,
    /// against a unit carrier, which is the C/N0 that [`Self::cn0_db`] reports.
    pub fn noise_sigma(&self, cnr_db: f32) -> f32 {
        (10f32.powf(-cnr_db / 10.0) * self.fs / self.if_bandwidth_hz / 2.0).sqrt()
    }
}

/// A channel preset: its name and its constructor.
pub type Preset = (&'static str, fn() -> FmChannelConfig);

/// The channel presets of [`FmChannelConfig::by_channel`]: radio path (`mic` for
/// mic/speaker audio, `data` for the flat data port) and channel spacing.
pub const CHANNELS: [Preset; 4] = [
    ("mic-12k5", FmChannelConfig::narrow),
    ("mic-25k", FmChannelConfig::mic_25k),
    ("data-25k", FmChannelConfig::wide),
    ("data-12k5", FmChannelConfig::data_12k5),
];

/// One transmission on the air: complex baseband at `fs`, before noise.
pub struct Rf {
    pub samples: Vec<Complex32>,
    /// The samples while the carrier is up.
    pub carrier: std::ops::Range<usize>,
}

/// Pass one transmission (audio at `cfg.fs`, full scale = peak deviation) through the channel.
pub fn transmit(cfg: &FmChannelConfig, audio: &[f32]) -> Vec<f32> {
    receive(cfg, &modulate(cfg, audio))
}

/// The transmitter: audio chain, deviation limiter and FM modulator. The carrier
/// is up for `audio`, with `cfg.lead_silence_s` before and `cfg.trail_silence_s`
/// after.
pub fn modulate(cfg: &FmChannelConfig, audio: &[f32]) -> Rf {
    let fs = cfg.fs;
    let lead = (cfg.lead_silence_s * fs) as usize;
    let trail = (cfg.trail_silence_s * fs) as usize;
    let total = lead + audio.len() + trail;
    let carrier = lead..lead + audio.len();

    // --- Transmitter audio chain ---
    let mut tx_chain = Chain::bandpass4(fs, cfg.tx_band.0, cfg.tx_band.1);
    let mut pre = cfg.emphasis_us.map(|us| FirstOrder::preemphasis(fs, us * 1e-6));
    let mut tx_audio = vec![0f32; total];
    for (i, &x) in audio.iter().enumerate() {
        let mut v = x;
        if let Some(p) = &mut pre {
            v = p.process(v);
        }
        v = v.clamp(-cfg.clip_level, cfg.clip_level);
        tx_audio[lead + i] = tx_chain.process(v);
    }

    // --- FM modulator ---
    let mut phase = 0f32;
    let k = 2.0 * PI * cfg.deviation_hz / fs;
    let samples = (0..total)
        .map(|i| {
            if carrier.contains(&i) {
                phase = (phase + k * tx_audio[i]) % (2.0 * PI);
                Complex32::from_polar(1.0, phase)
            } else {
                Complex32::new(0.0, 0.0)
            }
        })
        .collect();
    Rf { samples, carrier }
}

/// `rf` with the receiver's RF noise for `cfg.cnr_db`, seeded by `cfg.seed`, and the
/// RF interference of `cfg.interference`.
pub fn add_noise(cfg: &FmChannelConfig, rf: &Rf) -> Vec<Complex32> {
    let mut rng = StdRng::seed_from_u64(cfg.seed);
    let sigma = cfg.cnr_db.map(|c| cfg.noise_sigma(c));
    let normal = Normal::new(0.0f32, 1.0).unwrap();
    let mut noisy: Vec<Complex32> = (0..rf.samples.len())
        .map(|i| {
            let mut z = rf.samples[i];
            if let Some(s) = sigma {
                z += Complex32::new(normal.sample(&mut rng), normal.sample(&mut rng)) * s;
            } else if !rf.carrier.contains(&i) {
                // A tiny noise floor keeps the discriminator defined with no carrier.
                z += Complex32::new(normal.sample(&mut rng), normal.sample(&mut rng)) * 1e-3;
            }
            z
        })
        .collect();
    interference::add_rf(cfg, &mut noisy);
    noisy
}

/// The receiver: RF noise at `cfg.cnr_db` and RF interference, IF filter,
/// discriminator, de-emphasis, audio filter, squelch, audio interference and
/// sound-card clock.
pub fn receive(cfg: &FmChannelConfig, rf: &Rf) -> Vec<f32> {
    let fs = cfg.fs;
    let keyup = (cfg.keyup_delay_s * fs) as usize;
    let tail = (cfg.squelch_tail_s * fs) as usize;
    let total = rf.samples.len();
    let carrier = rf.carrier.clone();
    let rf = add_noise(cfg, rf);

    // --- Receiver: IF filter, discriminator, de-emphasis, audio filter ---
    let taps = design_lowpass_fir(129, cfg.if_bandwidth_hz / 2.0, fs);
    let delay = taps.len() / 2;
    let iff = fir_complex(&taps, &rf);
    let mut de = cfg.emphasis_us.map(|us| FirstOrder::deemphasis(fs, us * 1e-6));
    let mut rx_chain = Chain::bandpass4(fs, cfg.rx_band.0, cfg.rx_band.1);
    let scale = fs / (2.0 * PI * cfg.deviation_hz);
    let mut out = vec![0f32; total];
    let mut prev = Complex32::new(1.0, 0.0);
    for i in 0..total {
        let z = iff[(i + delay).min(total - 1)];
        let mut v = (z * prev.conj()).arg() * scale;
        prev = z;
        if let Some(d) = &mut de {
            v = d.process(v);
        }
        out[i] = rx_chain.process(v);
    }

    // --- Squelch: closed until the carrier is up, open for a short tail afterwards ---
    let open = carrier.start + keyup..carrier.end + tail;
    for (i, v) in out.iter_mut().enumerate() {
        if !open.contains(&i) {
            *v = 0.0;
        }
    }
    interference::add_audio(cfg, &mut out);

    if cfg.clock_offset_ppm != 0.0 {
        out = resample_cubic(&out, 1.0 + cfg.clock_offset_ppm as f64 * 1e-6);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The noise `receive` adds has the density that `cn0_db` reports: measured on a
    /// steady unit carrier, 10 log10(1 / N0) matches C/N0 within 0.1 dB, for two
    /// IF bandwidths and CNRs. Every threshold given as C/N0 rests on this.
    #[test]
    fn added_noise_has_the_reported_c_n0() {
        let n = 1 << 18;
        for (base, cnr) in [(FmChannelConfig::narrow(), 5.0), (FmChannelConfig::mic_25k(), 20.0)] {
            let cfg = FmChannelConfig { cnr_db: Some(cnr), ..base };
            let rf = Rf { samples: vec![Complex32::new(1.0, 0.0); n], carrier: 0..n };
            let noisy = add_noise(&cfg, &rf);
            let power: f64 = noisy.iter().map(|z| (z - 1.0).norm_sqr() as f64).sum::<f64>() / n as f64;
            let measured = -10.0 * (power / cfg.fs as f64).log10();
            let want = cfg.cn0_db(cnr) as f64;
            assert!(
                (measured - want).abs() < 0.1,
                "IF {}: C/N0 {measured:.2} dB-Hz, want {want:.2}",
                cfg.if_bandwidth_hz
            );
        }
    }

    #[test]
    fn clean_tone_passes_through() {
        let cfg = FmChannelConfig { lead_silence_s: 0.0, keyup_delay_s: 0.0, ..FmChannelConfig::narrow() };
        let tone: Vec<f32> = (0..48_000).map(|i| 0.3 * (2.0 * PI * 1000.0 * i as f32 / 48_000.0).sin()).collect();
        let out = transmit(&cfg, &tone);
        let peak = out[24_000..40_000].iter().fold(0f32, |m, v| m.max(v.abs()));
        assert!((peak - 0.3).abs() < 0.05, "peak {peak}");
    }

    /// Every listed channel parses; overrides apply; anything unknown or out of
    /// range is refused rather than replaced by a default.
    #[test]
    fn channels_parse_and_refuse_what_they_do_not_know() {
        for (name, _) in CHANNELS {
            assert!(FmChannelConfig::parse_channel(name).is_ok(), "{name}");
        }
        let c = FmChannelConfig::parse_channel("data-12k5:dev=2000:if=8000").unwrap();
        assert_eq!((c.deviation_hz, c.if_bandwidth_hz, c.emphasis_us), (2000.0, 8000.0, None));
        let n = FmChannelConfig::parse_channel("mic-12k5").unwrap();
        assert_eq!((n.deviation_hz, n.if_bandwidth_hz), (2500.0, 11_000.0));
        let w = FmChannelConfig::parse_channel("mic-25k").unwrap();
        assert_eq!((w.deviation_hz, w.if_bandwidth_hz, w.emphasis_us), (5000.0, 15_000.0, n.emphasis_us));
        let d = FmChannelConfig::parse_channel("data-12k5").unwrap();
        assert_eq!((d.deviation_hz, d.if_bandwidth_hz, d.emphasis_us), (1500.0, 11_000.0, None));
        // C/N0 is the CNR in the IF bandwidth plus 10 log10 of that bandwidth.
        let ten_k = FmChannelConfig { if_bandwidth_hz: 10_000.0, ..n };
        assert!((ten_k.cn0_db(0.0) - 40.0).abs() < 1e-4 && (ten_k.cn0_db(6.0) - 46.0).abs() < 1e-4);
        for bad in [
            "narrow",
            "mic-12k5:dev",
            "mic-12k5:dev=x",
            "mic-12k5:dev=-1",
            "mic-12k5:bw=9000",
            "data-25k:dev=9000",
            "mic-12k5:dev=2000:dev=3000",
            "data-25k:dev=5000",
        ] {
            assert!(FmChannelConfig::parse_channel(bad).is_err(), "{bad} parsed");
        }
        // An unknown name lists every preset, read from the same table.
        let e = FmChannelConfig::parse_channel("x").unwrap_err();
        assert!(CHANNELS.iter().all(|(n, _)| e.contains(n)), "{e}");
        // An unknown key is named as such, whatever its value.
        let e = FmChannelConfig::parse_channel("mic-12k5: bw =-1").unwrap_err();
        assert!(e.contains("unknown key \"bw\""), "{e}");
    }
}

#[cfg(test)]
mod probe {
    use super::*;
    #[test]
    #[ignore]
    fn tone_snr_probe() {
        for (name, base) in [("narrow", FmChannelConfig::narrow()), ("wide", FmChannelConfig::wide())] {
            for cnr in [None, Some(30.0), Some(20.0), Some(12.0)] {
                let cfg = FmChannelConfig { cnr_db: cnr, lead_silence_s: 0.0, keyup_delay_s: 0.0, ..base.clone() };
                let n = 96_000;
                let tone: Vec<f32> = (0..n).map(|i| 0.35 * (2.0 * PI * 1000.0 * i as f32 / 48_000.0).sin()).collect();
                let out = transmit(&cfg, &tone);
                // least-squares fit of a 1 kHz sine, residual = noise+distortion
                let seg = 24_000..90_000;
                let (mut ss, mut cc, mut sc) = (0f64, 0f64, 0f64);
                let (mut ys, mut yc) = (0f64, 0f64);
                for i in seg.clone() {
                    let w = 2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48_000.0;
                    let (s, c) = w.sin_cos();
                    ss += s * s;
                    cc += c * c;
                    sc += s * c;
                    ys += out[i] as f64 * s;
                    yc += out[i] as f64 * c;
                }
                let det = ss * cc - sc * sc;
                let a = (ys * cc - yc * sc) / det;
                let b = (yc * ss - ys * sc) / det;
                let (mut sig, mut err) = (0f64, 0f64);
                for i in seg {
                    let w = 2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48_000.0;
                    let fit = a * w.sin() + b * w.cos();
                    sig += fit * fit;
                    err += (out[i] as f64 - fit).powi(2);
                }
                println!("{name} cnr={cnr:?}: SNR {:.1} dB", 10.0 * (sig / err).log10());
            }
        }
    }
}
