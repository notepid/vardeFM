//! Interference other than the receiver's thermal noise.
//!
//! Two kinds enter at RF, before the IF filter, so the discriminator treats them as
//! a radio would (capture, beat tones, clicks):
//! - a steady carrier (`spur`), a given offset from ours and a given level below it;
//! - short bursts of wideband noise (`rfburst`), such as ignition noise.
//!
//! Two enter in the received audio, after the radio, where FM does nothing about
//! them:
//! - a steady tone (`tone`), such as mains hum or a switching supply's whine;
//! - short bursts of wideband noise (`pop`), such as a sound card's pops.
//!
//! Limits of the model:
//! - RF interference exists only inside one transmission's buffer, drawn from its
//!   seed: while the carrier is up and in the squelch tail after it. A receiver
//!   hears none of it between transmissions.
//! - Audio interference belongs to the receiver, not to a transmission.
//!   [`AudioInterference`] is one receiver's stream, to add to everything that
//!   receiver hears; `sim-pair` runs one per station, so a station hears it between
//!   transmissions too, busy detector included. [`crate::receive`] has only one
//!   transmission, and adds it to that buffer from its first sample to its last.
//! - Audio interference is added after the squelch gate, so a closed squelch does
//!   not mute it, and before the sound card's clock offset. The sum is then clipped
//!   at full scale, as a sound card's input clips. Audio with no audio interference
//!   is left as it is, unclipped.
//! - Bursts and pops arrive at random (Poisson) and independently of each other. An
//!   event that began before the first sample is left out, and where two overlap
//!   their noise adds.
//! - The noise of a burst or pop is white across the whole sample rate, with a
//!   rectangular envelope.

use crate::FmChannelConfig;
use num_complex::Complex32;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use rand_distr::{Distribution, Exp, Normal};
use std::f64::consts::TAU;
use std::ops::Range;

/// One source of interference. See the module documentation for where each enters.
#[derive(Clone, Debug, PartialEq)]
pub enum Interferer {
    /// A steady carrier `offset_hz` from ours (either sign), `ci_db` below it.
    Spur { offset_hz: f32, ci_db: f32 },
    /// Bursts of RF noise: `rate_hz` per second on average, each `duration_s` long.
    /// During a burst the carrier-to-noise ratio in the IF bandwidth is `cnr_db`
    /// (as [`FmChannelConfig::cnr_db`]), on top of the thermal noise.
    RfBursts { rate_hz: f32, duration_s: f32, cnr_db: f32 },
    /// A steady audio tone of peak amplitude `dbfs` (0 dBFS = full scale = the audio
    /// level of peak deviation).
    Tone { freq_hz: f32, dbfs: f32 },
    /// Bursts of audio noise: `rate_hz` per second on average, each `duration_s`
    /// long, of RMS level `dbfs` across the whole sample rate.
    Pops { rate_hz: f32, duration_s: f32, dbfs: f32 },
}

impl Interferer {
    /// The keys of [`Interferer::from_str`], for messages.
    pub const KEYS: [&'static str; 4] = ["spur", "rfburst", "tone", "pop"];

    /// Whether this enters in the received audio rather than at RF.
    pub fn is_audio(&self) -> bool {
        matches!(self, Interferer::Tone { .. } | Interferer::Pops { .. })
    }

    /// Refuses values the simulator cannot render at sample rate `fs`.
    pub fn check(&self, fs: f32) -> Result<(), String> {
        let positive = |what: &str, v: f32| {
            if v.is_finite() && v > 0.0 {
                Ok(())
            } else {
                Err(format!("{what} must be above 0"))
            }
        };
        let finite = |what: &str, v: f32| if v.is_finite() { Ok(()) } else { Err(format!("{what} must be a number")) };
        match *self {
            Interferer::Spur { offset_hz, ci_db } => {
                finite("the level", ci_db)?;
                if !(offset_hz.is_finite() && offset_hz.abs() < fs / 2.0) {
                    return Err(format!("the offset must be within ±{} Hz", fs / 2.0));
                }
            }
            Interferer::Tone { freq_hz, dbfs } => {
                finite("the level", dbfs)?;
                positive("the frequency", freq_hz)?;
                if freq_hz >= fs / 2.0 {
                    return Err(format!("the frequency must be below {} Hz", fs / 2.0));
                }
            }
            Interferer::RfBursts { rate_hz, duration_s, cnr_db: level }
            | Interferer::Pops { rate_hz, duration_s, dbfs: level } => {
                finite("the level", level)?;
                positive("the rate", rate_hz)?;
                positive("the duration", duration_s)?;
            }
        }
        Ok(())
    }
}

/// `spur=OFFSET_HZ@C/I_DB`, `rfburst=PER_SECOND@MS@CNR_DB`, `tone=HZ@DBFS` or
/// `pop=PER_SECOND@MS@DBFS`.
impl std::str::FromStr for Interferer {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, String> {
        let (key, value) = s.split_once('=').ok_or_else(|| format!("{s:?} is not key=value"))?;
        let key = key.trim();
        let numbers: Result<Vec<f32>, String> = value
            .split('@')
            .map(|v| v.trim().parse::<f32>().map_err(|_| format!("{s:?}: {v:?} is not a number")))
            .collect();
        let v = numbers?;
        let want = |n: usize, form: &str| {
            if v.len() == n {
                Ok(())
            } else {
                Err(format!("{s:?}: {key} takes {form}"))
            }
        };
        match key {
            "spur" => want(2, "OFFSET_HZ@C/I_DB").map(|()| Interferer::Spur { offset_hz: v[0], ci_db: v[1] }),
            "tone" => want(2, "HZ@DBFS").map(|()| Interferer::Tone { freq_hz: v[0], dbfs: v[1] }),
            "rfburst" => want(3, "PER_SECOND@MS@CNR_DB").map(|()| Interferer::RfBursts {
                rate_hz: v[0],
                duration_s: v[1] / 1000.0,
                cnr_db: v[2],
            }),
            "pop" => want(3, "PER_SECOND@MS@DBFS").map(|()| Interferer::Pops {
                rate_hz: v[0],
                duration_s: v[1] / 1000.0,
                dbfs: v[2],
            }),
            k => Err(format!("{s:?}: unknown key {k:?} ({})", Self::KEYS.join(", "))),
        }
    }
}

/// A list of interferers from `key=value[:key=value...]`, each as
/// [`Interferer::from_str`] and checked against sample rate `fs`. Empty for an
/// empty spec.
pub fn parse_interference(spec: &str, fs: f32) -> Result<Vec<Interferer>, String> {
    let parts = spec.split(':').map(str::trim).filter(|p| !p.is_empty());
    parts
        .map(|p| {
            let i: Interferer = p.parse()?;
            i.check(fs).map_err(|e| format!("{p:?}: {e}"))?;
            Ok(i)
        })
        .collect()
}

/// The random source of interferer `index` under `seed`: apart from the thermal
/// noise's (seeded by the seed itself), so adding interference leaves that noise as
/// it was.
fn rng_for(seed: u64, index: usize) -> StdRng {
    StdRng::seed_from_u64(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15).wrapping_add(0x1f3a_0001 + index as u64))
}

/// Where events arriving at random at `rate_hz`, each `duration_s` long, fall in
/// `n` samples.
fn events(rng: &mut StdRng, n: usize, fs: f32, rate_hz: f32, duration_s: f32) -> Vec<Range<usize>> {
    let gap = Exp::new(rate_hz as f64).expect("rate checked");
    let len = ((duration_s * fs).round() as usize).max(1);
    let mut out = Vec::new();
    let mut t = gap.sample(rng);
    while t * (fs as f64) < n as f64 {
        let start = (t * fs as f64) as usize;
        out.push(start..(start + len).min(n));
        t += gap.sample(rng);
    }
    out
}

fn amplitude(db: f32) -> f32 {
    10f32.powf(db / 20.0)
}

/// Add the RF interference of `cfg` to complex baseband `rf` (unit carrier).
pub(crate) fn add_rf(cfg: &FmChannelConfig, rf: &mut [Complex32]) {
    let normal = Normal::new(0.0f32, 1.0).unwrap();
    for (index, i) in cfg.interference.iter().enumerate() {
        let mut rng = rng_for(cfg.seed, index);
        match *i {
            Interferer::Spur { offset_hz, ci_db } => {
                let (amp, phase0) = (amplitude(-ci_db), rng.gen::<f64>() * TAU);
                let step = TAU * offset_hz as f64 / cfg.fs as f64;
                for (k, z) in rf.iter_mut().enumerate() {
                    *z += Complex32::from_polar(amp, ((phase0 + step * k as f64) % TAU) as f32);
                }
            }
            Interferer::RfBursts { rate_hz, duration_s, cnr_db } => {
                let sigma = cfg.noise_sigma(cnr_db);
                for r in events(&mut rng, rf.len(), cfg.fs, rate_hz, duration_s) {
                    for z in &mut rf[r] {
                        *z += Complex32::new(normal.sample(&mut rng), normal.sample(&mut rng)) * sigma;
                    }
                }
            }
            Interferer::Tone { .. } | Interferer::Pops { .. } => {}
        }
    }
}

/// One source of an [`AudioInterference`].
enum Source {
    Tone {
        amp: f32,
        phase0: f64,
        /// Radians per sample.
        step: f64,
    },
    Pops(Box<Pops>),
}

/// The state of one stream of pops.
struct Pops {
    rng: StdRng,
    gap: Exp<f64>,
    rms: f32,
    /// Length of a pop, samples.
    len: u64,
    /// When the next pop starts, in samples since the stream began.
    next: f64,
    /// Pops that have started and not yet ended: the samples they span.
    live: Vec<Range<u64>>,
}

/// The audio interference at one receiver, as a stream: [`AudioInterference::add`]
/// adds its next samples to the receiver's audio, chunk by chunk.
pub struct AudioInterference {
    sources: Vec<Source>,
    fs: f32,
    /// Samples added so far.
    pos: u64,
}

impl AudioInterference {
    /// The stream of the audio interferers among `interferers` (the RF ones are left
    /// out) at sample rate `fs`, drawn from `seed`.
    pub fn new(interferers: &[Interferer], fs: f32, seed: u64) -> Self {
        let source = |(index, i): (usize, &Interferer)| {
            let mut rng = rng_for(seed, index);
            match *i {
                Interferer::Tone { freq_hz, dbfs } => Some(Source::Tone {
                    amp: amplitude(dbfs),
                    phase0: rng.gen::<f64>() * TAU,
                    step: TAU * freq_hz as f64 / fs as f64,
                }),
                Interferer::Pops { rate_hz, duration_s, dbfs } => {
                    let gap = Exp::new(rate_hz as f64).expect("rate checked");
                    let next = gap.sample(&mut rng) * fs as f64;
                    let len = ((duration_s * fs).round() as u64).max(1);
                    Some(Source::Pops(Box::new(Pops { rng, gap, rms: amplitude(dbfs), len, next, live: Vec::new() })))
                }
                Interferer::Spur { .. } | Interferer::RfBursts { .. } => None,
            }
        };
        Self { sources: interferers.iter().enumerate().filter_map(source).collect(), pos: 0, fs }
    }

    /// Whether there is no audio interference: [`AudioInterference::add`] then
    /// changes nothing.
    pub fn is_empty(&self) -> bool {
        self.sources.is_empty()
    }

    /// Add the next `audio.len()` samples of interference to `audio`, and clip the
    /// sum at full scale as a sound card's input does. Without any audio
    /// interference `audio` is left as it is.
    pub fn add(&mut self, audio: &mut [f32]) {
        if self.sources.is_empty() {
            return;
        }
        let normal = Normal::new(0.0f32, 1.0).unwrap();
        let (from, to) = (self.pos, self.pos + audio.len() as u64);
        for source in &mut self.sources {
            match source {
                Source::Tone { amp, phase0, step } => {
                    for (k, v) in audio.iter_mut().enumerate() {
                        *v += *amp * ((*phase0 + *step * (from + k as u64) as f64) % TAU).sin() as f32;
                    }
                }
                Source::Pops(p) => {
                    while p.next < to as f64 {
                        let start = p.next as u64;
                        p.live.push(start..start + p.len);
                        p.next += p.gap.sample(&mut p.rng) * self.fs as f64;
                    }
                    for pop in &p.live {
                        for k in pop.start.max(from)..pop.end.min(to) {
                            audio[(k - from) as usize] += normal.sample(&mut p.rng) * p.rms;
                        }
                    }
                    p.live.retain(|pop| pop.end > to);
                }
            }
        }
        for v in audio.iter_mut() {
            *v = v.clamp(-1.0, 1.0);
        }
        self.pos = to;
    }
}

/// Add the audio interference of `cfg` to the received audio of one transmission.
pub(crate) fn add_audio(cfg: &FmChannelConfig, audio: &mut [f32]) {
    AudioInterference::new(&cfg.interference, cfg.fs, cfg.seed).add(audio);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{add_noise, transmit, Rf};

    fn steady_carrier(n: usize) -> Rf {
        Rf { samples: vec![Complex32::new(1.0, 0.0); n], carrier: 0..n }
    }

    fn with(base: FmChannelConfig, spec: &str) -> FmChannelConfig {
        FmChannelConfig { interference: parse_interference(spec, base.fs).unwrap(), ..base }
    }

    /// A spur adds a carrier of its level to every sample and leaves the thermal
    /// noise exactly as it was, so results with and without it compare like for like.
    #[test]
    fn spur_has_its_level_and_leaves_the_noise_alone() {
        let base = FmChannelConfig { cnr_db: Some(10.0), seed: 5, ..FmChannelConfig::narrow() };
        let rf = steady_carrier(4096);
        let plain = add_noise(&base, &rf);
        let spurred = add_noise(&with(base, "spur=1500@20"), &rf);
        for (a, b) in spurred.iter().zip(&plain) {
            assert!(((a - b).norm() - 0.1).abs() < 1e-4, "spur sample {}", (a - b).norm());
        }
    }

    /// A weak carrier `f` Hz off ours comes out of the discriminator as a tone at
    /// `f` of amplitude (I/C) · f / deviation: FM does not remove an in-channel spur.
    #[test]
    fn rf_spur_becomes_an_audio_tone() {
        let base = FmChannelConfig {
            lead_silence_s: 0.0,
            trail_silence_s: 0.0,
            keyup_delay_s: 0.0,
            ..FmChannelConfig::wide()
        };
        let want = amplitude(-30.0) * 1000.0 / base.deviation_hz;
        let out = transmit(&with(base, "spur=1000@30"), &vec![0.0; 48_000]);
        let seg = &out[12_000..36_000];
        let peak = (2.0 * seg.iter().map(|v| v * v).sum::<f32>() / seg.len() as f32).sqrt();
        assert!((peak / want - 1.0).abs() < 0.1, "tone {peak}, want {want}");
    }

    /// RF bursts carry the noise power of their CNR for their share of the time.
    #[test]
    fn rf_bursts_have_their_power_and_duty() {
        let n = 1 << 19;
        let base = FmChannelConfig { seed: 3, ..FmChannelConfig::narrow() };
        let sigma = base.noise_sigma(-6.0);
        let out = add_noise(&with(base, "rfburst=20@5@-6"), &steady_carrier(n));
        let hit = out.iter().filter(|z| (*z - 1.0).norm_sqr() > 0.0).count() as f32 / n as f32;
        assert!(hit > 0.07 && hit < 0.11, "{hit} of the samples are in a burst, want about 0.095");
        let power = out.iter().map(|z| (z - 1.0).norm_sqr()).sum::<f32>() / n as f32;
        let want = 20.0 * 0.005 * 2.0 * sigma * sigma;
        assert!((power / want - 1.0).abs() < 0.25, "mean power {power}, want {want}");
    }

    /// Audio interference is added to the received audio as it is: a tone of its
    /// peak level, pops of their RMS level for their share of the time.
    #[test]
    fn audio_interference_has_its_level() {
        let base = FmChannelConfig { seed: 9, ..FmChannelConfig::narrow() };
        let silence = vec![0.0; 10 * 48_000];
        // The radio's audio is clipped along with the interference (the key-up
        // transient passes full scale), so compare with it clipped.
        let plain = transmit(&base, &silence);
        let diff = |spec: &str| -> Vec<f32> {
            let with = transmit(&with(base.clone(), spec), &silence);
            with.iter().zip(&plain).map(|(a, b)| a - b.clamp(-1.0, 1.0)).collect()
        };
        let tone = diff("tone=1000@-20");
        let peak = tone.iter().fold(0f32, |m, v| m.max(v.abs()));
        let power = tone.iter().map(|v| v * v).sum::<f32>() / tone.len() as f32;
        assert!((peak - 0.1).abs() < 1e-3 && (power / 0.005 - 1.0).abs() < 0.01, "peak {peak}, power {power}");
        let pops = diff("pop=20@5@-10");
        let power = pops.iter().map(|v| v * v).sum::<f32>() / pops.len() as f32;
        let want = 20.0 * 0.005 * 0.1;
        assert!((power / want - 1.0).abs() < 0.25, "mean power {power}, want {want}");
    }

    /// A receiver's stream is the same audio however it is cut into chunks (a tone
    /// exactly; pops in their power), and it never leaves full scale.
    #[test]
    fn audio_stream_is_continuous_and_clipped() {
        let stream = |spec: &str| AudioInterference::new(&parse_interference(spec, 48e3).unwrap(), 48e3, 4);
        let n = 20 * 48_000;
        let mut whole = vec![0.0; n];
        stream("tone=1000@-20").add(&mut whole);
        let mut chunked = vec![0.0; n];
        let mut tone = stream("tone=1000@-20");
        chunked.chunks_mut(480).for_each(|c| tone.add(c));
        assert_eq!(whole, chunked);

        let mut pops = stream("pop=20@5@-10");
        let mut audio = vec![0.0; n];
        audio.chunks_mut(480).for_each(|c| pops.add(c));
        let power = audio.iter().map(|v| v * v).sum::<f32>() / n as f32;
        let want = 20.0 * 0.005 * 0.1;
        assert!((power / want - 1.0).abs() < 0.25, "mean power {power}, want {want}");

        let mut loud = vec![0.9; 48_000];
        stream("pop=20@5@10").add(&mut loud);
        let peak = loud.iter().fold(0f32, |m, v| m.max(v.abs()));
        assert!(peak == 1.0 && loud.iter().any(|&v| v == -1.0), "peak {peak}");
        // No audio interference: nothing is touched, not even clipped.
        let mut plain = vec![1.5; 100];
        let mut none = stream("spur=1000@20");
        assert!(none.is_empty());
        none.add(&mut plain);
        assert_eq!(plain, vec![1.5; 100]);
    }

    #[test]
    fn specs_parse_and_refuse_what_they_do_not_know() {
        let got = parse_interference("spur=1000@20: spur=-2000@30 :tone=50@-30:rfburst=5@2@-10:pop=2@20@-20", 48e3);
        let want = vec![
            Interferer::Spur { offset_hz: 1000.0, ci_db: 20.0 },
            Interferer::Spur { offset_hz: -2000.0, ci_db: 30.0 },
            Interferer::Tone { freq_hz: 50.0, dbfs: -30.0 },
            Interferer::RfBursts { rate_hz: 5.0, duration_s: 0.002, cnr_db: -10.0 },
            Interferer::Pops { rate_hz: 2.0, duration_s: 0.02, dbfs: -20.0 },
        ];
        assert_eq!(got, Ok(want));
        assert_eq!(parse_interference("", 48e3), Ok(Vec::new()));
        for bad in [
            "spur",
            "spur=1000",
            "spur=1000@x",
            "spur=30000@20",
            "spur=1000@20@3",
            "tone=0@-20",
            "tone=24000@-20",
            "pop=0@1@-20",
            "pop=1@2@3@4",
            "rfburst=5@0@-10",
            "rfburst=5@2",
            "hum=50@-20",
        ] {
            assert!(parse_interference(bad, 48e3).is_err(), "{bad} parsed");
        }
        // A channel spec takes them after the channel's name, any number of each.
        let c = FmChannelConfig::parse_channel("mic-12k5:dev=2000:spur=1000@20:spur=500@25").unwrap();
        assert_eq!((c.deviation_hz, c.interference.len()), (2000.0, 2));
        assert!(FmChannelConfig::parse_channel("mic-12k5:spur=30000@20").is_err());
    }
}
