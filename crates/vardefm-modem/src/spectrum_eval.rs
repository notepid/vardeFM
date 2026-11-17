//! Occupied bandwidth and adjacent-channel power of real bursts on the air, per
//! mode, channel and level: whether a mode fits a 12.5 kHz or 25 kHz channel.
//!
//! Each level is measured on its own bursts at its own drive (`Modem::tx_rms`),
//! leader and preambles included, through the channel's transmitter
//! (`vardefm_sim::modulate`): audio filter, pre-emphasis, deviation limiter and
//! FM modulator. The spectrum is a Welch estimate over the carrier only. Reported:
//! - `obw99`: the 99 % occupied bandwidth (ITU-R SM.328), and its edges;
//! - `out6k25` / `out12k5`: the power further than 6.25 kHz / 12.5 kHz from the
//!   carrier, which a 12.5 kHz / 25 kHz channel does not hold, dBc;
//! - `acp12k5`: adjacent-channel power for 12.5 kHz spacing, the worse side, in
//!   `ACP_BW_12K5_HZ` centred 12.5 kHz away, dBc. It is a rectangular band sum
//!   of the spectrum of real bursts, not the standard's measuring-receiver filter
//!   and test tone, so read it against the -60 dBc limit to a few dB, not finer.
//!   The 25 kHz adjacent channel
//!   (17 to 33 kHz away) lies beyond what the simulator's complex baseband holds
//!   at 48 kHz, so for 25 kHz spacing only `out12k5` is given;
//! - `far`: the power further than `FAR_HZ` from the carrier, which shows whether
//!   the complex baseband holds the signal; it must stay far below every figure
//!   above.
//!
//! The measurement itself is checked in `vardefm_sim::spectrum` on signals of known
//! width, one of them too wide for 12.5 kHz.
//!
//! `cargo test --release -p vardefm-modem occupied_bandwidth -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `CHANNELS` (channel specs as in
//! `FmChannelConfig::parse_channel`; unset, each mode on its own channel), `LEVELS`
//! (0,..,6), `BURSTS` per level (8).

use super::*;
use crate::detect_eval::{
    burst, channels_for, env_channels, env_list, env_modes, env_or, mode_on, modem, payload_seed,
};
use vardefm_sim::spectrum::Spectrum;
use vardefm_sim::FmChannelConfig;

/// FFT length of the Welch estimate: about 12 Hz bins at 48 kHz.
const NFFT: usize = 4096;
/// Adjacent-channel measurement bandwidth for 12.5 kHz spacing, Hz (ETSI EN 300 086).
const ACP_BW_12K5_HZ: f32 = 8_500.0;
/// The aliasing check: power beyond this offset, Hz.
const FAR_HZ: f32 = 20_000.0;
/// `far` must lie at least this far below each figure it could inflate, dB.
const FAR_MARGIN_DB: f64 = 10.0;
/// ... unless that figure is itself below this, where it no longer matters, dBc.
const FAR_IRRELEVANT_DBC: f64 = -100.0;
/// Codewords per burst at most, to keep the run short.
const CODEWORDS: usize = 8;

fn dbc(fraction: f64) -> f64 {
    10.0 * fraction.max(1e-15).log10()
}

/// The spectrum of `bursts` bursts at `level` through the transmitter of `channel`.
fn spectrum(m: &Modem, channel: &FmChannelConfig, level: u8, bursts: u32) -> Spectrum {
    let ncw = m.max_ncw(level).min(CODEWORDS);
    let rfs: Vec<_> =
        (0..bursts).map(|t| vardefm_sim::modulate(channel, &m.modulate(&burst(level, ncw, payload_seed(t))))).collect();
    Spectrum::welch(rfs.iter().map(|rf| &rf.samples[rf.carrier.clone()]), channel.fs, NFFT)
}

/// The worse of the two adjacent channels `spacing` Hz away, measured in `bw` Hz, dBc.
fn acp(s: &Spectrum, spacing: f32, bw: f32) -> f64 {
    let side = |c: f32| s.fraction_in(c - bw / 2.0, c + bw / 2.0);
    dbc(side(-spacing).max(side(spacing)))
}

#[test]
#[ignore]
fn occupied_bandwidth() {
    let modes = env_modes("narrow,wide");
    let channels = env_channels();
    let levels: Vec<u8> = env_list("LEVELS", "0,1,2,3,4,5,6");
    assert!(levels.iter().all(|&l| (l as usize) < LEVELS.len()), "LEVELS: 0 to {}", LEVELS.len() - 1);
    let bursts = env_or("BURSTS", 8u32);
    assert!(bursts > 0, "BURSTS must be at least 1");
    println!(
        "{:<36} {:>2} {:>5} {:>7} {:>16} {:>8} {:>8} {:>8} {:>8}",
        "mode on channel", "L", "drive", "obw99", "edges", "out6k25", "acp12k5", "out12k5", "far"
    );
    for mode in &modes {
        let m = modem(mode);
        for (name, channel) in &channels_for(mode, &channels) {
            let label = mode_on(mode, name.as_deref());
            assert!(FAR_HZ < channel.fs / 2.0, "{label}: fs {} Hz does not reach {FAR_HZ} Hz", channel.fs);
            for &level in &levels {
                let s = spectrum(&m, channel, level, bursts);
                let (lo, hi) = s.occupied(0.99);
                let (acp12k5, out12k5) = (acp(&s, 12_500.0, ACP_BW_12K5_HZ), dbc(s.fraction_outside(12_500.0)));
                let far = dbc(s.fraction_outside(FAR_HZ));
                for (what, x) in [("acp12k5", acp12k5), ("out12k5", out12k5)] {
                    assert!(
                        far <= x - FAR_MARGIN_DB || x < FAR_IRRELEVANT_DBC,
                        "{label} L{level}: power beyond {FAR_HZ} Hz ({far:.1} dBc) is too near {what} ({x:.1} dBc)"
                    );
                }
                println!(
                    "{:<36} {level:>2} {:>5.2} {:>7.0} {:>16} {:>8.1} {:>8.1} {:>8.1} {:>8.1}",
                    label,
                    m.tx_rms(level),
                    hi - lo,
                    format!("{lo:.0}..{hi:.0}"),
                    dbc(s.fraction_outside(6_250.0)),
                    acp12k5,
                    out12k5,
                    far,
                );
            }
        }
    }
    println!("{bursts} bursts of up to {CODEWORDS} codewords per level, Welch {NFFT} points");
}

/// The spectrum claims behind docs/RADIO.md in CI, not only in the ignored runs:
/// `narrow` on the mic path stays inside the 11.25 kHz authorised for 12.5 kHz
/// equipment, `narrow` on the data port at 2.5 kHz stays under the -60 dBc
/// adjacent-channel limit (measured -75 dBc), and `wide` on
/// its own channel spills far over the -60 dBc adjacent-channel limit. By 99 %
/// occupied bandwidth `wide` is only about 11 kHz, so the spill is the check that
/// tells it does not fit 12.5 kHz.
#[test]
fn presets_fit_or_spill_as_measured() {
    let s = spectrum(&modem("narrow"), &FmChannelConfig::parse_channel("mic-12k5").unwrap(), 0, 1);
    let (lo, hi) = s.occupied(0.99);
    assert!(hi - lo < 11_250.0, "narrow on mic-12k5: obw99 {:.0} Hz", hi - lo);
    let s = spectrum(&modem("narrow"), &FmChannelConfig::parse_channel("data-12k5:dev=2500").unwrap(), 0, 1);
    let a = acp(&s, 12_500.0, ACP_BW_12K5_HZ);
    assert!(a < -65.0, "narrow on data-12k5:dev=2500: ACP {a:.1} dBc, expected well under -60");
    let s = spectrum(&modem("wide"), &FmChannelConfig::parse_channel("data-25k").unwrap(), 0, 1);
    let a = acp(&s, 12_500.0, ACP_BW_12K5_HZ);
    assert!(a > -50.0, "wide on data-25k: ACP {a:.1} dBc, expected far over -60");
}
