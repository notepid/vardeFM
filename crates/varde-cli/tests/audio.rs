//! End-to-end tests: generate modem audio, pass it through a channel, decode it.
//!
//! Set `VARDE_TEST_WAV_DIR=<folder>` to save the audio of each test as WAV files
//! (`<test>_tx.wav` and `<test>_rx.wav`) for listening or inspection in Audacity.

use std::f32::consts::PI;
use varde_cli::{count_errors, decode_all, make_frame, read_wav, test_bits, write_wav};
use varde_dsp::{Modulation, Ofdm, OfdmParams};
use varde_sim::{transmit, FmChannelConfig};

const FS: u32 = 48_000;
const NSYM: usize = 30;
const TXDELAY: f32 = 0.1;

fn modem(mode: &str) -> Ofdm {
    Ofdm::new(OfdmParams::by_name(mode).unwrap())
}

fn channel(mode: &str) -> FmChannelConfig {
    FmChannelConfig::by_name(mode).unwrap()
}

/// Save audio for inspection if VARDE_TEST_WAV_DIR is set.
fn dump(name: &str, tx: &[f32], rx: &[f32]) {
    if let Ok(dir) = std::env::var("VARDE_TEST_WAV_DIR") {
        // Cargo runs tests from the crate folder, so anchor relative paths at the repo root.
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let dir = root.join(dir);
        std::fs::create_dir_all(&dir).unwrap();
        let dir = dir.canonicalize().unwrap();
        let shown = dir.display().to_string();
        eprintln!("saving {name} audio to {}", shown.trim_start_matches(r"\\?\"));
        write_wav(dir.join(format!("{name}_tx.wav")), FS, tx).unwrap();
        write_wav(dir.join(format!("{name}_rx.wav")), FS, rx).unwrap();
    }
}

fn pad(audio: &[f32], before: usize, after: usize) -> Vec<f32> {
    let mut v = vec![0.0; before];
    v.extend_from_slice(audio);
    v.extend(std::iter::repeat(0.0).take(after));
    v
}

/// Send `frames` test frames one transmission at a time; return the worst BER and frames lost.
fn run(name: &str, mode: &str, m: Modulation, cfg: &FmChannelConfig, frames: usize) -> (f64, usize) {
    let ofdm = modem(mode);
    let mut worst = 0f64;
    let mut lost = 0;
    for f in 0..frames {
        let bits = test_bits(&ofdm, m, NSYM, f);
        let tx = make_frame(&ofdm, m, &bits, TXDELAY);
        let rx = transmit(&FmChannelConfig { seed: 100 + f as u64, ..cfg.clone() }, &tx);
        if f == 0 {
            dump(name, &tx, &rx);
        }
        match decode_all(&ofdm, &rx, NSYM).first() {
            Some((_, frame)) => {
                worst = worst.max(count_errors(m, frame, &bits) as f64 / bits.len() as f64);
            }
            None => lost += 1,
        }
    }
    (worst, lost)
}

// ---------------------------------------------------------------- direct audio

#[test]
fn direct_loopback_is_error_free() {
    for mode in ["narrow", "wide"] {
        let ofdm = modem(mode);
        for m in Modulation::ALL {
            let bits = test_bits(&ofdm, m, NSYM, 0);
            let tx = pad(&make_frame(&ofdm, m, &bits, TXDELAY), 1234, 4000);
            let frames = decode_all(&ofdm, &tx, NSYM);
            assert_eq!(frames.len(), 1, "{mode} {m:?}: frames found");
            assert_eq!(count_errors(m, &frames[0].1, &bits), 0, "{mode} {m:?}");
        }
    }
}

#[test]
fn level_and_polarity_do_not_matter() {
    // Sound cards and radios scale audio arbitrarily, and some invert it.
    let ofdm = modem("narrow");
    let m = Modulation::Qam16;
    let bits = test_bits(&ofdm, m, NSYM, 3);
    let tx = pad(&make_frame(&ofdm, m, &bits, TXDELAY), 500, 4000);
    for gain in [0.01, 0.2, 5.0, -1.0, -0.05] {
        let rx: Vec<f32> = tx.iter().map(|x| x * gain).collect();
        let frames = decode_all(&ofdm, &rx, NSYM);
        assert_eq!(frames.len(), 1, "gain {gain}");
        assert_eq!(count_errors(m, &frames[0].1, &bits), 0, "gain {gain}");
    }
}

#[test]
fn survives_16bit_wav_roundtrip() {
    let dir = std::env::temp_dir().join(format!("varde-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("roundtrip.wav");
    for mode in ["narrow", "wide"] {
        let ofdm = modem(mode);
        let m = Modulation::Qam64;
        let bits = test_bits(&ofdm, m, NSYM, 0);
        let tx = pad(&make_frame(&ofdm, m, &bits, TXDELAY), 2400, 2400);
        write_wav(&path, FS, &tx).unwrap();
        let (fs, rx) = read_wav(&path).unwrap();
        assert_eq!(fs, FS);
        let frames = decode_all(&ofdm, &rx, NSYM);
        assert_eq!(frames.len(), 1, "{mode}");
        assert_eq!(count_errors(m, &frames[0].1, &bits), 0, "{mode}");
    }
    std::fs::remove_dir_all(&dir).ok();
}

// ---------------------------------------------------------------- false alarms

#[test]
fn no_frames_found_in_non_modem_audio() {
    let n = FS as usize * 2;
    let mut rng = varde_dsp::prng::Prng::new(99);
    let noise: Vec<f32> = (0..n).map(|_| (rng.next_u32() as f32 / u32::MAX as f32 - 0.5) * 0.6).collect();
    let tone = |f: f32| (0..n).map(|i| 0.3 * (2.0 * PI * f * i as f32 / FS as f32).sin()).collect::<Vec<f32>>();
    let cases: Vec<(&str, Vec<f32>)> = vec![
        ("silence", vec![0.0; n]),
        ("white noise", noise),
        // 1500 Hz and 750 Hz repeat exactly every nfft/2 samples, like preamble A.
        ("1500 Hz tone", tone(1500.0)),
        ("750 Hz tone", tone(750.0)),
        ("1000 Hz tone", tone(1000.0)),
    ];
    for mode in ["narrow", "wide"] {
        let ofdm = modem(mode);
        for (name, audio) in &cases {
            assert!(ofdm.detect(audio, 0).is_none(), "{mode}: false frame in {name}");
        }
    }
}

#[test]
fn squelch_noise_is_not_a_frame() {
    // An open squelch with no carrier: pure discriminator noise.
    for mode in ["narrow", "wide"] {
        let cfg = FmChannelConfig { cnr_db: Some(-10.0), keyup_delay_s: 0.0, ..channel(mode) };
        let rx = transmit(&cfg, &vec![0.0; FS as usize * 2]);
        assert!(modem(mode).detect(&rx, 0).is_none(), "{mode}");
    }
}

// ---------------------------------------------------------------- through the radio

#[test]
fn clean_radio_link() {
    for mode in ["narrow", "wide"] {
        for m in [Modulation::Bpsk, Modulation::Qpsk, Modulation::Qam16] {
            let (ber, lost) = run(&format!("clean_{mode}_{}", m.name()), mode, m, &channel(mode), 2);
            assert_eq!(lost, 0, "{mode} {m:?}");
            // Narrow 16QAM has a small floor from the radio's deviation limiter.
            let max = if mode == "narrow" && m == Modulation::Qam16 { 1e-3 } else { 0.0 };
            assert!(ber <= max, "{mode} {m:?}: BER {ber:.2e}");
        }
    }
}

#[test]
fn clean_radio_link_64qam() {
    // Narrow mode clips a little in the radio's deviation limiter, so 64QAM has an
    // error floor without FEC. Guard against it getting worse.
    let (ber, lost) = run("clean_narrow_64qam", "narrow", Modulation::Qam64, &channel("narrow"), 2);
    assert_eq!(lost, 0);
    assert!(ber < 2e-2, "narrow 64qam BER {ber}");
    let (ber, lost) = run("clean_wide_64qam", "wide", Modulation::Qam64, &channel("wide"), 2);
    assert_eq!(lost, 0);
    assert!(ber < 1e-3, "wide 64qam BER {ber}");
}

#[test]
fn noisy_radio_link() {
    // (mode, modulation, CNR dB, max BER), uncoded. Limits are about 3x the worst
    // frame measured when the test was written; FEC will make these error-free.
    let cases = [
        ("narrow", Modulation::Bpsk, 10.0, 1e-3),
        ("narrow", Modulation::Qpsk, 12.0, 1e-3),
        ("narrow", Modulation::Qam16, 20.0, 1.5e-3),
        ("wide", Modulation::Bpsk, 15.0, 1e-2),
        ("wide", Modulation::Qpsk, 20.0, 7e-3),
        ("wide", Modulation::Qam16, 30.0, 1e-3),
    ];
    let mut failures = Vec::new();
    for (mode, m, cnr, max) in cases {
        let cfg = FmChannelConfig { cnr_db: Some(cnr), ..channel(mode) };
        let (ber, lost) = run(&format!("noisy_{mode}_{}_{cnr}dB", m.name()), mode, m, &cfg, 3);
        println!("{mode:>6} {:>6} @ {cnr:>4} dB: worst BER {ber:.2e} (limit {max:.0e}), lost {lost}", m.name());
        if lost > 0 || ber > max {
            failures.push(format!("{mode} {m:?} @ {cnr} dB: BER {ber:.2e}, lost {lost}"));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("
"));
}

#[test]
fn tolerates_sound_card_clock_offset() {
    for mode in ["narrow", "wide"] {
        for ppm in [-150.0, -50.0, 50.0, 150.0] {
            let cfg = FmChannelConfig { clock_offset_ppm: ppm, ..channel(mode) };
            let (ber, lost) = run(&format!("clock_{mode}_{ppm}ppm"), mode, Modulation::Qam16, &cfg, 1);
            assert_eq!(lost, 0, "{mode} {ppm} ppm");
            assert!(ber < 1e-3, "{mode} {ppm} ppm: BER {ber}");
        }
    }
}

#[test]
fn leader_covers_radio_keyup() {
    let ofdm = modem("narrow");
    let m = Modulation::Qpsk;
    let bits = test_bits(&ofdm, m, NSYM, 0);
    let cfg = FmChannelConfig { keyup_delay_s: 0.08, ..channel("narrow") };

    // Without a leader the preamble is lost while the radio keys up.
    let rx = transmit(&cfg, &make_frame(&ofdm, m, &bits, 0.0));
    let ok = decode_all(&ofdm, &rx, NSYM)
        .first()
        .is_some_and(|(_, f)| count_errors(m, f, &bits) == 0);
    assert!(!ok, "frame decoded without a leader");

    // With a leader longer than the key-up time it decodes.
    let tx = make_frame(&ofdm, m, &bits, 0.12);
    let rx = transmit(&cfg, &tx);
    dump("keyup_with_leader", &tx, &rx);
    let frames = decode_all(&ofdm, &rx, NSYM);
    assert_eq!(frames.len(), 1);
    assert_eq!(count_errors(m, &frames[0].1, &bits), 0);
}

#[test]
fn many_frames_in_one_long_recording() {
    // One continuous transmission with frames separated by gaps, as a receiver
    // would hear during an exchange. Every frame must be found, in order.
    for mode in ["narrow", "wide"] {
        let ofdm = modem(mode);
        let m = Modulation::Qpsk;
        let n_frames = 6;
        let mut tx = Vec::new();
        for f in 0..n_frames {
            tx.extend(vec![0.0; 3000 + 1700 * f]);
            tx.extend(make_frame(&ofdm, m, &test_bits(&ofdm, m, NSYM, f), TXDELAY));
        }
        tx.extend(vec![0.0; 5000]);
        let cnr = if mode == "narrow" { 15.0 } else { 25.0 };
        let rx = transmit(&FmChannelConfig { cnr_db: Some(cnr), clock_offset_ppm: 60.0, ..channel(mode) }, &tx);
        dump(&format!("stream_{mode}"), &tx, &rx);

        let frames = decode_all(&ofdm, &rx, NSYM);
        assert_eq!(frames.len(), n_frames, "{mode}: frames found");
        for (f, (_, frame)) in frames.iter().enumerate() {
            let bits = test_bits(&ofdm, m, NSYM, f);
            let ber = count_errors(m, frame, &bits) as f64 / bits.len() as f64;
            assert!(ber < 1e-3, "{mode} frame {f}: BER {ber}");
        }
    }
}
