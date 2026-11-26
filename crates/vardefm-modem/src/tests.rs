use super::*;
use std::sync::Arc;
use vardefm_dsp::prng::Prng;
use vardefm_sim::{transmit, FmChannelConfig};

fn modem(mode: &str) -> Arc<Modem> {
    Arc::new(Modem::new(OfdmParams::by_name(mode).unwrap(), 0.1))
}

fn burst(level: u8, ncw: usize, seed: u32) -> Burst {
    let mut rng = Prng::new(seed);
    let n = Modem::payload_bytes(level);
    Burst {
        header: Header {
            kind: Kind::Data,
            session: 0x42,
            level,
            ncw: ncw as u8,
            req_level: 2,
            burst_id: 3,
            ack_burst: 5,
            ack_offset: 12345,
            bitmap: 0b1011,
            flags: 0,
        },
        payload: (0..ncw).map(|_| (0..n).map(|_| rng.next_u32() as u8).collect()).collect(),
    }
}

/// Push audio in 10 ms chunks, like a sound card would deliver it.
fn receive(rx: &mut Receiver, audio: &[f32]) -> Vec<RxBurst> {
    let mut out = Vec::new();
    for chunk in audio.chunks(480) {
        out.extend(rx.push(chunk));
    }
    out.extend(rx.push(&vec![0.0; 48_000]));
    out
}

#[test]
fn every_level_roundtrips_clean() {
    for mode in ["narrow", "wide"] {
        let m = modem(mode);
        for level in 0..LEVELS.len() as u8 {
            let b = burst(level, 2, level as u32 + 1);
            let mut audio = vec![0.0; 5000];
            audio.extend(m.modulate(&b));
            assert_eq!(audio.len() - 5000, m.burst_samples(level, 2));
            let got = receive(&mut Receiver::new(m.clone()), &audio);
            assert_eq!(got.len(), 1, "{mode} level {level}");
            assert_eq!(got[0].header, b.header);
            let payload: Vec<Vec<u8>> = got[0].payload.iter().map(|p| p.clone().expect("codeword failed")).collect();
            assert_eq!(payload, b.payload, "{mode} level {level}");
        }
    }
}

#[test]
fn header_only_and_back_to_back_bursts() {
    let m = modem("narrow");
    let mut ack = burst(0, 0, 1);
    ack.header.kind = Kind::DiscAck;
    let data = burst(3, 3, 2);
    let mut audio = vec![0.0; 2000];
    audio.extend(m.modulate(&ack));
    audio.extend(vec![0.0; 7000]);
    audio.extend(m.modulate(&data));
    let got = receive(&mut Receiver::new(m.clone()), &audio);
    assert_eq!(got.len(), 2);
    assert_eq!(got[0].header, ack.header);
    assert!(got[0].payload.is_empty());
    assert_eq!(got[1].header, data.header);
    assert!(got[1].payload.iter().all(|p| p.is_some()));
}

#[test]
fn decodes_through_noisy_radio() {
    // (mode, level, CNR): all codewords must decode.
    for (mode, level, cnr) in [("narrow", 1, 12.0), ("narrow", 4, 22.0), ("wide", 1, 18.0), ("wide", 4, 30.0)] {
        let m = modem(mode);
        let b = burst(level, m.max_ncw(level).min(6), 7);
        let cfg =
            FmChannelConfig { cnr_db: Some(cnr), clock_offset_ppm: 70.0, ..FmChannelConfig::by_name(mode).unwrap() };
        let rx_audio = transmit(&cfg, &m.modulate(&b));
        let got = receive(&mut Receiver::new(m.clone()), &rx_audio);
        assert_eq!(got.len(), 1, "{mode} level {level}");
        let ok = got[0].payload.iter().filter(|p| p.is_some()).count();
        assert_eq!(ok, b.payload.len(), "{mode} level {level} @ {cnr} dB, SNR {:.1}", got[0].snr_db);
    }
}

/// Prints, per level, the codeword success rate and estimated SNR across CNR.
/// Used to set the rate-adaptation thresholds in vardefm-link.
/// `cargo test --release -p vardefm-modem level_table -- --ignored --nocapture`
#[test]
#[ignore]
fn level_table() {
    for mode in ["narrow", "wide"] {
        let m = modem(mode);
        println!("{mode}: net rates {:?}", (0..7).map(|l| m.net_rate(l) as u32).collect::<Vec<_>>());
        for level in 0..LEVELS.len() as u8 {
            let mut line = format!("{mode} L{level} {:>10}:", LEVELS[level as usize].name);
            for cnr in [6.0, 8.0, 10.0, 12.0, 14.0, 17.0, 20.0, 24.0, 28.0, 33.0, 40.0] {
                let (mut ok, mut total, mut snr, mut nf) = (0, 0, 0.0, 0);
                for t in 0..4 {
                    let b = burst(level, m.max_ncw(level).min(4), 100 + t);
                    let cfg = FmChannelConfig {
                        cnr_db: Some(cnr),
                        seed: t as u64 + 1,
                        ..FmChannelConfig::by_name(mode).unwrap()
                    };
                    let got = receive(&mut Receiver::new(m.clone()), &transmit(&cfg, &m.modulate(&b)));
                    total += b.payload.len();
                    if let Some(g) = got.first() {
                        ok += g.payload.iter().filter(|p| p.is_some()).count();
                        snr += g.snr_db;
                        nf += 1;
                    }
                }
                line += &format!(" {cnr}:{:.0}%/{:.1}", 100.0 * ok as f32 / total as f32, snr / nf.max(1) as f32);
            }
            println!("{line}");
        }
    }
}

#[test]
fn threaded_decoding_matches_single_thread() {
    // A noisy link where some codewords fail: the result must not depend on the thread count.
    let params = OfdmParams::by_name("narrow").unwrap();
    let b = burst(6, 9, 11);
    let cfg = FmChannelConfig { cnr_db: Some(16.0), seed: 5, ..FmChannelConfig::by_name("narrow").unwrap() };
    let audio = transmit(&cfg, &Modem::new(params.clone(), 0.1).modulate(&b));
    let mut results = Vec::new();
    for threads in [1, 2, 4, 16] {
        let mut m = Modem::new(params.clone(), 0.1);
        m.decode_threads = threads;
        let got = receive(&mut Receiver::new(Arc::new(m)), &audio);
        assert_eq!(got.len(), 1);
        results.push(got[0].payload.clone());
    }
    let ok = results[0].iter().filter(|p| p.is_some()).count();
    assert!(ok > 0 && ok < 9, "want a mix of good and failed codewords, got {ok}/9");
    assert!(results.iter().all(|r| *r == results[0]));
}

/// Received audio of `b` through the simulator, and where its preamble B starts.
fn received(m: &Modem, mode: &str, b: &Burst, cnr: f32) -> (Vec<f32>, usize) {
    let cfg = FmChannelConfig { cnr_db: Some(cnr), ..FmChannelConfig::by_name(mode).unwrap() };
    let audio = transmit(&cfg, &m.modulate(b));
    let start = m.ofdm.detect(&audio, 0).at.expect("frame not detected");
    (audio, start)
}

#[test]
fn header_training_raises_snr() {
    for mode in ["narrow", "wide"] {
        let m = modem(mode);
        let b = burst(4, 2, 21);
        let (audio, start) = received(&m, mode, &b, 30.0);
        let nsym = m.header_symbols() + 2 * m.symbols_per_codeword(4);
        let plain = m.ofdm.demodulate(&audio, start, nsym).unwrap();
        let known = m.header_carriers(&b.header);
        let trained = m.ofdm.demodulate_known(&audio, start, nsym, &known).unwrap();
        assert!(
            trained.snr_db > plain.snr_db + 0.5,
            "{mode}: SNR {:.1} dB with header training, {:.1} dB without",
            trained.snr_db,
            plain.snr_db
        );
        assert!(m.decode_payload(&b.header, &trained).iter().all(|p| p.is_some()), "{mode}");
    }
}

#[test]
fn survives_a_click() {
    // An FM threshold click or a squelch pop wipes out one whole OFDM symbol. Its
    // per-symbol noise scale must keep its LLRs from swamping the decoder.
    let (mode, level, ncw) = ("narrow", 6, 4);
    let m = modem(mode);
    let b = burst(level, ncw, 31);
    let (mut audio, start) = received(&m, mode, &b, 24.0);
    let p = &m.ofdm.p;
    let l = p.symbol_len();
    let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
    // Data symbol i, cyclic prefix included, starts at start + (i+1)*l - ncp.
    let hit = m.header_symbols() + ncw * m.symbols_per_codeword(level) / 2;
    let from = start + (hit + 1) * l - p.ncp;
    let sig = &audio[start..start + nsym * l];
    let rms = (sig.iter().map(|x| x * x).sum::<f32>() / sig.len() as f32).sqrt();
    // Uniform noise in ±a has RMS a/sqrt(3): make it 3x the signal.
    let a = 3.0 * rms * 3f32.sqrt();
    let mut rng = Prng::new(77);
    for x in &mut audio[from..from + l] {
        *x = a * rng.uniform();
    }

    let known = m.header_carriers(&b.header);
    let mut frame = m.ofdm.demodulate_known(&audio, start, nsym, &known).unwrap();
    let scale = frame.symbol_noise_scale.clone();
    assert!(scale[hit] > 10.0, "scales {scale:?}");
    assert!(scale.iter().enumerate().all(|(i, &s)| i == hit || s < scale[hit] / 10.0), "scales {scale:?}");
    let ok = m.decode_payload(&b.header, &frame).iter().filter(|p| p.is_some()).count();
    assert_eq!(ok, ncw);
    // Without the scale the click symbol's LLRs are far too confident. One pass only:
    // the later passes must not get to rescue what the missing scale loses.
    frame.symbol_noise_scale.iter_mut().for_each(|s| *s = 1.0);
    let ok_flat = m.decode_payload_passes(&b.header, &frame, 1).data.iter().filter(|p| p.is_some()).count();
    assert!(ok_flat < ncw, "{ok_flat} codewords decode even without the per-symbol scale");

    // And the same through the streaming receiver.
    let got = receive(&mut Receiver::new(m.clone()), &audio);
    assert_eq!(got.len(), 1);
    assert!(got[0].payload.iter().all(|p| p.is_some()));
}

#[test]
fn header_only_burst_reports_the_same_snr() {
    // Rate adaptation uses the SNR of every burst, acks included.
    for (mode, cnr) in [("narrow", 15.0), ("wide", 22.0)] {
        let m = modem(mode);
        let mut snr = Vec::new();
        for ncw in [0, 3] {
            let mut sum = 0.0;
            for seed in 1..=4 {
                let b = burst(3, ncw, seed);
                let cfg =
                    FmChannelConfig { cnr_db: Some(cnr), seed: seed as u64, ..FmChannelConfig::by_name(mode).unwrap() };
                let got = receive(&mut Receiver::new(m.clone()), &transmit(&cfg, &m.modulate(&b)));
                assert_eq!(got.len(), 1, "{mode} ncw {ncw} seed {seed}");
                sum += got[0].snr_db;
            }
            snr.push(sum / 4.0);
        }
        eprintln!("{mode}: header-only {:.2} dB, data {:.2} dB", snr[0], snr[1]);
        // A data burst's SNR includes the clipping noise of its payload symbols (the
        // per-symbol noise scale picks them up), which a header-only burst cannot see:
        // about 0.8 dB in narrow mode at the 0.30 drive, 0.3 dB at 0.20. So the header-only
        // burst reads the same or a little higher, not meaningfully lower: measured without
        // the header as training it reads 0.49 dB (narrow) and 0.97 dB (wide) lower, which
        // a symmetric 1 dB bound lets through.
        let diff = snr[0] - snr[1];
        assert!(diff > -0.25 && diff < 1.0, "{mode}: header-only {:.2} dB, data {:.2} dB", snr[0], snr[1]);
    }
}

/// Error vector magnitude (dB, relative to the symbol power) of the equalized header
/// symbols of `f` against the transmitted values, skipping symbol `skip`.
fn header_evm_db(f: &Frame, known: &[Vec<Complex32>], skip: usize) -> f32 {
    let (mut err, mut sig) = (0.0, 0.0);
    for (i, k) in known.iter().enumerate().filter(|(i, _)| *i != skip) {
        for (y, x) in f.symbols[i].iter().zip(k) {
            err += (y - x).norm_sqr();
            sig += x.norm_sqr();
        }
    }
    10.0 * (err / sig).log10()
}

#[test]
fn survives_a_click_in_the_header() {
    // A click on a header symbol: the header still decodes (its code is rate 0.24),
    // so the symbol is offered as channel training. Averaged in at full weight it
    // would corrupt the estimate for every carrier of the burst, which no later
    // per-symbol scale can undo. The training must leave it out.
    let (mode, level, ncw) = ("narrow", 6, 4);
    let m = modem(mode);
    let b = burst(level, ncw, 31);
    let (clean, start) = received(&m, mode, &b, 24.0);
    let p = &m.ofdm.p;
    let l = p.symbol_len();
    let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
    let hit = m.header_symbols() / 2;
    let from = start + (hit + 1) * l - p.ncp;
    let sig = &clean[start..start + nsym * l];
    let rms = (sig.iter().map(|x| x * x).sum::<f32>() / sig.len() as f32).sqrt();
    let a = 3.0 * rms * 3f32.sqrt();
    let mut audio = clean.clone();
    let mut rng = Prng::new(78);
    for x in &mut audio[from..from + l] {
        *x = a * rng.uniform();
    }

    let known = m.header_carriers(&b.header);
    let reference = m.ofdm.demodulate_known(&clean, start, nsym, &known).unwrap();
    let frame = m.ofdm.demodulate_known(&audio, start, nsym, &known).unwrap();
    assert!(frame.symbol_noise_scale[hit] > 10.0, "scales {:?}", frame.symbol_noise_scale);
    let (evm_ref, evm) = (header_evm_db(&reference, &known, hit), header_evm_db(&frame, &known, hit));
    assert!(evm < evm_ref + 1.0, "header EVM {evm:.1} dB with the click, {evm_ref:.1} dB without");
    let ok = m.decode_payload(&b.header, &frame).iter().filter(|p| p.is_some()).count();
    assert_eq!(ok, ncw);

    // And through the streaming receiver, where the header is decoded first.
    let got = receive(&mut Receiver::new(m.clone()), &audio);
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].header, b.header);
    assert!(got[0].payload.iter().all(|p| p.is_some()));
}

#[test]
fn burst_level_ignores_a_squelch_pop_after_the_burst() {
    let m = modem("narrow");
    let mut ack = burst(0, 0, 7);
    ack.header.kind = Kind::DiscAck;
    for b in [burst(2, 2, 6), ack] {
        // The burst at -6 dB, then a full-scale pop right after its last symbol.
        let tx: Vec<f32> = m.modulate(&b).iter().map(|x| x * 0.5).collect();
        let expected = AudioLevel::of(&tx);
        let mut audio = vec![0.0; 3000];
        audio.extend(&tx);
        audio.extend(vec![0.0; 100]);
        audio.extend(vec![1.0; 200]);
        let got = receive(&mut Receiver::new(m.clone()), &audio);
        assert_eq!(got.len(), 1);
        let level = got[0].level;
        assert!(level.peak_dbfs < -5.0, "pop counted: {level:?}");
        // The whole burst is measured (preamble A included): its peak is the burst's own.
        assert!((level.peak_dbfs - expected.peak_dbfs).abs() < 0.5, "{level:?} vs {expected:?}");
    }
}

/// `rms` sine at `f` Hz, `n` samples starting at `phase`.
fn sine(f: f32, rms: f32, n: usize, phase: f32) -> Vec<f32> {
    let w = 2.0 * std::f32::consts::PI * f / 48_000.0;
    (0..n).map(|i| rms * 2f32.sqrt() * (i as f32 * w + phase).sin()).collect()
}

/// A steady CTCSS tone (a keyed repeater) heard before and after our own transmission.
/// The receiver is fed nothing while we transmit. After it, the radio is muted until
/// it is back on receive (tens of ms for a handheld, a VOX interface's hang time of a
/// few hundred ms), and then the tone comes back abruptly, at any phase.
#[test]
fn ctcss_tone_after_our_own_transmission_is_not_busy() {
    let m = modem("wide");
    let mut seed = 1u32;
    for rms in [0.177, 0.354] {
        for muted_ms in [0, 5, 30, 100, 300, 600] {
            for jump in [0.5, 1.0, 1.5] {
                for floor in [0.0, 3e-4] {
                    let mut rx = Receiver::new(m.clone());
                    for chunk in sine(254.1, rms, 3 * 48_000, 0.0).chunks(480) {
                        rx.push(chunk);
                    }
                    assert!(!rx.busy());
                    rx.reset();
                    // The muted radio, with or without a faint noise floor from the sound card.
                    let mut audio: Vec<f32> = (0..muted_ms * 48)
                        .map(|_| {
                            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                            floor * (seed as f32 / u32::MAX as f32 * 2.0 - 1.0)
                        })
                        .collect();
                    audio.extend(sine(254.1, rms, 24_000, jump * std::f32::consts::PI));
                    for (i, chunk) in audio.chunks(480).enumerate() {
                        rx.push(chunk);
                        assert!(
                            !rx.busy(),
                            "{rms} RMS, muted {muted_ms} ms, jump {jump} pi, floor {floor}: busy after {} ms",
                            10 * (i + 1)
                        );
                    }
                }
            }
        }
    }
}

/// A burst sent with a CTCSS tone on the carrier decodes, on every receive path of
/// `detector_eval`'s CTCSS pass. Each case missed the burst before detection was
/// high-passed and confirmed on in-band energy (#12): 94.8 Hz is in antiphase at the
/// wide lag of nfft/2 and keeps the Schmidl-Cox metric below threshold; 186.2 Hz is
/// in phase and starts the plateau in the leader; in narrow mode, 141.3 Hz swells the
/// energy that confirms preamble B.
#[test]
fn a_burst_with_a_ctcss_tone_decodes() {
    use crate::detect_eval::{ctcss_cases, ctcss_channel, with_ctcss};
    let mut missed = Vec::new();
    for (mode, f, dev) in [("wide", 94.8, 1500.0), ("wide", 186.2, 1500.0), ("narrow", 141.3, 1500.0)] {
        let m = modem(mode);
        let b = burst(1, 1, 7);
        for case in ctcss_cases(mode) {
            let cfg = FmChannelConfig { cnr_db: None, ..FmChannelConfig::by_name(mode).unwrap() };
            let audio = with_ctcss(&m, &ctcss_channel(&cfg, case), case.tap, &b, f, dev, 0.0);
            let got = receive(&mut Receiver::new(m.clone()), &audio);
            if !got
                .iter()
                .any(|g| g.header == b.header && g.payload.first().and_then(|p| p.as_ref()) == b.payload.first())
            {
                missed.push(format!("{mode}, {}: {f} Hz at {dev} Hz deviation, {} burst(s)", case.name, got.len()));
            }
        }
    }
    assert!(missed.is_empty(), "{missed:#?}");
}

/// Busy holds for a while after the last loud audio, but not across our own
/// transmission: the channel is not busy when reception resumes just because it was
/// busy shortly before we keyed up.
#[test]
fn a_busy_hold_does_not_outlast_our_own_transmission() {
    let m = modem("narrow");
    let mut rx = Receiver::new(m.clone());
    rx.push(&vec![0.0; 48_000]);
    for chunk in sine(993.0, 0.1, 4800, 0.0).chunks(480) {
        rx.push(chunk);
    }
    rx.push(&vec![0.0; 4800]);
    assert!(rx.busy(), "the hold runs 100 ms after the signal");
    rx.reset();
    rx.push(&[0.0; 48]);
    assert!(!rx.busy(), "busy after our transmission");
}

/// A NaN, an Inf or a sample far beyond full scale from the sound card in a steady CTCSS tone, at its peak, up to
/// 1500 Hz deviation on the wide port.
#[test]
fn a_non_finite_sample_in_a_ctcss_tone_is_not_busy() {
    let m = modem("wide");
    for rms in [0.177, 0.354] {
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, f32::MAX, -1e30, 5.0] {
            let mut audio = sine(254.1, rms, 4 * 48_000, 0.0);
            let peak = (3 * 48_000..3 * 48_000 + 200).max_by(|&a, &b| audio[a].total_cmp(&audio[b])).unwrap();
            audio[peak] = bad;
            let mut rx = Receiver::new(m.clone());
            for (i, chunk) in audio.chunks(480).enumerate() {
                rx.push(chunk);
                assert!(!rx.busy(), "{rms} RMS, {bad}: busy after {} ms", 10 * (i + 1));
            }
        }
    }
}

/// A NaN, an Inf or a sample far beyond full scale from the sound card before or inside a burst.
#[test]
fn a_non_finite_sample_does_not_hide_a_burst() {
    for mode in ["narrow", "wide"] {
        let m = modem(mode);
        let b = burst(2, 2, 9);
        for bad in [f32::NAN, f32::INFINITY, f32::MAX] {
            // From 1000 samples before the burst's leader to well into its payload.
            for at in (19_000..31_000).step_by(1000) {
                let mut audio = vec![0.0; 20_000];
                audio.extend(m.modulate(&b));
                audio[at] = bad;
                let got = receive(&mut Receiver::new(m.clone()), &audio);
                assert_eq!(got.len(), 1, "{mode}, {bad} at {at}");
                assert!(got[0].payload.iter().all(|p| p.is_some()), "{mode}, {bad} at {at}");
            }
        }
    }
}

/// A link, its channel, and the audio of a level 1 burst sent on it.
struct Link {
    name: &'static str,
    chan: FmChannelConfig,
    audio: Vec<f32>,
}

impl Link {
    /// What the receiver gets at `cnr_db` (`None`: no noise) with noise `seed`.
    fn received(&self, cnr_db: Option<f32>, seed: u64) -> Vec<f32> {
        transmit(&FmChannelConfig { cnr_db, seed, ..self.chan.clone() }, &self.audio)
    }

    /// Where detection finds the burst without noise.
    fn clean_position(&self, rx_modem: &Modem) -> usize {
        let found = rx_modem.ofdm.detect(&self.received(None, 0), 0).at;
        found.unwrap_or_else(|| panic!("{}: not detected without noise", self.name))
    }
}

/// The matched link of `mode`, and links whose audio response differs from the one the
/// sender shaped its audio for: emphasis at one end only, and a sender tilted
/// 6 dB/octave off.
fn links(mode: &str) -> Vec<Link> {
    let us = FmChannelConfig::narrow().tx_emphasis_us;
    let base = FmChannelConfig::by_name(mode).unwrap();
    let links = [
        ("matched", base.clone(), 0.0),
        ("tx-pre-only", FmChannelConfig { tx_emphasis_us: us, rx_emphasis_us: None, ..base.clone() }, 0.0),
        ("rx-de-only", FmChannelConfig { tx_emphasis_us: None, rx_emphasis_us: us, ..base.clone() }, 0.0),
        ("tilt -6", base.clone(), -6.0),
        ("tilt +6", base, 6.0),
    ];
    let sent = |tilt: f32| {
        let mut p = OfdmParams::by_name(mode).unwrap();
        p.tilt_db_per_octave += tilt;
        Modem::new(p, 0.15).modulate(&burst(1, 1, 0))
    };
    links.into_iter().map(|(name, chan, tilt)| Link { name, chan, audio: sent(tilt) }).collect()
}

fn link<'a>(links: &'a [Link], name: &str) -> &'a Link {
    links.iter().find(|l| l.name == name).unwrap_or_else(|| panic!("no link named {name}"))
}

/// Whether `at` is `want`, as near as timing has to be: within a quarter cyclic prefix.
fn is_at(rx_modem: &Modem, at: usize, want: usize) -> bool {
    at.abs_diff(want) <= rx_modem.ofdm.p.ncp / 4
}

/// A link whose audio response differs from the one the sender shaped its audio for
/// (emphasis at one end only, or a sender tilted 6 dB/octave off) still has its burst
/// found, and where the matched link's is. Detection's filters move the Schmidl-Cox
/// plateau with the audio's tilt, and preamble B used to be looked for only one cyclic
/// prefix either side of it: the two wide links that tilt down were never found without
/// noise, and narrow with the sender tilted down was one sample from that.
#[test]
fn mismatched_links_are_detected_where_the_matched_link_is() {
    for mode in ["narrow", "wide"] {
        let rx_modem = modem(mode);
        let links = links(mode);
        let want = link(&links, "matched").clean_position(&rx_modem);
        for l in &links {
            // Noiseless, then strong signals with noise, which moves the plateau too.
            let cases = [(None, 0), (Some(20.0), 1), (Some(20.0), 2), (Some(20.0), 3)];
            for (cnr_db, seed) in cases {
                let got = rx_modem.ofdm.detect(&l.received(cnr_db, seed), 0).at;
                assert!(
                    got.is_some_and(|b| is_at(&rx_modem, b, want)),
                    "{mode} {} cnr {cnr_db:?} seed {seed}: found {got:?}, matched link at {want}",
                    l.name
                );
            }
        }
    }
}

/// The two wide links whose audio tilts down are found 2 dB above the sensitivity knee
/// too. Their plateau mostly puts preamble B more than a cyclic prefix late, so it has
/// to reach the higher correlation asked for out there, and noise lowers what it
/// reaches: a threshold much above what keeps other plateaus out would lose these links
/// again.
#[test]
fn wide_links_that_tilt_down_are_detected_above_the_knee() {
    let rx_modem = modem("wide");
    let links = links("wide");
    for name in ["rx-de-only", "tilt -6"] {
        let l = link(&links, name);
        let want = l.clean_position(&rx_modem);
        let seeds = 1..=20;
        let found = seeds
            .clone()
            .filter(|&seed| {
                rx_modem.ofdm.detect(&l.received(Some(9.0), seed), 0).at.is_some_and(|b| is_at(&rx_modem, b, want))
            })
            .count();
        assert!(found >= 18, "{name}: found {found} of {} bursts at 9 dB CNR", seeds.count());
    }
}

/// What the preamble B search's span and thresholds in `vardefm-dsp` rest on
/// (`FINE_TIMING_SPAN_NCP`, `MIN_FAR_PREAMBLE_CORR`): per link and CNR, how far from
/// preamble B the Schmidl-Cox plateau puts it, and what correlation preamble B and
/// other places reach within one cyclic prefix of that ("near") and further off
/// ("far"). Prints; asserts nothing.
///
/// ```sh
/// cargo test --release -p vardefm-modem preamble_b_search_eval -- --ignored --nocapture
/// ```
/// TRIALS bursts per link and CNR (200), NOISE_SECONDS of carrier without a burst (60),
/// with noise seeds from SEED on (1000). The threshold was chosen on the default seeds
/// and checked on SEED=50000 and SEED=90000.
#[test]
#[ignore]
fn preamble_b_search_eval() {
    use crate::detect_eval::{env_or, on_every_core};
    use vardefm_dsp::ofdm::PlateauPeaks;
    let trials: u32 = env_or("TRIALS", 200);
    let noise_s: f32 = env_or("NOISE_SECONDS", 60.0);
    let seed0: u64 = env_or("SEED", 1000);
    assert!(noise_s > 0.0 && noise_s.is_finite(), "NOISE_SECONDS={noise_s}: must be a finite number more than 0");
    assert!(trials > 0, "TRIALS must be at least 1");
    let far_min = PlateauPeaks::MIN_FAR_CORR;
    let max = |v: &[f32]| v.iter().copied().fold(0.0f32, f32::max);
    // The value `f` of the way through sorted `v`.
    let at = |v: &[f32], f: f32| v.get(((v.len() as f32 - 1.0) * f).round() as usize).copied().unwrap_or(f32::NAN);
    for mode in ["narrow", "wide"] {
        let rx_modem = modem(mode);
        let ofdm = &rx_modem.ofdm;
        let ncp = ofdm.p.ncp;
        let knee = if mode == "narrow" { 5.0 } else { 7.0 };
        println!("{mode}: cyclic prefix {ncp} samples, {trials} bursts per line, seeds from {seed0}");
        println!("  plateau: where it puts preamble B, in prefixes late of where it is (lowest, median, highest)");
        println!("  found: bursts detected at preamble B, of them by a far place; wrong: detected elsewhere");
        println!("  far B: preamble B's correlation when it is far (lowest, 5th percentile), and bursts where it is below {far_min}");
        println!("  shoulder: highest far correlation at a plateau whose near place is preamble B");
        println!("  miss: highest correlation, near or far, at a plateau with preamble B inside its search but at neither place");
        println!("  stray: highest correlation, near or far, at a plateau with preamble B outside its search");
        // The highest correlation at a plateau that is not a frame's, over the mode.
        let mut stray_all = 0.0f32;
        for l in &links(mode) {
            let truth = l.clean_position(&rx_modem);
            let is_b = |at: usize| is_at(&rx_modem, at, truth);
            for cnr_db in [None, Some(knee), Some(knee + 2.0), Some(20.0)] {
                // Each burst: where it was detected, and every plateau looked at on the way.
                let bursts = on_every_core(if cnr_db.is_none() { 1 } else { trials }, |t| {
                    let mut plateaus = Vec::new();
                    let got = ofdm.detect_traced(&l.received(cnr_db, seed0 + t as u64), 0, |p| plateaus.push(p));
                    (got, plateaus)
                });
                let (mut late, mut far_b, mut shoulder, mut miss, mut stray) = (vec![], vec![], vec![], vec![], vec![]);
                let (mut found, mut found_far, mut wrong, mut far_b_low) = (0, 0, 0, 0);
                for (got, plateaus) in &bursts {
                    match got {
                        Some(b) if is_b(*b) => found += 1,
                        Some(_) => wrong += 1,
                        None => {}
                    }
                    // Detection stops at the plateau it confirms, so that one is the last.
                    let by_far = plateaus.last().is_some_and(|p| Some(p.far.1) == *got);
                    found_far += usize::from(got.is_some_and(is_b) && by_far);
                    let mut low = false;
                    for p in plateaus {
                        if p.expected.abs_diff(truth) > PlateauPeaks::SPAN_NCP * ncp + ncp / 4 {
                            stray.push(p.best_corr());
                        } else {
                            late.push((p.expected as f32 - truth as f32) / ncp as f32);
                            if is_b(p.near.1) {
                                shoulder.push(p.far.0);
                            } else if is_b(p.far.1) {
                                far_b.push(p.far.0);
                                low |= p.far.0 < far_min;
                            } else {
                                miss.push(p.best_corr());
                            }
                        }
                    }
                    far_b_low += usize::from(low);
                }
                late.sort_by(f32::total_cmp);
                far_b.sort_by(f32::total_cmp);
                stray_all = stray_all.max(max(&stray));
                println!(
                    "  {:11} {:>7} | plateau {:+.2} {:+.2} {:+.2} | found {found:3} (far {found_far:3}) wrong {wrong} | far B {:.2} {:.2}, {far_b_low} below | shoulder {:.2} | miss {:.2} of {} | stray {:.2} of {}",
                    l.name,
                    cnr_db.map_or("clean".into(), |c| format!("{c} dB")),
                    at(&late, 0.0),
                    at(&late, 0.5),
                    at(&late, 1.0),
                    at(&far_b, 0.0),
                    at(&far_b, 0.05),
                    max(&shoulder),
                    max(&miss),
                    miss.len(),
                    max(&stray),
                    stray.len(),
                );
            }
        }
        // A carrier without a burst: every plateau is a stray one.
        for cnr in [-10.0, 5.0, 25.0] {
            let cfg = FmChannelConfig {
                cnr_db: Some(cnr),
                seed: seed0 + trials as u64,
                ..FmChannelConfig::by_name(mode).unwrap()
            };
            let rx = transmit(&cfg, &vec![0.0; (noise_s * cfg.fs) as usize]);
            let (mut stray, mut confirmed, mut from) = (Vec::new(), 0, 0);
            while let Some(b) = ofdm.detect_traced(&rx, from, |p| stray.push(p.best_corr())) {
                confirmed += 1;
                from = b + ofdm.p.symbol_len();
            }
            stray_all = stray_all.max(max(&stray));
            println!(
                "  carrier, no burst, {cnr} dB: stray {:.2} of {}, {confirmed} confirmed",
                max(&stray),
                stray.len()
            );
        }
        println!("  highest stray correlation in {mode}: {stray_all:.2}");
    }
}

/// A modem whose bursts start straight with preamble A, without a leader, so that a
/// test can place the preambles to the sample.
fn modem_without_leader(mode: &str, detector: Detector) -> Arc<Modem> {
    Arc::new(Modem::new(OfdmParams::by_name(mode).unwrap(), 0.0).with_detector(detector))
}

/// Preambles A and B of `b` on their own, as a burst cut off before its header would
/// leave them, then `gap` samples of silence. A detector finds a burst there, and its
/// header fails. (Preamble A alone is not enough: Schmidl-Cox confirms on preamble
/// B, and finds nothing there in narrow mode.)
fn cut_off_burst(m: &Modem, b: &Burst, gap: usize) -> Vec<f32> {
    assert_eq!(m.leader_symbols(), 0, "the burst must start with preamble A");
    let mut out = m.modulate(b)[..2 * m.ofdm.p.symbol_len()].to_vec();
    out.extend(vec![0.0; gap]);
    out
}

/// What a receiver finds must not depend on how its audio is chunked. A burst cut off
/// before its header sits right after the first burst, so the search after that
/// burst finds a header that fails just before the second burst, and searches on
/// from there.
///
/// The bursts must match exactly; where they end may move by a few samples, as a
/// detector may round differently depending on where each search starts.
#[test]
fn chunking_does_not_change_what_is_received() {
    for mode in ["narrow", "wide"] {
        for detector in Detector::ALL {
            let m = modem_without_leader(mode, detector);
            let ncp = m.ofdm.p.ncp;
            let (b1, b2) = (burst(2, 1, 11), burst(1, 1, 12));
            // At least two cyclic prefixes `after` the first burst: the search after it
            // starts where it ends, and Schmidl-Cox can miss a preamble A that starts
            // closer than that to the start of its search (see
            // `a_search_starting_just_before_a_frame_finds_it_in_a_tone` in vardefm-dsp).
            let l = m.ofdm.p.symbol_len();
            for (after, gap) in [(2 * ncp, ncp / 2), (3 * ncp, 1), (l, ncp - 1)] {
                let mut audio = vec![0.0; 10_000];
                audio.extend(m.modulate(&b1));
                audio.extend(vec![0.0; after]);
                audio.extend(cut_off_burst(&m, &b2, gap));
                audio.extend(m.modulate(&b2));
                audio.extend(vec![0.0; 48_000]);
                // The case this test is for must happen: searching from the end of the
                // first burst finds the cut-off burst before the second one, and its
                // header fails.
                let b1_end = 10_000 + m.modulate(&b1).len();
                let b2_at = b1_end + after + 2 * l + gap + l + ncp;
                let false_at = m.ofdm.detect(&audio, b1_end).at;
                let at = format!("{mode} {detector} after {after} gap {gap}");
                assert!(false_at.is_some_and(|b| b + l < b2_at), "{at}: found {false_at:?}, second burst at {b2_at}");
                let hdr = m.ofdm.demodulate(&audio, false_at.unwrap(), m.header_symbols());
                assert!(hdr.and_then(|f| m.decode_header(&f)).is_none(), "{at}: the cut-off burst's header decoded");
                let run = |chunk: usize| {
                    let mut rx = Receiver::new(m.clone());
                    let got: Vec<RxBurst> = audio.chunks(chunk).flat_map(|c| rx.push(c)).collect();
                    got.iter().map(|g| (g.header, g.payload.clone(), g.end_sample)).collect::<Vec<_>>()
                };
                let whole = run(audio.len());
                let headers: Vec<Header> = whole.iter().map(|g| g.0).collect();
                assert_eq!(headers, [b1.header, b2.header], "{mode} {detector} after {after} gap {gap}");
                for chunk in [37, 480, 4801] {
                    let got = run(chunk);
                    let at = format!("{mode} {detector} after {after} gap {gap}, chunks of {chunk}");
                    assert_eq!(got.len(), whole.len(), "{at}");
                    for (g, w) in got.iter().zip(&whole) {
                        assert_eq!((g.0, &g.1), (w.0, &w.1), "{at}");
                        assert!(g.2.abs_diff(w.2) <= (ncp / 8) as u64, "{at}: ends at {} and {}", g.2, w.2);
                    }
                }
            }
        }
    }
}
