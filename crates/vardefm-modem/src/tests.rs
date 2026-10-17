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
