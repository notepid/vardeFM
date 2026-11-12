//! Header error rate per channel estimator, side by side on the same received audio.
//!
//! Each trial sends a one-codeword burst at level 0's drive (random header fields,
//! as in `level_eval`) through the FM channel simulator and decodes its header from
//! the known burst position with each estimator:
//! - `smooth`: preamble B alone, smoothed [1 2 1]/4: the receiver's estimate before
//!   the header fit;
//! - `fitN`: a Legendre fit of order N across carriers to preamble B and the
//!   header's pilots (`Ofdm::demodulate_header` uses one order of it);
//! - `loo`: a reference that knows the header. Symbol `i` is equalized with the
//!   smoothed estimate trained on preamble B and every other header symbol, so no
//!   symbol trains on its own noise. It is not a limit: it takes the timing slope
//!   from neighbouring carriers, as the smoothed estimate does, and below about
//!   2 dB (narrow) and 4 dB (wide) the fit beats it.
//!
//! `cargo test --release -p vardefm-modem header_estimator_eval -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `TRIALS` per CNR (1600), `ORDERS` (fit
//! orders, 2,4), `CNRS` (dB list, default per mode).

use super::*;
use crate::detect_eval::{env_list, env_modes, env_or, modem, on_every_core, through_channel, true_position};
use crate::level_eval::trial_burst;
use vardefm_dsp::Estimator;

/// Header decoded with estimator `est` from the burst at `b`.
fn header_with(m: &Modem, rx: &[f32], b: usize, est: Estimator) -> Option<Header> {
    let f = m.ofdm.demodulate_with(rx, b, m.header_symbols(), &[], est)?;
    m.decode_header(&f)
}

/// Header decoded with the leave-one-out genie estimate (see the module doc).
fn header_loo(m: &Modem, rx: &[f32], b: usize, sent: &Header) -> Option<Header> {
    let hs = m.header_symbols();
    let carriers = m.header_carriers(sent);
    let mut slots = Vec::new();
    for i in 0..hs {
        let known: Vec<Option<&[Complex32]>> = (0..hs).map(|j| (j != i).then_some(carriers[j].as_slice())).collect();
        let f = m.ofdm.demodulate_with(rx, b, hs, &known, Estimator::Smooth)?;
        let scale = f.symbol_noise_scale[i];
        for (y, &nv) in f.symbols[i].iter().zip(&f.noise_var) {
            Modulation::Bpsk.demap_llr(*y, nv * scale, &mut slots);
        }
    }
    m.decode_header_slots(&slots)
}

#[test]
#[ignore]
fn header_estimator_eval() {
    let modes = env_modes("narrow,wide");
    let trials: u32 = env_or("TRIALS", 1600);
    assert!(trials > 0, "TRIALS must be at least 1");
    let orders: Vec<usize> = env_list("ORDERS", "2,4");
    let mut names = vec!["smooth".to_string(), "loo".to_string()];
    names.extend(orders.iter().map(|o| format!("fit{o}")));
    for mode in &modes {
        let m = modem(mode);
        let default_cnrs = if mode == "narrow" { "1.5,2,2.5,3,3.5,4,5,10,30" } else { "3.5,4,4.5,5,5.5,6,7,10,30" };
        let cnrs: Vec<f32> = env_list("CNRS", default_cnrs);
        let truth = true_position(&m, mode, 0, 1);
        println!("{mode}: header error rate, {trials} trials per CNR, level 0 drive");
        println!("{:>6} {}", "CNR", names.iter().map(|n| format!("{n:>8}")).collect::<String>());
        for &cnr in &cnrs {
            let fails: Vec<Vec<bool>> = on_every_core(trials, |t| {
                let sent = trial_burst(0, 1, t);
                let rx = through_channel(&m, mode, cnr, &sent, t);
                let fail = |h: Option<Header>| h != Some(sent.header);
                let mut v = vec![
                    fail(header_with(&m, &rx, truth, Estimator::Smooth)),
                    fail(header_loo(&m, &rx, truth, &sent.header)),
                ];
                v.extend(orders.iter().map(|&order| fail(header_with(&m, &rx, truth, Estimator::Fit { order }))));
                v
            });
            let rates: String = (0..names.len())
                .map(|i| format!("{:>8.4}", fails.iter().filter(|f| f[i]).count() as f32 / trials as f32))
                .collect();
            println!("{cnr:>6.2} {rates}");
        }
    }
}
