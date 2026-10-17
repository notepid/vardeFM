//! RadioEngine with fake sound cards and PTT lines: two stations, a full
//! session, and checks that audio only ever leaves while PTT is keyed.

use super::*;
use std::collections::VecDeque;
use std::sync::Mutex;

const CHUNK: usize = 480; // 10 ms
const KEYUP_STEPS: usize = 5; // radio needs 50 ms after PTT before it transmits

#[derive(Default)]
struct Card {
    queue: VecDeque<f32>,
    consumed: u64,
    queued: u64,
    ptt: bool,
    /// Steps since PTT went on.
    keyed_for: usize,
    /// Non-silent samples played while PTT was off.
    leaked: usize,
    /// Steps between the last audio sample and PTT release, per transmission.
    tails: Vec<usize>,
    since_audio: usize,
    ptt_events: usize,
    /// Set to stop the card from consuming (stuck driver).
    stuck: bool,
}

type Shared = Arc<Mutex<Card>>;

struct FakeOut(Shared);

impl AudioSink for FakeOut {
    fn play(&mut self, samples: &[f32]) -> u64 {
        let mut c = self.0.lock().unwrap();
        c.queue.extend(samples);
        c.queued += samples.len() as u64;
        c.queued
    }
    fn consumed(&self) -> u64 {
        self.0.lock().unwrap().consumed
    }
    fn flush(&mut self) {
        let mut c = self.0.lock().unwrap();
        c.consumed += c.queue.len() as u64;
        c.queue.clear();
    }
}

struct FakePtt(Shared);

impl Ptt for FakePtt {
    fn set(&mut self, on: bool) -> io::Result<()> {
        let mut c = self.0.lock().unwrap();
        if c.ptt && !on {
            let t = c.since_audio;
            c.tails.push(t);
        }
        if on != c.ptt {
            c.ptt_events += 1;
            c.keyed_for = 0;
        }
        c.ptt = on;
        Ok(())
    }
    fn describe(&self) -> String {
        "fake".into()
    }
}

impl Card {
    /// One 10 ms tick of the sound card. Returns what the radio puts on the air.
    fn tick(&mut self) -> Vec<f32> {
        let mut out = vec![0.0; CHUNK];
        if !self.stuck {
            for o in out.iter_mut() {
                let Some(x) = self.queue.pop_front() else { break };
                self.consumed += 1;
                *o = x;
            }
        }
        let audible = out.iter().any(|x| *x != 0.0);
        if audible {
            self.since_audio = 0;
            if !self.ptt {
                self.leaked += out.iter().filter(|x| **x != 0.0).count();
            }
        } else {
            self.since_audio += 1;
        }
        if self.ptt {
            self.keyed_for += 1;
        }
        // The transmitter only radiates once keyed long enough.
        if self.ptt && self.keyed_for > KEYUP_STEPS {
            out
        } else {
            vec![0.0; CHUNK]
        }
    }
}

fn engine(card: &Shared, seed: u32) -> RadioEngine<FakeOut> {
    let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.15));
    RadioEngine::new(modem, Box::new(FakePtt(card.clone())), FakeOut(card.clone()), 1.0, 0.05, None, seed)
}

#[derive(Default)]
struct HostSide {
    lines: Vec<String>,
    data: Vec<u8>,
}

impl HostSide {
    fn take(&mut self, out: Vec<HostOut>) {
        for o in out {
            match o {
                HostOut::Line(l) => self.lines.push(l),
                HostOut::Data(d) => self.data.extend(d),
            }
        }
    }
    fn has(&self, l: &str) -> bool {
        self.lines.iter().any(|x| x == l)
    }
}

#[test]
fn session_over_fake_sound_cards() {
    let cards: [Shared; 2] = [Arc::default(), Arc::default()];
    let mut eng = [engine(&cards[0], 1), engine(&cards[1], 2)];
    let mut host = [HostSide::default(), HostSide::default()];
    let line = |s: &str| HostIn::Line(s.into());
    let mut inbox: [Vec<HostIn>; 2] = [
        vec![line("MYCALL LA1AAA"), line("CONNECT LA1AAA LA2BBB")],
        vec![line("MYCALL LA2BBB"), line("LISTEN ON")],
    ];
    let to_a: Vec<u8> = (0..2000u32).map(|i| (i * 7) as u8).collect();
    let to_b: Vec<u8> = (0..1500u32).map(|i| (i * 13 + 5) as u8).collect();
    let mut air = [vec![0.0f32; CHUNK], vec![0.0f32; CHUNK]];
    let (mut sent_b, mut sent_a, mut disc) = (false, false, false);

    for step in 0..60_000 {
        // Each station hears what the other radiated in the previous tick.
        for i in 0..2 {
            let heard = air[1 - i].clone();
            let out = eng[i].step(Some(&heard), std::mem::take(&mut inbox[i]));
            host[i].take(out);
        }
        for i in 0..2 {
            air[i] = cards[i].lock().unwrap().tick();
        }
        // Host behaviour: B sends a banner once connected, A answers, then A disconnects.
        if !sent_b && host[1].lines.iter().any(|l| l.starts_with("CONNECTED")) {
            inbox[1].push(HostIn::Data(to_a.clone()));
            sent_b = true;
        }
        if !sent_a && host[0].data.len() >= to_a.len() {
            inbox[0].push(HostIn::Data(to_b.clone()));
            sent_a = true;
        }
        if !disc && host[1].data.len() >= to_b.len() && host[0].lines.last().is_some_and(|l| l == "BUFFER 0") {
            inbox[0].push(line("DISCONNECT"));
            disc = true;
        }
        if host[0].has("DISCONNECTED") && host[1].has("DISCONNECTED") {
            println!("session done after {:.1} s", step as f64 * 0.01);
            break;
        }
    }
    assert_eq!(host[0].data, to_a, "A's received data");
    assert_eq!(host[1].data, to_b, "B's received data");
    assert!(host[0].has("DISCONNECTED") && host[1].has("DISCONNECTED"), "A: {:?}", host[0].lines);
    for (i, c) in cards.iter().enumerate() {
        let c = c.lock().unwrap();
        assert_eq!(c.leaked, 0, "station {i}: audio played with PTT off");
        assert!(!c.ptt, "station {i}: left keyed");
        assert!(c.ptt_events >= 4, "station {i}: only {} PTT changes", c.ptt_events);
        // Tail: released 50 ms (5 ticks) after the last sample, give or take a tick; never early.
        for &t in &c.tails {
            assert!((5..=7).contains(&t), "station {i}: PTT released {t} ticks after the last sample");
        }
    }
    for i in 0..2 {
        assert!(!eng[i].log.iter().any(|l| l.contains("watchdog")), "{:?}", eng[i].log);
        let ptt_lines = host[i].lines.iter().filter(|l| l.starts_with("PTT")).count();
        assert_eq!(ptt_lines, cards[i].lock().unwrap().ptt_events, "PTT lines to host match the real PTT");
    }
}

#[test]
fn watchdog_unkeys_a_stuck_sound_card() {
    let card: Shared = Arc::default();
    card.lock().unwrap().stuck = true;
    let mut eng = engine(&card, 3);
    let mut out = eng.step(None, vec![HostIn::Line("MYCALL LA1AAA".into()), HostIn::Line("CONNECT LA1AAA LA9ZZZ".into())]);
    let silence = vec![0.0f32; CHUNK];
    let mut keyed_at = None;
    for step in 0..2000 {
        out.extend(eng.step(Some(&silence), Vec::new()));
        card.lock().unwrap().tick();
        let ptt = card.lock().unwrap().ptt;
        if ptt && keyed_at.is_none() {
            keyed_at = Some(step);
        }
        if keyed_at.is_some() && !ptt {
            let held = (step - keyed_at.unwrap()) as f64 * 0.01;
            assert!(held < 6.0, "held PTT for {held} s");
            assert!(eng.log.iter().any(|l| l.contains("watchdog")));
            return;
        }
    }
    panic!("never unkeyed");
}
