//! Link tests over a fake channel: bursts take their real airtime, collide when
//! both stations transmit, and can lose whole bursts or single codewords.

use super::*;
use vardefm_modem::OfdmParams;

const DT: f64 = 0.01;

struct Node {
    link: Link,
    /// (start, end, burst) while transmitting.
    tx: Option<(f64, f64, Burst)>,
    host_rx: Vec<u8>,
    events: Vec<LinkEvent>,
    /// Stop receiving and transmitting (station switched off).
    dead: bool,
}

struct Net {
    nodes: [Node; 2],
    modem: Arc<Modem>,
    now: f64,
    /// Bursts on their way: (deliver_at, to, burst, start, end).
    air: Vec<(f64, usize, Burst, f64, f64)>,
    rng: Prng,
    burst_loss: f64,
    cw_loss: f64,
    airtime: f64,
}

impl Net {
    fn new(seed: u32, burst_loss: f64, cw_loss: f64) -> Self {
        let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
        let node = |s| Node {
            link: Link::new(modem.clone(), LinkConfig::default(), s),
            tx: None,
            host_rx: Vec::new(),
            events: Vec::new(),
            dead: false,
        };
        let mut n = Self {
            nodes: [node(seed), node(seed + 1)],
            modem,
            now: 0.0,
            air: Vec::new(),
            rng: Prng::new(seed + 99),
            burst_loss,
            cw_loss,
            airtime: 0.0,
        };
        n.nodes[0].link.set_mycalls(vec!["LA1AAA".into()]);
        n.nodes[1].link.set_mycalls(vec!["LA2BBB-1".into()]);
        n.nodes[1].link.set_listen(true);
        n
    }

    fn chance(&mut self, p: f64) -> bool {
        (self.rng.next_u32() as f64 / u32::MAX as f64) < p
    }

    fn transmitting(&self, i: usize, a: f64, b: f64) -> bool {
        self.nodes[i].tx.as_ref().is_some_and(|(s, e, _)| *s < b && *e > a)
            || self.air.iter().any(|(_, to, _, s, e)| *to != i && *s < b && *e > a)
    }

    fn busy(&self, i: usize) -> bool {
        // Hear the other station from key-up until the squelch tail ends.
        let other = &self.nodes[1 - i];
        other.tx.as_ref().is_some_and(|(s, _, _)| self.now >= s + 0.1)
            || self.air.iter().any(|(_, to, _, _, e)| *to == i && self.now < e + 0.25)
    }

    fn step(&mut self) {
        self.now += DT;
        let now = self.now;
        // Finish transmissions and deliver.
        for i in 0..2 {
            if let Some((start, end, _)) = &self.nodes[i].tx {
                if now >= *end {
                    let (start, end) = (*start, *end);
                    let (_, _, burst) = self.nodes[i].tx.take().unwrap();
                    self.nodes[i].link.tx_done(now);
                    self.airtime += end - start;
                    self.air.push((end + 0.02, 1 - i, burst, start, end));
                }
            }
        }
        let mut arrived = Vec::new();
        self.air.retain(|a| {
            if a.0 <= now {
                arrived.push(a.clone());
                false
            } else {
                true
            }
        });
        for (_, to, burst, start, end) in arrived {
            if self.nodes[to].dead || self.transmitting(to, start, end) || self.chance(self.burst_loss) {
                if std::env::var("LINK_TRACE").is_ok() {
                    eprintln!("{now:7.2} lost burst to {}", ["A", "B"][to]);
                }
                continue;
            }
            let payload = burst
                .payload
                .iter()
                .map(|cw| if self.chance(self.cw_loss) { None } else { Some(cw.clone()) })
                .collect();
            let rx = RxBurst { header: burst.header, payload, snr_db: 30.0, end_sample: 0 };
            self.nodes[to].link.on_burst(now, &rx);
        }
        // Poll for new transmissions.
        for i in 0..2 {
            if self.nodes[i].dead || self.nodes[i].tx.is_some() {
                continue;
            }
            let busy = self.busy(i);
            if let Some(b) = self.nodes[i].link.poll(now, busy) {
                let dur = self.modem.burst_seconds(b.header.level, b.header.ncw as usize) as f64;
                if std::env::var("LINK_TRACE").is_ok() {
                    eprintln!("{now:7.2} {} TX {:?} ncw={} lvl={} id={} ack={}/{} bm={:b} {:.2}s", ["A", "B"][i], b.header.kind, b.header.ncw, b.header.level, b.header.burst_id, b.header.ack_burst, b.header.ack_offset, b.header.bitmap, dur);
                }
                self.nodes[i].tx = Some((now, now + dur, b));
            }
        }
        for n in &mut self.nodes {
            for e in n.link.take_events() {
                if let LinkEvent::Data(d) = &e {
                    n.host_rx.extend_from_slice(d);
                }
                n.events.push(e);
            }
        }
    }

    /// Run until `done` or `limit` seconds pass. Returns the time taken.
    fn run_until(&mut self, limit: f64, mut done: impl FnMut(&Net) -> bool) -> f64 {
        let start = self.now;
        while !done(self) {
            assert!(self.now - start < limit, "timed out after {limit} s at t={:.1}", self.now);
            self.step();
        }
        self.now - start
    }

    fn has(&self, i: usize, e: &LinkEvent) -> bool {
        self.nodes[i].events.contains(e)
    }
}

fn data(n: usize, seed: u32) -> Vec<u8> {
    // Every byte value, including CR and 0x00, in pseudo-random order.
    let mut rng = Prng::new(seed);
    (0..n).map(|i| if i < 256 { i as u8 } else { rng.next_u32() as u8 }).collect()
}

fn session(burst_loss: f64, cw_loss: f64, seed: u32) {
    let mut net = Net::new(seed, burst_loss, cw_loss);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    assert!(net.has(0, &LinkEvent::Connected { source: "LA1AAA".into(), destination: "LA2BBB-1".into() }));
    assert!(net.has(1, &LinkEvent::Connected { source: "LA1AAA".into(), destination: "LA2BBB-1".into() }));
    assert!(net.has(1, &LinkEvent::Pending));

    let (a, b) = (data(10_000, seed), data(10_000, seed + 7));
    net.nodes[1].link.send_data(&b[..300]); // banner first
    let t = net.run_until(600.0, |n| n.nodes[0].host_rx.len() >= 300);
    assert!(t < 60.0);
    net.nodes[1].link.send_data(&b[300..]);
    net.nodes[0].link.send_data(&a);
    let t = net.run_until(900.0, |n| n.nodes[0].host_rx.len() >= b.len() && n.nodes[1].host_rx.len() >= a.len());
    assert_eq!(net.nodes[0].host_rx, b);
    assert_eq!(net.nodes[1].host_rx, a);
    net.run_until(60.0, |n| n.has(0, &LinkEvent::Buffer(0)) && n.has(1, &LinkEvent::Buffer(0)));

    net.nodes[0].link.disconnect(net.now);
    net.run_until(60.0, |n| n.nodes[0].link.is_idle() && n.nodes[1].link.is_idle());
    for i in 0..2 {
        assert_eq!(net.nodes[i].events.iter().filter(|e| **e == LinkEvent::Disconnected).count(), 1);
    }
    println!(
        "loss {burst_loss}/{cw_loss}: 20 kB in {t:.0} s, airtime {:.0} s, retries {}/{}, resent {}/{} bytes",
        net.airtime,
        net.nodes[0].link.stats.retries,
        net.nodes[1].link.stats.retries,
        net.nodes[0].link.stats.bytes_resent,
        net.nodes[1].link.stats.bytes_resent
    );
}

#[test]
fn clean_session() {
    session(0.0, 0.0, 1);
}

#[test]
fn lossy_session() {
    session(0.15, 0.15, 2);
    session(0.3, 0.1, 3);
}

#[test]
fn connect_to_absent_station_gives_up() {
    let mut net = Net::new(4, 0.0, 0.0);
    net.nodes[1].dead = true;
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    let t = net.run_until(60.0, |n| n.nodes[0].link.is_idle());
    assert!(t < 25.0, "took {t} s");
    assert_eq!(net.nodes[0].events, vec![LinkEvent::Disconnected]);
}

#[test]
fn not_listening_or_wrong_call_is_ignored() {
    let mut net = Net::new(5, 0.0, 0.0);
    net.nodes[1].link.set_listen(false);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_idle());
    assert!(net.nodes[1].events.is_empty());

    let mut net = Net::new(6, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA9ZZZ").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_idle());
    assert!(net.nodes[1].events.is_empty());
}

#[test]
fn abort_tells_the_peer() {
    let mut net = Net::new(7, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    net.nodes[0].link.abort(net.now);
    assert!(net.nodes[0].link.is_idle());
    let t = net.run_until(10.0, |n| n.nodes[1].link.is_idle());
    assert!(t < 5.0);
}

#[test]
fn dead_peer_is_detected() {
    let mut net = Net::new(8, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    net.nodes[1].dead = true;
    net.nodes[0].link.send_data(b"hello?");
    let t = net.run_until(120.0, |n| n.nodes[0].link.is_idle());
    assert!(t < 60.0, "took {t} s");
    assert!(net.has(0, &LinkEvent::Disconnected));
}

#[test]
fn idle_link_is_kept_alive_then_dropped_when_peer_dies() {
    let mut net = Net::new(9, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    // Quiet for two minutes: keepalives keep both ends connected.
    net.run_until(130.0, |n| n.now > 120.0);
    assert!(net.nodes[0].link.is_connected() && net.nodes[1].link.is_connected());
    // Then the caller disappears without a word; the called side notices.
    net.nodes[0].dead = true;
    net.run_until(200.0, |n| n.nodes[1].link.is_idle());
}
