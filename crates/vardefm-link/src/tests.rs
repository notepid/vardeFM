//! Link tests over a fake channel: bursts take their real airtime, collide when
//! two stations a receiver hears transmit at once (or the receiver itself does),
//! and can lose whole bursts or single codewords. Stations may hear only some of
//! the others, and may run a digipeater.

use super::*;
use vardefm_modem::OfdmParams;

const DT: f64 = 0.01;

/// A station on the fake channel. `node` is the shipped link and digipeater.
struct Peer {
    name: String,
    node: Node,
    /// (start, end, burst, from the digipeater) while transmitting.
    tx: Option<(f64, f64, Burst, bool)>,
    host_rx: Vec<u8>,
    events: Vec<LinkEvent>,
    /// Stop receiving and transmitting (station switched off).
    dead: bool,
}

struct Net {
    nodes: Vec<Peer>,
    /// `hears[to][from]`.
    hears: Vec<Vec<bool>>,
    modem: Arc<Modem>,
    now: f64,
    /// Bursts on their way: (deliver_at, to, burst, start, end, from).
    air: Vec<(f64, usize, Burst, f64, f64, usize)>,
    /// Every transmission still relevant for collisions and busy: (from, start, end).
    history: Vec<(usize, f64, f64)>,
    rng: Prng,
    burst_loss: f64,
    cw_loss: f64,
    airtime: f64,
    /// Every burst a station's own link sent (repeats left out): (from, start, end, kind).
    sent: Vec<(usize, f64, f64, Kind)>,
    /// (from, to): the next burst `to` would receive from `from` is lost. It is still on
    /// the air, so `to` hears the channel busy, but nothing decodes.
    lose_next: Option<(usize, usize)>,
    /// The header of each burst in `sent`: (from, start, header).
    sent_headers: Vec<(usize, f64, Header)>,
}

const A: usize = 0;

impl Net {
    /// Two stations that hear each other: A (LA1AAA) and B (LA2BBB-1, listening).
    fn new(seed: u32, burst_loss: f64, cw_loss: f64) -> Self {
        Self::build(seed, burst_loss, cw_loss, &[], LinkConfig::default())
    }

    /// A, the digipeaters `digis` in order, then B. Each station hears only its
    /// neighbours along the chain.
    fn chain(seed: u32, cw_loss: f64, digis: &[&str], cfg: LinkConfig) -> Self {
        Self::build(seed, 0.0, cw_loss, digis, cfg)
    }

    fn build(seed: u32, burst_loss: f64, cw_loss: f64, digis: &[&str], cfg: LinkConfig) -> Self {
        let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
        let count = digis.len() + 2;
        let mut nodes: Vec<Peer> = (0..count)
            .map(|i| Peer {
                name: String::new(),
                node: Node::new(Link::new(modem.clone(), cfg.clone(), seed + i as u32)),
                tx: None,
                host_rx: Vec::new(),
                events: Vec::new(),
                dead: false,
            })
            .collect();
        nodes[A].name = "A".into();
        nodes[A].node.link.set_mycalls(vec!["LA1AAA".into()]);
        for (i, d) in digis.iter().enumerate() {
            nodes[i + 1].name = format!("D{}", i + 1);
            nodes[i + 1].node.set_digi(&[d.to_string()]);
        }
        let b = count - 1;
        nodes[b].name = "B".into();
        nodes[b].node.link.set_mycalls(vec!["LA2BBB-1".into()]);
        nodes[b].node.link.set_listen(true);
        let hears = (0..count).map(|to| (0..count).map(|from| to.abs_diff(from) == 1).collect()).collect();
        Self {
            nodes,
            hears,
            modem,
            now: 0.0,
            air: Vec::new(),
            history: Vec::new(),
            rng: Prng::new(seed + 99),
            burst_loss,
            cw_loss,
            airtime: 0.0,
            sent: Vec::new(),
            lose_next: None,
            sent_headers: Vec::new(),
        }
    }

    fn b(&self) -> usize {
        self.nodes.len() - 1
    }

    fn chance(&mut self, p: f64) -> bool {
        (self.rng.next_u32() as f64 / u32::MAX as f64) < p
    }

    /// True if `to` cannot receive a burst from `from` over `[a, b]`: it transmitted
    /// itself, or heard another station at the same time.
    fn collided(&self, to: usize, from: usize, a: f64, b: f64) -> bool {
        self.history.iter().any(|&(g, s, e)| g != from && (g == to || self.hears[to][g]) && s < b && e > a)
    }

    fn busy(&self, i: usize) -> bool {
        // Hear a station from key-up until the squelch tail ends.
        self.history.iter().any(|&(g, s, e)| self.hears[i][g] && self.now >= s + 0.1 && self.now < e + 0.25)
    }

    fn trace(&self) -> bool {
        std::env::var("LINK_TRACE").is_ok()
    }

    fn step(&mut self) {
        self.now += DT;
        let now = self.now;
        // Finish transmissions.
        for i in 0..self.nodes.len() {
            if self.nodes[i].tx.as_ref().is_some_and(|t| now >= t.1) {
                let (start, end, ..) = self.nodes[i].tx.take().unwrap();
                self.nodes[i].node.tx_done(now);
                self.airtime += end - start;
            }
        }
        // Deliver.
        let mut arrived = Vec::new();
        self.air.retain(|a| {
            if a.0 <= now {
                arrived.push(a.clone());
                false
            } else {
                true
            }
        });
        for (_, to, burst, start, end, from) in arrived {
            let lost = self.lose_next == Some((from, to));
            if lost {
                self.lose_next = None;
            }
            if lost || self.nodes[to].dead || self.collided(to, from, start, end) || self.chance(self.burst_loss) {
                if self.trace() {
                    eprintln!("{now:7.2} lost burst {}->{}", self.nodes[from].name, self.nodes[to].name);
                }
                continue;
            }
            let payload = burst
                .payload
                .iter()
                .map(|cw| if self.chance(self.cw_loss) { None } else { Some(cw.clone()) })
                .collect();
            let rx = RxBurst { header: burst.header, payload, snr_db: 30.0, end_sample: 0, level: Default::default() };
            self.nodes[to].node.on_burst(now, &rx);
        }
        self.history.retain(|&(_, _, e)| e + 30.0 > now);
        // Poll for new transmissions.
        for i in 0..self.nodes.len() {
            if self.nodes[i].dead || self.nodes[i].tx.is_some() {
                continue;
            }
            let busy = self.busy(i);
            let Some((b, digi)) = self.nodes[i].node.poll(now, busy) else { continue };
            let dur = self.modem.burst_seconds(b.header.level, b.header.ncw as usize) as f64;
            if self.trace() {
                eprintln!(
                    "{now:7.2} {} TX {:?} ncw={} lvl={} id={} ack={}/{} bm={:b} up={} hop={} {:.2}s",
                    self.nodes[i].name,
                    b.header.kind,
                    b.header.ncw,
                    b.header.level,
                    b.header.burst_id,
                    b.header.ack_burst,
                    b.header.ack_offset,
                    b.header.bitmap,
                    b.header.up(),
                    b.header.hop(),
                    dur
                );
            }
            let end = now + dur;
            for to in 0..self.nodes.len() {
                if self.hears[to][i] {
                    self.air.push((end + 0.02, to, b.clone(), now, end, i));
                }
            }
            self.history.push((i, now, end));
            if !digi {
                self.sent.push((i, now, end, b.header.kind));
                self.sent_headers.push((i, now, b.header));
            }
            self.nodes[i].tx = Some((now, end, b, digi));
        }
        for n in &mut self.nodes {
            for e in n.node.link.take_events() {
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

    /// Sets who hears whom: station `to` hears `from` if `f(to, from)`.
    fn set_hears(&mut self, f: impl Fn(usize, usize) -> bool) {
        for (to, row) in self.hears.iter_mut().enumerate() {
            for (from, h) in row.iter_mut().enumerate() {
                *h = f(to, from);
            }
        }
    }

    fn both_connected(&self) -> bool {
        self.nodes[A].node.link.is_connected() && self.nodes[self.b()].node.link.is_connected()
    }

    fn both_idle(&self) -> bool {
        self.nodes[A].node.link.is_idle() && self.nodes[self.b()].node.link.is_idle()
    }
}

#[test]
fn write_is_reported_even_when_the_buffer_length_is_unchanged() {
    // A write that lands in the same tick as the ack draining an equal amount leaves
    // the buffer length as last reported. The host still needs a report newer than
    // its write, so one must follow.
    let mut net = Net::new(9, 0.0, 0.0);
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, Net::both_connected);
    let d = data(10, 9);
    net.nodes[A].node.link.send_data(&d);
    net.run_until(60.0, |n| n.nodes[1].host_rx.len() >= d.len());
    assert_eq!(net.nodes[A].events.last(), Some(&LinkEvent::Buffer(10)));
    // Stop one tick before B's ack reaches A, then write as much again.
    net.run_until(60.0, |n| n.air.iter().any(|a| a.1 == A && a.0 <= n.now + DT));
    let before = net.nodes[A].events.len();
    net.nodes[A].node.link.send_data(&d);
    net.step();
    let reports: Vec<_> = net.nodes[A].events[before..].iter().filter(|e| matches!(e, LinkEvent::Buffer(_))).collect();
    // Buffer(20) would mean the ack missed this tick (the setup drifted), not a bug.
    assert_eq!(reports, [&LinkEvent::Buffer(10)], "none = write went unreported; Buffer(20) = ack not in this tick");
}

#[test]
fn write_that_changes_the_buffer_length_is_reported_once() {
    let mut net = Net::new(9, 0.0, 0.0);
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, Net::both_connected);
    let before = net.nodes[A].events.len();
    net.nodes[A].node.link.send_data(&data(10, 9));
    // No ack can come back within two ticks, so the length stays 10 throughout.
    net.step();
    net.step();
    let reports: Vec<_> = net.nodes[A].events[before..].iter().filter(|e| matches!(e, LinkEvent::Buffer(_))).collect();
    assert_eq!(reports, [&LinkEvent::Buffer(10)]);
}

fn data(n: usize, seed: u32) -> Vec<u8> {
    // Every byte value, including CR and 0x00, in pseudo-random order.
    let mut rng = Prng::new(seed);
    (0..n).map(|i| if i < 256 { i as u8 } else { rng.next_u32() as u8 }).collect()
}

fn session(burst_loss: f64, cw_loss: f64, seed: u32) {
    let mut net = Net::new(seed, burst_loss, cw_loss);
    net.nodes[0].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].node.link.is_connected() && n.nodes[1].node.link.is_connected());
    assert!(net.has(0, &LinkEvent::Connected { source: "LA1AAA".into(), destination: "LA2BBB-1".into() }));
    assert!(net.has(1, &LinkEvent::Connected { source: "LA1AAA".into(), destination: "LA2BBB-1".into() }));
    assert!(net.has(1, &LinkEvent::Pending));

    let (a, b) = (data(10_000, seed), data(10_000, seed + 7));
    net.nodes[1].node.link.send_data(&b[..300]); // banner first
    let t = net.run_until(600.0, |n| n.nodes[0].host_rx.len() >= 300);
    assert!(t < 60.0);
    net.nodes[1].node.link.send_data(&b[300..]);
    net.nodes[0].node.link.send_data(&a);
    let t = net.run_until(900.0, |n| n.nodes[0].host_rx.len() >= b.len() && n.nodes[1].host_rx.len() >= a.len());
    assert_eq!(net.nodes[0].host_rx, b);
    assert_eq!(net.nodes[1].host_rx, a);
    net.run_until(60.0, |n| n.has(0, &LinkEvent::Buffer(0)) && n.has(1, &LinkEvent::Buffer(0)));

    net.nodes[0].node.link.disconnect(net.now);
    net.run_until(60.0, |n| n.nodes[0].node.link.is_idle() && n.nodes[1].node.link.is_idle());
    for node in &net.nodes[..2] {
        assert_eq!(node.events.iter().filter(|e| **e == LinkEvent::Disconnected).count(), 1);
    }
    println!(
        "loss {burst_loss}/{cw_loss}: 20 kB in {t:.0} s, airtime {:.0} s, retries {}/{}, resent {}/{} bytes",
        net.airtime,
        net.nodes[0].node.link.stats.retries,
        net.nodes[1].node.link.stats.retries,
        net.nodes[0].node.link.stats.bytes_resent,
        net.nodes[1].node.link.stats.bytes_resent
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
    net.nodes[0].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    let t = net.run_until(60.0, |n| n.nodes[0].node.link.is_idle());
    assert!(t < 25.0, "took {t} s");
    assert_eq!(net.nodes[0].events, vec![LinkEvent::Disconnected]);
}

#[test]
fn not_listening_or_wrong_call_is_ignored() {
    let mut net = Net::new(5, 0.0, 0.0);
    net.nodes[1].node.link.set_listen(false);
    net.nodes[0].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].node.link.is_idle());
    assert!(net.nodes[1].events.is_empty());

    let mut net = Net::new(6, 0.0, 0.0);
    net.nodes[0].node.link.connect(0.0, "LA1AAA", "LA9ZZZ", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].node.link.is_idle());
    assert!(net.nodes[1].events.is_empty());
}

#[test]
fn abort_tells_the_peer() {
    let mut net = Net::new(7, 0.0, 0.0);
    net.nodes[0].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].node.link.is_connected() && n.nodes[1].node.link.is_connected());
    net.nodes[0].node.link.abort(net.now);
    assert!(net.nodes[0].node.link.is_idle());
    let t = net.run_until(10.0, |n| n.nodes[1].node.link.is_idle());
    assert!(t < 5.0);
}

#[test]
fn dead_peer_is_detected() {
    let mut net = Net::new(8, 0.0, 0.0);
    net.nodes[0].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].node.link.is_connected() && n.nodes[1].node.link.is_connected());
    net.nodes[1].dead = true;
    net.nodes[0].node.link.send_data(b"hello?");
    let t = net.run_until(120.0, |n| n.nodes[0].node.link.is_idle());
    assert!(t < 60.0, "took {t} s");
    assert!(net.has(0, &LinkEvent::Disconnected));
}

#[test]
fn idle_link_is_kept_alive_then_dropped_when_peer_dies() {
    let mut net = Net::new(9, 0.0, 0.0);
    net.nodes[0].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].node.link.is_connected() && n.nodes[1].node.link.is_connected());
    // Quiet for two minutes: keepalives keep both ends connected.
    net.run_until(130.0, |n| n.now > 120.0);
    assert!(net.nodes[0].node.link.is_connected() && net.nodes[1].node.link.is_connected());
    // Then the caller disappears without a word; the called side notices.
    net.nodes[0].dead = true;
    net.run_until(200.0, |n| n.nodes[1].node.link.is_idle());
}

fn via(digis: &[&str]) -> Vec<String> {
    digis.iter().map(|s| s.to_string()).collect()
}

/// Connect A to B through `net`'s digipeaters, move 10 kB each way, disconnect.
fn routed_session(mut net: Net, digis: &[&str], seed: u32) -> f64 {
    let b = net.b();
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    let connected = LinkEvent::Connected { source: "LA1AAA".into(), destination: "LA2BBB-1".into() };
    assert!(net.has(A, &connected) && net.has(b, &connected));

    let (da, db) = (data(10_000, seed), data(10_000, seed + 7));
    net.nodes[b].node.link.send_data(&db);
    net.nodes[A].node.link.send_data(&da);
    let t = net.run_until(3000.0, |n| n.nodes[A].host_rx.len() >= db.len() && n.nodes[b].host_rx.len() >= da.len());
    assert_eq!(net.nodes[A].host_rx, db);
    assert_eq!(net.nodes[b].host_rx, da);

    net.run_until(300.0, |n| n.has(A, &LinkEvent::Buffer(0)) && n.has(b, &LinkEvent::Buffer(0)));
    net.nodes[A].node.link.disconnect(net.now);
    net.run_until(300.0, |n| n.both_idle());
    for i in [A, b] {
        assert_eq!(net.nodes[i].events.iter().filter(|e| **e == LinkEvent::Disconnected).count(), 1);
    }
    let repeats: Vec<u64> = net.nodes[1..b].iter().map(|n| n.node.repeats()).collect();
    println!(
        "via {digis:?}: 20 kB in {t:.0} s, airtime {:.0} s, retries {}/{}, repeats {repeats:?}",
        net.airtime, net.nodes[A].node.link.stats.retries, net.nodes[b].node.link.stats.retries
    );
    assert!(repeats.iter().all(|&r| r > 0), "every digipeater relayed");
    t
}

#[test]
fn session_via_one_and_two_digipeaters() {
    let mut times = Vec::new();
    for (i, digis) in [&["LA3DDD"][..], &["LA3DDD", "LA4EEE-2"]].iter().enumerate() {
        let seed = 20 + i as u32;
        times.push(routed_session(Net::chain(seed, 0.0, digis, LinkConfig::default()), digis, seed));
    }
    // Each hop costs airtime, so longer chains are slower.
    assert!(times[0] < times[1], "{times:?}");
}

#[test]
fn lossy_routed_session_resends_voided_codewords() {
    let digis = ["LA3DDD", "LA4EEE-2"];
    routed_session(Net::chain(30, 0.1, &digis, LinkConfig::default()), &digis, 30);
}

#[test]
fn endpoints_that_hear_each_other_ignore_the_direct_copy() {
    // Everyone hears everyone. A routed session must still deliver each byte once,
    // although A and B also decode each other's bursts directly.
    let digis = ["LA3DDD"];
    let mut net = Net::chain(31, 0.0, &digis, LinkConfig::default());
    net.set_hears(|to, from| to != from);
    routed_session(net, &digis, 31);
}

/// A Data burst asking for an answer, with UP and the raw 2-bit route field `route`
/// (0 = direct, else 1 + digipeater repeats).
fn probe(session: u8, up: bool, route: u8) -> RxBurst {
    let flags = FLAG_POLL | if up { FLAG_UP } else { 0 } | route << 2;
    let header = Header { kind: Kind::Data, session, flags, ..Header::default() };
    RxBurst { header, payload: Vec::new(), snr_db: 30.0, end_sample: 0, level: Default::default() }
}

/// Every (UP, route field) a burst can carry, except `own`.
fn all_but(own: (bool, u8)) -> Vec<(bool, u8)> {
    [false, true].into_iter().flat_map(|up| (0..4).map(move |r| (up, r))).filter(|&c| c != own).collect()
}

/// Once `net` has gone quiet, feeds each end every other (UP, route field) and then
/// its peer's own: only the peer's own is answered.
fn only_own_answered(net: &mut Net, own_a: (bool, u8), own_b: (bool, u8)) {
    let settled = net.now + 20.0;
    net.run_until(30.0, |n| n.now >= settled);
    let now = net.now;
    for (i, own) in [(A, own_a), (net.b(), own_b)] {
        let link = &mut net.nodes[i].node.link;
        let State::Connected(s) = &link.state else { panic!("not connected") };
        let id = s.id;
        assert!(link.poll(now + 1.0, false).is_none(), "node {i} not quiet");
        for (up, route) in all_but(own) {
            link.on_burst(now, &probe(id, up, route));
            assert!(link.poll(now + 1.0, false).is_none(), "node {i} answered up {up} route {route}");
        }
        link.on_burst(now, &probe(id, own.0, own.1));
        assert!(link.poll(now + 1.0, false).is_some(), "node {i} answers the peer's own burst");
    }
}

#[test]
fn direct_session_ignores_bursts_a_digipeater_repeated() {
    // A direct session's own bursts are unrouted without UP, as every version sends
    // them. Anything else with its id is not the peer's: most of all a routed caller's
    // unrepeated burst (route 1, UP 0) heard directly, and copies a digipeater repeated.
    let mut net = Net::new(33, 0.0, 0.0);
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.both_connected());
    only_own_answered(&mut net, (false, 0), (false, 0));

    // A caller still connecting takes only the called station's ConnectAck as it arrives.
    connecting_takes_only_its_ack(&[], (false, 0));
}

/// A caller connecting via `digis` ignores Data with its session id, even with the
/// called station's flags `own`, and a ConnectAck with any other flags. It connects on
/// the ConnectAck with `own`.
fn connecting_takes_only_its_ack(digis: &[&str], own: (bool, u8)) {
    let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
    let mut link = Link::new(modem, LinkConfig::default(), 3);
    link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
    let State::Connecting { session, .. } = &link.state else { panic!("not connecting") };
    let id = *session;
    let ack = |(up, route)| connect_burst(Kind::ConnectAck, id, ("LA1AAA", "LA2BBB-1"), digis, up, route);
    for (up, route) in all_but(own).into_iter().chain([own]) {
        link.on_burst(0.5, &probe(id, up, route));
        assert!(!link.is_connected(), "via {digis:?}: connected by Data up {up} route {route}");
    }
    for (up, route) in all_but(own) {
        link.on_burst(0.5, &ack((up, route)));
        assert!(!link.is_connected(), "via {digis:?}: connected by ConnectAck up {up} route {route}");
    }
    link.on_burst(0.5, &ack(own));
    assert!(link.is_connected(), "via {digis:?}");
}

#[test]
fn routed_session_ignores_direct_bursts() {
    // Through one digipeater each end takes only the other's burst after one repeat
    // (route field 2), with UP from the called station. A direct burst with the same id
    // (route 0) is another session's.
    let digis = ["LA3DDD"];
    let mut net = Net::chain(38, 0.0, &digis, LinkConfig::default());
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&digis)).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    only_own_answered(&mut net, (true, 2), (false, 2));

    // A routed caller still connecting is not answered by a direct ConnectAck either.
    connecting_takes_only_its_ack(&digis, (true, 2));
}

#[test]
fn bursts_carry_the_route_field() {
    // Direct: route field 0 and no UP from either end, exactly as released versions.
    // Routed: route field 1 (not yet repeated) from both ends, UP from the called station.
    for (digis, a_flags, b_flags) in [(&[][..], 0, 0), (&["LA3DDD"][..], FLAG_ROUTED, FLAG_ROUTED | FLAG_UP)] {
        let mut net = Net::chain(40, 0.0, digis, LinkConfig::default());
        let b = net.b();
        let mut sent: Vec<(usize, Kind, u8)> = Vec::new();
        // Steps until `done`, noting each end's bursts as they start.
        let run = |net: &mut Net, sent: &mut Vec<(usize, Kind, u8)>, done: &dyn Fn(&Net) -> bool| {
            while !done(net) {
                assert!(net.now < 600.0, "via {digis:?}: timed out");
                net.step();
                for i in [A, b] {
                    if let Some((start, _, burst, false)) = &net.nodes[i].tx {
                        if (*start - net.now).abs() < DT / 2.0 {
                            sent.push((i, burst.header.kind, burst.header.flags & !FLAG_POLL));
                        }
                    }
                }
            }
        };
        net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
        run(&mut net, &mut sent, &|n| n.both_connected());
        net.nodes[A].node.link.send_data(b"hello");
        net.nodes[b].node.link.send_data(b"there");
        run(&mut net, &mut sent, &|n| n.nodes[A].host_rx.len() >= 5 && n.nodes[n.b()].host_rx.len() >= 5);
        net.nodes[A].node.link.disconnect(net.now);
        run(&mut net, &mut sent, &|n| n.both_idle());
        for kind in [Kind::ConnectReq, Kind::ConnectAck, Kind::Data, Kind::Disc, Kind::DiscAck] {
            assert!(sent.iter().any(|&(_, k, _)| k == kind), "via {digis:?}: no {kind:?} sent");
        }
        for (i, kind, flags) in sent {
            let want = if i == A { a_flags } else { b_flags };
            assert_eq!(flags, want, "via {digis:?}: node {i} {kind:?}");
        }
    }
}

#[test]
fn digipeater_switched_off_drops_the_link_on_both_ends() {
    let mut net = Net::chain(32, 0.0, &["LA3DDD"], LinkConfig::default());
    let b = net.b();
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&["LA3DDD"])).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    net.nodes[1].dead = true;
    net.nodes[A].node.link.send_data(b"hello?");
    net.nodes[b].node.link.send_data(b"anyone?");
    net.run_until(600.0, |n| n.both_idle());
    assert!(net.has(A, &LinkEvent::Disconnected) && net.has(b, &LinkEvent::Disconnected));
}

#[test]
fn connect_via_absent_digipeater_gives_up() {
    // The station between A and B is not a digipeater for LA9ZZZ.
    let mut net = Net::chain(33, 0.0, &["LA3DDD"], LinkConfig::default());
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&["LA9ZZZ"])).unwrap();
    let t = net.run_until(120.0, |n| n.nodes[A].node.link.is_idle());
    assert!(t < 60.0, "took {t} s");
    assert_eq!(net.nodes[A].events, vec![LinkEvent::Disconnected]);
    assert!(net.nodes[net.b()].events.is_empty());
}

#[test]
fn more_than_two_digipeaters_is_refused() {
    let mut net = Net::new(34, 0.0, 0.0);
    for path in [&["LA3A", "LA3B", "LA3C"][..], &["LA3A", "LA3B", "LA3C", "LA3D"]] {
        assert!(net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(path)).is_err(), "via {path:?}");
        assert!(net.nodes[A].node.link.is_idle());
    }
    // Nor is a request through three answered. The header's hop field tops out at
    // MAX_DIGIS, so such a request never looks fully repeated and is refused even without
    // the payload's digipeater-count check (that limit is tested in connect_payload_parsing).
    let mut b = Link::new(net.modem.clone(), LinkConfig::default(), 1);
    b.set_mycalls(vec!["LA2BBB-1".into()]);
    b.set_listen(true);
    let mut header = Header { kind: Kind::ConnectReq, session: 9, ncw: 1, ..Header::default() };
    header.set_hop(2);
    let payload = vec![Some(calls_payload("LA1AAA", "LA2BBB-1", &via(&["LA3A", "LA3B", "LA3C"])))];
    b.on_burst(1.0, &RxBurst { header, payload, snr_db: 30.0, end_sample: 0, level: Default::default() });
    assert!(b.is_idle() && b.take_events().is_empty());
}

#[test]
fn retry_wait_grows_with_hops_and_margin() {
    let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
    let link = Link::new(modem.clone(), LinkConfig::default(), 1);
    let wide_margin = Link::new(modem, LinkConfig { hop_margin_s: 3.0, ..LinkConfig::default() }, 1);
    let cfg = LinkConfig::default();
    assert_eq!(link.response_wait(0, 4.0), cfg.response_timeout_s);
    let w: Vec<f64> = (0..=MAX_DIGIS as u8).map(|n| link.response_wait(n, 4.0)).collect();
    assert!(w.windows(2).all(|p| p[1] > p[0] + 8.0), "{w:?}");
    assert!((wide_margin.response_wait(2, 4.0) - link.response_wait(2, 4.0) - 2.0 * 2.5).abs() < 1e-9);
    // Keepalive never fires while an answer may still be coming.
    for n in 1..=MAX_DIGIS as u8 {
        assert!(link.keepalive_s(n) >= 2.0 * link.response_wait(n, link.max_burst_s));
    }
    assert_eq!(link.keepalive_s(0), cfg.keepalive_s);
    for params in [OfdmParams::narrow(), OfdmParams::wide()] {
        let l = Link::new(Arc::new(Modem::new(params, 0.1)), LinkConfig::default(), 1);
        let w: Vec<String> =
            (1..=MAX_DIGIS as u8).map(|n| format!("{:.1}", l.response_wait(n, l.max_burst_s))).collect();
        println!("{}: longest burst {:.2} s, wait after a full burst {w:?} s", l.modem.ofdm.p.name, l.max_burst_s);
    }
}

/// A sends 2 kB to B through one digipeater that repeats `delay` s after each burst.
/// Returns the bursts sent again for want of an answer (connect requests and data),
/// or None if the data did not arrive.
fn slow_digipeater_resends(delay: f64, hop_margin_s: f64, seed: u32) -> Option<u64> {
    let digis = ["LA3DDD"];
    let mut net = Net::chain(seed, 0.0, &digis, LinkConfig { hop_margin_s, ..LinkConfig::default() });
    net.nodes[1].node.set_digipeater(Digipeater::new(&["LA3DDD".to_string()], delay));
    let b = net.b();
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&digis)).unwrap();
    let da = data(2000, seed);
    let mut queued = false;
    while net.now < 900.0 && net.nodes[b].host_rx != da {
        net.step();
        if net.has(A, &LinkEvent::Disconnected) || net.has(b, &LinkEvent::Disconnected) {
            return None;
        }
        if !queued && net.both_connected() {
            queued = true;
            net.nodes[A].node.link.send_data(&da);
        }
    }
    let requests = net.sent.iter().filter(|s| s.3 == Kind::ConnectReq).count() as u64;
    let retries = net.nodes[A].node.link.stats.retries + net.nodes[b].node.link.stats.retries;
    (net.nodes[b].host_rx == da).then_some(requests - 1 + retries)
}

#[test]
fn slow_digipeater_needs_a_larger_hop_margin() {
    // The digipeater repeats 2 s after each burst instead of 0.2 s. B cannot hear A,
    // so only the hop margin covers the delay. An answer passes the digipeater twice,
    // as our burst and then as the answer, so the margin has to cover the delay twice.
    // With less, a station resends while the answer is still on its way.
    for seed in 35..39 {
        let (default, once, twice) = (0.5, 2.0, 4.0);
        assert_ne!(slow_digipeater_resends(2.0, default, seed), Some(0), "seed {seed}, default margin");
        assert_ne!(slow_digipeater_resends(2.0, once, seed), Some(0), "seed {seed}, the delay once");
        assert_eq!(slow_digipeater_resends(2.0, twice, seed), Some(0), "seed {seed}, the delay twice");
    }
}

#[test]
fn called_station_banner_does_not_collide_with_connect_ack_repeats() {
    // A BBS queues its banner as soon as it reports CONNECTED, which happens when the
    // ConnectReq arrives, before its ConnectAck has gone out.
    for digis in [&["LA3DDD"][..], &["LA3DDD", "LA4EEE-2"]] {
        let mut net = Net::chain(36, 0.0, digis, LinkConfig::default());
        let b = net.b();
        net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
        net.run_until(120.0, |n| n.nodes[n.b()].node.link.is_connected());
        let banner = data(500, 36);
        net.nodes[b].node.link.send_data(&banner);
        net.run_until(300.0, |n| n.nodes[A].host_rx.len() >= banner.len());
        assert_eq!(net.nodes[A].host_rx, banner);
        let (ra, rb) = (net.nodes[A].node.link.stats.retries, net.nodes[b].node.link.stats.retries);
        assert_eq!((ra, rb), (0, 0), "via {digis:?}: retries A {ra} B {rb}");
    }
}

#[test]
fn path_naming_one_digipeater_twice() {
    let digis = ["LA3DDD"];
    let mut net = Net::chain(37, 0.0, &digis, LinkConfig::default());
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&["LA3DDD", "LA3DDD"])).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    net.nodes[A].node.link.send_data(b"twice");
    net.run_until(120.0, |n| n.nodes[n.b()].host_rx.len() >= 5);
    assert_eq!(net.nodes[net.b()].host_rx, b"twice");
}

/// Both ends of a quiet routed link queue `size` bytes, B `offset` s after A, so their
/// bursts meet at a digipeater and are both lost. Returns the time to deliver both ways
/// and each end's retries, or what went wrong.
fn simultaneous_start(digis: &[&str], seed: u32, size: usize, offset: f64) -> Result<(f64, u64, u64), String> {
    let mut net = Net::chain(seed, 0.0, digis, LinkConfig::default());
    let b = net.b();
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    let quiet = net.now + 25.0;
    net.run_until(30.0, |n| n.now >= quiet);
    let (da, db) = (data(size, seed), data(size, seed + 7));
    net.nodes[A].node.link.send_data(&da);
    let at = net.now + offset;
    net.run_until(5.0, |n| n.now >= at);
    net.nodes[b].node.link.send_data(&db);
    let lost = |n: &Net| n.has(A, &LinkEvent::Disconnected) || n.has(n.b(), &LinkEvent::Disconnected);
    let t = net.run_until(1500.0, |n| {
        lost(n) || n.nodes[A].host_rx.len() >= db.len() && n.nodes[n.b()].host_rx.len() >= da.len()
    });
    let (ra, rb) = (net.nodes[A].node.link.stats.retries, net.nodes[b].node.link.stats.retries);
    let what = format!("via {digis:?} seed {seed} {size} B offset {offset:.1} s");
    println!("{what}: {t:.0} s, retries {ra}/{rb}");
    if lost(&net) {
        return Err(format!("{what}: link dropped after {t:.0} s, retries {ra}/{rb}"));
    }
    assert_eq!(net.nodes[A].host_rx, db, "{what}");
    assert_eq!(net.nodes[b].host_rx, da, "{what}");
    Ok((t, ra, rb))
}

#[test]
fn routed_ends_starting_together_do_not_collide_forever() {
    // The two ends do not hear each other, so neither can tell that the other is
    // sending. With equal retry timers their retries met at a digipeater every time.
    // The longest path again, on more seeds.
    let chains = [&["LA3DDD"][..], &["LA3DDD", "LA4EEE-2"], &["LA3DDD", "LA4EEE-2"]];
    let mut runs = Vec::new();
    for (i, digis) in chains.iter().enumerate() {
        for k in 0..4u32 {
            let size = if k == 3 { 300 } else { 40 };
            runs.push(simultaneous_start(digis, 60 + 10 * i as u32 + k, size, 0.2 * k as f64));
        }
    }
    runs.push(simultaneous_start(chains[0], 59, 6000, 0.3));
    let failed: Vec<&String> = runs.iter().filter_map(|r| r.as_ref().err()).collect();
    assert!(failed.is_empty(), "{} of {} runs failed: {failed:#?}", failed.len(), runs.len());
    // The first bursts did collide: every run needed a retry.
    assert!(runs.iter().flatten().all(|&(_, ra, rb)| ra + rb > 0), "a run had no collision");
}

#[test]
fn called_station_outwaits_the_callers_retry() {
    for params in [OfdmParams::narrow(), OfdmParams::wide()] {
        let modem = Arc::new(Modem::new(params, 0.1));
        for cfg in [LinkConfig::default(), LinkConfig { hop_margin_s: 3.0, ..LinkConfig::default() }] {
            let link = Link::new(modem.clone(), cfg.clone(), 1);
            let max = link.max_burst_s;
            let lengths = [modem.burst_seconds(0, 0) as f64, 1.0, max];
            // Direct sessions and the caller keep the plain response wait.
            assert_eq!(link.retry_wait(0, false, 1.0), link.response_wait(0, 1.0));
            let mut waits = Vec::new();
            for n in 1..=MAX_DIGIS as u8 {
                for own in lengths {
                    assert_eq!(link.retry_wait(n, true, own), link.response_wait(n, own));
                }
                // The caller's longest retry period, start to start, and the time its
                // retry takes to reach the called station.
                let caller = (0..=20).map(|i| max * i as f64 / 20.0).map(|t| t + link.response_wait(n, t));
                let caller = caller.fold(0.0, f64::max) + RETRY_BACKOFF_S * Link::retry_spread(n);
                let reach = n as f64 * (max + cfg.turnaround_s + cfg.hop_margin_s);
                for own in lengths {
                    let called = own + link.retry_wait(n, false, own);
                    let what = format!("{n} hops, {own:.2} s burst: {called:.1} < {caller:.1} + {reach:.1}");
                    assert!(called >= caller + reach - 1e-9, "{what}");
                }
                // Keepalive never fires while the called station's retry may be coming.
                assert!(link.keepalive_s(n) >= 2.0 * link.retry_wait(n, false, 0.0));
                let (short, full) = (link.retry_wait(n, false, lengths[0]), link.retry_wait(n, false, max));
                waits.push(format!("{short:.1}/{full:.1}"));
            }
            let name = modem.ofdm.p.name;
            let margin = cfg.hop_margin_s;
            println!("{name}, hop margin {margin}: called station's retry wait after a short/full burst {waits:?} s");
        }
    }
}

#[test]
fn routed_session_with_lost_bursts_completes() {
    let digis = ["LA3DDD", "LA4EEE-2"];
    for seed in 70..82 {
        println!("seed {seed}");
        routed_session(Net::build(seed, 0.1, 0.0, &digis, LinkConfig::default()), &digis, seed);
    }
}

#[test]
fn rate_choice_accounts_for_drive_per_level() {
    for params in [OfdmParams::narrow(), OfdmParams::wide()] {
        let modem = Modem::new(params, 0.1);
        let name = modem.ofdm.p.name;
        let top = LEVELS.len() as u8 - 1;
        for l in 0..=top {
            // Just above the threshold, measured on a burst at that level.
            let at = SNR_THRESHOLDS_DB[l as usize] + SNR_MARGIN_DB + 0.05;
            assert!(level_by_snr(&modem, at, l) >= l, "{name} level {l}");
            // The same channel measured on a louder level-0 burst (the model is invertible
            // below the ceilings).
            let at0 = modem.predict_snr_db(at, l, 0);
            assert!(level_by_snr(&modem, at0, 0) >= l, "{name} level {l}: {at:.2} dB -> {at0:.2} dB at level 0");
            // A level never predicts above its own ceiling, nor above the measurement
            // plus the drive difference.
            for to in 0..=top {
                let pred = modem.predict_snr_db(at, l, to);
                assert!(pred <= modem.ofdm.p.level_snr_ceiling_db[to as usize] + 0.01);
                assert!(pred <= at + modem.level_gain_db(l, to).max(0.0) + 0.01, "{name} {l}->{to}: {pred}");
            }
        }
        // Well below the ceilings, a loud burst's SNR overstates what a softer, faster
        // level would get by about the drive difference.
        let gain = modem.level_gain_db(top, 0);
        assert!(gain > 0.0);
        let pred = modem.predict_snr_db(5.0, 0, top);
        assert!((pred - (5.0 - gain)).abs() < 0.5, "{name}: {pred} vs {}", 5.0 - gain);
        // At the ceiling the measurement says nothing about the channel.
        let ceiling0 = modem.ofdm.p.level_snr_ceiling_db[0];
        assert_eq!(level_by_snr(&modem, ceiling0 + 1.0, 0), top);
        assert_eq!(level_by_snr(&modem, -10.0, top), 0);
        assert_eq!(level_by_snr(&modem, 60.0, 0), top);
    }
}

#[test]
fn predicted_snr_matches_the_simulator() {
    // Measured with the `level_table` benchmark (wide mode): at 24 dB CNR a level-4
    // burst reports 18.0 dB and a level-5 burst 16.9 dB; at 12 dB CNR a level-0 burst
    // reports 8.3 dB and a level-5 burst 4.7 dB.
    let modem = Modem::new(OfdmParams::wide(), 0.1);
    let p = modem.predict_snr_db(18.0, 4, 5);
    assert!((p - 16.9).abs() < 0.7, "L4 -> L5 at 24 dB CNR: {p:.1}");
    let p = modem.predict_snr_db(8.3, 0, 5);
    assert!((p - 4.7).abs() < 1.0, "L0 -> L5 at 12 dB CNR: {p:.1}");
    // Narrow: at 14 dB CNR a level-4 burst reports 15.9 dB and a level-5 burst 15.3 dB.
    let modem = Modem::new(OfdmParams::narrow(), 0.1);
    let p = modem.predict_snr_db(15.9, 4, 5);
    assert!((p - 15.3).abs() < 1.0, "narrow L4 -> L5 at 14 dB CNR: {p:.1}");
}

#[test]
fn longer_resend_of_a_stored_segment_is_kept() {
    // A segment that arrived ahead of a gap is resent at a faster level, where one
    // codeword holds more bytes from the same offset. The receiver acknowledges the
    // resend, so the sender never sends those extra bytes again.
    let mut net = Net::new(50, 0.0, 0.0);
    net.nodes[0].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.both_connected());
    let now = net.now;
    let link = &mut net.nodes[1].node.link;
    let State::Connected(s) = &link.state else { panic!("not connected") };
    let id = s.id;
    link.take_events();

    let top = LEVELS.len() as u8 - 1;
    let cap = |level: u8| Modem::payload_bytes(level) - SEGMENT_HEADER;
    let (short, long) = (cap(0), cap(top));
    assert!(long > short);
    let d = data(short + long, 51);
    // One Data burst with one segment: `bytes` at offset `off`.
    let segment = |burst_id: u8, level: u8, off: usize, bytes: &[u8]| {
        let mut cw = (off as u32).to_be_bytes()[1..].to_vec();
        cw.push(bytes.len() as u8);
        cw.extend_from_slice(bytes);
        cw.resize(Modem::payload_bytes(level), 0);
        let header = Header { kind: Kind::Data, session: id, level, ncw: 1, burst_id, ..Header::default() };
        RxBurst { header, payload: vec![Some(cw)], snr_db: 30.0, end_sample: 0, level: Default::default() }
    };
    // Ahead of the gap, at level 0; then the same offset at the top level, with more bytes.
    link.on_burst(now, &segment(1, 0, short, &d[short..2 * short]));
    link.on_burst(now + 1.0, &segment(2, top, short, &d[short..]));
    assert!(link.take_events().iter().all(|e| !matches!(e, LinkEvent::Data(_))), "delivered past the gap");
    // The gap is filled: everything acknowledged must come out.
    link.on_burst(now + 2.0, &segment(3, 0, 0, &d[..short]));
    let got: Vec<u8> = link
        .take_events()
        .into_iter()
        .filter_map(|e| if let LinkEvent::Data(x) = e { Some(x) } else { None })
        .flatten()
        .collect();
    assert_eq!(got.len(), d.len(), "bytes of the longer resend were dropped");
    assert_eq!(got, d);
}

/// A ConnectReq or ConnectAck naming `digis`, with UP and the raw route field.
fn connect_burst(kind: Kind, session: u8, calls: (&str, &str), digis: &[&str], up: bool, route: u8) -> RxBurst {
    let flags = if up { FLAG_UP } else { 0 } | route << 2;
    let header = Header { kind, session, ncw: 1, flags, ..Header::default() };
    let payload = vec![Some(calls_payload(calls.0, calls.1, &via(digis)))];
    RxBurst { header, payload, snr_db: 30.0, end_sample: 0, level: Default::default() }
}

/// B (LA2BBB-1) on its own, listening.
fn called_station() -> Link {
    let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
    let mut b = Link::new(modem, LinkConfig::default(), 1);
    b.set_mycalls(vec!["LA2BBB-1".into()]);
    b.set_listen(true);
    b
}

#[test]
fn connect_request_must_arrive_as_its_path_says() {
    // A direct request is unrouted; one through n digipeaters has been repeated n
    // times (route field n + 1). Neither carries UP. Any other header on the same
    // payload is a copy heard part way, or not what it claims to be.
    let request = |digis: &[&str], (up, route): (bool, u8)| {
        connect_burst(Kind::ConnectReq, 9, ("LA1AAA", "LA2BBB-1"), digis, up, route)
    };
    for (digis, own) in [(&[][..], (false, 0)), (&["LA3A"][..], (false, 2)), (&["LA3A", "LA3B"][..], (false, 3))] {
        for other in all_but(own) {
            let mut b = called_station();
            b.on_burst(1.0, &request(digis, other));
            assert!(b.is_idle() && b.take_events().is_empty(), "via {digis:?}: answered {other:?}");
        }
        let mut b = called_station();
        b.on_burst(1.0, &request(digis, own));
        assert!(b.is_connected(), "via {digis:?}");
    }
}

#[test]
fn connect_ack_must_name_our_request() {
    let mut net = Net::new(41, 0.0, 0.0);
    let link = &mut net.nodes[A].node.link;
    link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&["LA3DDD"])).unwrap();
    let State::Connecting { session, .. } = &link.state else { panic!("not connecting") };
    let id = *session;
    // The header is the called station's, through one digipeater. The payload is not ours.
    let ack = |calls, digis: &[&str]| connect_burst(Kind::ConnectAck, id, calls, digis, true, 2);
    let ours = ("LA1AAA", "LA2BBB-1");
    for (calls, digis) in [
        (ours, &["LA3EEE"][..]),
        (ours, &["LA3DDD", "LA3DDD"]),
        (ours, &[]),
        (("LA9ZZZ", "LA2BBB-1"), &["LA3DDD"]),
        (("LA1AAA", "LA9ZZZ"), &["LA3DDD"]),
    ] {
        link.on_burst(0.5, &ack(calls, digis));
        assert!(!link.is_connected(), "connected by {calls:?} via {digis:?}");
    }
    link.on_burst(0.5, &ack(ours, &["LA3DDD"]));
    assert!(link.is_connected());
}

#[test]
fn closed_session_answers_only_its_peers_disc() {
    // The caller did not hear our DiscAck and sends its Disc again. We answer with the
    // session's own flags, and only a Disc that came the session's way.
    for (digis, peer, own_flags) in [(&[][..], (false, 0), 0), (&["LA3DDD"][..], (false, 2), FLAG_ROUTED | FLAG_UP)] {
        let mut net = Net::chain(42, 0.0, digis, LinkConfig::default());
        let b = net.b();
        net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
        net.run_until(120.0, |n| n.both_connected());
        let State::Connected(s) = &net.nodes[b].node.link.state else { panic!("not connected") };
        let id = s.id;
        net.nodes[A].node.link.disconnect(net.now);
        net.run_until(120.0, |n| n.both_idle());
        let now = net.now + 1.0;
        let link = &mut net.nodes[b].node.link;
        let disc = |(up, route): (bool, u8)| RxBurst {
            header: Header { kind: Kind::Disc, ..probe(id, up, route).header },
            ..probe(id, up, route)
        };
        for other in all_but(peer) {
            link.on_burst(now, &disc(other));
            assert!(link.poll(now, false).is_none(), "via {digis:?}: answered {other:?}");
        }
        link.on_burst(now, &disc(peer));
        let ack = link.poll(now, false).expect("answers its peer's Disc");
        assert_eq!(
            (ack.header.kind, ack.header.session, ack.header.flags),
            (Kind::DiscAck, id, own_flags),
            "via {digis:?}"
        );
    }
}

#[test]
fn abort_reaches_the_peer_through_a_digipeater() {
    let digis = ["LA3DDD"];
    let mut net = Net::chain(43, 0.0, &digis, LinkConfig::default());
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&digis)).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    let quiet = net.now + 20.0;
    net.run_until(30.0, |n| n.now >= quiet);
    net.nodes[A].node.link.abort(net.now);
    assert!(net.nodes[A].node.link.is_idle());
    // One Disc, repeated once: well before B would miss A by itself.
    let t = net.run_until(30.0, |n| n.nodes[n.b()].node.link.is_idle());
    assert!(t < 5.0, "took {t} s");
}

#[test]
fn connect_payload_parsing() {
    let (a, b) = ("LA1AAA".to_string(), "LA2BBB-1".to_string());
    for digis in [&[][..], &["LA3A"], &["LA3A", "LA3B-2"]] {
        let p = calls_payload(&a, &b, &via(digis));
        assert_eq!(parse_calls_payload(&p), Some((a.clone(), b.clone(), via(digis))), "via {digis:?}");
    }
    // More digipeaters than a path can have.
    assert_eq!(parse_calls_payload(&calls_payload(&a, &b, &via(&["LA3A", "LA3B", "LA3C"]))), None);

    let good = calls_payload(&a, &b, &via(&["LA3A"]));
    let count_at = 4 + 1 + a.len() + 1 + b.len();
    assert_eq!(good[count_at], 1);
    let with = |at: usize, byte: u8| {
        let mut p = good.clone();
        p[at] = byte;
        p
    };
    // A routed payload names at least one digipeater, and at most MAX_DIGIS. The
    // padding after the first call would read as empty calls.
    assert_eq!(parse_calls_payload(&with(count_at, 0)), None);
    assert_eq!(parse_calls_payload(&with(count_at, MAX_DIGIS as u8 + 1)), None);
    assert!(parse_calls_payload(&with(count_at, MAX_DIGIS as u8)).is_some());
    // Not our payload, not text, or cut short.
    assert_eq!(parse_calls_payload(&with(3, b'3')), None);
    assert_eq!(parse_calls_payload(&with(5, 0xff)), None);
    assert_eq!(parse_calls_payload(&good[..count_at]), None);
    assert_eq!(parse_calls_payload(&good[..count_at + 3]), None);
    assert_eq!(parse_calls_payload(&[]), None);
}

/// Longest burst any level can send, from the modem alone.
fn longest_burst(modem: &Modem) -> f64 {
    (0..LEVELS.len() as u8).map(|l| modem.burst_seconds(l, modem.max_ncw(l)) as f64).fold(0.0, f64::max)
}

/// A quiet session through `digis`, with the far end of `who` then switched off.
/// `act` makes `who` start sending; returns the (start, end) of each burst of `kind`
/// it sent until it gave up.
fn unanswered_bursts(
    digis: &[&str],
    seed: u32,
    who: usize,
    kind: Kind,
    act: impl Fn(&mut Link, f64),
) -> Vec<(f64, f64)> {
    let mut net = Net::chain(seed, 0.0, digis, LinkConfig::default());
    let (who, other) = if who == A { (A, net.b()) } else { (net.b(), A) };
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    // Past the time the caller could still be retrying its ConnectReq: until then a
    // called station not yet answered retries with its ConnectAck, not Data.
    let window = net.nodes[A].node.link.connect_window_s(digis.len() as u8);
    let quiet = net.now + window.max(20.0);
    net.run_until(window + 30.0, |n| n.now >= quiet);
    net.nodes[other].dead = true;
    let from = net.now;
    act(&mut net.nodes[who].node.link, from);
    net.run_until(1200.0, |n| n.nodes[who].node.link.is_idle());
    net.sent.iter().filter(|s| s.0 == who && s.1 >= from && s.3 == kind).map(|s| (s.1, s.2)).collect()
}

#[test]
fn retries_are_spaced_as_the_protocol_document_says() {
    // Measured on the channel, from the figures in docs/PROTOCOL.md 3.2 and 3.4, not
    // from the link's own helpers.
    let cfg = LinkConfig::default();
    let (timeout, turnaround, margin, backoff) = (1.2, 0.2, 0.5, 0.4);
    assert_eq!((cfg.response_timeout_s, cfg.turnaround_s, cfg.hop_margin_s), (timeout, turnaround, margin));
    // One step of the fake channel either side of each timer.
    let slack = 5.0 * DT;
    let longest = longest_burst(&Modem::new(OfdmParams::narrow(), 0.1));
    let mut widest_spread = [0.0f64; 3];
    for (hops, digis) in [&[][..], &["LA3DDD"], &["LA3DDD", "LA4EEE-2"]].into_iter().enumerate() {
        let n = hops as f64;
        let spread = backoff * (1.0 + 2.0 * n);
        // Wait after a burst of `own` s before it is sent again.
        let wait = |own: f64| timeout + n * (own + longest + 2.0 * turnaround + margin);
        let mut check = |what: &str, got: f64, base: f64| {
            let over = got - base;
            assert!(
                over > -slack && over < spread + slack,
                "via {digis:?}, {what}: {got:.2} s, not {base:.2} + 0..{spread:.1}"
            );
            widest_spread[hops] = widest_spread[hops].max(over);
        };
        for seed in 90..93 {
            // The caller resends after the plain wait, counted from the end of its burst.
            let sent = unanswered_bursts(digis, seed, A, Kind::Data, |link, _| link.send_data(b"anyone?"));
            assert_eq!(sent.len(), 1 + cfg.max_retries as usize, "via {digis:?}");
            for pair in sent.windows(2) {
                check("caller", pair[1].0 - pair[0].1, wait(pair[0].1 - pair[0].0));
            }
            // The called station waits out the caller's longest retry period, counted
            // from the start of its own burst, plus the time the caller's retry takes
            // to reach it. Without digipeaters it waits like the caller.
            let sent = unanswered_bursts(digis, seed, 1, Kind::Data, |link, _| link.send_data(b"anyone?"));
            assert_eq!(sent.len(), 1 + cfg.max_retries as usize, "via {digis:?}");
            for pair in sent.windows(2) {
                if hops == 0 {
                    check("called station", pair[1].0 - pair[0].1, wait(pair[0].1 - pair[0].0));
                } else {
                    let caller_period = longest + wait(longest) + spread;
                    let reach = n * (longest + turnaround + margin);
                    check("called station", pair[1].0 - pair[0].0, caller_period + reach);
                }
            }
            // Its Disc is resent after the plain wait: a digipeater forgets the session
            // soon after the DiscAck.
            let sent = unanswered_bursts(digis, seed, 1, Kind::Disc, |link, now| link.disconnect(now));
            assert_eq!(sent.len(), 4, "via {digis:?}");
            for pair in sent.windows(2) {
                check("called station's Disc", pair[1].0 - pair[0].1, wait(pair[0].1 - pair[0].0));
            }
        }
    }
    // Through digipeaters the random spread really is wider than a direct session's.
    assert!(widest_spread[0] < backoff + slack);
    assert!(widest_spread[1] > backoff + slack && widest_spread[2] > backoff + slack, "{widest_spread:?}");
}

#[test]
fn hop_margin_limits() {
    for ok in [0.0, 0.5, MAX_HOP_MARGIN_S] {
        assert!(valid_hop_margin(ok), "{ok}");
    }
    for bad in [-0.1, MAX_HOP_MARGIN_S + 0.1, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(!valid_hop_margin(bad), "{bad}");
    }
    assert!(valid_hop_margin(LinkConfig::default().hop_margin_s));
}

#[test]
fn digipeater_carries_its_own_session_while_relaying() {
    // A talks to B through D, while D itself talks to C. A, B and C hear only D.
    // This shows that both sessions can run and that their data is not mixed up, not
    // that they always do: C and A or B cannot hear each other, and their bursts
    // collide at D, so both sessions retry a lot (see the limits in docs/PROTOCOL.md).
    for seed in 44..48 {
        let digis = ["LA3DDD"];
        let mut net = Net::chain(seed, 0.0, &digis, LinkConfig::default());
        let (d, b, c) = (1, 2, 3);
        let mut peer = Peer {
            name: "C".into(),
            node: Node::new(Link::new(net.modem.clone(), LinkConfig::default(), seed + 9)),
            tx: None,
            host_rx: Vec::new(),
            events: Vec::new(),
            dead: false,
        };
        peer.node.link.set_mycalls(vec!["LA5CCC".into()]);
        peer.node.link.set_listen(true);
        net.nodes.push(peer);
        for row in net.hears.iter_mut() {
            row.push(false);
        }
        net.hears.push(vec![false, true, false, false]);
        net.hears[d][c] = true;

        net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&digis)).unwrap();
        net.nodes[d].node.link.connect(0.0, "LA3DDD", "LA5CCC", &[]).unwrap();
        net.run_until(300.0, |n| n.nodes.iter().all(|p| p.node.link.is_connected()));
        // 2 kB each way on both sessions, all queued at once.
        let sent: Vec<Vec<u8>> = (0..4).map(|i| data(2000, seed + i)).collect();
        for (i, bytes) in sent.iter().enumerate() {
            net.nodes[i].node.link.send_data(bytes);
        }
        let from = [b, c, A, d];
        net.run_until(1500.0, |n| (0..4).all(|i| n.nodes[i].host_rx.len() >= sent[from[i]].len()));
        for i in 0..4 {
            assert_eq!(net.nodes[i].host_rx, sent[from[i]], "seed {seed}: what {} received", net.nodes[i].name);
            assert!(!net.has(i, &LinkEvent::Disconnected), "seed {seed}: {} dropped", net.nodes[i].name);
        }
        assert!(net.nodes[d].node.repeats() >= 4, "seed {seed}: {} repeats", net.nodes[d].node.repeats());
    }
}

/// Connects A to B through `digis`, silences B, and once A's next burst is over keys
/// up a carrier at A's neighbour that never ends. If `gaps`, the carrier drops out for
/// 0.3 s every 30 s, long enough for A to see a quiet channel for a moment. If `disc`,
/// A's host then asks to disconnect, so a Disc is pending too. Returns how long A took
/// to drop the session, or None if it was still connected after 900 s, and the latest
/// A should have dropped.
fn jammed_with(seed: u32, digis: &[&str], disc: bool, gaps: bool, cfg: LinkConfig) -> (Option<f64>, f64) {
    let mut net = Net::chain(seed, 0.0, digis, cfg);
    let b = net.b();
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    net.run_until(120.0, |n| n.now > 60.0);
    net.nodes[b].dead = true;
    net.nodes[A].node.link.send_data(&data(500, seed));
    net.run_until(60.0, |n| n.nodes[A].tx.is_some());
    net.run_until(60.0, |n| n.nodes[A].tx.is_none());
    let now = net.now;
    if gaps {
        for k in 0..31 {
            let start = now + 30.0 * k as f64;
            net.history.push((1, start, start + 29.7));
        }
    } else {
        net.history.push((1, now, f64::MAX));
    }
    if disc {
        net.nodes[A].node.link.disconnect(now);
    }
    let t0 = net.now;
    while !net.has(A, &LinkEvent::Disconnected) && net.now - t0 < 900.0 {
        net.step();
    }
    // A has data or a Disc held back from the start, so its busy timeout is the limit.
    let limit = net.nodes[A].node.link.busy_timeout_s(digis.len() as u8);
    (net.has(A, &LinkEvent::Disconnected).then_some(net.now - t0), limit)
}

/// As `jammed_with`, and checks A dropped after the busy timeout for its route, but no
/// later than that plus some slack.
fn jammed(seed: u32, digis: &[&str], disc: bool, gaps: bool, cfg: LinkConfig) {
    let (t, limit) = jammed_with(seed, digis, disc, gaps, cfg);
    let t = t.unwrap_or_else(|| panic!("A still in session after 900 s on a jammed channel via {digis:?}"));
    assert!(t > limit && t < limit + 5.0, "via {digis:?}, disc {disc}, gaps {gaps}: took {t:.0} s, limit {limit:.0} s");
}

#[test]
fn direct_retry_gives_up_on_a_jammed_channel() {
    // The peer goes silent and the channel then stays busy (a carrier, or a station we
    // are not talking to). A retry that gives way to a busy channel must still be
    // bounded: the session has to drop rather than wait for the channel forever.
    jammed(1, &[], false, false, LinkConfig::default());
}

#[test]
fn routed_retry_and_pending_disc_give_up_on_a_jammed_channel() {
    for (i, digis) in [&["LA3DDD"][..], &["LA3DDD", "LA4EEE-2"]].iter().enumerate() {
        for disc in [false, true] {
            jammed(50 + i as u32, digis, disc, false, LinkConfig::default());
        }
    }
    jammed(52, &[], true, false, LinkConfig::default());
}

#[test]
fn infinite_busy_timeout_never_drops() {
    let cfg = LinkConfig { busy_timeout_s: f64::INFINITY, ..LinkConfig::default() };
    assert_eq!(jammed_with(1, &[], false, false, cfg).0, None);
}

#[test]
fn busy_timeout_setting_is_raised_to_a_floor_and_bad_values_rejected() {
    let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
    let cfg = |s: f64| LinkConfig { busy_timeout_s: s, ..LinkConfig::default() };
    let with = |s: f64| Link::new(modem.clone(), cfg(s), 1);
    let mut last = 0.0;
    for hops in 0..=MAX_DIGIS as u8 {
        // 0 gives the floor: more than a burst, and longer with every hop.
        let floor = with(0.0).busy_timeout_s(hops);
        assert!(floor > with(0.0).max_burst_s && floor > last, "hops {hops}: {floor}");
        last = floor;
        assert_eq!(with(floor / 2.0).busy_timeout_s(hops), floor);
        assert_eq!(with(floor + 100.0).busy_timeout_s(hops), floor + 100.0);
        assert_eq!(with(f64::INFINITY).busy_timeout_s(hops), f64::INFINITY);
    }
    for bad in [-1.0, f64::NEG_INFINITY, f64::NAN] {
        let m = modem.clone();
        let made = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| Link::new(m, cfg(bad), 1)));
        assert!(made.is_err(), "busy_timeout_s {bad} accepted");
    }
}

#[test]
fn idle_session_survives_a_busy_channel_until_its_keepalive_is_due() {
    // Neither end has anything to send, and other stations hold the channel for longer
    // than the busy timeout. Nothing is held back until the keepalive is due, so the
    // session must stay up.
    let cfg = LinkConfig { busy_timeout_s: 0.0, keepalive_s: 300.0, ..LinkConfig::default() };
    let mut net = Net::chain(71, 0.0, &[], cfg);
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.both_connected());
    net.run_until(10.0, |n| n.nodes.iter().all(|x| x.tx.is_none()));
    let busy_timeout = net.nodes[A].node.link.busy_timeout_s(0);
    let (start, end) = (net.now, net.now + 2.0 * busy_timeout);
    net.history.push((1, start, end));
    net.run_until(3.0 * busy_timeout, |n| n.now > end + 2.0);
    assert!(net.both_connected());
    assert!(!net.has(A, &LinkEvent::Disconnected));
}

#[test]
fn busy_timeout_never_drops_a_working_session() {
    // With no configured busy timeout, only the floor counts; a short keepalive keeps
    // something to send held back most of the time. Two topologies: every
    // station hears every other, so the channel stays busy from a burst through its
    // repeats and the answer's; and neighbours only, where the ends do not hear each
    // other and a digipeater's repeat of the peer's answer is the first we hear of it.
    // Both must finish without dropping.
    let cfg = LinkConfig { busy_timeout_s: 0.0, keepalive_s: 1.0, ..LinkConfig::default() };
    for all_hear in [true, false] {
        for (i, digis) in [&[][..], &["LA3DDD"], &["LA3DDD", "LA4EEE-2"]].iter().enumerate() {
            let seed = 60 + i as u32;
            let mut net = Net::chain(seed, 0.05, digis, cfg.clone());
            if all_hear {
                net.set_hears(|to, from| to != from);
            }
            routed_session(net, digis, seed);
        }
    }
}

#[test]
fn held_back_timer_restarts_on_our_burst_not_a_quiet_gap_and_spares_a_burst_on_air() {
    let mut net = Net::new(70, 0.0, 0.0);
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.both_connected());
    net.nodes[A].node.link.send_data(&data(100, 70));
    net.run_until(10.0, |n| n.nodes[A].tx.is_some());
    let link = &mut net.nodes[A].node.link;
    let bt = link.busy_timeout_s(0);
    // The peer is long silent.
    let t = net.now + 1000.0;
    // Our burst is on the air: a long busy stretch does not drop the session.
    assert!(link.poll(t, true).is_none());
    assert!(link.poll(t + bt + 1.0, true).is_none());
    assert!(link.is_connected());
    // Our burst ends, so the timer starts again from zero. We now wait for the answer
    // with a retry pending, which the busy channel holds back.
    let t = t + bt + 1.0;
    link.tx_done(t);
    link.poll(t + 0.01, true);
    link.poll(t + 0.9 * bt, true);
    assert!(link.is_connected());
    // A quiet moment too short for the retry to go out is no progress: the timer runs on.
    assert!(link.poll(t + 0.9 * bt + 0.1, false).is_none());
    link.poll(t + 0.9 * bt + 0.2, true);
    assert!(link.is_connected());
    link.poll(t + bt + 0.1, true);
    assert!(!link.is_connected());
    assert!(link.take_events().contains(&LinkEvent::Disconnected));
}

#[test]
fn short_quiet_gaps_do_not_keep_a_jammed_session_open() {
    // A carrier that drops out for a moment every 30 s: each gap is too short for the
    // held-back retry to go out, so it is no progress and the session must still drop.
    for (i, digis) in [&[][..], &["LA3DDD"], &["LA3DDD", "LA4EEE-2"]].iter().enumerate() {
        jammed(80 + i as u32, digis, false, true, LinkConfig::default());
    }
}

#[test]
fn a_long_keepalive_does_not_delay_the_drop_when_something_is_held_back() {
    // The keepalive matters only to an idle session. With data or a Disc held back, the
    // busy timeout alone decides.
    for keepalive_s in [600.0, f64::INFINITY] {
        let cfg = LinkConfig { keepalive_s, ..LinkConfig::default() };
        for disc in [false, true] {
            jammed(90, &[], disc, false, cfg.clone());
            jammed(91, &["LA3DDD"], disc, false, cfg.clone());
        }
    }
}

#[test]
fn connect_gives_up_on_a_jammed_channel() {
    // The channel is busy before our first request and stays busy. The request is held
    // back, so the connect attempt limit is never reached: the busy timeout must end it.
    for (i, digis) in [&[][..], &["LA3DDD"], &["LA3DDD", "LA4EEE-2"]].iter().enumerate() {
        let mut net = Net::chain(100 + i as u32, 0.0, digis, LinkConfig::default());
        net.history.push((1, -1.0, f64::MAX));
        net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
        let limit = net.nodes[A].node.link.busy_timeout_s(digis.len() as u8);
        let t = net.run_until(900.0, |n| n.has(A, &LinkEvent::Disconnected));
        assert!(t > limit && t < limit + 5.0, "via {digis:?}: took {t:.0} s, limit {limit:.0} s");
        assert!(net.sent.iter().all(|&(from, ..)| from != A), "via {digis:?}: A keyed up on a busy channel");
    }
}

/// Station 1 of a chain via one digipeater, which `jammed_live_peer` uses as a jammer.
const JAMMER: usize = 1;

/// A direct session, plus a third station (`JAMMER`, not on the route) whose carrier
/// only A hears. B hears A, but A cannot decode B under the carrier, so A drops the
/// session. Runs until A has dropped, and returns the net and when the carrier began.
fn jammed_live_peer(seed: u32) -> (Net, f64) {
    let cfg = LinkConfig { keepalive_s: 600.0, ..LinkConfig::default() };
    let mut net = Net::chain(seed, 0.0, &["LA3DDD"], cfg);
    net.set_hears(|to, from| to != from && !(from == JAMMER && to != A));
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.both_connected());
    net.run_until(60.0, |n| n.now > 20.0 && n.nodes.iter().all(|x| x.tx.is_none()));
    let start = net.now;
    net.history.push((JAMMER, start, start + 1000.0));
    net.nodes[A].node.link.send_data(&data(200, seed));
    net.run_until(300.0, |n| n.has(A, &LinkEvent::Disconnected));
    (net, start)
}

/// Ends the jammer's carrier, which began at `start`, at `end` instead.
fn end_jam(net: &mut Net, start: f64, end: f64) {
    net.history.retain(|&(g, ..)| g != JAMMER);
    net.history.push((JAMMER, start, end));
}

#[test]
fn dropped_session_sends_a_disc_once_the_channel_is_quiet() {
    // B is alive: A tells it with a Disc, but not into the carrier.
    let (mut net, start) = jammed_live_peer(110);
    let b = net.b();
    let end = net.now + 5.0;
    end_jam(&mut net, start, end);
    net.run_until(30.0, |n| n.has(b, &LinkEvent::Disconnected));
    let discs: Vec<f64> = net.sent.iter().filter(|&&(from, .., k)| from == A && k == Kind::Disc).map(|x| x.1).collect();
    assert_eq!(discs.len(), 1, "A's Discs at {discs:?}");
    assert!(discs[0] >= end, "Disc at {:.2} into the carrier (ends {end:.2})", discs[0]);
}

#[test]
fn new_connect_attempt_does_not_inherit_the_held_back_timer() {
    // A connect attempt held back for most of the busy timeout, then aborted. A new
    // attempt started at once gets the whole timeout again.
    let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
    let mut link = Link::new(modem, LinkConfig::default(), 1);
    let bt = link.busy_timeout_s(0);
    link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    assert!(link.poll(0.0, true).is_none());
    assert!(link.poll(0.9 * bt, true).is_none());
    let t = 0.9 * bt;
    link.abort(t);
    link.connect(t, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    assert!(link.poll(t + 0.01, true).is_none());
    assert!(link.poll(t + 0.5 * bt, true).is_none());
    assert!(!link.is_idle(), "new attempt ended after {:.1} s", 0.5 * bt);
}

#[test]
fn a_disc_held_back_past_the_busy_timeout_is_not_sent() {
    // The carrier outlasts the drop by more than the busy timeout. A Disc that late
    // could end the peer's next session if it got the same id: drop it.
    let (mut net, start) = jammed_live_peer(120);
    let end = net.now + net.nodes[A].node.link.busy_timeout_s(0) + 5.0;
    end_jam(&mut net, start, end);
    net.run_until(400.0, |n| n.now > end + 10.0);
    assert!(!net.sent.iter().any(|&(from, .., k)| from == A && k == Kind::Disc), "stale Disc sent");
}

#[test]
fn an_accepted_peer_burst_restarts_the_held_back_timer() {
    // A is closing, and its Disc is held back. Accepting B's ack is progress even though
    // it leaves the Disc still waiting, so the timer starts again.
    let mut net = Net::new(130, 0.0, 0.0);
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.both_connected());
    net.run_until(60.0, |n| n.now > 20.0 && n.nodes.iter().all(|x| x.tx.is_none()));
    net.nodes[A].node.link.send_data(&data(50, 130));
    net.run_until(30.0, |n| n.nodes[1].tx.is_some());
    let (.., burst, _) = net.nodes[1].tx.clone().unwrap();
    assert_eq!(burst.header.ncw, 0, "expected B's ack");
    let rx =
        RxBurst { header: burst.header, payload: Vec::new(), snr_db: 30.0, end_sample: 0, level: Default::default() };
    let link = &mut net.nodes[A].node.link;
    let bt = link.busy_timeout_s(0);
    let t = net.now;
    link.disconnect(t);
    assert!(link.poll(t, true).is_none());
    assert!(link.poll(t + 0.9 * bt, true).is_none());
    link.on_burst(t + 0.9 * bt, &rx);
    assert!(link.poll(t + 1.1 * bt, true).is_none());
    assert!(link.is_connected(), "dropped although B's ack came in {:.1} s ago", 0.2 * bt);
}

#[test]
fn jam_after_an_answer_that_needs_no_reply_still_drops() {
    // B acks A's data. Its ack needs no reply, so B waits without a retry pending. A
    // then goes silent and the channel stays busy, and B gets data to send: the busy
    // channel holds that data back, so B must still drop.
    let mut net = Net::new(140, 0.0, 0.0);
    let b = net.b();
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.both_connected());
    net.run_until(60.0, |n| n.now > 20.0 && n.nodes.iter().all(|x| x.tx.is_none()));
    net.nodes[A].node.link.send_data(&data(50, 140));
    net.run_until(30.0, |n| n.nodes[b].tx.is_some());
    assert_eq!(net.nodes[b].tx.as_ref().unwrap().2.header.ncw, 0, "expected B's ack");
    net.run_until(30.0, |n| n.nodes[b].tx.is_none());
    net.nodes[A].dead = true;
    net.history.push((A, net.now - 1.0, f64::MAX));
    net.nodes[b].node.link.send_data(&data(50, 141));
    let limit = net.nodes[b].node.link.busy_timeout_s(0);
    let t = net.run_until(900.0, |n| n.has(b, &LinkEvent::Disconnected));
    assert!(t > limit && t < limit + 5.0, "took {t:.0} s, limit {limit:.0} s");
}

#[test]
fn direct_ends_retrying_together_recover() {
    // Two stations that hear each other, no digipeater. Once connected and quiet, both
    // get 2 kB at the same instant, so both start a burst of their own and some of
    // these collide. Then both retry after the same wait, give or take the back-off,
    // and without giving way to each other every retry collides until the session drops.
    let mut dropped = Vec::new();
    // Without giving way, 4 of these 15 seeds drop (0, 2, 12, 14).
    for seed in 0..15 {
        let mut net = Net::new(seed, 0.0, 0.0);
        net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
        net.run_until(60.0, |n| n.both_connected());
        net.run_until(60.0, |n| n.now > 15.0);
        let (a, b) = (data(2000, seed), data(2000, seed + 1));
        net.nodes[A].node.link.send_data(&a);
        net.nodes[1].node.link.send_data(&b);
        let lost = |n: &Net| n.has(A, &LinkEvent::Disconnected) || n.has(1, &LinkEvent::Disconnected);
        net.run_until(300.0, |n| lost(n) || (n.nodes[A].host_rx == b && n.nodes[1].host_rx == a));
        if lost(&net) {
            dropped.push(seed);
        }
    }
    assert!(dropped.is_empty(), "sessions dropped for seeds {dropped:?}");
}

#[test]
fn connecting_caller_ignores_data_with_its_session_id() {
    // Issue #27: a Data burst carries no call signs. One with our session id while we
    // are connecting belongs to another session that happens to use the same id.
    let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
    let mut c = Link::new(modem, LinkConfig::default(), 7);
    c.connect(0.0, "LA5CCC", "LA9ZZZ", &[]).unwrap();
    let State::Connecting { session, .. } = &c.state else { panic!("not connecting") };
    let header = Header { kind: Kind::Data, session: *session, ..Header::default() };
    let payload = vec![Some(vec![0; Modem::payload_bytes(0)])];
    c.on_burst(0.5, &RxBurst { header, payload, snr_db: 30.0, end_sample: 0, level: Default::default() });
    assert!(!c.is_connected());
    assert!(c.take_events().iter().all(|e| !matches!(e, LinkEvent::Connected { .. } | LinkEvent::Data(_))));
}

#[test]
fn caller_does_not_pick_a_session_id_heard_recently() {
    let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
    let keep = Link::new(modem.clone(), LinkConfig::default(), 0).heard_id_s();
    // Every id but 77 heard at t = 100: each caller picks 77.
    for seed in 0..20 {
        let mut c = Link::new(modem.clone(), LinkConfig::default(), seed);
        for id in (1..=255).filter(|&id| id != 77) {
            c.on_burst(100.0, &probe(id, false, 0));
        }
        c.connect(100.0 + keep - 1.0, "LA5CCC", "LA9ZZZ", &[]).unwrap();
        let State::Connecting { session, .. } = &c.state else { panic!("not connecting") };
        assert_eq!(*session, 77, "seed {seed}");
    }
    // Once that time has passed, the ids are free again.
    let picked: std::collections::HashSet<u8> = (0..20)
        .map(|seed| {
            let mut c = Link::new(modem.clone(), LinkConfig::default(), seed);
            for id in (1..=255).filter(|&id| id != 77) {
                c.on_burst(100.0, &probe(id, false, 0));
            }
            c.connect(100.0 + keep + 1.0, "LA5CCC", "LA9ZZZ", &[]).unwrap();
            let State::Connecting { session, .. } = &c.state else { panic!("not connecting") };
            *session
        })
        .collect();
    assert!(picked.len() > 1, "still avoiding expired ids: {picked:?}");
    // Every id heard: still connects, with a non-zero id.
    let mut c = Link::new(modem, LinkConfig::default(), 3);
    for id in 0..=255 {
        c.on_burst(100.0, &probe(id, false, 0));
    }
    c.connect(101.0, "LA5CCC", "LA9ZZZ", &[]).unwrap();
    let State::Connecting { session, .. } = &c.state else { panic!("not connecting") };
    assert_ne!(*session, 0);
}

#[test]
fn caller_does_not_join_a_session_it_hears() {
    // Issue #27. A and B run a session. C hears both and calls LA9ZZZ, who is absent.
    // C's random generator is set so that a random pick alone would give it A and B's
    // id: C would then take their Data as LA9ZZZ's answer and break their session.
    let mut net = Net::new(5, 0.0, 0.0);
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    let State::Connecting { session: id, .. } = net.nodes[A].node.link.state else { panic!("not connecting") };
    let mut peer = Peer {
        name: "C".into(),
        node: Node::new(Link::new(net.modem.clone(), LinkConfig::default(), 77)),
        tx: None,
        host_rx: Vec::new(),
        events: Vec::new(),
        dead: false,
    };
    peer.node.link.set_mycalls(vec!["LA5CCC".into()]);
    // C goes before B, which `Net` takes to be the last station.
    net.nodes.insert(1, peer);
    let (c, b) = (1, 2);
    assert_eq!((net.nodes[A].name.as_str(), net.nodes[b].name.as_str(), net.b()), ("A", "B", b));
    net.hears = vec![vec![false, true, true], vec![true, false, true], vec![true, true, false]];

    net.run_until(60.0, |n| n.both_connected());
    let (to_b, to_a) = (data(3000, 1), data(3000, 2));
    net.nodes[A].node.link.send_data(&to_b);
    net.nodes[b].node.link.send_data(&to_a);
    // Every poll draws from the generator, so set it just before the pick. The seed is
    // one with which a caller that has heard nothing picks A and B's id.
    let unaware_pick = |seed| {
        let mut fresh = Link::new(net.modem.clone(), LinkConfig::default(), 0);
        fresh.rng = Prng::new(seed);
        fresh.connect(net.now, "LA5CCC", "LA9ZZZ", &[]).unwrap();
        let State::Connecting { session, .. } = fresh.state else { panic!("not connecting") };
        session
    };
    let seed = (0..10_000).find(|&s| unaware_pick(s) == id).expect("no seed picks the id");
    net.nodes[c].node.link.rng = Prng::new(seed);
    net.nodes[c].node.link.connect(net.now, "LA5CCC", "LA9ZZZ", &[]).unwrap();
    let State::Connecting { session, .. } = net.nodes[c].node.link.state else { panic!("not connecting") };
    assert_ne!(session, id, "C took the id of the session it hears");

    net.run_until(400.0, |n| n.nodes[A].host_rx.len() >= to_a.len() && n.nodes[b].host_rx.len() >= to_b.len());
    assert_eq!(net.nodes[A].host_rx, to_a);
    assert_eq!(net.nodes[b].host_rx, to_b);
    assert!(!net.has(c, &LinkEvent::Connected { source: "LA5CCC".into(), destination: "LA9ZZZ".into() }));
    assert!(net.nodes[c].host_rx.is_empty(), "C received A and B's data");
}

#[test]
fn call_colliding_with_a_session_on_air_does_not_join_it() {
    // Issue #27: A and B are exchanging data. C hears both and calls LA9ZZZ, which is
    // not there, under the session id A and B use (1 call in 255). C must not report
    // CONNECTED or take their data, and A and B must not take C's bursts as each other's.
    let mut net = Net::new(5, 0.0, 0.0);
    let peer = Peer {
        name: "C".into(),
        node: Node::new(Link::new(net.modem.clone(), LinkConfig::default(), 77)),
        tx: None,
        host_rx: Vec::new(),
        events: Vec::new(),
        dead: false,
    };
    // Before B, so that `net.b()` is still B.
    net.nodes.insert(1, peer);
    let (c, b) = (1, 2);
    assert_eq!(net.b(), b);
    net.nodes[c].node.link.set_mycalls(vec!["LA5CCC".into()]);
    net.hears = (0..3).map(|to| (0..3).map(|from| to != from).collect()).collect();

    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.both_connected());
    let State::Connected(s) = &net.nodes[A].node.link.state else { panic!("A not connected") };
    let id = s.id;
    let (to_b, to_a) = (data(3000, 1), data(3000, 2));
    net.nodes[A].node.link.send_data(&to_b);
    net.nodes[b].node.link.send_data(&to_a);
    let link = &mut net.nodes[c].node.link;
    link.connect(net.now, "LA5CCC", "LA9ZZZ", &[]).unwrap();
    let State::Connecting { session, .. } = &mut link.state else { panic!("C not connecting") };
    *session = id;

    net.run_until(400.0, |n| n.nodes[A].host_rx.len() >= to_a.len() && n.nodes[b].host_rx.len() >= to_b.len());
    assert!(!net.nodes[c].events.iter().any(|e| matches!(e, LinkEvent::Connected { .. })), "C joined session {id}");
    assert!(net.nodes[c].host_rx.is_empty(), "C took {} bytes of session {id}", net.nodes[c].host_rx.len());
    assert_eq!(net.nodes[A].host_rx, to_a);
    assert_eq!(net.nodes[b].host_rx, to_b);
    assert!(!net.has(A, &LinkEvent::Disconnected) && !net.has(b, &LinkEvent::Disconnected));
}

/// Runs `link` alone on a quiet channel from `*t` until it transmits, then finishes the
/// burst. Returns its kind.
fn next_tx(link: &mut Link, t: &mut f64) -> Kind {
    let limit = *t + 300.0;
    loop {
        assert!(*t < limit, "nothing sent by t={t:.1}");
        if let Some(b) = link.poll(*t, false) {
            *t += link.modem.burst_seconds(b.header.level, b.payload.len()) as f64;
            link.tx_done(*t);
            return b.header.kind;
        }
        *t += DT;
    }
}

#[test]
fn called_station_retries_connect_ack_until_caller_answers() {
    // The called station sends its ConnectAck and then its banner. If the caller never
    // answers, it may not have heard the ConnectAck (a caller connects on nothing else),
    // so the retry is the ConnectAck again, followed by the banner. Once the caller has
    // sent anything, retries are Data.
    let mut b = called_station();
    b.on_burst(0.0, &connect_burst(Kind::ConnectReq, 9, ("LA1AAA", "LA2BBB-1"), &[], false, 0));
    assert!(b.is_connected());
    b.send_data(&data(500, 1));
    let mut t = 0.0;
    let sent: Vec<Kind> = (0..4).map(|_| next_tx(&mut b, &mut t)).collect();
    assert_eq!(sent, [Kind::ConnectAck, Kind::Data, Kind::ConnectAck, Kind::Data]);

    b.on_burst(t, &probe(9, false, 0));
    let sent: Vec<Kind> = (0..2).map(|_| next_tx(&mut b, &mut t)).collect();
    assert_eq!(sent, [Kind::Data, Kind::Data], "answer, then its retry");

    // Once the caller would have stopped retrying its ConnectReq, a caller that missed
    // the ConnectAck has given up: no point sending it again.
    let mut b = called_station();
    b.on_burst(0.0, &connect_burst(Kind::ConnectReq, 9, ("LA1AAA", "LA2BBB-1"), &[], false, 0));
    let mut t = 0.0;
    assert_eq!(next_tx(&mut b, &mut t), Kind::ConnectAck);
    t = b.connect_window_s(0) + 0.1;
    b.send_data(&data(500, 1));
    let sent: Vec<Kind> = (0..2).map(|_| next_tx(&mut b, &mut t)).collect();
    assert_eq!(sent, [Kind::Data, Kind::Data], "after the caller's connect window");
}

#[test]
fn lost_connect_ack_rarely_leaves_a_half_open_session() {
    // The caller connects only on the ConnectAck. When that is lost, the called station
    // must get it through again before the caller gives up, also while it is sending
    // its banner: otherwise the caller reports DISCONNECTED and the called station
    // stays connected to nobody. It cannot always win: with every burst at 25 % loss,
    // the caller's last ConnectReqs and the called station's resent ConnectAcks are all
    // lost now and then (about 1 seed in 300). Without the resent ConnectAck, 10 in 300.
    // Direct sessions only: through a digipeater at this loss, about 1 seed in 4 ends
    // half open with or without the resend, and without this change too.
    let mut half_open = Vec::new();
    for seed in 0..100 {
        let mut net = Net::new(seed, 0.25, 0.0);
        let b = net.b();
        net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
        let gave_up = |n: &Net| n.has(A, &LinkEvent::Disconnected);
        net.run_until(120.0, |n| n.nodes[n.b()].node.link.is_connected() || gave_up(n));
        if gave_up(&net) {
            continue; // B never heard a request: nothing half open.
        }
        let banner = data(500, seed);
        net.nodes[b].node.link.send_data(&banner);
        net.run_until(400.0, |n| n.nodes[A].host_rx.len() >= banner.len() || gave_up(n));
        if gave_up(&net) && net.nodes[b].node.link.is_connected() {
            half_open.push(seed);
        }
    }
    assert!(half_open.len() <= 1, "caller gave up, called station connected, for seeds {half_open:?}");
}

#[test]
fn called_station_answers_only_its_callers_repeated_request() {
    // A repeated ConnectReq means the caller missed our ConnectAck. One under our
    // session id that names other stations or another path is another caller that
    // happens to use the same id: answering it would step on our own session.
    let mut b = called_station();
    let ours = ("LA1AAA", "LA2BBB-1");
    b.on_burst(0.0, &connect_burst(Kind::ConnectReq, 9, ours, &[], false, 0));
    let mut t = 0.0;
    assert_eq!(next_tx(&mut b, &mut t), Kind::ConnectAck);
    for (calls, digis) in [(("LA5CCC", "LA9ZZZ"), &[][..]), (("LA5CCC", "LA2BBB-1"), &[]), (ours, &["LA3DDD"])] {
        b.on_burst(t, &connect_burst(Kind::ConnectReq, 9, calls, digis, false, 0));
        assert!(b.poll(t + 1.0, false).is_none(), "answered {calls:?} via {digis:?}");
    }
    b.on_burst(t, &connect_burst(Kind::ConnectReq, 9, ours, &[], false, 0));
    t += 1.0;
    assert_eq!(next_tx(&mut b, &mut t), Kind::ConnectAck);
}

#[test]
fn caller_gives_up_within_the_called_stations_connect_window() {
    // The called station resends its ConnectAck only while the caller could still be
    // retrying its ConnectReq. That window is computed on the called station's side, so
    // check it against a caller that really retries until it gives up, nobody answering.
    for digis in [&[][..], &["LA3DDD"], &["LA3DDD", "LA4EEE-2"]] {
        for seed in 0..5 {
            let mut net = Net::chain(seed, 0.0, digis, LinkConfig::default());
            let b = net.b();
            net.nodes[b].dead = true;
            net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
            let gave_up = net.run_until(300.0, |n| n.nodes[A].node.link.is_idle());
            let window = net.nodes[A].node.link.connect_window_s(digis.len() as u8);
            assert!(
                gave_up <= window,
                "via {digis:?}, seed {seed}: gave up after {gave_up:.1} s, window {window:.1} s"
            );
        }
    }
    // The called station's own bursts hold the caller's retries back: the channel is
    // busy with one of its longest bursts just as each retry is due.
    let mut net = Net::new(0, 0.0, 0.0);
    let b = net.b();
    net.nodes[b].dead = true;
    let longest = net.nodes[A].node.link.max_burst_s;
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    let mut held = 0;
    while !net.nodes[A].node.link.is_idle() {
        assert!(net.now < 300.0, "caller never gave up");
        if let State::Connecting { attempts, next_tx, sending: false, .. } = net.nodes[A].node.link.state {
            if attempts > held && net.now >= next_tx - 0.05 {
                held = attempts;
                net.history.push((b, net.now - 0.1, net.now - 0.1 + longest));
            }
        }
        net.step();
    }
    let window = net.nodes[A].node.link.connect_window_s(0);
    assert!(held >= 4, "held back {held} retries");
    assert!(net.now <= window, "held back: gave up after {:.1} s, window {window:.1} s", net.now);
}

#[test]
fn another_callers_request_does_not_disturb_a_data_retry() {
    // A called station is waiting to resend unanswered data when another caller's
    // ConnectReq arrives under the same session id. Its retry must go out as planned,
    // not be replaced by a ConnectAck for a request that is not its caller's.
    let mut b = called_station();
    b.on_burst(0.0, &connect_burst(Kind::ConnectReq, 9, ("LA1AAA", "LA2BBB-1"), &[], false, 0));
    let mut t = 0.0;
    assert_eq!(next_tx(&mut b, &mut t), Kind::ConnectAck);
    b.on_burst(t, &probe(9, false, 0));
    b.send_data(&data(500, 1));
    assert_eq!(next_tx(&mut b, &mut t), Kind::Data);
    let State::Connected(s) = &b.state else { panic!("not connected") };
    let Phase::Wait { deadline, retry: true } = s.phase else { panic!("not waiting to retry: {:?}", s.phase) };
    b.on_burst((t + deadline) / 2.0, &connect_burst(Kind::ConnectReq, 9, ("LA5CCC", "LA9ZZZ"), &[], false, 0));
    t = (t + deadline) / 2.0;
    assert_eq!(next_tx(&mut b, &mut t), Kind::Data);
    assert_eq!(b.stats.retries, 1, "sent as a retry");
    assert!(t >= deadline, "retried at {t:.2} s, before {deadline:.2} s");
}

// ------------------------------------------------------------ ABORT's Disc (#58)

/// Bursts of `kind` that station `who`'s own link sent from `since` on: (start, end).
fn sent_by(net: &Net, who: usize, kind: Kind, since: f64) -> Vec<(f64, f64)> {
    net.sent.iter().filter(|s| s.0 == who && s.3 == kind && s.1 >= since).map(|s| (s.1, s.2)).collect()
}

/// How many times station `who`'s host has been told DISCONNECTED.
fn disconnects(net: &Net, who: usize) -> usize {
    net.nodes[who].events.iter().filter(|e| **e == LinkEvent::Disconnected).count()
}

fn overlaps(a: (f64, f64), b: (f64, f64)) -> bool {
    a.0 < b.1 && b.0 < a.1
}

/// Like [`Net::run_until`], but returns whether `done` came within `limit` seconds
/// instead of failing, so that a test can say what went wrong.
fn run_for(net: &mut Net, limit: f64, mut done: impl FnMut(&Net) -> bool) -> bool {
    let start = net.now;
    while !done(net) {
        if net.now - start >= limit {
            return false;
        }
        net.step();
    }
    true
}

/// A session between A and B through `digis`, connected, then quiet: past the time the
/// caller could still be retrying its ConnectReq, and nothing on the air. Keepalives
/// are far off, so none goes out during a test.
fn quiet_session(seed: u32, digis: &[&str]) -> Net {
    quiet_session_with(seed, digis, LinkConfig::default())
}

/// As `quiet_session`, with `cfg` (its `keepalive_s` is set to 600 s).
fn quiet_session_with(seed: u32, digis: &[&str], cfg: LinkConfig) -> Net {
    let cfg = LinkConfig { keepalive_s: 600.0, ..cfg };
    let mut net = Net::chain(seed, 0.0, digis, cfg);
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    let window = net.nodes[A].node.link.connect_window_s(digis.len() as u8);
    let quiet = net.now + window.max(20.0);
    net.run_until(window + 60.0, |n| n.now >= quiet && n.nodes.iter().all(|x| x.tx.is_none()));
    net
}

/// The session id A uses.
fn session_id(net: &Net) -> u8 {
    let State::Connected(s) = &net.nodes[A].node.link.state else { panic!("A is not connected") };
    s.id
}

/// ABORT while A owes B an answer: A has decoded B's burst, and its answer is due
/// `turnaround_s` later. The Disc goes in that slot, as the answer would have, and B
/// answers DiscAck and closes. A's host is told DISCONNECTED once, at the ABORT; a
/// second ABORT or a DISCONNECT then changes nothing.
fn abort_after_the_peers_burst(digis: &[&str], seed: u32) {
    let mut net = quiet_session(seed, digis);
    let b = net.b();
    net.nodes[b].node.link.send_data(&data(100, seed));
    let owes_answer = |n: &Net| {
        matches!(&n.nodes[A].node.link.state,
            State::Connected(s) if matches!(s.phase, Phase::Respond { initiated: false, .. }))
    };
    net.run_until(120.0, owes_answer);
    let State::Connected(s) = &net.nodes[A].node.link.state else { unreachable!() };
    let Phase::Respond { at: slot, .. } = s.phase else { unreachable!() };
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.step();
    assert_eq!(disconnects(&net, A), 1, "via {digis:?}: A's host not told at once");
    let t = net.run_until(30.0, |n| n.has(b, &LinkEvent::Disconnected));
    let discs = sent_by(&net, A, Kind::Disc, t0);
    assert!(!discs.is_empty(), "via {digis:?}: no Disc");
    assert!(
        (discs[0].0 - slot).abs() <= 2.0 * DT,
        "via {digis:?}: Disc at {:.2}, A's answer slot at {slot:.2} (ABORT at {t0:.2})",
        discs[0].0
    );
    assert_eq!(discs.len(), 1, "via {digis:?}: A's Discs {discs:?}, but B answered the first");
    assert!(t < 10.0, "via {digis:?}: B took {t:.1} s to close");
    // Settle, then a second ABORT and a DISCONNECT: no second DISCONNECTED, nothing sent.
    let settled = net.now + 30.0;
    net.run_until(60.0, |n| n.now >= settled);
    let t1 = net.now;
    net.nodes[A].node.link.abort(t1);
    net.nodes[A].node.link.disconnect(t1);
    net.run_until(30.0, |n| n.now >= t1 + 20.0);
    assert!(!net.sent.iter().any(|s| s.0 == A && s.1 >= t1), "via {digis:?}: A sent after a second ABORT");
    assert_eq!(disconnects(&net, A), 1, "via {digis:?}: A's host told DISCONNECTED twice");
    assert_eq!(disconnects(&net, b), 1, "via {digis:?}");
    assert!(!net.nodes[A].node.link.is_connected() && !net.nodes[b].node.link.is_connected());
}

#[test]
fn abort_after_the_peers_burst_sends_the_disc_in_our_answer_slot() {
    abort_after_the_peers_burst(&[], 201);
}

#[test]
fn abort_after_the_peers_burst_sends_the_disc_in_our_answer_slot_through_a_digipeater() {
    abort_after_the_peers_burst(&["LA3DDD"], 202);
}

/// ABORT just after A's own burst, while B is about to answer it (direct). The Disc
/// must not go into B's answer, or B, transmitting, never hears it and only drops the
/// session after its retries. It follows the answer in A's slot. If `lose_reply`, A
/// hears B's answer as a busy channel but cannot decode it: the Disc then goes once
/// the channel is clear and A's wait for the answer has run out.
fn abort_just_after_our_burst(seed: u32, lose_reply: bool) {
    let mut net = quiet_session(seed, &[]);
    let b = net.b();
    let turnaround = net.nodes[A].node.link.config().turnaround_s;
    net.nodes[A].node.link.send_data(&data(100, seed));
    net.run_until(60.0, |n| n.nodes[A].tx.is_some());
    net.run_until(60.0, |n| n.nodes[A].tx.is_none());
    if lose_reply {
        net.lose_next = Some((b, A));
    }
    let wait_until = match &net.nodes[A].node.link.state {
        State::Connected(s) => match s.phase {
            Phase::Wait { deadline, .. } => deadline,
            _ => panic!("A is not waiting for B's answer"),
        },
        _ => panic!("A is not connected"),
    };
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    let what = if lose_reply { "lost answer" } else { "answer heard" };
    let b_closed = run_for(&mut net, 10.0, |n| n.has(b, &LinkEvent::Disconnected));
    let reply = *sent_by(&net, b, Kind::Data, t0).first().unwrap_or_else(|| panic!("{what}: B did not answer"));
    let discs = sent_by(&net, A, Kind::Disc, t0);
    assert!(!discs.is_empty(), "{what}: no Disc");
    for d in &discs {
        assert!(!overlaps(*d, reply), "{what}: Disc at {d:.2?} into B's answer at {reply:.2?}");
    }
    if lose_reply {
        assert!(discs[0].0 >= reply.1, "{what}: Disc at {:.2} before B's answer ended at {:.2}", discs[0].0, reply.1);
        assert!(
            discs[0].0 >= wait_until - DT,
            "{what}: Disc at {:.2}, before A's wait for the answer ran out at {wait_until:.2}",
            discs[0].0
        );
    } else {
        // B's answer reaches A 0.02 s after it ends (the fake channel's delay).
        let slot = reply.1 + 0.02 + turnaround;
        assert!(
            discs[0].0 >= slot - 2.0 * DT && discs[0].0 <= slot + 3.0 * DT,
            "{what}: Disc at {:.2}, A's answer slot at {slot:.2}",
            discs[0].0
        );
    }
    assert!(b_closed, "{what}: B still connected 10 s after A's ABORT");
    let settled = net.now + 30.0;
    net.run_until(60.0, |n| n.now >= settled);
    assert_eq!((disconnects(&net, A), disconnects(&net, b)), (1, 1), "{what}");
}

#[test]
fn abort_just_after_our_burst_waits_for_the_peers_answer() {
    abort_just_after_our_burst(203, false);
}

#[test]
fn abort_just_after_our_burst_whose_answer_is_lost_sends_the_disc_on_a_clear_channel() {
    abort_just_after_our_burst(204, true);
}

/// CONNECT right after ABORT is accepted, but waits until the Disc exchange is over:
/// the ConnectReq goes out only after B's DiscAck, then both ends connect on a new
/// session id. A's host gets DISCONNECTED for the old session first. B misses A's
/// first Disc, so the exchange takes a repeat: a CONNECT that went out before it would
/// find B still in the old session.
///
/// Longest wait, on a quiet channel, with the Disc never answered (default config,
/// narrow mode). Each Disc lasts 0.408 s and is followed by a wait for its DiscAck
/// (`response_wait`: 1.2 s direct, 6.876 s through one digipeater) and, before the
/// next Disc, up to RETRY_BACKOFF_S × `retry_spread` of backoff (0.4 s direct, 1.2 s
/// through one digipeater). After the 4th Disc's wait the abort ends.
/// - ABORT on a clear channel: the first Disc goes at once.
///   Direct: 4 × 0.408 + 4 × 1.2 + 3 × 0.4 ≈ 7.6 s; through one digipeater:
///   4 × 0.408 + 4 × 6.876 + 3 × 1.2 ≈ 32.7 s.
/// - ABORT while our burst is on the air: first the rest of that burst (up to the
///   longest burst, 4.368 s), then its answer wait (1.2 s direct; through digipeaters
///   `response_wait` for that burst or, for one we started, the longer `retry_wait`)
///   and one backoff. Direct: up to 4.368 + 1.2 + 0.4 + 7.6 ≈ 13.6 s, or about 9.2 s
///   for an ABORT during the answer wait.
///
/// A busy channel, or a peer that cannot hear us and keeps resending, lengthens the
/// waits, but the abort ends at its deadline from the ABORT at the latest: the route's
/// busy timeout (the larger of `busy_timeout_s`, 60 s by default, and a floor for the
/// route, which is below 60 s direct (6.8 s) and through one digipeater (47.3 s), but
/// 78.3 s through two), or the longest case above if that is longer.
#[test]
fn connect_right_after_abort_waits_for_the_disc_exchange() {
    let mut net = quiet_session(205, &[]);
    let b = net.b();
    let old = session_id(&net);
    let ev0 = net.nodes[A].events.len();
    let t0 = net.now;
    net.lose_next = Some((A, b));
    net.nodes[A].node.link.abort(t0);
    net.nodes[A].node.link.connect(t0, "LA1AAA", "LA2BBB-1", &[]).expect("CONNECT accepted after ABORT");
    let connected = run_for(&mut net, 60.0, |n| n.both_connected());
    let req = *sent_by(&net, A, Kind::ConnectReq, t0).first().expect("A sent no ConnectReq");
    let ack = sent_by(&net, b, Kind::DiscAck, t0).first().map(|a| a.1);
    assert!(
        ack.is_some_and(|end| req.0 >= end),
        "ConnectReq at {:.2}, but B's DiscAck ended at {ack:.2?}: A's Discs at {:.2?}",
        req.0,
        sent_by(&net, A, Kind::Disc, t0)
    );
    assert!(connected, "not connected 60 s after the CONNECT");
    assert_ne!(session_id(&net), old, "the new session reuses the aborted session's id");
    let events = &net.nodes[A].events[ev0..];
    let first_connected = events.iter().position(|e| matches!(e, LinkEvent::Connected { .. }));
    let first_disconnected = events.iter().position(|e| *e == LinkEvent::Disconnected);
    assert!(first_disconnected.is_some() && first_disconnected < first_connected, "{events:?}");
}

/// ABORT while a CONNECT waits for the Disc repeats drops the CONNECT: it is never
/// sent, and A's host gets DISCONNECTED for it, as for an ABORT while connecting. The
/// Disc repeats go on (B is switched off, so all 4 go out).
#[test]
fn abort_drops_a_connect_waiting_for_the_disc_repeats() {
    let mut net = quiet_session(206, &[]);
    let b = net.b();
    net.nodes[b].dead = true;
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.nodes[A].node.link.connect(t0, "LA1AAA", "LA2BBB-1", &[]).expect("CONNECT accepted after ABORT");
    // After the first Disc, while A waits for its DiscAck.
    net.run_until(30.0, |n| n.now >= t0 + 1.5);
    net.nodes[A].node.link.abort(net.now);
    net.run_until(60.0, |n| n.now >= t0 + 40.0);
    let reqs = sent_by(&net, A, Kind::ConnectReq, t0);
    assert!(reqs.is_empty(), "the CONNECT went out during the Disc repeats: ConnectReqs at {reqs:.2?}");
    assert_eq!(disconnects(&net, A), 2, "one DISCONNECTED for the session, one for the dropped CONNECT");
    assert_eq!(sent_by(&net, A, Kind::Disc, t0).len(), 4, "the Disc repeats were cut short");
}

/// An ABORT whose Disc is never answered (B switched off) sends 4 Discs, spaced as
/// DISCONNECT's are: each waits `response_wait` for a DiscAck, then up to
/// RETRY_BACKOFF_S more. Then A sends nothing more, and its host hears DISCONNECTED
/// only the once.
#[test]
fn an_unanswered_abort_disc_is_sent_four_times() {
    let mut net = quiet_session(207, &[]);
    let b = net.b();
    net.nodes[b].dead = true;
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.run_until(60.0, |n| n.now >= t0 + 40.0);
    let discs = sent_by(&net, A, Kind::Disc, t0);
    assert_eq!(discs.len(), 4, "A's Discs at {discs:.2?}");
    let link = &net.nodes[A].node.link;
    let disc_s = link.modem.burst_seconds(0, 0) as f64;
    let lo = disc_s + link.response_wait(0, disc_s);
    let hi = lo + RETRY_BACKOFF_S;
    for w in discs.windows(2) {
        let gap = w[1].0 - w[0].0;
        assert!(gap >= lo - DT && gap <= hi + 3.0 * DT, "Discs {gap:.2} s apart, want {lo:.2}..{hi:.2}: {discs:.2?}");
    }
    // DISCONNECT, for comparison, also sends its Disc 4 times.
    let by_disconnect = unanswered_bursts(&[], 207, A, Kind::Disc, |l, t| l.disconnect(t));
    assert_eq!(by_disconnect.len(), discs.len(), "DISCONNECT's Discs at {by_disconnect:.2?}");
    assert!(!net.sent.iter().any(|s| s.0 == A && s.1 > discs[3].1), "A sent after its last Disc");
    assert_eq!(disconnects(&net, A), 1);
}

/// ABORT on a channel jammed for longer than the busy timeout: the Disc gives way to the
/// carrier like any burst we start, and the abort ends at the busy timeout. No Disc
/// had gone, so A sends the one parting Disc a session dropped at the busy timeout
/// gets, once the carrier ends (within the busy timeout again), and B closes on it.
#[test]
fn a_jammed_channel_ends_the_abort_and_its_disc_follows_the_carrier() {
    let mut net = quiet_session(208, &[]);
    let b = net.b();
    let t0 = net.now;
    let timeout = net.nodes[A].node.link.busy_timeout_s(0);
    let end = t0 + timeout + 5.0;
    // B's carrier, from just before the ABORT: A hears the channel busy, B hears nothing.
    net.history.push((b, t0 - 0.2, end));
    net.nodes[A].node.link.abort(t0);
    net.run_until(200.0, |n| n.now >= t0 + timeout - 1.0);
    assert!(matches!(net.nodes[A].node.link.state, State::Aborting(_)), "the abort ended before the busy timeout");
    net.run_until(200.0, |n| n.now >= t0 + timeout + 1.0);
    assert!(matches!(net.nodes[A].node.link.state, State::Idle), "the abort did not end at the busy timeout");
    net.run_until(200.0, |n| n.now >= end + 30.0);
    let discs = sent_by(&net, A, Kind::Disc, t0);
    let into: Vec<_> = discs.iter().filter(|d| d.0 < end).collect();
    assert!(into.is_empty(), "Disc into the carrier (until {end:.2}): {into:.2?}");
    assert_eq!(discs.len(), 1, "A sent no Disc, or more than one, after the carrier: {discs:.2?}");
    assert_eq!((disconnects(&net, A), disconnects(&net, b)), (1, 1));
}

/// While its Disc is being repeated, A does not take another station's ConnectReq, as it
/// would not while connected: the repeats go on, and no session starts.
#[test]
fn a_connect_request_is_ignored_while_the_abort_disc_is_repeated() {
    let mut net = quiet_session(209, &[]);
    let b = net.b();
    net.nodes[b].dead = true;
    net.nodes[A].node.link.set_listen(true);
    let old = session_id(&net);
    let ev0 = net.nodes[A].events.len();
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.run_until(30.0, |n| n.now >= t0 + 1.0);
    let other = old.wrapping_add(1).max(1);
    let req = connect_burst(Kind::ConnectReq, other, ("LA9ZZZ", "LA1AAA"), &[], false, 0);
    let now = net.now;
    net.nodes[A].node.link.on_burst(now, &req);
    net.run_until(60.0, |n| n.now >= t0 + 40.0);
    let events = &net.nodes[A].events[ev0..];
    let connected = events.iter().any(|e| matches!(e, LinkEvent::Connected { .. }));
    assert!(!connected, "A took LA9ZZZ's ConnectReq while repeating its Disc: {events:?}");
    assert!(sent_by(&net, A, Kind::ConnectAck, t0).is_empty(), "A answered LA9ZZZ's ConnectReq");
    assert_eq!(sent_by(&net, A, Kind::Disc, t0).len(), 4, "the Disc repeats were cut short");
}

// ------------------------------------------------------------ ABORT's Disc: review round

/// Where A's abort has just ended by B's DiscAck, with a CONNECT waiting for it, and
/// before A's next `poll`: the DiscAck is handed straight to A's link. Returns the net
/// and the time.
fn abort_ended_with_a_connect_waiting(seed: u32) -> (Net, f64) {
    let mut net = quiet_session(seed, &[]);
    let b = net.b();
    net.nodes[b].dead = true;
    net.nodes[A].node.link.set_listen(true);
    let id = session_id(&net);
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.nodes[A].node.link.connect(t0, "LA1AAA", "LA2BBB-1", &[]).expect("CONNECT accepted after ABORT");
    net.run_until(30.0, |n| n.now >= t0 + 0.5 && n.nodes[A].tx.is_none());
    assert_eq!(sent_by(&net, A, Kind::Disc, t0).len(), 1, "A's first Disc");
    let now = net.now;
    let disc_ack =
        RxBurst { header: Header { kind: Kind::DiscAck, ..probe(id, false, 0).header }, ..probe(id, false, 0) };
    net.nodes[A].node.link.on_burst(now, &disc_ack);
    (net, now)
}

// The abort's end starts the waiting CONNECT at once: there is no moment, before the
// next `poll`, where A looks idle with a CONNECT still waiting. The three tests below
// act in that moment.

/// Another station's ConnectReq, decoded in the same batch of audio as the DiscAck,
/// does not start a session: the waiting CONNECT has started.
#[test]
fn a_connect_request_as_the_abort_ends_does_not_start_a_session() {
    let (mut net, now) = abort_ended_with_a_connect_waiting(301);
    let link = &mut net.nodes[A].node.link;
    let ev0 = link.events.len();
    link.on_burst(now, &connect_burst(Kind::ConnectReq, 77, ("LA9ZZZ", "LA1AAA"), &[], false, 0));
    let took = link.events[ev0..].iter().any(|e| matches!(e, LinkEvent::Connected { .. }));
    assert!(!took, "LA9ZZZ's ConnectReq, decoded as the abort ended, started a session");
    assert!(matches!(link.state, State::Connecting { .. }), "the waiting CONNECT did not start as the abort ended");
}

/// The host's DISCONNECT as the abort ends drops the waiting CONNECT, with
/// DISCONNECTED, and it never goes out.
#[test]
fn disconnect_as_the_abort_ends_drops_the_waiting_connect() {
    let (mut net, now) = abort_ended_with_a_connect_waiting(302);
    let before =
        disconnects(&net, A) + net.nodes[A].node.link.events.iter().filter(|e| **e == LinkEvent::Disconnected).count();
    net.nodes[A].node.link.disconnect(now);
    let after =
        disconnects(&net, A) + net.nodes[A].node.link.events.iter().filter(|e| **e == LinkEvent::Disconnected).count();
    assert_eq!(after, before + 1, "DISCONNECT as the abort ended did not drop the waiting CONNECT");
    let t = net.now;
    net.run_until(30.0, |n| n.now >= t + 20.0);
    let reqs = sent_by(&net, A, Kind::ConnectReq, t);
    assert!(reqs.is_empty(), "the CONNECT went out after the host's DISCONNECT: {reqs:.2?}");
}

/// A second CONNECT as the abort ends fails: the first one is under way.
#[test]
fn a_second_connect_as_the_abort_ends_fails() {
    let (mut net, now) = abort_ended_with_a_connect_waiting(303);
    let second = net.nodes[A].node.link.connect(now, "LA1AAA", "LA2BBB-1", &[]);
    assert!(second.is_err(), "a second CONNECT was accepted while the first one waits");
}

/// Every Disc of an abort counts toward the 4, the ones sent in an answer slot too.
/// Here B never hears A, so it keeps retrying its data, and A answers each retry it
/// decodes: still 4 Discs at most, then the abort ends.
#[test]
fn abort_discs_in_answer_slots_count_toward_the_four() {
    let mut net = quiet_session(304, &[]);
    let b = net.b();
    net.set_hears(|to, from| to != from && !(to == b && from == A));
    net.nodes[b].node.link.send_data(&data(100, 304));
    // A hears B's burst: a Disc on a clear channel now would go into it.
    net.run_until(30.0, |n| n.nodes[b].tx.is_some() && n.busy(A));
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.run_until(120.0, |n| n.now >= t0 + 90.0);
    let discs = sent_by(&net, A, Kind::Disc, t0);
    let b_bursts = sent_by(&net, b, Kind::Data, t0).len();
    assert!(b_bursts > 4, "B sent only {b_bursts} bursts: the test needs more than 4");
    assert_eq!(discs.len(), 4, "A's Discs at {discs:.2?}, answering {b_bursts} bursts of B");
    assert!(matches!(net.nodes[A].node.link.state, State::Idle), "the abort did not end after 4 Discs");
}

/// ABORT during a DISCONNECT keeps the Discs DISCONNECT has already sent, including
/// one on the air at the ABORT: never more than 4 in all. B is switched off.
#[test]
fn abort_during_a_disconnect_sends_no_more_than_four_discs_in_all() {
    let mut net = quiet_session(305, &[]);
    let b = net.b();
    net.nodes[b].dead = true;
    let t0 = net.now;
    net.nodes[A].node.link.disconnect(t0);
    // ABORT while DISCONNECT's second Disc is on the air.
    net.run_until(30.0, |n| sent_by(n, A, Kind::Disc, t0).len() == 2 && n.nodes[A].tx.is_some());
    net.nodes[A].node.link.abort(net.now);
    net.run_until(60.0, |n| n.now >= t0 + 50.0);
    let discs = sent_by(&net, A, Kind::Disc, t0);
    assert_eq!(discs.len(), 4, "Discs of DISCONNECT and then ABORT at {discs:.2?}");
    assert_eq!(disconnects(&net, A), 1);
}

/// ABORT while a burst we started waits out its random start (`Respond` with
/// `initiated`): the Disc goes at that time, not at once, so it keeps the spread that
/// stops two ends from starting together.
#[test]
fn abort_keeps_the_start_time_of_a_burst_we_were_about_to_start() {
    let mut net = quiet_session(306, &[]);
    net.nodes[A].node.link.send_data(&data(100, 306));
    let starting = |n: &Net| {
        matches!(&n.nodes[A].node.link.state,
            State::Connected(s) if matches!(s.phase, Phase::Respond { initiated: true, .. }))
    };
    net.run_until(30.0, starting);
    let State::Connected(s) = &net.nodes[A].node.link.state else { unreachable!() };
    let Phase::Respond { at, .. } = s.phase else { unreachable!() };
    let t0 = net.now;
    assert!(at - t0 > 3.0 * DT, "start at {at:.2}, too close to {t0:.2} for the test");
    net.nodes[A].node.link.abort(t0);
    net.run_until(30.0, |n| !sent_by(n, A, Kind::Disc, t0).is_empty());
    let disc = sent_by(&net, A, Kind::Disc, t0)[0];
    assert!((disc.0 - at).abs() <= 2.0 * DT, "Disc at {:.2}, the burst was due at {at:.2} (ABORT at {t0:.2})", disc.0);
}

/// After ABORT, the peer's data is not acknowledged: the Disc's ack offset is what had
/// reached the host before the ABORT, and its bitmap is empty, so the peer does not
/// count as delivered data we threw away. Nor does `bytes_delivered`. B's data burst is
/// on the air at the ABORT, and A answers it with the Disc.
#[test]
fn abort_disc_acks_only_what_was_acked_before_the_abort() {
    let mut net = quiet_session(307, &[]);
    let b = net.b();
    net.nodes[b].node.link.send_data(&data(100, 307));
    // A hears B's burst: a Disc on a clear channel now would go into it.
    net.run_until(30.0, |n| n.nodes[b].tx.is_some() && n.busy(A));
    let State::Connected(s) = &net.nodes[A].node.link.state else { panic!("A is not connected") };
    let acked = ((s.rx_next % OFFSET_MOD) as u32, s.ack_burst);
    let delivered = net.nodes[A].node.link.stats.bytes_delivered;
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.run_until(30.0, |n| n.has(b, &LinkEvent::Disconnected));
    let (_, at, h) = *net
        .sent_headers
        .iter()
        .find(|&&(from, at, h)| from == A && at >= t0 && h.kind == Kind::Disc)
        .expect("A sent no Disc");
    let on_air = net
        .sent
        .iter()
        .any(|&(from, start, end, k)| from == b && k == Kind::Data && start <= t0 && end > t0 && end < at);
    assert!(on_air, "B's data was not on the air at the ABORT, ending before the Disc");
    assert_eq!((h.ack_offset, h.ack_burst), acked, "the Disc acks data received after the ABORT");
    assert_eq!(h.bitmap, 0, "the Disc's bitmap acks data received after the ABORT");
    assert_eq!(net.nodes[A].node.link.stats.bytes_delivered, delivered, "data after the ABORT counted as delivered");
}

/// ABORT while A's DiscAck to B's Disc is due: the DiscAck goes, not a Disc, and that
/// ends the abort. Both ends close, A's host hears DISCONNECTED once.
#[test]
fn abort_while_a_disc_ack_is_due_sends_the_disc_ack() {
    let mut net = quiet_session(308, &[]);
    let b = net.b();
    net.nodes[b].node.link.disconnect(net.now);
    let acking = |n: &Net| {
        matches!(&n.nodes[A].node.link.state,
            State::Connected(s) if matches!(s.phase, Phase::Respond { kind: Kind::DiscAck, .. }))
    };
    net.run_until(30.0, acking);
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.run_until(30.0, |n| n.has(b, &LinkEvent::Disconnected));
    let settled = net.now + 20.0;
    net.run_until(30.0, |n| n.now >= settled);
    assert_eq!(sent_by(&net, A, Kind::DiscAck, t0).len(), 1, "A's DiscAck");
    assert!(sent_by(&net, A, Kind::Disc, t0).is_empty(), "A sent a Disc after the peer's");
    assert!(matches!(net.nodes[A].node.link.state, State::Idle));
    assert_eq!((disconnects(&net, A), disconnects(&net, b)), (1, 1));
}

/// ABORT on the called station while its ConnectAck is due: we have never answered,
/// so the caller has never heard us. The session ends at once, with no ConnectAck and
/// no Disc, and the host hears DISCONNECTED once for it. B stops listening, so the
/// caller's retries are not taken as a new call.
#[test]
fn abort_before_our_connect_ack_ends_at_once_without_a_disc() {
    let mut net = Net::chain(309, 0.0, &[], LinkConfig::default());
    let b = net.b();
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    let acking = |n: &Net| {
        matches!(&n.nodes[b].node.link.state,
            State::Connected(s) if matches!(s.phase, Phase::Respond { kind: Kind::ConnectAck, .. }))
    };
    net.run_until(30.0, acking);
    let t0 = net.now;
    let ev0 = net.nodes[b].events.len();
    net.nodes[b].node.link.abort(t0);
    net.nodes[b].node.link.set_listen(false);
    assert!(matches!(net.nodes[b].node.link.state, State::Idle), "the abort did not end at once");
    // Until the caller's attempts are over.
    net.run_until(120.0, |n| n.nodes[A].node.link.is_idle());
    assert!(sent_by(&net, b, Kind::ConnectAck, t0).is_empty(), "B sent its ConnectAck after ABORT");
    let discs = sent_by(&net, b, Kind::Disc, t0);
    assert!(discs.is_empty(), "B's Discs at {discs:.2?}, but the caller never heard B");
    let told = net.nodes[b].events[ev0..].iter().filter(|e| **e == LinkEvent::Disconnected).count();
    assert_eq!(told, 1, "B's host told DISCONNECTED {told} times");
}

/// ABORT on the called station just after its ConnectAck went out: the caller may have
/// heard it, so the usual Disc rules apply. The Disc goes on the clear channel, the
/// caller (now connected) answers DiscAck, and both ends close.
#[test]
fn abort_after_our_connect_ack_sends_the_disc() {
    let mut net = Net::chain(311, 0.0, &[], LinkConfig::default());
    let b = net.b();
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(30.0, |n| !sent_by(n, b, Kind::ConnectAck, 0.0).is_empty() && n.nodes[b].tx.is_none());
    let t0 = net.now;
    net.nodes[b].node.link.abort(t0);
    net.run_until(30.0, |n| n.has(A, &LinkEvent::Disconnected));
    let discs = sent_by(&net, b, Kind::Disc, t0);
    assert_eq!(discs.len(), 1, "B's Discs at {discs:.2?}");
    assert_eq!(sent_by(&net, A, Kind::DiscAck, t0).len(), 1, "A's DiscAck");
    let settled = net.now + 20.0;
    net.run_until(30.0, |n| n.now >= settled);
    assert!(matches!(net.nodes[b].node.link.state, State::Idle), "B's abort did not end");
    assert_eq!((disconnects(&net, A), disconnects(&net, b)), (1, 1));
}

/// The peer's own Disc while A's Disc is repeated (B cannot hear A): A answers it with
/// a DiscAck in its slot, and that ends the abort: no Disc after it.
#[test]
fn the_peers_disc_while_aborting_is_answered_and_ends_the_abort() {
    let mut net = quiet_session(310, &[]);
    let b = net.b();
    net.set_hears(|to, from| to != from && !(to == b && from == A));
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.run_until(30.0, |n| !sent_by(n, A, Kind::Disc, t0).is_empty() && n.nodes[A].tx.is_none());
    net.nodes[b].node.link.disconnect(net.now);
    net.run_until(60.0, |n| !sent_by(n, A, Kind::DiscAck, t0).is_empty());
    let turnaround = net.nodes[A].node.link.config().turnaround_s;
    let peer_disc = *sent_by(&net, b, Kind::Disc, t0).last().expect("B sent no Disc");
    let ack = sent_by(&net, A, Kind::DiscAck, t0)[0];
    let slot = peer_disc.1 + 0.02 + turnaround;
    assert!(ack.0 >= slot - 2.0 * DT && ack.0 <= slot + 3.0 * DT, "DiscAck at {:.2}, slot at {slot:.2}", ack.0);
    let t = net.now;
    net.run_until(60.0, |n| n.now >= t + 40.0);
    let late: Vec<_> = sent_by(&net, A, Kind::Disc, ack.1);
    assert!(late.is_empty(), "A's Discs after its DiscAck: {late:.2?}");
    assert!(matches!(net.nodes[A].node.link.state, State::Idle));
    assert_eq!(disconnects(&net, A), 1);
}

// ------------------------------------------------------------ ABORT: second review

/// The called station's session id, once B has taken A's call.
fn called_id(net: &Net) -> u8 {
    match &net.nodes[net.b()].node.link.state {
        State::Connected(s) | State::Aborting(s) => s.id,
        _ => panic!("B has no session"),
    }
}

/// B, listening, takes A's call and ABORTs it while its ConnectAck is due, so the caller
/// never heard B. A keeps retrying its ConnectReq for its call window: B, still
/// listening, does not take those retries as a new call, and sends nothing.
#[test]
fn an_aborted_call_is_not_taken_again_while_listening() {
    let mut net = Net::chain(320, 0.0, &[], LinkConfig::default());
    let b = net.b();
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    let acking = |n: &Net| {
        matches!(&n.nodes[b].node.link.state,
            State::Connected(s) if matches!(s.phase, Phase::Respond { kind: Kind::ConnectAck, .. }))
    };
    net.run_until(30.0, acking);
    let t0 = net.now;
    let ev0 = net.nodes[b].events.len();
    net.nodes[b].node.link.abort(t0);
    net.run_until(200.0, |n| n.nodes[A].node.link.is_idle() || n.both_connected());
    let again: Vec<_> = net.nodes[b].events[ev0..]
        .iter()
        .filter(|e| matches!(e, LinkEvent::Connected { .. } | LinkEvent::Pending))
        .collect();
    assert!(again.is_empty(), "B took the aborted call again: {again:?}");
    let sent: Vec<_> = net.sent.iter().filter(|&&(from, start, ..)| from == b && start >= t0).collect();
    assert!(sent.is_empty(), "B sent after its ABORT: {sent:.2?}");
}

/// As above, but B's ConnectAck went out and was lost, so the caller, still connecting,
/// retries: B answers those retries with its Discs (which a station still connecting
/// ignores), and after them still does not take the call again.
#[test]
fn an_aborted_call_is_not_taken_again_after_its_discs() {
    let cfg = LinkConfig { connect_attempts: 10, ..LinkConfig::default() };
    let mut net = Net::chain(321, 0.0, &[], cfg);
    let b = net.b();
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.lose_next = Some((b, A));
    // Until B's ConnectAck has ended and been lost on its way to A.
    net.run_until(30.0, |n| n.lose_next.is_none() && n.nodes[b].tx.is_none());
    assert_eq!(sent_by(&net, b, Kind::ConnectAck, 0.0).len(), 1, "the lost burst was not B's ConnectAck");
    let t0 = net.now;
    let ev0 = net.nodes[b].events.len();
    net.nodes[b].node.link.abort(t0);
    net.run_until(300.0, |n| n.nodes[A].node.link.is_idle() || n.both_connected());
    let discs = sent_by(&net, b, Kind::Disc, t0);
    assert!(!discs.is_empty() && discs.len() <= 4, "B's Discs at {discs:.2?}");
    let again: Vec<_> = net.nodes[b].events[ev0..]
        .iter()
        .filter(|e| matches!(e, LinkEvent::Connected { .. } | LinkEvent::Pending))
        .collect();
    assert!(again.is_empty(), "B took the aborted call again: {again:?}");
    assert!(sent_by(&net, b, Kind::ConnectAck, t0).is_empty(), "B sent a ConnectAck after ABORT");
}

/// B ABORTs A's call before answering it. Within the call window a ConnectReq with the
/// aborted id is ignored, but a new call from the same caller (a new id) connects; so
/// does the aborted id once the window is over.
#[test]
fn after_an_aborted_call_a_new_call_still_connects() {
    let mut net = Net::chain(322, 0.0, &[], LinkConfig::default());
    let b = net.b();
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    let acking = |n: &Net| {
        matches!(&n.nodes[b].node.link.state,
            State::Connected(s) if matches!(s.phase, Phase::Respond { kind: Kind::ConnectAck, .. }))
    };
    net.run_until(30.0, acking);
    let id = called_id(&net);
    let t0 = net.now;
    let window = net.nodes[b].node.link.connect_window_s(0);
    net.nodes[b].node.link.abort(t0);
    let connected = |link: &mut Link, now: f64, id: u8| {
        let ev0 = link.events.len();
        link.on_burst(now, &connect_burst(Kind::ConnectReq, id, ("LA1AAA", "LA2BBB-1"), &[], false, 0));
        let took = link.events[ev0..].iter().any(|e| matches!(e, LinkEvent::Connected { .. }));
        if took {
            link.abort(now);
        }
        took
    };
    let link = &mut net.nodes[b].node.link;
    assert!(!connected(link, t0 + 1.0, id), "the aborted id was taken again within the call window");
    assert!(connected(link, t0 + window + 1.0, id), "the aborted id was still refused after the call window");
    let other = id.wrapping_add(1).max(1);
    assert!(connected(link, t0 + window + 2.0, other), "a new call (id {other}) from the same caller did not connect");
}

/// The abort ends at the session's busy timeout at the latest, counted from the ABORT:
/// Discs that go out in short gaps of a jammed channel do not restart it. B is switched
/// off, and the carrier drops for 1 s every 20 s.
#[test]
fn the_abort_ends_at_the_busy_timeout_though_discs_go_out_in_gaps() {
    let mut net = quiet_session(323, &[]);
    let b = net.b();
    net.nodes[b].dead = true;
    let t0 = net.now;
    for k in 0..10 {
        let start = t0 - 0.2 + 20.0 * k as f64;
        net.history.push((b, start, start + 19.0));
    }
    let timeout = net.nodes[A].node.link.busy_timeout_s(0);
    net.nodes[A].node.link.abort(t0);
    net.run_until(300.0, |n| n.now >= t0 + 150.0);
    let discs = sent_by(&net, A, Kind::Disc, t0);
    assert!(!discs.is_empty(), "no Disc went out in the gaps");
    let late: Vec<_> = discs.iter().filter(|d| d.0 > t0 + timeout + DT).collect();
    assert!(late.is_empty(), "Discs after the busy timeout ({timeout:.0} s from the ABORT at {t0:.2}): {late:.2?}");
    assert!(matches!(net.nodes[A].node.link.state, State::Idle));
    assert_eq!(disconnects(&net, A), 1);
}

/// The abort ends at the busy timeout on a jammed channel after its first Disc went out:
/// no parting Disc follows the carrier, since the peer has had one. B cannot hear A.
#[test]
fn a_jammed_channel_after_the_first_abort_disc_sends_no_parting_disc() {
    let mut net = quiet_session(324, &[]);
    let b = net.b();
    net.set_hears(|to, from| to != from && !(to == b && from == A));
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.run_until(30.0, |n| !sent_by(n, A, Kind::Disc, t0).is_empty() && n.nodes[A].tx.is_none());
    let start = net.now;
    let end = t0 + net.nodes[A].node.link.busy_timeout_s(0) + 5.0;
    net.history.push((b, start, end));
    net.run_until(200.0, |n| n.now >= end + 30.0);
    let discs = sent_by(&net, A, Kind::Disc, t0);
    assert_eq!(discs.len(), 1, "A's Discs at {discs:.2?}, carrier from {start:.2} to {end:.2}");
    assert!(matches!(net.nodes[A].node.link.state, State::Idle));
    assert_eq!(disconnects(&net, A), 1);
}

/// The Disc's ack bitmap is empty after ABORT, even right after the peer's data burst
/// that set it: the peer must not take any data past the ack offset (held out of order,
/// then thrown away) as delivered. The ack offset (what reached the host) and the burst
/// stay as they were.
#[test]
fn the_abort_disc_carries_an_empty_bitmap() {
    let mut net = quiet_session(325, &[]);
    let b = net.b();
    net.nodes[b].node.link.send_data(&data(100, 325));
    let got = |n: &Net| matches!(&n.nodes[A].node.link.state, State::Connected(s) if s.ack_bitmap != 0);
    net.run_until(30.0, got);
    let State::Connected(s) = &net.nodes[A].node.link.state else { unreachable!() };
    let before = ((s.rx_next % OFFSET_MOD) as u32, s.ack_burst);
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.run_until(30.0, |n| n.has(b, &LinkEvent::Disconnected));
    let (_, _, h) = *net
        .sent_headers
        .iter()
        .find(|&&(from, at, h)| from == A && at >= t0 && h.kind == Kind::Disc)
        .expect("A sent no Disc");
    assert_eq!(h.bitmap, 0, "the Disc acks the data burst A had just decoded");
    assert_eq!((h.ack_offset, h.ack_burst), before, "the Disc's ack offset or burst changed");
}

// ------------------------------------------------------------ ABORT: third review

/// With the smallest busy timeout (0: the route's floor, about 6.8 s direct, less than
/// the 7.6 s four unanswered Discs take), an ABORT on a clear channel still sends all
/// 4 Discs: the abort's deadline is at least the time they need. B is switched off.
#[test]
fn abort_with_the_smallest_busy_timeout_still_sends_four_discs() {
    let mut net = quiet_session_with(330, &[], LinkConfig { busy_timeout_s: 0.0, ..LinkConfig::default() });
    let b = net.b();
    net.nodes[b].dead = true;
    let floor = net.nodes[A].node.link.busy_timeout_s(0);
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.run_until(60.0, |n| n.now >= t0 + 40.0);
    let discs = sent_by(&net, A, Kind::Disc, t0);
    assert_eq!(discs.len(), 4, "A's Discs at {discs:.2?} (ABORT at {t0:.2}, busy timeout {floor:.2} s)");
    assert_eq!(disconnects(&net, A), 1);
}

/// As above, with the ABORT just as a long data burst of ours starts: the Disc waits
/// for the rest of it and for its answer, and the 4 Discs still all go.
#[test]
fn abort_at_the_start_of_a_long_burst_still_sends_four_discs() {
    let mut net = quiet_session_with(331, &[], LinkConfig { busy_timeout_s: 0.0, ..LinkConfig::default() });
    let b = net.b();
    net.nodes[b].dead = true;
    net.nodes[A].node.link.send_data(&data(5000, 331));
    net.run_until(30.0, |n| n.nodes[A].tx.is_some());
    let t0 = net.now;
    let burst = *net.sent.iter().rfind(|&&(from, ..)| from == A).unwrap();
    net.nodes[A].node.link.abort(t0);
    net.run_until(60.0, |n| n.now >= t0 + 40.0);
    let discs = sent_by(&net, A, Kind::Disc, t0);
    assert_eq!(discs.len(), 4, "A's Discs at {discs:.2?}, after its burst {burst:.2?} (ABORT at {t0:.2})");
    assert_eq!(disconnects(&net, A), 1);
}

/// The refusal of an aborted call is for that call only: the same caller calling our
/// other call sign with the same session id is a new call, and connects.
#[test]
fn a_call_to_our_other_call_sign_with_the_aborted_id_connects() {
    let mut net = Net::chain(332, 0.0, &[], LinkConfig::default());
    let b = net.b();
    net.nodes[b].node.link.set_mycalls(vec!["LA2BBB-1".into(), "LA2BBB-2".into()]);
    net.nodes[A].node.link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    let acking = |n: &Net| {
        matches!(&n.nodes[b].node.link.state,
            State::Connected(s) if matches!(s.phase, Phase::Respond { kind: Kind::ConnectAck, .. }))
    };
    net.run_until(30.0, acking);
    let id = called_id(&net);
    let t0 = net.now;
    let link = &mut net.nodes[b].node.link;
    link.abort(t0);
    let ev0 = link.events.len();
    link.on_burst(t0 + 1.0, &connect_burst(Kind::ConnectReq, id, ("LA1AAA", "LA2BBB-2"), &[], false, 0));
    let took = link.events[ev0..]
        .iter()
        .any(|e| matches!(e, LinkEvent::Connected { destination, .. } if destination == "LA2BBB-2"));
    assert!(took, "a call to LA2BBB-2 with the aborted id {id} was refused");
}

/// A busy channel holds A's data back for most of the busy timeout, then A ABORTs: the
/// abort's deadline runs from the ABORT, so the busy time before it does not count. The
/// channel clears after the busy timeout from the data, and all 4 Discs go. B is
/// switched off.
#[test]
fn a_busy_timer_running_before_abort_does_not_shorten_it() {
    let mut net = quiet_session(333, &[]);
    let b = net.b();
    net.nodes[b].dead = true;
    let timeout = net.nodes[A].node.link.busy_timeout_s(0);
    let start = net.now;
    let end = start + timeout + 2.0;
    net.history.push((b, start - 0.2, end));
    net.nodes[A].node.link.send_data(&data(100, 333));
    net.run_until(120.0, |n| n.now >= start + timeout - 10.0);
    assert!(sent_by(&net, A, Kind::Data, start).is_empty(), "A's data went out into the carrier");
    let t0 = net.now;
    net.nodes[A].node.link.abort(t0);
    net.run_until(120.0, |n| n.now >= end + 30.0);
    let discs = sent_by(&net, A, Kind::Disc, t0);
    assert_eq!(
        discs.len(),
        4,
        "A's Discs at {discs:.2?} (data held back from {start:.2}, ABORT at {t0:.2}, carrier until {end:.2})"
    );
    assert!(discs[0].0 >= end, "Disc into the carrier");
    assert_eq!(disconnects(&net, A), 1);
}
