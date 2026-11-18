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
            if self.nodes[to].dead || self.collided(to, from, start, end) || self.chance(self.burst_loss) {
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
    for (to, row) in net.hears.iter_mut().enumerate() {
        for (from, h) in row.iter_mut().enumerate() {
            *h = to != from;
        }
    }
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
