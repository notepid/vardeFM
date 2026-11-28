//! VardeFM ARQ link layer.
//!
//! A pure state machine: the caller passes in the current time, received bursts,
//! host commands and whether the channel is busy. It asks [`Node::poll`] what to
//! transmit, and reads host notifications from [`Link::take_events`]. A [`Node`] is a
//! [`Link`] with the radio's side of it: bursts in, bursts out.
//!
//! # Protocol summary
//! - **Connect:** the caller sends `ConnectReq` (session id + call signs) and the
//!   called station answers `ConnectAck`. Requests are retried
//!   `connect_attempts` times.
//! - **Data:** a byte stream in each direction, numbered by byte offset. Each
//!   payload codeword carries one segment: `offset (24 bits) | length (8 bits) | data`.
//! - **Acknowledgements:** every burst carries `ack_offset`, the next byte expected
//!   from the peer, plus a bitmap of which codewords of the peer's last burst
//!   decoded. A sender resends everything not covered by either.
//! - **Turns:** after receiving a burst that carried data (or a POLL), a station
//!   answers after `turnaround_s`. Its answer carries acks and any data of its
//!   own. If neither side has anything to send, the link goes quiet. A station
//!   that then gets data from its host starts a burst once the channel is clear.
//!   After `ConnectAck`, the called station may send first; the caller waits one
//!   response period for it.
//! - **Rate adaptation:** each burst names the speed level the peer should use
//!   next. It is chosen from the measured SNR (normalized for the per-level
//!   transmit drive) and stepped down after codeword failures or timeouts.
//! - **Disconnect:** after the host's DISCONNECT, once all sent data is
//!   acknowledged, the station sends `Disc` and the peer answers `DiscAck`. Both
//!   then report DISCONNECTED.
//! - **Digipeaters:** a session may run through up to two digipeaters
//!   ([`Digipeater`]), which repeat each burst. Every burst of such a session is
//!   marked routed in its header; direct sessions' bursts are not, and are never
//!   repeated. The endpoints accept only the copy that has passed every digipeater,
//!   and wait longer for answers.

mod digi;
mod node;
pub use digi::Digipeater;
pub use node::Node;

use std::collections::{BTreeMap, VecDeque};
use std::sync::Arc;
use vardefm_dsp::prng::Prng;
use vardefm_modem::{Burst, Header, Kind, Modem, RxBurst, FLAG_POLL, FLAG_ROUTED, FLAG_UP, LEVELS, MAX_DIGIS};

/// Estimated SNR (dB, as reported by the receiver) on a burst sent at each level, at
/// which that level decodes reliably in the FM channel simulator (see the
/// `level_table` benchmark). Each level has its own transmit drive, so a burst
/// measured at one level is normalized to the drive of another before comparing.
///
/// Each value is the highest SNR of any burst at that level with a failed codeword,
/// or the lowest SNR from which every burst decoded fully if higher, rounded up to
/// 0.5 dB, in the worse of the two modes. So with the margin, a level is only
/// requested at least 1 dB above every failure seen. Below about 10 dB the SNR
/// estimate falls fast with CNR (FM threshold), so the low levels' values are small
/// or negative; level 0's is never used.
pub const SNR_THRESHOLDS_DB: [f32; 7] = [-2.0, 1.5, 4.5, 7.0, 11.0, 15.0, 18.0];
const SNR_MARGIN_DB: f32 = 1.0;

/// Highest level whose threshold (plus margin) the SNR would reach, given `snr_db`
/// measured on a burst sent at `rx_level`. Each level has its own drive and clipping
/// floor, so the SNR is first predicted for that level ([`Modem::predict_snr_db`]).
pub fn level_by_snr(modem: &Modem, snr_db: f32, rx_level: u8) -> u8 {
    SNR_THRESHOLDS_DB
        .iter()
        .enumerate()
        .rposition(|(l, &t)| modem.predict_snr_db(snr_db, rx_level, l as u8) >= t + SNR_MARGIN_DB)
        .unwrap_or(0) as u8
}

const SEGMENT_HEADER: usize = 4;
/// Largest random back-off before a burst we start (a routed retry's is spread wider).
const RETRY_BACKOFF_S: f64 = 0.4;
/// Largest random addition to the wait between connect requests.
const CONNECT_JITTER_S: f64 = 0.5;
/// While the channel is busy, a burst that is due is put off by this much, again and
/// again until the channel is clear.
const BUSY_RECHECK_S: f64 = 0.3;
/// Most Discs a session sends when it closes, by DISCONNECT and ABORT together.
const MAX_DISC_TRIES: u32 = 4;
/// Largest `hop_margin_s`. A digipeater does not know the end stations' margin, so the
/// lifetimes of its sessions are sized for this one.
pub const MAX_HOP_MARGIN_S: f64 = 10.0;
const OFFSET_MOD: u64 = 1 << 24;
/// Segment length byte of a codeword a digipeater could not decode (a real segment
/// holds at most 242 bytes). The receiver treats it as a failed codeword.
const VOID_LEN: u8 = 0xff;

#[derive(Clone, Debug, PartialEq)]
pub struct LinkConfig {
    /// Gap between receiving a burst and answering it.
    pub turnaround_s: f64,
    /// How long to wait for the peer to start answering before retrying.
    /// The wait is extended while the channel is busy.
    pub response_timeout_s: f64,
    pub connect_attempts: u32,
    /// Consecutive unanswered bursts before the link is declared lost.
    pub max_retries: u32,
    /// Delay between DISCONNECT and closing, so data written just before it
    /// (on the separate data socket) is still sent.
    pub disconnect_grace_s: f64,
    /// Probe a silent link after this long.
    pub keepalive_s: f64,
    /// Level used until the peer asks for another.
    pub start_level: u8,
    pub max_level: u8,
    /// Extra wait per digipeater hop, beyond airtime and turnaround: a digipeater's
    /// decoding time and the delay before its transmitter is up. From 0 to
    /// [`MAX_HOP_MARGIN_S`].
    pub hop_margin_s: f64,
    /// Give up once a busy channel has held back a burst we want to start (a connect
    /// request, a retry, data, a keepalive or a Disc) this long, with no progress: none
    /// of our own link's bursts went out and we accepted none from the peer. A quiet
    /// moment too short for the burst to go out does not count as progress, nor does a
    /// digipeater repeat. Without this a carrier or other stations' traffic could keep a
    /// dead session, or a connect attempt, open forever. A dropped session's Disc goes
    /// out once the channel is quiet, in case the peer is there but we cannot decode it,
    /// unless the channel stays busy for this long again. An aborted session ends at the
    /// latest this long after the ABORT (as raised to the route's floor), or after the
    /// longest an unanswered abort takes on a clear channel if that is longer. It gets
    /// the parting Disc only if none of its Discs went out, and no CONNECT was waiting.
    ///
    /// The link raises a smaller value to a floor chosen to cover an exchange with the
    /// peer through the session's digipeaters (the longest burst plus two of the called
    /// station's longest retry waits), so that the peer's own bursts and their repeats do
    /// not trigger it; 0 means "use the floor". `f64::INFINITY` turns the drop off.
    /// [`Link::new`] panics on a negative or NaN value.
    pub busy_timeout_s: f64,
}

impl Default for LinkConfig {
    fn default() -> Self {
        Self {
            turnaround_s: 0.2,
            response_timeout_s: 1.2,
            connect_attempts: 5,
            max_retries: 8,
            disconnect_grace_s: 0.3,
            keepalive_s: 30.0,
            start_level: 1,
            max_level: LEVELS.len() as u8 - 1,
            hop_margin_s: 0.5,
            busy_timeout_s: 60.0,
        }
    }
}

/// Notifications for the host application.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LinkEvent {
    Pending,
    CancelPending,
    /// `source` is the calling station.
    Connected {
        source: String,
        destination: String,
    },
    Disconnected,
    /// Bytes queued for transmission and not yet acknowledged.
    Buffer(usize),
    Data(Vec<u8>),
}

/// What the last transmitted burst contained (for logging).
#[derive(Clone, Debug, Default)]
pub struct TxInfo {
    pub kind: Kind,
    pub level: u8,
    pub ncw: usize,
    /// Payload bytes in this burst, including resent bytes.
    pub bytes: usize,
    pub resent_bytes: usize,
    /// Consecutive retry number (0 = first attempt).
    pub retry: u32,
}

#[derive(Clone, Debug, Default)]
pub struct LinkStats {
    pub bursts_sent: u64,
    pub retries: u64,
    pub payload_bytes_sent: u64,
    pub bytes_resent: u64,
    pub bytes_delivered: u64,
    pub codewords_received: u64,
    pub codewords_failed: u64,
}

/// The digipeaters of a session, in order from the calling station.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Route {
    digis: Vec<String>,
    caller: bool,
}

impl Route {
    fn hops(&self) -> u8 {
        self.digis.len() as u8
    }

    /// Flags for our bursts. A routed session marks every burst routed, not yet
    /// repeated, with UP from the called station. Direct sessions set none, as
    /// released versions do.
    fn out_flags(&self) -> u8 {
        match (self.digis.is_empty(), self.caller) {
            (true, _) => 0,
            (false, true) => FLAG_ROUTED,
            (false, false) => FLAG_ROUTED | FLAG_UP,
        }
    }

    /// True if a burst is the peer's, after every digipeater. On a routed session this
    /// drops copies heard directly or part way along the path, our own echoes, and
    /// direct bursts that reuse the id. A direct session takes only unrouted bursts
    /// without UP, as every version sends them: a routed caller's burst heard directly,
    /// or a copy repeated by a digipeater, is not the peer's.
    fn accepts(&self, h: &Header) -> bool {
        if self.digis.is_empty() {
            !h.routed() && !h.up()
        } else {
            h.routed() && h.up() == self.caller && h.hop() == self.hops()
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    /// Nothing to do until something happens.
    Idle,
    /// Transmit a burst of `kind` at time `at`. `initiated`: we start the exchange
    /// ourselves (not answering the peer), or it is a retry: give way if the channel
    /// turns busy.
    Respond { at: f64, kind: Kind, initiated: bool },
    /// Our burst is on the air. `initiated` as for `Respond`.
    Sending { kind: Kind, retry: bool, initiated: bool },
    /// Waiting for the peer. `retry`: resend if it does not answer.
    Wait { deadline: f64, retry: bool },
}

struct Session {
    id: u8,
    local: String,
    remote: String,
    route: Route,
    /// Airtime of our last burst, seconds.
    last_tx_s: f64,
    // Transmit side: tx_buf holds every byte from tx_base that is not yet
    // cumulatively acknowledged. `delivered` lists ranges the peer has received
    // out of order; `in_flight` the ranges sent in our last burst, by codeword.
    tx_base: u64,
    tx_buf: VecDeque<u8>,
    delivered: Vec<(u64, u64)>,
    in_flight: Vec<(u64, u64)>,
    /// Highest byte offset ever sent (anything below it is a resend).
    sent_high: u64,
    burst_id: u8,
    // Receive side.
    rx_next: u64,
    rx_ooo: BTreeMap<u64, Vec<u8>>,
    ack_burst: u8,
    ack_bitmap: u32,
    /// Level the peer asked us to use.
    peer_level: u8,
    /// Level we ask the peer to use.
    req_level: u8,
    phase: Phase,
    retries: u32,
    closing: Option<f64>,
    disc_tries: u32,
    /// We have sent a burst of this session, so the peer may have heard of it: the
    /// caller from the start (its ConnectReq), the called station once its ConnectAck
    /// goes out. ABORT before that ends the session without a Disc.
    answered: bool,
    last_heard: f64,
    /// Called station: until then, unless the caller has sent a burst of the session,
    /// the caller may have missed our ConnectAck and still be retrying its ConnectReq.
    /// Our retries then send the ConnectAck again instead of Data. After it, a caller
    /// that missed the ConnectAck has given up. `NEG_INFINITY` once answered, and for
    /// the caller.
    ack_until: f64,
    /// Called station: the end of the caller's call window, from its first ConnectReq.
    /// Unlike `ack_until` it is kept once answered. `NEG_INFINITY` for the caller.
    call_until: f64,
    /// After ABORT: when the abort ends at the latest (see `abort_s`).
    abort_until: f64,
}

/// A recently closed session.
struct Closed {
    id: u8,
    route: Route,
    local: String,
    remote: String,
    /// After ABORT of a call: until then the caller may still be retrying its
    /// ConnectReq, and that is not taken as a new call. `NEG_INFINITY` otherwise.
    refuse_until: f64,
}

impl Closed {
    fn of(s: &Session) -> Closed {
        Closed {
            id: s.id,
            route: s.route.clone(),
            local: s.local.clone(),
            remote: s.remote.clone(),
            refuse_until: f64::NEG_INFINITY,
        }
    }

    /// As `of`, for a session the host aborted. Only a called station refuses anything:
    /// a caller's `call_until` is `NEG_INFINITY`.
    fn aborted(s: &Session) -> Closed {
        Closed { refuse_until: s.call_until, ..Closed::of(s) }
    }
}

enum State {
    Idle,
    Connecting {
        session: u8,
        mycall: String,
        target: String,
        digis: Vec<String>,
        attempts: u32,
        next_tx: f64,
        sending: bool,
    },
    Connected(Box<Session>),
    /// After ABORT: the host has been told DISCONNECTED, but the session is kept for its
    /// Disc. The Disc goes in our answer slot after the peer's next burst, or on a clear
    /// channel, and is repeated as DISCONNECT's is, giving way to a busy channel:
    /// `MAX_DISC_TRIES` in all, counting those a DISCONNECT already sent. It ends on the
    /// peer's DiscAck, or its own Disc (which we answer with a DiscAck, the one other
    /// burst we send), once the tries are used up, or at its deadline (`abort_s`).
    /// No data is delivered or acked meanwhile; a CONNECT waits in `queued_connect`.
    Aborting(Box<Session>),
}

/// A CONNECT that arrived while an aborted session's Disc was being repeated.
struct QueuedConnect {
    mycall: String,
    target: String,
    via: Vec<String>,
}

pub struct Link {
    modem: Arc<Modem>,
    cfg: LinkConfig,
    mycalls: Vec<String>,
    listen: bool,
    state: State,
    events: Vec<LinkEvent>,
    last_buffer: usize,
    /// Report BUFFER at the next `take_events` even if unchanged (after host data).
    force_buffer: bool,
    rng: Prng,
    /// A burst to send once even though we are disconnected (a DiscAck re-sent to a closed
    /// session's repeated Disc).
    oneshot: Option<Burst>,
    /// A Disc for a session a busy channel ended, and the time after which it is too late
    /// to send. It goes out once the channel is quiet, so that it is not lost under the
    /// carrier. Dropped if a new session starts first, or once it is too late: the peer
    /// has given up by then, and a late Disc could end its next session if that one got
    /// the same id.
    parting: Option<(f64, Burst)>,
    /// Recently closed session, so a repeated Disc can still be answered, and an aborted
    /// call is not taken again.
    closed: Option<Closed>,
    /// A CONNECT accepted while `State::Aborting`, started once the Disc exchange ends.
    queued_connect: Option<QueuedConnect>,
    /// When each session id was last heard on the channel, in any burst.
    heard_ids: [f64; 256],
    /// Longest burst any level can send, seconds.
    max_burst_s: f64,
    /// Since when something we want to send has been held back by a busy channel. Only
    /// progress restarts it: a burst of our own link going out, or one from the peer
    /// accepted. A quiet moment does not, as it may be too short for the burst to go out.
    /// It is cleared whenever nothing waits to be sent.
    held_since: Option<f64>,
    last_tx: TxInfo,
    pub stats: LinkStats,
}

/// True if `s` can be a `hop_margin_s`: from 0 to [`MAX_HOP_MARGIN_S`].
pub fn valid_hop_margin(s: f64) -> bool {
    (0.0..=MAX_HOP_MARGIN_S).contains(&s)
}

fn norm_call(c: &str) -> String {
    c.trim().to_ascii_uppercase()
}

impl Link {
    /// Panics if `cfg.busy_timeout_s` is negative or NaN.
    pub fn new(modem: Arc<Modem>, cfg: LinkConfig, seed: u32) -> Self {
        assert!(
            cfg.busy_timeout_s >= 0.0,
            "LinkConfig::busy_timeout_s must be 0 or more (or infinite), got {}",
            cfg.busy_timeout_s
        );
        let max_burst_s =
            (0..LEVELS.len() as u8).map(|l| modem.burst_seconds(l, modem.max_ncw(l)) as f64).fold(0.0, f64::max);
        Self {
            max_burst_s,
            modem,
            cfg,
            mycalls: Vec::new(),
            listen: false,
            state: State::Idle,
            events: Vec::new(),
            last_buffer: 0,
            force_buffer: false,
            rng: Prng::new(seed),
            oneshot: None,
            parting: None,
            closed: None,
            queued_connect: None,
            heard_ids: [f64::NEG_INFINITY; 256],
            held_since: None,
            last_tx: TxInfo::default(),
            stats: LinkStats::default(),
        }
    }

    pub fn config(&self) -> &LinkConfig {
        &self.cfg
    }

    pub fn set_mycalls(&mut self, calls: Vec<String>) {
        self.mycalls = calls;
    }

    pub fn set_listen(&mut self, on: bool) {
        self.listen = on;
    }

    pub fn is_connected(&self) -> bool {
        matches!(self.state, State::Connected(_))
    }

    /// The host's view: no session and no connect attempt. True while an aborted
    /// session's Disc is still being repeated (`State::Aborting`), since the host has
    /// been told DISCONNECTED, unless a CONNECT waits for it to end. Code that decides
    /// whether to transmit or start something must also check for `State::Aborting`.
    pub fn is_idle(&self) -> bool {
        matches!(self.state, State::Idle | State::Aborting(_)) && self.queued_connect.is_none()
    }

    pub fn last_tx(&self) -> &TxInfo {
        &self.last_tx
    }

    pub fn take_events(&mut self) -> Vec<LinkEvent> {
        let buf = match &self.state {
            State::Connected(s) => s.tx_buf.len(),
            _ => 0,
        };
        // Take the flag first: behind `||` it would stay set when the length changed,
        // and the next call would send a second report with nothing new.
        let forced = std::mem::take(&mut self.force_buffer);
        if buf != self.last_buffer || forced {
            self.last_buffer = buf;
            self.events.push(LinkEvent::Buffer(buf));
        }
        std::mem::take(&mut self.events)
    }

    fn rand_s(&mut self, max: f64) -> f64 {
        (self.rng.next_u32() as f64 / u32::MAX as f64) * max
    }

    /// Extra time an answer takes through `hops` digipeaters: each repeats our burst
    /// (`own_s`) and then the answer (at most `reply_s`), after its turnaround.
    fn relay_s(&self, hops: u8, own_s: f64, reply_s: f64) -> f64 {
        hops as f64 * (own_s + reply_s + 2.0 * self.cfg.turnaround_s + self.cfg.hop_margin_s)
    }

    /// Longest time a caller keeps retrying its ConnectReq through `hops` digipeaters:
    /// `connect_attempts` periods of a connect burst, the wait after it and the jitter,
    /// plus the time one of our longest bursts can hold the retry back: the burst, the
    /// busy detector's hang time after it (taken as at most one recheck) and the recheck.
    /// Assumes the caller uses our `connect_attempts` and `response_timeout_s`. If the
    /// window ends too soon, the caller's next ConnectReq is still answered.
    fn connect_window_s(&self, hops: u8) -> f64 {
        let held_back = self.max_burst_s + 2.0 * BUSY_RECHECK_S;
        let period = self.connect_s() + self.connect_wait_s(hops) + CONNECT_JITTER_S + held_back;
        self.cfg.connect_attempts as f64 * period
    }

    /// Airtime of a ConnectReq or ConnectAck.
    fn connect_s(&self) -> f64 {
        self.modem.burst_seconds(0, 1) as f64
    }

    /// Wait after a ConnectReq through `hops` digipeaters before retrying it, not counting
    /// the jitter.
    fn connect_wait_s(&self, hops: u8) -> f64 {
        let connect_s = self.connect_s();
        self.cfg.response_timeout_s + self.relay_s(hops, connect_s, connect_s)
    }

    /// Wait after our burst of `own_s` before retrying.
    fn response_wait(&self, hops: u8, own_s: f64) -> f64 {
        self.cfg.response_timeout_s + self.relay_s(hops, own_s, self.max_burst_s)
    }

    /// Retries are spread by a random 0..RETRY_BACKOFF_S s times this. Through
    /// digipeaters a retry has further to go, so the spread is wider.
    fn retry_spread(hops: u8) -> f64 {
        1.0 + 2.0 * hops as f64
    }

    /// Wait before resending a burst of `own_s` that we started ourselves (not an answer
    /// to the peer), or a retry. On a routed session the two ends do not hear each other:
    /// if both send at once, both bursts can be lost at a digipeater, and with equal
    /// timers every retry would meet there again. So the called station waits out the
    /// caller's longest retry period (counted from the start of a burst, with its random
    /// spread), plus the time the caller's retry takes to reach it through every
    /// digipeater. After each such collision the called station's next burst starts at
    /// least that reach later, relative to the caller's, than its last one did, so it soon
    /// hears the caller's retry coming through and answers it.
    fn retry_wait(&self, hops: u8, caller: bool, own_s: f64) -> f64 {
        if hops == 0 || caller {
            return self.response_wait(hops, own_s);
        }
        let caller_period =
            self.max_burst_s + self.response_wait(hops, self.max_burst_s) + RETRY_BACKOFF_S * Self::retry_spread(hops);
        let reach = hops as f64 * (self.max_burst_s + self.cfg.turnaround_s + self.cfg.hop_margin_s);
        caller_period - own_s + reach
    }

    /// Keepalive interval: on a routed session, long enough that a probe never goes
    /// out while an answer may still be on its way, even the called station's retry
    /// after its longest wait.
    fn keepalive_s(&self, hops: u8) -> f64 {
        if hops == 0 {
            self.cfg.keepalive_s
        } else {
            self.cfg.keepalive_s.max(2.0 * self.retry_wait(hops, false, 0.0))
        }
    }

    /// True if there is a burst we would start ourselves as soon as the channel is quiet
    /// (or once its time comes): a connect request, a retry, data, a keepalive or a
    /// Disc. Answers to the peer go out whether the channel is busy or not. A wait with
    /// no retry pending ends in `Phase::Idle`, so it holds back what `Idle` would start.
    fn wants_to_send(&self, now: f64, keepalive: f64) -> bool {
        match &self.state {
            State::Idle => false,
            State::Connecting { sending, .. } => !sending,
            State::Connected(s) | State::Aborting(s) => match s.phase {
                Phase::Sending { .. } => false,
                Phase::Wait { retry: true, .. } => true,
                Phase::Respond { initiated, .. } => initiated,
                Phase::Wait { retry: false, .. } | Phase::Idle => {
                    s.closing.is_some() || s.has_data() || s.keepalive_due(now, keepalive)
                }
            },
        }
    }

    /// How long a busy channel may hold back a burst we want to start before we give up.
    /// The configured value, but at least the peer's longest burst plus two of the
    /// called station's longest retry waits, a floor chosen to cover the channel staying
    /// busy from our burst's repeats through the peer's answer and its repeats before we
    /// decode the answer. `f64::INFINITY` turns the drop off.
    /// How long an abort may last from the ABORT: the busy timeout for its route, but at
    /// least the longest an unanswered abort takes on a clear channel: the rest of our
    /// longest burst and its answer wait, then `MAX_DISC_TRIES` Discs, each after a
    /// backoff and followed by its wait for a DiscAck.
    fn abort_s(&self, route: &Route) -> f64 {
        let hops = route.hops();
        let answer =
            self.response_wait(hops, self.max_burst_s).max(self.retry_wait(hops, route.caller, self.max_burst_s));
        let disc = self.modem.burst_seconds(0, 0) as f64;
        let backoff = RETRY_BACKOFF_S * Self::retry_spread(hops);
        let clear =
            self.max_burst_s + answer + MAX_DISC_TRIES as f64 * (backoff + disc + self.response_wait(hops, disc));
        self.busy_timeout_s(hops).max(clear)
    }

    fn busy_timeout_s(&self, hops: u8) -> f64 {
        self.cfg.busy_timeout_s.max(self.max_burst_s + 2.0 * self.retry_wait(hops, false, 0.0))
    }

    // ------------------------------------------------------------ host commands

    /// Start a connection, through the digipeaters `via` (in order, at most two).
    /// Fails if the link is not idle. While an aborted session's Disc is being repeated,
    /// the attempt waits until that ends (accepted, not started); a second CONNECT
    /// meanwhile fails.
    #[allow(clippy::result_unit_err)]
    pub fn connect(&mut self, now: f64, mycall: &str, target: &str, via: &[String]) -> Result<(), ()> {
        if via.len() > MAX_DIGIS {
            return Err(());
        }
        let q = QueuedConnect { mycall: norm_call(mycall), target: norm_call(target), via: via.to_vec() };
        match self.state {
            State::Idle => self.start_connect(now, q),
            State::Aborting(_) if self.queued_connect.is_none() => self.queued_connect = Some(q),
            _ => return Err(()),
        }
        Ok(())
    }

    fn start_connect(&mut self, now: f64, q: QueuedConnect) {
        let session = self.pick_session_id(now);
        self.state = State::Connecting {
            session,
            mycall: q.mycall,
            target: q.target,
            digis: q.via.iter().map(|c| norm_call(c)).collect(),
            attempts: 0,
            next_tx: now,
            sending: false,
        };
    }

    /// How long a caller avoids a session id after hearing it, so it does not share an id
    /// with a session it can hear (issue #27): as long as a digipeater keeps an idle
    /// session, and three of the longest keepalive intervals (a routed session's), so a
    /// quiet session's id is not freed.
    fn heard_id_s(&self) -> f64 {
        digi::IDLE_EXPIRY_S.max(3.0 * self.keepalive_s(MAX_DIGIS as u8))
    }

    /// A random non-zero session id not heard in the last [`Self::heard_id_s`]. If every
    /// id has been heard, any non-zero id.
    fn pick_session_id(&mut self, now: f64) -> u8 {
        let keep = self.heard_id_s();
        let free: Vec<u8> = (1..=255).filter(|&id| now - self.heard_ids[id as usize] > keep).collect();
        let n = self.rng.next_u32();
        if free.is_empty() {
            (n % 255 + 1) as u8
        } else {
            free[n as usize % free.len()]
        }
    }

    /// Graceful close: send everything still queued, then close the session.
    pub fn disconnect(&mut self, now: f64) {
        match &mut self.state {
            State::Idle => {}
            State::Aborting(_) => self.drop_queued_connect(),
            State::Connecting { .. } => self.close(now),
            State::Connected(s) => {
                s.closing.get_or_insert(now);
            }
        }
    }

    /// Hard stop. The host is told DISCONNECTED at once. A session's `Disc` still goes
    /// out, so the peer does not wait for a timeout: in our answer slot after the peer's
    /// next burst, or on a clear channel, repeated until the peer's DiscAck
    /// ([`State::Aborting`]). An ABORT then only drops a CONNECT waiting for that.
    pub fn abort(&mut self, now: f64) {
        match std::mem::replace(&mut self.state, State::Idle) {
            State::Idle => {}
            State::Aborting(s) => {
                self.state = State::Aborting(s);
                self.drop_queued_connect();
            }
            state @ State::Connecting { .. } => {
                self.state = state;
                self.close(now);
            }
            // A session we never answered (our ConnectAck has not gone out): the caller
            // has never heard us, so there is no one to send a Disc to.
            State::Connected(s) if !s.answered => {
                let closed = Closed::aborted(&s);
                self.state = State::Connected(s);
                self.close(now);
                self.closed = Some(closed);
            }
            State::Connected(mut s) => {
                // Only the Disc is left to send: due at once (no grace for host data).
                // The Discs a DISCONNECT has already sent count: MAX_DISC_TRIES in all.
                s.tx_buf.clear();
                s.in_flight.clear();
                s.closing = Some(now - self.cfg.disconnect_grace_s);
                s.retries = 0;
                // The Disc's ack offset covers what reached the host before the ABORT; its
                // bitmap is empty, so the peer takes nothing past that offset (data we
                // held out of order, now thrown away) as delivered.
                s.ack_bitmap = 0;
                // An answer still due goes as the Disc, in its slot. A burst we were about
                // to start goes as the Disc at its own time, still giving way to a busy
                // channel. The abort has its own deadline from the ABORT, whatever happens
                // meanwhile (our Discs, the peer's bursts); a busy timer already running
                // does not shorten it.
                s.abort_until = now + self.abort_s(&s.route);
                // We have used the id until now: a new session must not pick it.
                self.heard_ids[s.id as usize] = now;
                self.closed = Some(Closed::aborted(&s));
                self.state = State::Aborting(s);
                self.events.push(LinkEvent::Disconnected);
            }
        }
    }

    /// Drops a CONNECT waiting for an aborted session's Disc exchange, and tells the
    /// host, as for a connect attempt that ends.
    fn drop_queued_connect(&mut self) {
        if self.queued_connect.take().is_some() {
            self.events.push(LinkEvent::Disconnected);
        }
    }

    /// Bytes from the host's data port. Ignored when not connected. Like VARA,
    /// data written after DISCONNECT is still sent (until our Disc goes out).
    /// Every accepted write is followed by a BUFFER report, so a host can tell
    /// that a report is newer than its write.
    pub fn send_data(&mut self, data: &[u8]) {
        if let State::Connected(s) = &mut self.state {
            if s.disc_tries == 0 {
                s.tx_buf.extend(data);
                self.force_buffer = true;
            }
        }
    }

    /// A one-off `Disc` for the current session: if connected, or aborted with no Disc
    /// sent yet (the peer has had none).
    fn disc_burst(&self) -> Option<Burst> {
        let s = match &self.state {
            State::Connected(s) => s,
            State::Aborting(s) if s.disc_tries == 0 => s,
            _ => return None,
        };
        Some(Burst {
            header: Header { kind: Kind::Disc, session: s.id, flags: s.route.out_flags(), ..Header::default() },
            payload: Vec::new(),
        })
    }

    fn close(&mut self, now: f64) {
        if matches!(self.state, State::Aborting(_)) {
            self.end_abort(now);
            return;
        }
        if let State::Connected(s) = &self.state {
            self.closed = Some(Closed::of(s));
        }
        self.state = State::Idle;
        self.held_since = None;
        self.events.push(LinkEvent::Disconnected);
    }

    /// The aborted session's Disc exchange is over (DiscAck, the peer's own Disc
    /// answered, tries used up, or a busy channel held it back too long). The host
    /// already has its DISCONNECTED. A queued CONNECT starts here and now, so the link
    /// is never idle with a CONNECT still waiting: a ConnectReq, DISCONNECT, ABORT or
    /// CONNECT that comes before the next `poll` finds it under way.
    fn end_abort(&mut self, now: f64) {
        self.state = State::Idle;
        self.held_since = None;
        if let Some(q) = self.queued_connect.take() {
            self.start_connect(now, q);
        }
    }

    // ------------------------------------------------------------ receive

    fn level_for(&self, rx: &RxBurst) -> u8 {
        let by_snr = level_by_snr(&self.modem, rx.snr_db, rx.header.level);
        let failed = rx.payload.iter().any(|p| p.is_none());
        let cap = if failed { rx.header.level.saturating_sub(1) } else { u8::MAX };
        by_snr.min(cap).min(self.cfg.max_level)
    }

    /// Handle a decoded burst.
    pub(crate) fn on_burst(&mut self, now: f64, rx: &RxBurst) {
        let voided;
        let rx = if has_void(rx) {
            voided = without_voids(rx);
            &voided
        } else {
            rx
        };
        let h = rx.header;
        self.heard_ids[h.session as usize] = now;
        self.stats.codewords_received += rx.payload.len() as u64;
        self.stats.codewords_failed += rx.payload.iter().filter(|p| p.is_none()).count() as u64;
        let req = self.level_for(rx);
        let turnaround = self.cfg.turnaround_s;

        match &mut self.state {
            State::Idle => {
                if h.kind == Kind::ConnectReq {
                    self.on_connect_req(now, rx, req);
                } else if h.kind == Kind::Disc {
                    // Peer did not hear our DiscAck: answer again.
                    if let Some(c) = &self.closed {
                        if c.id == h.session && c.route.accepts(&h) {
                            self.oneshot = Some(Burst {
                                header: Header {
                                    kind: Kind::DiscAck,
                                    session: h.session,
                                    flags: c.route.out_flags(),
                                    ..Header::default()
                                },
                                payload: Vec::new(),
                            });
                        }
                    }
                }
            }
            State::Connecting { session, mycall, target, digis, .. } => {
                let route = Route { digis: digis.clone(), caller: true };
                if h.session != *session || !route.accepts(&h) {
                    return;
                }
                // Only a ConnectAck naming our request answers it. Other bursts carry no
                // call signs: one with our id may be another session's that uses the same
                // id. If the called station's ConnectAck is lost, it sends it again.
                let accepted = h.kind == Kind::ConnectAck
                    && parse_calls(rx).is_some_and(|(s, d, v)| s == *mycall && d == *target && v == *digis);
                if !accepted {
                    return;
                }
                let (id, local, remote) = (*session, mycall.clone(), target.clone());
                self.events.push(LinkEvent::Connected { source: local.clone(), destination: remote.clone() });
                // Let the called station speak first.
                let wait = self.response_wait(route.hops(), 0.0);
                let mut s = self.new_session(id, local, remote, route, now, req, h.req_level);
                s.phase = Phase::Wait { deadline: now + wait, retry: false };
                self.state = State::Connected(Box::new(s));
            }
            State::Connected(s) | State::Aborting(s) => {
                if h.session != s.id || !s.route.accepts(&h) {
                    return;
                }
                match h.kind {
                    Kind::ConnectReq if !s.route.caller => {
                        // Our ConnectAck was lost; send it again. Only to our caller: a
                        // request naming other stations or another path is another
                        // caller's that happens to use our session id.
                        let ours = parse_calls(rx).is_some_and(|(src, dst, digis)| {
                            src == s.remote && dst == norm_call(&s.local) && digis == s.route.digis
                        });
                        if !ours {
                            return;
                        }
                        s.last_heard = now;
                        self.held_since = None;
                        s.phase = Phase::Respond { at: now + turnaround, kind: Kind::ConnectAck, initiated: false };
                    }
                    Kind::ConnectReq | Kind::ConnectAck => {}
                    _ => self.on_session_burst(now, rx, req),
                }
            }
        }
    }

    fn on_connect_req(&mut self, now: f64, rx: &RxBurst, req: u8) {
        let Some((src, dst, digis)) = parse_calls(rx) else { return };
        let route = Route { digis, caller: false };
        // A routed request counts only once it has passed every digipeater, and its header
        // must agree with its payload: a VFM1 request unrouted, a VFM2 one routed.
        if !self.listen || !route.accepts(&rx.header) {
            return;
        }
        let Some(local) = self.mycalls.iter().find(|c| norm_call(c) == dst).cloned() else { return };
        // The caller of a call we aborted, still retrying within its call window.
        let aborted = self.closed.as_ref().is_some_and(|c| {
            c.id == rx.header.session
                && c.remote == src
                && norm_call(&c.local) == dst
                && c.route == route
                && now < c.refuse_until
        });
        if aborted {
            return;
        }
        self.events.push(LinkEvent::Pending);
        self.events.push(LinkEvent::Connected { source: src.clone(), destination: local.clone() });
        // The caller has not heard us yet, so its requested level is just a default.
        // Our own measurement of its request is a better guess (same radio path).
        let window = self.connect_window_s(route.hops());
        let mut s = self.new_session(rx.header.session, local, src, route, now, req, req);
        s.ack_until = now + window;
        s.call_until = now + window;
        s.phase = Phase::Respond { at: now + self.cfg.turnaround_s, kind: Kind::ConnectAck, initiated: false };
        self.state = State::Connected(Box::new(s));
    }

    #[allow(clippy::too_many_arguments)]
    fn new_session(
        &self,
        id: u8,
        local: String,
        remote: String,
        route: Route,
        now: f64,
        req: u8,
        peer_req: u8,
    ) -> Session {
        let answered = route.caller;
        Session {
            id,
            local,
            remote,
            route,
            last_tx_s: 0.0,
            tx_base: 0,
            tx_buf: VecDeque::new(),
            delivered: Vec::new(),
            in_flight: Vec::new(),
            sent_high: 0,
            burst_id: 0,
            rx_next: 0,
            rx_ooo: BTreeMap::new(),
            ack_burst: 0,
            ack_bitmap: 0,
            peer_level: peer_req.min(self.cfg.max_level),
            req_level: req,
            phase: Phase::Idle,
            retries: 0,
            closing: None,
            disc_tries: 0,
            answered,
            last_heard: now,
            ack_until: f64::NEG_INFINITY,
            call_until: f64::NEG_INFINITY,
            abort_until: f64::INFINITY,
        }
    }

    fn on_session_burst(&mut self, now: f64, rx: &RxBurst, req: u8) {
        let h = rx.header;
        let turnaround = self.cfg.turnaround_s;
        let aborting = matches!(self.state, State::Aborting(_));
        self.held_since = None;
        let (State::Connected(s) | State::Aborting(s)) = &mut self.state else { return };
        s.last_heard = now;
        s.ack_until = f64::NEG_INFINITY;
        s.retries = 0;
        s.peer_level = h.req_level.min(self.cfg.max_level);
        s.req_level = req;

        // Acknowledgements for our data.
        if h.ack_burst == s.burst_id {
            for (i, r) in s.in_flight.iter().enumerate() {
                if h.bitmap & (1 << i) != 0 {
                    s.delivered.push(*r);
                }
            }
            s.in_flight.clear();
            s.delivered = merge(std::mem::take(&mut s.delivered));
        }
        let acked = expand(h.ack_offset, s.tx_base);
        if acked > s.tx_base && acked <= s.tx_base + s.tx_buf.len() as u64 {
            s.tx_buf.drain(..(acked - s.tx_base) as usize);
            s.tx_base = acked;
        }
        s.delivered.retain(|&(_, end)| end > s.tx_base);

        // Data from the peer. After ABORT it is neither kept nor acked: the Disc acks only
        // what was acked before, so the peer does not take data we threw away as
        // delivered.
        let mut delivered = Vec::new();
        if !aborting {
            let mut bitmap = 0u32;
            for (i, cw) in rx.payload.iter().enumerate() {
                let Some(cw) = cw else { continue };
                bitmap |= 1 << i;
                let off = u32::from_be_bytes([0, cw[0], cw[1], cw[2]]);
                let len = (cw[3] as usize).min(cw.len() - SEGMENT_HEADER);
                let start = expand(off, s.rx_next);
                let data = &cw[SEGMENT_HEADER..SEGMENT_HEADER + len];
                if start + len as u64 > s.rx_next && len > 0 {
                    // A resend at another level can start at the same offset and hold more
                    // bytes. We acknowledge it, so the sender will not send those bytes
                    // again: keep the longer copy.
                    let kept = s.rx_ooo.entry(start).or_default();
                    if data.len() > kept.len() {
                        *kept = data.to_vec();
                    }
                }
            }
            while let Some((&start, _)) = s.rx_ooo.iter().next() {
                if start > s.rx_next {
                    break;
                }
                let data = s.rx_ooo.remove(&start).unwrap();
                let end = start + data.len() as u64;
                if end > s.rx_next {
                    delivered.extend_from_slice(&data[(s.rx_next - start) as usize..]);
                    s.rx_next = end;
                }
            }
            s.ack_burst = h.burst_id;
            s.ack_bitmap = bitmap;
        }

        match h.kind {
            Kind::Disc => {
                s.phase = Phase::Respond { at: now + turnaround, kind: Kind::DiscAck, initiated: false };
            }
            Kind::DiscAck => {
                if s.closing.is_some() {
                    self.stats.bytes_delivered += delivered.len() as u64;
                    if !delivered.is_empty() {
                        self.events.push(LinkEvent::Data(delivered));
                    }
                    self.close(now);
                    return;
                }
            }
            // Aborted: answer whatever the peer sent with the Disc, in our slot.
            _ if aborting => {
                s.phase = Phase::Respond { at: now + turnaround, kind: Kind::Disc, initiated: false };
            }
            _ => {
                let has_data = s.has_data();
                s.phase = if h.ncw > 0 || h.flags & FLAG_POLL != 0 || has_data {
                    Phase::Respond { at: now + turnaround, kind: Kind::Data, initiated: false }
                } else {
                    Phase::Idle
                };
            }
        }
        self.stats.bytes_delivered += delivered.len() as u64;
        if !delivered.is_empty() {
            self.events.push(LinkEvent::Data(delivered));
        }
    }

    // ------------------------------------------------------------ transmit

    /// Called regularly while not transmitting. Returns a burst to transmit now.
    pub(crate) fn poll(&mut self, now: f64, busy: bool) -> Option<Burst> {
        // A new session, or a Disc too late to send, drops the parting Disc.
        if !matches!(self.state, State::Idle) || self.parting.as_ref().is_some_and(|&(until, _)| now > until) {
            self.parting = None;
        }
        let quiet_parting = || if busy { None } else { self.parting.take().map(|(_, b)| b) };
        if let Some(b) = self.oneshot.take().or_else(quiet_parting) {
            self.last_tx = TxInfo { kind: b.header.kind, ..TxInfo::default() };
            self.stats.bursts_sent += 1;
            return Some(b);
        }
        let cfg = self.cfg.clone();
        let backoff = self.rand_s(RETRY_BACKOFF_S);
        let (keepalive, busy_timeout) = match &self.state {
            State::Connected(s) | State::Aborting(s) => {
                (self.keepalive_s(s.route.hops()), self.busy_timeout_s(s.route.hops()))
            }
            State::Connecting { digis, .. } => (cfg.keepalive_s, self.busy_timeout_s(digis.len() as u8)),
            State::Idle => (cfg.keepalive_s, f64::INFINITY),
        };
        if !self.wants_to_send(now, keepalive) {
            self.held_since = None;
        } else if busy {
            self.held_since.get_or_insert(now);
        }
        // A busy channel holds back everything we would start ourselves: a connect
        // request, retries, data, keepalives, a pending Disc. If it has held something
        // back this long with no progress, give up rather than wait for the channel
        // forever. A peer we hear but cannot decode is no progress. An aborted session has
        // its own deadline instead (see `abort_s`).
        let timed_out = match &self.state {
            State::Aborting(s) => now > s.abort_until,
            _ => self.held_since.is_some_and(|t| now - t > busy_timeout),
        };
        if timed_out {
            // Tell the peer, in case it is there but we cannot decode it under the carrier.
            // An aborted session gets that Disc only if none of its own has gone out. A
            // CONNECT that waited for the abort starts now, and that drops it (as any new
            // session does).
            self.parting = self.disc_burst().map(|b| (now + busy_timeout, b));
            self.close(now);
            return None;
        }
        let aborting = matches!(self.state, State::Aborting(_));
        match &mut self.state {
            State::Idle => None,
            State::Connecting { session, mycall, target, digis, attempts, next_tx, sending } => {
                if *sending {
                    return None;
                }
                if busy {
                    *next_tx = next_tx.max(now + BUSY_RECHECK_S);
                    return None;
                }
                if now < *next_tx {
                    return None;
                }
                if *attempts >= cfg.connect_attempts {
                    self.close(now);
                    return None;
                }
                *attempts += 1;
                *sending = true;
                let flags = Route { digis: digis.clone(), caller: true }.out_flags();
                let burst = Burst {
                    header: Header {
                        kind: Kind::ConnectReq,
                        session: *session,
                        level: 0,
                        ncw: 1,
                        req_level: cfg.start_level,
                        flags,
                        ..Header::default()
                    },
                    payload: vec![calls_payload(mycall, target, digis)],
                };
                self.last_tx = TxInfo { kind: Kind::ConnectReq, ncw: 1, retry: *attempts - 1, ..TxInfo::default() };
                self.stats.bursts_sent += 1;
                Some(burst)
            }
            State::Connected(s) | State::Aborting(s) => {
                // Advance timers.
                match s.phase {
                    Phase::Sending { .. } => return None,
                    Phase::Wait { deadline, retry } => {
                        if busy {
                            s.phase = Phase::Wait { deadline: deadline.max(now + BUSY_RECHECK_S), retry };
                            return None;
                        }
                        if now < deadline {
                            return None;
                        }
                        if retry {
                            s.retries += 1;
                            self.stats.retries += 1;
                            if s.retries > cfg.max_retries || (s.closing.is_some() && s.disc_tries >= MAX_DISC_TRIES) {
                                self.close(now);
                                return None;
                            }
                            // Unanswered: the peer may not have decoded us. Step down.
                            s.peer_level = s.peer_level.saturating_sub(1);
                            // A called station not yet answered sends its ConnectAck again: the
                            // caller takes nothing else as the answer to its request.
                            let kind = if aborting || s.closing.is_some() && s.disc_tries > 0 {
                                Kind::Disc
                            } else if now < s.ack_until {
                                Kind::ConnectAck
                            } else {
                                Kind::Data
                            };
                            // Through digipeaters, both ends may retry while a repeat is
                            // on the air; a wider spread keeps them from colliding again.
                            // On any route, a busy channel is most likely the peer's burst
                            // (its retry, or coming through a digipeater), so the retry
                            // gives way to it and answers it instead. Without that, two ends
                            // that retry together collide on every retry: their waits
                            // differ by less than a burst.
                            let hops = s.route.hops();
                            let at = now + backoff * Self::retry_spread(hops);
                            s.phase = Phase::Respond { at, kind, initiated: true };
                        } else {
                            s.phase = Phase::Idle;
                        }
                        return None;
                    }
                    Phase::Respond { at, .. } if now < at => return None,
                    Phase::Respond { .. } => {}
                    Phase::Idle => {
                        if busy {
                            return None;
                        }
                        let ready_to_close =
                            s.closing.is_some_and(|t| now >= t + cfg.disconnect_grace_s) && s.tx_buf.is_empty();
                        if ready_to_close {
                            s.phase = Phase::Respond { at: now, kind: Kind::Disc, initiated: true };
                        } else if s.has_data() || s.keepalive_due(now, keepalive) {
                            // Start a burst on a quiet channel, after a short random wait.
                            s.phase =
                                Phase::Respond { at: now + 0.05 + backoff * 0.5, kind: Kind::Data, initiated: true };
                        }
                        return None;
                    }
                }
                let Phase::Respond { kind, initiated, .. } = s.phase else { return None };
                // Aborted: the only burst left is the Disc, or a DiscAck if the peer sent
                // its own Disc.
                let kind = if aborting && kind != Kind::DiscAck { Kind::Disc } else { kind };
                // Every Disc counts, the ones in an answer slot too: after the last one,
                // the abort ends instead.
                if aborting && kind == Kind::Disc && s.disc_tries >= MAX_DISC_TRIES {
                    self.close(now);
                    return None;
                }
                if busy && initiated {
                    // Someone else started talking; go back to listening.
                    s.phase = Phase::Idle;
                    return None;
                }
                let retry = s.retries;
                let (burst, info) = s.build(kind, &self.modem, now, keepalive);
                s.last_tx_s = self.modem.burst_seconds(burst.header.level, burst.payload.len()) as f64;
                let needs_reply = matches!(kind, Kind::Disc)
                    || burst.header.ncw > 0 && kind == Kind::Data
                    || burst.header.flags & FLAG_POLL != 0;
                if kind == Kind::Disc {
                    s.disc_tries += 1;
                }
                s.answered = true;
                s.phase = Phase::Sending { kind, retry: needs_reply, initiated };
                self.stats.bursts_sent += 1;
                self.stats.payload_bytes_sent += info.bytes as u64;
                self.stats.bytes_resent += info.resent_bytes as u64;
                self.last_tx = TxInfo { retry, ..info };
                Some(burst)
            }
        }
    }

    /// Called when the burst returned by `poll` has finished transmitting.
    pub(crate) fn tx_done(&mut self, now: f64) {
        self.held_since = None;
        let timeout = self.cfg.response_timeout_s;
        let connect_s = self.connect_s();
        // Draw the retry jitter only while connecting, as before digipeating, so direct
        // sessions keep their random sequence.
        let jitter = if matches!(self.state, State::Connecting { .. }) { self.rand_s(CONNECT_JITTER_S) } else { 0.0 };
        let (wait, echo) = match &self.state {
            State::Connecting { digis, .. } => (self.connect_wait_s(digis.len() as u8), 0.0),
            State::Connected(s) | State::Aborting(s) => {
                // Time for every digipeater to repeat our ConnectAck.
                let hops = s.route.hops() as f64;
                let echo = hops * (connect_s + self.cfg.turnaround_s + self.cfg.hop_margin_s);
                // A burst we started ourselves, or a retry, may have met one of the peer's.
                // Not a Disc: the digipeaters forget a session soon after its DiscAck, so
                // a Disc resent much later would no longer be repeated. A Disc and the
                // peer's burst can still meet, but the Disc is tried only 4 times.
                let started = matches!(s.phase, Phase::Sending { retry: true, initiated: true, kind }
                    if kind != Kind::Disc);
                let wait = if started {
                    self.retry_wait(s.route.hops(), s.route.caller, s.last_tx_s)
                } else {
                    self.response_wait(s.route.hops(), s.last_tx_s)
                };
                (wait, echo)
            }
            State::Idle => (timeout, 0.0),
        };
        match &mut self.state {
            State::Connecting { next_tx, sending, .. } => {
                *sending = false;
                *next_tx = now + wait + jitter;
            }
            State::Connected(s) | State::Aborting(s) => {
                if let Phase::Sending { kind, retry, .. } = s.phase {
                    match kind {
                        Kind::DiscAck => {
                            self.close(now);
                        }
                        Kind::ConnectAck => {
                            // We may speak first. Through digipeaters, only once they have
                            // repeated the ConnectAck: our burst would collide with a repeat.
                            s.phase = if echo > 0.0 {
                                Phase::Wait { deadline: now + echo, retry: false }
                            } else {
                                Phase::Idle
                            };
                        }
                        _ => s.phase = Phase::Wait { deadline: now + wait, retry },
                    }
                }
            }
            State::Idle => {}
        }
    }
}

impl Session {
    /// True once the peer has been quiet long enough for a keepalive (half as long
    /// again on the called side).
    fn keepalive_due(&self, now: f64, keepalive: f64) -> bool {
        now - self.last_heard > keepalive * if self.route.caller { 1.0 } else { 1.5 }
    }

    /// True if there are bytes the peer has not received.
    fn has_data(&self) -> bool {
        let covered: u64 = self.delivered.iter().map(|(a, b)| b - a.max(&self.tx_base)).sum();
        (self.tx_buf.len() as u64) > covered
    }

    /// Byte ranges to send, in order, skipping ranges the peer already has.
    fn pending_ranges(&self) -> Vec<(u64, u64)> {
        let mut out = Vec::new();
        let mut pos = self.tx_base;
        let end = self.tx_base + self.tx_buf.len() as u64;
        let mut d = self.delivered.clone();
        d.sort_unstable();
        for (a, b) in d {
            if a > pos {
                out.push((pos, a.min(end)));
            }
            pos = pos.max(b);
        }
        if pos < end {
            out.push((pos, end));
        }
        out
    }

    fn build(&mut self, kind: Kind, modem: &Modem, now: f64, keepalive: f64) -> (Burst, TxInfo) {
        self.burst_id = (self.burst_id + 1) % 8;
        let level = if kind == Kind::ConnectAck { 0 } else { self.peer_level };
        let mut payload = Vec::new();
        if kind == Kind::ConnectAck {
            payload.push(calls_payload(&self.remote, &self.local, &self.route.digis));
        }
        let mut info = TxInfo { kind, level, ..TxInfo::default() };
        self.in_flight.clear();
        if kind == Kind::Data {
            let cw_bytes = Modem::payload_bytes(level);
            let cap = cw_bytes - SEGMENT_HEADER;
            let max = modem.max_ncw(level);
            'outer: for (a, b) in self.pending_ranges() {
                let mut pos = a;
                while pos < b {
                    if payload.len() >= max {
                        break 'outer;
                    }
                    let len = ((b - pos) as usize).min(cap);
                    let off = (pos % OFFSET_MOD) as u32;
                    let mut cw = Vec::with_capacity(cw_bytes);
                    cw.extend_from_slice(&off.to_be_bytes()[1..]);
                    cw.push(len as u8);
                    let i = (pos - self.tx_base) as usize;
                    cw.extend(self.tx_buf.range(i..i + len));
                    cw.resize(cw_bytes, 0);
                    payload.push(cw);
                    self.in_flight.push((pos, pos + len as u64));
                    info.bytes += len;
                    info.resent_bytes += (self.sent_high.min(pos + len as u64)).saturating_sub(pos) as usize;
                    self.sent_high = self.sent_high.max(pos + len as u64);
                    pos += len as u64;
                }
            }
        }
        info.ncw = payload.len();
        let poll = kind == Kind::Data && payload.is_empty() && now - self.last_heard > keepalive;
        let header = Header {
            kind,
            session: self.id,
            level: if payload.is_empty() { 0 } else { level },
            ncw: payload.len() as u8,
            req_level: self.req_level,
            burst_id: self.burst_id,
            ack_burst: self.ack_burst,
            ack_offset: (self.rx_next % OFFSET_MOD) as u32,
            bitmap: self.ack_bitmap,
            flags: (if poll { FLAG_POLL } else { 0 }) | self.route.out_flags(),
        };
        (Burst { header, payload }, info)
    }
}

/// Sort and merge overlapping or touching ranges.
fn merge(mut r: Vec<(u64, u64)>) -> Vec<(u64, u64)> {
    r.sort_unstable();
    let mut out: Vec<(u64, u64)> = Vec::with_capacity(r.len());
    for (a, b) in r {
        match out.last_mut() {
            Some(last) if a <= last.1 => last.1 = last.1.max(b),
            _ => out.push((a, b)),
        }
    }
    out
}

/// Expand a 24-bit wrapped offset to the 64-bit value closest to `near`.
fn expand(off: u32, near: u64) -> u64 {
    let base = near - near % OFFSET_MOD;
    let cands = [base.wrapping_sub(OFFSET_MOD), base, base + OFFSET_MOD];
    cands.into_iter().map(|c| c.wrapping_add(off as u64)).min_by_key(|&c| c.abs_diff(near)).unwrap()
}

/// ConnectReq/ConnectAck payload, zero padded:
/// - direct: `"VFM1" | len | source | len | destination`
/// - through digipeaters: `"VFM2" | len | source | len | destination | count | (len | digi) × count`
///
/// Stations that predate digipeating ignore `VFM2`, so they never answer a routed
/// request they hear directly.
fn calls_payload(src: &str, dst: &str, digis: &[String]) -> Vec<u8> {
    let mut p = if digis.is_empty() { b"VFM1".to_vec() } else { b"VFM2".to_vec() };
    for c in [src, dst] {
        p.push(c.len() as u8);
        p.extend_from_slice(c.as_bytes());
    }
    if !digis.is_empty() {
        p.push(digis.len() as u8);
        for c in digis {
            p.push(c.len() as u8);
            p.extend_from_slice(c.as_bytes());
        }
    }
    p.resize(Modem::payload_bytes(0), 0);
    p
}

fn parse_calls(rx: &RxBurst) -> Option<(String, String, Vec<String>)> {
    parse_calls_payload(rx.payload.first()?.as_ref()?)
}

/// Source, destination and digipeaters of a ConnectReq/ConnectAck payload.
fn parse_calls_payload(p: &[u8]) -> Option<(String, String, Vec<String>)> {
    let routed = match p.get(..4)? {
        b"VFM1" => false,
        b"VFM2" => true,
        _ => return None,
    };
    let call = |i: &mut usize| -> Option<String> {
        let n = *p.get(*i)? as usize;
        let s = std::str::from_utf8(p.get(*i + 1..*i + 1 + n)?).ok()?;
        *i += 1 + n;
        Some(norm_call(s))
    };
    let mut i = 4;
    let src = call(&mut i)?;
    let dst = call(&mut i)?;
    let mut digis = Vec::new();
    if routed {
        let count = *p.get(i)? as usize;
        i += 1;
        if count == 0 || count > MAX_DIGIS {
            return None;
        }
        for _ in 0..count {
            digis.push(call(&mut i)?);
        }
    }
    Some((src, dst, digis))
}

/// A codeword that stands for one a digipeater could not decode.
fn is_void(cw: &[u8]) -> bool {
    cw.get(3) == Some(&VOID_LEN)
}

/// A data-carrying burst with void codewords (connect payloads are never void).
fn has_void(rx: &RxBurst) -> bool {
    !matches!(rx.header.kind, Kind::ConnectReq | Kind::ConnectAck)
        && rx.payload.iter().any(|p| p.as_deref().is_some_and(is_void))
}

/// The burst with void codewords marked as failed.
fn without_voids(rx: &RxBurst) -> RxBurst {
    let mut out = rx.clone();
    for p in &mut out.payload {
        if p.as_deref().is_some_and(is_void) {
            *p = None;
        }
    }
    out
}

/// A segment that marks a codeword the digipeater could not decode.
fn void_codeword(level: u8) -> Vec<u8> {
    let mut cw = vec![0u8; Modem::payload_bytes(level)];
    cw[3] = VOID_LEN;
    cw
}

#[cfg(test)]
mod tests;
