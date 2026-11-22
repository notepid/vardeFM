//! Modems driven only through the C ABI: a full session, and what a host may do
//! from inside a callback. The tests that need no receiver also run under Miri.

use std::cell::Cell;
use std::collections::VecDeque;
use std::ffi::{c_char, c_void, CStr, CString};
use std::rc::Rc;
use std::sync::Mutex;
use vardefm::{
    vfm_advance, vfm_command, vfm_config_default, vfm_free, vfm_new, vfm_rx_audio, vfm_send, vfm_tx_played,
    vfm_version, VfmCallbacks, VfmModem, VfmRxReport, VfmTxReport, VFM_KIND_CONNECT_REQ, VFM_MODE_WIDE, VFM_RATE,
};

const CHUNK: usize = 480;
const KEYUP: usize = 5;

#[derive(Default)]
struct Side {
    queue: VecDeque<f32>,
    /// Samples that really went out. Flushed ones are not counted.
    played: u64,
    /// The sound card takes no samples (stuck driver).
    stuck: bool,
    ptt: bool,
    keyed_for: usize,
    /// True if every `tx_audio` so far arrived while PTT was already on.
    ptt_before_audio: bool,
    audio_bursts: usize,
    /// Ticks since the last sample went out.
    since_audio: usize,
    /// `since_audio` at each un-key.
    tails: Vec<usize>,
    lines: Vec<String>,
    data: Vec<u8>,
    logs: Vec<String>,
    tx_kinds: Vec<u8>,
    rx_kinds: Vec<u8>,
    /// The callbacks in the order they fired.
    events: Vec<&'static str>,
}

/// What a callback tells the test's hook.
enum Event<'a> {
    TxAudio,
    TxFlush,
    Ptt(bool),
    Line(&'a str),
    Other,
}

/// The `ctx` of a modem: what the callbacks record, and a hook that runs at the
/// end of each callback and may call back into the modem.
struct Host {
    side: Mutex<Side>,
    modem: Cell<*mut VfmModem>,
    hook: fn(&Host, Event),
}

impl Host {
    /// A host and its modem.
    fn open(seed: u32, hook: fn(&Host, Event)) -> Rc<Host> {
        let host = Rc::new(Host { side: Mutex::default(), modem: Cell::new(std::ptr::null_mut()), hook });
        let mut cfg = unsafe {
            let mut d = std::mem::zeroed();
            vfm_config_default(&mut d);
            d
        };
        cfg.seed = seed;
        cfg.decode_threads = 1;
        if cfg!(miri) {
            // Miri takes minutes to modulate one second of audio: make the bursts short.
            cfg.mode = VFM_MODE_WIDE;
            cfg.txdelay_s = 0.0;
        }
        let cbs = VfmCallbacks {
            tx_audio: Some(on_tx_audio),
            tx_flush: Some(on_tx_flush),
            ptt: Some(on_ptt),
            host_line: Some(on_host_line),
            host_data: Some(on_host_data),
            log: Some(on_log),
            tx_report: Some(on_tx_report),
            rx_report: Some(on_rx_report),
        };
        let m = unsafe { vfm_new(&cfg, &cbs, Rc::as_ptr(&host) as *mut c_void) };
        assert!(!m.is_null());
        host.modem.set(m);
        host
    }

    fn side(&self) -> std::sync::MutexGuard<'_, Side> {
        self.side.lock().unwrap()
    }

    fn cmd(&self, line: &str) {
        let c = CString::new(line).unwrap();
        unsafe { vfm_command(self.modem.get(), c.as_ptr()) };
    }

    fn advance(&self, samples: u64) {
        unsafe { vfm_advance(self.modem.get(), samples) };
    }

    fn rx(&self, audio: &[f32]) {
        unsafe { vfm_rx_audio(self.modem.get(), audio.as_ptr(), audio.len()) };
    }

    fn send(&self, data: &[u8]) {
        unsafe { vfm_send(self.modem.get(), data.as_ptr(), data.len()) };
    }

    /// 10 ms of the sound card: play what is queued, tell the modem how much has
    /// been played, and return what the radio puts on the air.
    fn tick(&self) -> Vec<f32> {
        let mut out = vec![0.0f32; CHUNK];
        let played = {
            let mut s = self.side();
            if s.ptt {
                s.keyed_for += 1;
            }
            let mut n = 0;
            while n < CHUNK && !s.stuck {
                let Some(x) = s.queue.pop_front() else { break };
                out[n] = x;
                n += 1;
            }
            s.played += n as u64;
            s.since_audio = if n > 0 { 0 } else { s.since_audio + 1 };
            if !(s.ptt && s.keyed_for > KEYUP) {
                out.fill(0.0);
            }
            s.played
        };
        unsafe { vfm_tx_played(self.modem.get(), played) };
        out
    }

    fn free(&self) {
        unsafe { vfm_free(self.modem.replace(std::ptr::null_mut())) };
    }
}

/// Record a callback, then run the hook (with the lock released: it may call the modem).
fn fired(ctx: *mut c_void, name: &'static str, event: Event, record: impl FnOnce(&mut Side)) {
    let host = unsafe { &*(ctx as *const Host) };
    {
        let mut s = host.side();
        s.events.push(name);
        record(&mut s);
    }
    (host.hook)(host, event);
}

extern "C" fn on_tx_audio(ctx: *mut c_void, samples: *const f32, n: usize) {
    let audio = unsafe { std::slice::from_raw_parts(samples, n) };
    fired(ctx, "tx_audio", Event::TxAudio, |s| {
        s.ptt_before_audio = s.ptt && (s.audio_bursts == 0 || s.ptt_before_audio);
        s.audio_bursts += 1;
        s.queue.extend(audio);
    });
}

extern "C" fn on_tx_flush(ctx: *mut c_void) {
    // The dropped samples were not played: `played` stays as it is.
    fired(ctx, "tx_flush", Event::TxFlush, |s| s.queue.clear());
}

extern "C" fn on_ptt(ctx: *mut c_void, on: bool) {
    fired(ctx, if on { "ptt on" } else { "ptt off" }, Event::Ptt(on), |s| {
        if on && !s.ptt {
            s.keyed_for = 0;
        }
        if !on && s.ptt {
            let tail = s.since_audio;
            s.tails.push(tail);
        }
        s.ptt = on;
    });
}

extern "C" fn on_host_line(ctx: *mut c_void, line: *const c_char) {
    let text = unsafe { CStr::from_ptr(line) }.to_string_lossy().into_owned();
    fired(ctx, "host_line", Event::Line(&text), |s| s.lines.push(text.clone()));
}

extern "C" fn on_host_data(ctx: *mut c_void, data: *const u8, n: usize) {
    let bytes = unsafe { std::slice::from_raw_parts(data, n) };
    fired(ctx, "host_data", Event::Other, |s| s.data.extend_from_slice(bytes));
}

extern "C" fn on_log(ctx: *mut c_void, line: *const c_char) {
    let text = unsafe { CStr::from_ptr(line) }.to_string_lossy().into_owned();
    fired(ctx, "log", Event::Other, |s| s.logs.push(text));
}

extern "C" fn on_tx_report(ctx: *mut c_void, r: *const VfmTxReport) {
    let kind = unsafe { (*r).kind };
    fired(ctx, "tx_report", Event::Other, |s| s.tx_kinds.push(kind));
}

extern "C" fn on_rx_report(ctx: *mut c_void, r: *const VfmRxReport) {
    let kind = unsafe { (*r).kind };
    fired(ctx, "rx_report", Event::Other, |s| s.rx_kinds.push(kind));
}

fn no_hook(_: &Host, _: Event) {}

/// A station that has started to call one that is not there, stepped until it keys.
fn calling(seed: u32, hook: fn(&Host, Event)) -> Rc<Host> {
    let host = Host::open(seed, hook);
    host.cmd("MYCALL LA1AAA");
    host.cmd("CONNECT LA1AAA LA9ZZZ");
    for _ in 0..500 {
        if host.side().audio_bursts > 0 || host.modem.get().is_null() {
            break;
        }
        host.advance(CHUNK as u64);
    }
    host
}

/// Two modems complete a session. The keyed station advances its clock with
/// `vfm_advance` only.
#[test]
#[cfg_attr(miri, ignore = "the receiver is too slow under Miri")]
fn session_through_c_abi_with_advance_while_keyed() {
    let hosts = [Host::open(1, no_hook), Host::open(2, no_hook)];
    hosts[0].cmd("MYCALL LA1AAA");
    hosts[0].cmd("CONNECT LA1AAA LA2BBB");
    hosts[1].cmd("MYCALL LA2BBB");
    hosts[1].cmd("LISTEN ON");

    let to_a: Vec<u8> = (0..1500u32).map(|i| (i * 7) as u8).collect();
    let to_b: Vec<u8> = (0..1000u32).map(|i| (i * 13 + 5) as u8).collect();
    let mut air = [vec![0.0f32; CHUNK], vec![0.0f32; CHUNK]];
    let (mut sent_b, mut sent_a, mut disc) = (false, false, false);

    for _ in 0..60_000 {
        for i in 0..2 {
            let keyed = hosts[i].side().ptt;
            if i == 0 && keyed {
                hosts[i].advance(CHUNK as u64);
            } else {
                hosts[i].rx(&air[1 - i]);
            }
        }
        for i in 0..2 {
            air[i] = hosts[i].tick();
        }

        if !sent_b && hosts[1].side().lines.iter().any(|l| l.starts_with("CONNECTED")) {
            hosts[1].send(&to_a);
            sent_b = true;
        }
        if !sent_a && hosts[0].side().data.len() >= to_a.len() {
            hosts[0].send(&to_b);
            sent_a = true;
        }
        if !disc
            && hosts[1].side().data.len() >= to_b.len()
            && hosts[0].side().lines.last().is_some_and(|l| l == "BUFFER 0")
        {
            hosts[0].cmd("DISCONNECT");
            disc = true;
        }
        let done = hosts.iter().all(|h| h.side().lines.iter().any(|l| l == "DISCONNECTED"));
        if done {
            {
                let (a, b) = (hosts[0].side(), hosts[1].side());
                assert_eq!(a.data, to_a);
                assert_eq!(b.data, to_b);
                assert!(a.ptt_before_audio, "PTT must be keyed before tx_audio");
                assert_eq!(a.tx_kinds.first(), Some(&VFM_KIND_CONNECT_REQ));
                assert_eq!(b.rx_kinds.first(), Some(&VFM_KIND_CONNECT_REQ));
                for s in [&a, &b] {
                    assert!(!s.logs.iter().any(|l| l.contains("watchdog")), "{:?}", s.logs);
                }
            }
            hosts[0].free();
            hosts[1].free();
            return;
        }
    }
    panic!("session did not finish");
}

/// `vfm_command` from inside `host_line` is queued and drained by the outer call.
#[test]
fn command_from_inside_host_line_is_queued() {
    fn hook(host: &Host, e: Event) {
        // Only for the first OK, or the VERSION reply's OK would loop forever.
        if matches!(e, Event::Line("OK")) && host.side().lines.len() == 1 {
            host.cmd("VERSION");
        }
    }
    let host = Host::open(9, hook);
    host.cmd("MYCALL LA1AAA");
    let lines = host.side().lines.clone();
    assert_eq!(lines.len(), 3, "{lines:?}");
    assert!(lines[2].starts_with("VERSION VardeFM "), "{lines:?}");
    host.free();
}

/// `ptt`, `tx_audio` and `tx_flush` fire in the middle of an engine step. The host
/// may still submit commands and data and report its played count from there;
/// feeding audio or advancing the clock from there is refused.
#[test]
fn calls_from_inside_ptt_and_audio_callbacks() {
    fn hook(host: &Host, e: Event) {
        match e {
            Event::Ptt(true) => host.cmd("VERSION"),
            Event::TxAudio => {
                host.cmd("LISTEN ON");
                host.send(b"dropped: not connected");
                host.rx(&[0.0; 16]);
                host.advance(CHUNK as u64);
            }
            Event::TxFlush => {
                let played = host.side().played;
                unsafe { vfm_tx_played(host.modem.get(), played) };
            }
            _ => {}
        }
    }
    let host = calling(3, hook);
    {
        let s = host.side();
        assert!(s.ptt && s.audio_bursts == 1, "{:?}", s.events);
        // MYCALL, CONNECT, and the two commands from inside the callbacks.
        assert_eq!(s.lines.iter().filter(|l| *l == "OK").count(), 4, "{:?}", s.lines);
        assert!(s.lines.iter().any(|l| l.starts_with("VERSION VardeFM ")), "{:?}", s.lines);
        for refused in ["vfm_rx_audio ignored", "vfm_advance ignored"] {
            assert!(s.logs.iter().any(|l| l.starts_with(refused)), "{:?}", s.logs);
        }
    }
    // Nothing gets played: the watchdog flushes and un-keys.
    host.advance(10 * VFM_RATE as u64);
    {
        let s = host.side();
        assert!(!s.ptt, "{:?}", s.events);
        assert!(s.events.contains(&"tx_flush"), "{:?}", s.events);
        assert!(s.logs.iter().any(|l| l.contains("watchdog")), "{:?}", s.logs);
    }
    host.free();
}

/// A host that counts only the samples it really played is behind the queued
/// count for good after a flush. The next burst must still end on time.
#[test]
#[cfg_attr(miri, ignore = "two bursts take too long to modulate under Miri")]
fn burst_after_a_watchdog_flush_unkeys_on_time() {
    let host = calling(4, no_hook);
    host.side().stuck = true;
    for _ in 0..3000 {
        host.advance(CHUNK as u64);
        host.tick();
        let mut s = host.side();
        // Once the watchdog has un-keyed the first burst, the sound card works again.
        s.stuck = s.tails.is_empty();
        if s.tails.len() == 2 {
            break;
        }
    }
    {
        let s = host.side();
        assert_eq!(s.tails.len(), 2, "no second burst: {:?}", s.logs);
        assert_eq!(s.logs.iter().filter(|l| l.contains("watchdog")).count(), 1, "{:?}", s.logs);
        // Released 50 ms (5 ticks) after the last sample, give or take a tick.
        assert!((5..=7).contains(&s.tails[1]), "PTT released {} ticks after the last sample", s.tails[1]);
    }
    host.free();
}

/// `vfm_free` from inside `host_line`: the rest of the output is dropped, and the
/// outer call flushes, un-keys and frees.
#[test]
fn free_from_inside_host_line() {
    fn hook(host: &Host, e: Event) {
        if matches!(e, Event::Line("OK")) {
            host.side().events.clear();
            host.free();
        }
    }
    let host = Host::open(5, hook);
    // Answers OK, then the version: the OK frees the modem.
    host.cmd("VERSION");
    let s = host.side();
    assert_eq!(s.lines, ["OK"]);
    assert_eq!(s.events, ["tx_flush", "ptt off"]);
}

/// `vfm_free` from inside `ptt`, in the middle of an engine step.
#[test]
fn free_from_inside_ptt() {
    fn hook(host: &Host, e: Event) {
        if matches!(e, Event::Ptt(true)) {
            host.side().events.clear();
            host.free();
        }
    }
    let host = calling(6, hook);
    assert!(host.modem.get().is_null(), "never keyed");
    let s = host.side();
    assert_eq!(s.events, ["tx_flush", "ptt off"]);
    assert_eq!(s.audio_bursts, 0);
}

/// A command that is not UTF-8 gets an answer like any other bad command.
#[test]
fn command_that_is_not_utf8_is_wrong() {
    let host = Host::open(7, no_hook);
    unsafe { vfm_command(host.modem.get(), c"MYCALL LA1\xffAA".as_ptr()) };
    assert_eq!(host.side().lines, ["WRONG"]);
    host.free();
}

#[test]
fn version_is_one_static_string() {
    let v = vfm_version();
    assert_eq!(v, vfm_version());
    assert!(unsafe { CStr::from_ptr(v) }.to_str().unwrap().starts_with("VardeFM "));
}

/// A burst that arrives clipped is reported through `log`.
#[test]
#[cfg_attr(miri, ignore = "the receiver is too slow under Miri")]
fn level_warnings_reach_the_log() {
    let a = calling(1, no_hook);
    let b = Host::open(2, no_hook);
    for _ in 0..300 {
        a.advance(CHUNK as u64);
        let heard: Vec<f32> = a.tick().iter().map(|x| (x * 4.0).clamp(-1.0, 1.0)).collect();
        b.rx(&heard);
        if !b.side().rx_kinds.is_empty() {
            break;
        }
    }
    {
        let s = b.side();
        assert_eq!(s.rx_kinds, [VFM_KIND_CONNECT_REQ], "B heard nothing");
        assert!(s.logs.iter().any(|l| l.contains("clipping")), "{:?}", s.logs);
    }
    a.free();
    b.free();
}
