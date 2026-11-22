//! C ABI for embedding the VardeFM radio engine.
//!
//! The modem has no thread of its own apart from scoped decode workers inside a
//! receive step. Call one function at a time; callbacks fire on the calling thread
//! before the call returns. A call may decode a burst, allocate, start threads and
//! wait in `ptt`, so call from a worker thread, not from a real-time audio callback.
//! The sample clock is the samples given to [`vfm_rx_audio`] plus [`vfm_advance`]:
//! the caller must keep it running while keyed.
//!
//! See `docs/EMBEDDING.md` and the header in `include/CVardeFM/vardefm.h`.
//!
//! # Safety
//!
//! Every `extern "C"` entry point takes raw pointers from the host. Unless noted
//! otherwise, `cfg` / `cbs` / `line` / audio buffers must be valid for the call,
//! `m` must be a pointer from [`vfm_new`] (or null, which is ignored), and the
//! host must not call in from two threads at once.
//!
//! A callback may call back in, so several entry points can be on the stack for one
//! modem. Each makes only a shared reference to the [`VfmModem`]; what changes sits
//! in `Cell`s, `RefCell`s and atomics. Only the outermost call borrows the engine,
//! and nested calls never touch it.

#![allow(clippy::missing_safety_doc)]

use std::cell::{Cell, RefCell};
use std::ffi::{c_char, c_void, CStr, CString};
use std::io;
use std::mem;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use vardefm_engine::{AudioSink, EngineConfig, HostIn, HostOut, Ptt, RadioEngine, Report, RATE};
use vardefm_modem::{Header, Kind, LEVELS};

/// Narrow (mic/speaker) mode.
pub const VFM_MODE_NARROW: u8 = 0;
/// Wide (9600 data port) mode.
pub const VFM_MODE_WIDE: u8 = 1;

/// Sample rate of all audio in and out (Hz).
pub const VFM_RATE: u32 = 48_000;
/// Number of speed levels: they run from 0 to `VFM_LEVEL_COUNT - 1`.
pub const VFM_LEVEL_COUNT: u32 = 7;

/// Burst kind (`kind` in the reports): connection request.
pub const VFM_KIND_CONNECT_REQ: u8 = 0;
/// Burst kind: connection accepted.
pub const VFM_KIND_CONNECT_ACK: u8 = 1;
/// Burst kind: data and/or acknowledgements.
pub const VFM_KIND_DATA: u8 = 2;
/// Burst kind: close the session.
pub const VFM_KIND_DISC: u8 = 3;
/// Burst kind: session closed.
pub const VFM_KIND_DISC_ACK: u8 = 4;

// cbindgen copies literals into the header; these tie them to the engine's values.
const _: () = {
    assert!(VFM_RATE == RATE);
    assert!(VFM_LEVEL_COUNT as usize == LEVELS.len());
    assert!(VFM_KIND_CONNECT_REQ == Kind::ConnectReq as u8);
    assert!(VFM_KIND_CONNECT_ACK == Kind::ConnectAck as u8);
    assert!(VFM_KIND_DATA == Kind::Data as u8);
    assert!(VFM_KIND_DISC == Kind::Disc as u8);
    assert!(VFM_KIND_DISC_ACK == Kind::DiscAck as u8);
};

/// Configuration for [`vfm_new`].
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VfmConfig {
    /// [`VFM_MODE_NARROW`] or [`VFM_MODE_WIDE`].
    pub mode: u8,
    pub txdelay_s: f32,
    pub txtail_s: f32,
    pub tx_gain: f32,
    /// Highest speed level to send or request (0–6).
    pub max_level: u8,
    /// 0 = one decode thread per CPU core; 1 = decode on the calling thread.
    pub decode_threads: u8,
    /// 0 = pick a random seed; any other value is used as-is.
    pub seed: u32,
}

/// Per-burst transmit statistics.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VfmTxReport {
    pub time_s: f64,
    pub repeat: bool,
    /// One of the `VFM_KIND_*` values.
    pub kind: u8,
    pub level: u8,
    pub codewords: u8,
    pub bytes: u32,
    pub resent_bytes: u32,
    pub retry: u32,
    pub air_s: f32,
    pub net_bps: f32,
}

/// Per-burst receive statistics.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VfmRxReport {
    pub time_s: f64,
    /// One of the `VFM_KIND_*` values.
    pub kind: u8,
    pub level: u8,
    pub codewords: u8,
    pub codewords_ok: u8,
    pub snr_db: f32,
    pub peak_dbfs: f32,
    pub rms_dbfs: f32,
    pub net_bps: f32,
}

/// Host-supplied callbacks. Optional pointers may be null.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct VfmCallbacks {
    /// One transmitted burst, 48 kHz mono float samples. Required.
    pub tx_audio: Option<extern "C" fn(ctx: *mut c_void, samples: *const f32, n: usize)>,
    /// Drop any queued transmit audio. The dropped samples do not count as played
    /// (see `vfm_tx_played`). Required.
    pub tx_flush: Option<extern "C" fn(ctx: *mut c_void)>,
    /// Key or un-key the radio. May block. Required.
    pub ptt: Option<extern "C" fn(ctx: *mut c_void, on: bool)>,
    /// One host command-port line (no CR). Required.
    pub host_line: Option<extern "C" fn(ctx: *mut c_void, line: *const c_char)>,
    /// Bytes for the host data port. Required.
    pub host_data: Option<extern "C" fn(ctx: *mut c_void, data: *const u8, n: usize)>,
    /// Diagnostic log line: bursts, failures, and warnings about the receive level. Optional.
    pub log: Option<extern "C" fn(ctx: *mut c_void, line: *const c_char)>,
    /// Structured TX burst report. Optional.
    pub tx_report: Option<extern "C" fn(ctx: *mut c_void, report: *const VfmTxReport)>,
    /// Structured RX burst report. Optional.
    pub rx_report: Option<extern "C" fn(ctx: *mut c_void, report: *const VfmRxReport)>,
}

/// Opaque modem handle.
pub struct VfmModem {
    cbs: VfmCallbacks,
    ctx: *mut c_void,
    engine: RefCell<RadioEngine<CallbackSink>>,
    /// Commands and data submitted from inside a callback, for the outermost call to run.
    pending: RefCell<Vec<HostIn>>,
    /// Entry points on the stack. Above 1, the current one was called from a callback.
    depth: Cell<u32>,
    /// The flush and un-key that follow a panic or `vfm_free` have been done.
    released: Cell<bool>,
    /// `vfm_free` was called: the outermost call frees the modem on its way out.
    free_requested: Cell<bool>,
    shared: Arc<Shared>,
}

// The host is responsible for synchronizing access; we only promise one call at a time.
unsafe impl Send for VfmModem {}

/// What the modem shares with its audio sink and PTT, which live inside the engine.
struct Shared {
    /// Samples given to `tx_audio`.
    queued: AtomicU64,
    /// Samples the host has played, as `vfm_tx_played` reported them.
    host_played: AtomicU64,
    /// Samples the host dropped in `tx_flush`. Always `queued - host_played` as of the last flush.
    flushed: AtomicU64,
    /// Set by a panic or `vfm_free`. From then on only `tx_flush` and `ptt(false)` are called.
    dead: AtomicBool,
}

impl Shared {
    fn dead(&self) -> bool {
        self.dead.load(Ordering::Acquire)
    }

    /// Samples that have left the host's queue, played or flushed.
    fn consumed(&self) -> u64 {
        let gone = self.host_played.load(Ordering::Acquire) + self.flushed.load(Ordering::Acquire);
        gone.min(self.queued.load(Ordering::Acquire))
    }

    fn set_played(&self, total: u64) {
        // The host cannot have played what it was never given or what it flushed.
        let most = self.queued.load(Ordering::Acquire) - self.flushed.load(Ordering::Acquire);
        self.host_played.store(total.min(most), Ordering::Release);
    }
}

struct CallbackSink {
    cbs: VfmCallbacks,
    ctx: *mut c_void,
    shared: Arc<Shared>,
}

unsafe impl Send for CallbackSink {}

impl AudioSink for CallbackSink {
    fn play(&mut self, samples: &[f32]) -> u64 {
        if let (Some(f), false) = (self.cbs.tx_audio, self.shared.dead()) {
            f(self.ctx, samples.as_ptr(), samples.len());
        }
        self.shared.queued.fetch_add(samples.len() as u64, Ordering::AcqRel) + samples.len() as u64
    }
    fn consumed(&self) -> u64 {
        self.shared.consumed()
    }
    fn flush(&mut self) {
        if let Some(f) = self.cbs.tx_flush {
            f(self.ctx);
        }
        // Read the played count after the callback: the host may report it from inside.
        // The rest of what was queued is gone and will never be reported as played.
        let queued = self.shared.queued.load(Ordering::Acquire);
        let played = self.shared.host_played.load(Ordering::Acquire);
        self.shared.flushed.store(queued - played, Ordering::Release);
    }
}

struct CallbackPtt {
    cbs: VfmCallbacks,
    ctx: *mut c_void,
    shared: Arc<Shared>,
}

unsafe impl Send for CallbackPtt {}

impl Ptt for CallbackPtt {
    fn set(&mut self, on: bool) -> io::Result<()> {
        if on && self.shared.dead() {
            return Ok(());
        }
        if let Some(f) = self.cbs.ptt {
            f(self.ctx, on);
        }
        Ok(())
    }
}

fn kind_u8(kind: Kind) -> u8 {
    match kind {
        Kind::ConnectReq => VFM_KIND_CONNECT_REQ,
        Kind::ConnectAck => VFM_KIND_CONNECT_ACK,
        Kind::Data => VFM_KIND_DATA,
        Kind::Disc => VFM_KIND_DISC,
        Kind::DiscAck => VFM_KIND_DISC_ACK,
    }
}

/// A burst report in its C form.
enum BurstReport {
    Tx(VfmTxReport),
    Rx(VfmRxReport),
}

/// What one engine step left for the host.
struct StepOutput {
    host: Vec<HostOut>,
    reports: Vec<BurstReport>,
    log: Vec<String>,
}

impl VfmModem {
    fn dead(&self) -> bool {
        self.shared.dead()
    }

    /// True if this entry point was called from inside a callback.
    fn nested(&self) -> bool {
        self.depth.get() > 1
    }

    /// Host input: run it now, or leave it for the outermost call if this one is nested.
    fn host_in(&self, input: HostIn) {
        if self.nested() {
            self.pending.borrow_mut().push(input);
        } else {
            self.run(|engine| engine.step(None, vec![input]));
        }
    }

    /// Step the engine and give the host its output, then the same for whatever the
    /// callbacks submitted meanwhile. For the outermost call only.
    fn run(&self, step: impl FnOnce(&mut RadioEngine<CallbackSink>) -> Vec<HostOut>) {
        let mut out = self.step(step);
        loop {
            self.emit(out);
            let more = mem::take(&mut *self.pending.borrow_mut());
            if more.is_empty() || self.dead() {
                return;
            }
            out = self.step(|engine| engine.step(None, more));
        }
    }

    /// One engine step. The engine is borrowed only in here, so it is free again
    /// before any of the output reaches the host.
    fn step(&self, step: impl FnOnce(&mut RadioEngine<CallbackSink>) -> Vec<HostOut>) -> StepOutput {
        let mut engine = self.engine.borrow_mut();
        let host = step(&mut engine);
        let reports = mem::take(&mut engine.reports);
        let mut log = mem::take(&mut engine.log);
        log.append(&mut engine.notices);
        let net_bps = |h: &Header| if h.ncw > 0 { engine.station.modem().net_rate(h.level) } else { 0.0 };
        let reports = reports
            .into_iter()
            .map(|r| match r {
                Report::Tx { time_s, repeat, header, info, air_s } => BurstReport::Tx(VfmTxReport {
                    time_s,
                    repeat,
                    kind: kind_u8(header.kind),
                    level: header.level,
                    codewords: header.ncw,
                    bytes: info.bytes as u32,
                    resent_bytes: info.resent_bytes as u32,
                    retry: info.retry,
                    air_s,
                    net_bps: net_bps(&header),
                }),
                Report::Rx { time_s, header, codewords_ok, snr_db, level } => BurstReport::Rx(VfmRxReport {
                    time_s,
                    kind: kind_u8(header.kind),
                    level: header.level,
                    codewords: header.ncw,
                    codewords_ok: codewords_ok.min(255) as u8,
                    snr_db,
                    peak_dbfs: level.peak_dbfs,
                    rms_dbfs: level.rms_dbfs,
                    net_bps: net_bps(&header),
                }),
            })
            .collect();
        StepOutput { host, reports, log }
    }

    /// Hand a step's output to the host. Stops as soon as a callback frees the modem.
    fn emit(&self, out: StepOutput) {
        for o in out.host {
            if self.dead() {
                return;
            }
            match o {
                HostOut::Line(l) => {
                    if let (Some(f), Ok(c)) = (self.cbs.host_line, CString::new(l)) {
                        f(self.ctx, c.as_ptr());
                    }
                }
                HostOut::Data(d) => {
                    if let Some(f) = self.cbs.host_data {
                        f(self.ctx, d.as_ptr(), d.len());
                    }
                }
            }
        }
        for r in out.reports {
            if self.dead() {
                return;
            }
            match r {
                BurstReport::Tx(rep) => {
                    if let Some(f) = self.cbs.tx_report {
                        f(self.ctx, &rep);
                    }
                }
                BurstReport::Rx(rep) => {
                    if let Some(f) = self.cbs.rx_report {
                        f(self.ctx, &rep);
                    }
                }
            }
        }
        for l in out.log {
            if self.dead() {
                return;
            }
            self.log(&l);
        }
    }

    fn log(&self, line: &str) {
        if let (Some(f), Ok(c)) = (self.cbs.log, CString::new(line)) {
            f(self.ctx, c.as_ptr());
        }
    }

    /// Flush and un-key a dead modem. A panic is logged; a modem the host freed stays silent.
    fn release(&self) {
        self.engine.borrow_mut().unkey();
        if !self.free_requested.get() {
            self.log("modem panicked; further calls ignored");
        }
    }
}

/// Fill `cfg` with the defaults used by the desktop TNC.
#[no_mangle]
pub unsafe extern "C" fn vfm_config_default(cfg: *mut VfmConfig) {
    if cfg.is_null() {
        return;
    }
    let e = EngineConfig::default();
    *cfg = VfmConfig {
        mode: VFM_MODE_NARROW,
        txdelay_s: e.txdelay_s,
        txtail_s: e.txtail_s,
        tx_gain: e.tx_gain,
        max_level: e.max_level,
        decode_threads: 0,
        seed: 0,
    };
}

/// Create a modem. Returns null if `cfg` or `cbs` is refused (missing callbacks,
/// bad mode, etc.).
#[no_mangle]
pub unsafe extern "C" fn vfm_new(cfg: *const VfmConfig, cbs: *const VfmCallbacks, ctx: *mut c_void) -> *mut VfmModem {
    if cfg.is_null() || cbs.is_null() {
        return ptr::null_mut();
    }
    let cfg = *cfg;
    let cbs = *cbs;
    if cbs.tx_audio.is_none()
        || cbs.tx_flush.is_none()
        || cbs.ptt.is_none()
        || cbs.host_line.is_none()
        || cbs.host_data.is_none()
    {
        return ptr::null_mut();
    }
    match catch_unwind(AssertUnwindSafe(|| new_modem(cfg, cbs, ctx))) {
        Ok(Ok(m)) => Box::into_raw(m),
        _ => ptr::null_mut(),
    }
}

fn new_modem(cfg: VfmConfig, cbs: VfmCallbacks, ctx: *mut c_void) -> io::Result<Box<VfmModem>> {
    let mode = match cfg.mode {
        VFM_MODE_NARROW => "narrow",
        VFM_MODE_WIDE => "wide",
        _ => return Err(io::Error::new(io::ErrorKind::InvalidInput, "mode must be narrow or wide")),
    };
    let eng_cfg = EngineConfig {
        mode: mode.into(),
        txdelay_s: cfg.txdelay_s,
        txtail_s: cfg.txtail_s,
        tx_gain: cfg.tx_gain,
        max_level: cfg.max_level,
        ..EngineConfig::default()
    };
    let decode_threads = (cfg.decode_threads > 0).then_some(cfg.decode_threads as usize);
    let seed = if cfg.seed == 0 { vardefm_engine::random_seed() } else { cfg.seed };
    let shared = Arc::new(Shared {
        queued: AtomicU64::new(0),
        host_played: AtomicU64::new(0),
        flushed: AtomicU64::new(0),
        dead: AtomicBool::new(false),
    });
    let engine = RadioEngine::open(
        &eng_cfg,
        decode_threads,
        Box::new(CallbackPtt { cbs, ctx, shared: shared.clone() }),
        CallbackSink { cbs, ctx, shared: shared.clone() },
        seed,
    )?;
    Ok(Box::new(VfmModem {
        cbs,
        ctx,
        engine: RefCell::new(engine),
        pending: RefCell::new(Vec::new()),
        depth: Cell::new(0),
        released: Cell::new(false),
        free_requested: Cell::new(false),
        shared,
    }))
}

/// Flush audio, un-key, and free the modem: `tx_flush` and `ptt(false)` are the
/// last callbacks. Do not use `m` afterwards.
///
/// May be called from inside a callback. Then no other callback fires any more,
/// and the flush, the un-key and the free happen when the call that fired the
/// callback returns: `ctx` must stay valid until it does.
#[no_mangle]
pub unsafe extern "C" fn vfm_free(m: *mut VfmModem) {
    if m.is_null() {
        return;
    }
    {
        let modem = &*m;
        modem.free_requested.set(true);
        modem.shared.dead.store(true, Ordering::Release);
        modem.depth.set(modem.depth.get() + 1);
    }
    leave(m);
}

/// Submit one host command line (without CR). May be called from inside a callback:
/// the command then runs before the call that fired the callback returns.
#[no_mangle]
pub unsafe extern "C" fn vfm_command(m: *mut VfmModem, line: *const c_char) {
    if m.is_null() || line.is_null() {
        return;
    }
    // Bytes that are not UTF-8 become U+FFFD, which no command accepts: the answer is WRONG.
    let line = CStr::from_ptr(line).to_string_lossy().into_owned();
    enter(m, |modem| modem.host_in(HostIn::Line(line)));
}

/// Bytes for the data port. May be called from inside a callback, like `vfm_command`.
#[no_mangle]
pub unsafe extern "C" fn vfm_send(m: *mut VfmModem, data: *const u8, n: usize) {
    if m.is_null() || (n > 0 && data.is_null()) {
        return;
    }
    let bytes = if n == 0 { Vec::new() } else { std::slice::from_raw_parts(data, n).to_vec() };
    enter(m, |modem| modem.host_in(HostIn::Data(bytes)));
}

/// Feed received audio (48 kHz mono float) and advance the sample clock by `n`.
///
/// Not for a real-time audio callback: this may decode a burst, allocate, start
/// threads and wait in `ptt`. Ignored (and logged) from inside a callback.
#[no_mangle]
pub unsafe extern "C" fn vfm_rx_audio(m: *mut VfmModem, samples: *const f32, n: usize) {
    if m.is_null() || (n > 0 && samples.is_null()) {
        return;
    }
    let chunk = if n == 0 { &[][..] } else { std::slice::from_raw_parts(samples, n) };
    enter(m, |modem| {
        if modem.nested() {
            modem.log("vfm_rx_audio ignored: re-entrant call");
        } else {
            modem.run(|engine| engine.step(Some(chunk), Vec::new()));
        }
    });
}

/// Advance the sample clock by `samples` without feeding audio content.
///
/// Required while keyed when the radio is not delivering RX audio, so the PTT
/// tail and watchdog can fire. Ignored (and logged) from inside a callback.
#[no_mangle]
pub unsafe extern "C" fn vfm_advance(m: *mut VfmModem, samples: u64) {
    if m.is_null() {
        return;
    }
    enter(m, |modem| {
        if modem.nested() {
            modem.log("vfm_advance ignored: re-entrant call");
        } else {
            modem.run(|engine| engine.advance(samples, Vec::new()));
        }
    });
}

/// Report how many transmit samples the host has played so far: a running total
/// since `vfm_new` of samples that really went out, clamped to what was queued.
///
/// Samples dropped in `tx_flush` were not played: never add them. The count the
/// modem holds when `tx_flush` returns is taken as exact, so a host that reports
/// late should call this from inside `tx_flush`. May be called from inside any
/// callback. Un-key happens on the next rx/advance call.
#[no_mangle]
pub unsafe extern "C" fn vfm_tx_played(m: *mut VfmModem, total_samples: u64) {
    if m.is_null() {
        return;
    }
    let modem = &*m;
    modem.shared.set_played(total_samples);
}

/// Library version string, e.g. `"VardeFM 0.3.2"`. Valid for the life of the program.
#[no_mangle]
pub extern "C" fn vfm_version() -> *const c_char {
    static VERSION: &CStr = vardefm_engine::VERSION_CSTR;
    VERSION.as_ptr()
}

/// Run `f` on the modem behind `m`, unless it is dead. A panic kills the modem.
unsafe fn enter(m: *mut VfmModem, f: impl FnOnce(&VfmModem)) {
    {
        let modem = &*m;
        if modem.dead() {
            return;
        }
        modem.depth.set(modem.depth.get() + 1);
        if catch_unwind(AssertUnwindSafe(|| f(modem))).is_err() {
            modem.shared.dead.store(true, Ordering::Release);
        }
    }
    leave(m);
}

/// Leave an entry point. The outermost one finishes a modem that died inside it:
/// flush and un-key, and free it if `vfm_free` asked.
unsafe fn leave(m: *mut VfmModem) {
    let free = {
        let modem = &*m;
        let depth = modem.depth.get();
        if depth > 1 {
            modem.depth.set(depth - 1);
            return;
        }
        // Still at depth 1, so a `vfm_free` from these last callbacks leaves the freeing to us.
        if modem.dead() && !modem.released.replace(true) {
            let _ = catch_unwind(AssertUnwindSafe(|| modem.release()));
        }
        modem.depth.set(0);
        modem.free_requested.get()
    };
    // No reference to the modem is left on the stack: every entry point has returned.
    if free {
        drop(Box::from_raw(m));
    }
}
