# Embedding VardeFM

VardeFM can be linked into another application as a static library. The app owns
audio and PTT and steps the library from one of its own threads. The library has
no worker thread of its own (decode may use scoped threads inside a receive step).

## Crates

| Crate | Role |
|---|---|
| `vardefm-engine` | Station, host protocol, `RadioEngine`, `AudioSink` / `Ptt` traits |
| `vardefm-ffi` | C ABI (`libvardefm.a`) and header in `include/CVardeFM/` |

`vardefm-ffi` depends only on the DSP/modem/link/engine stack. It does not pull
in sound-card, serial, HID, TCP or WAV recording crates.

## C API

Header: [`crates/vardefm-ffi/include/CVardeFM/vardefm.h`](../crates/vardefm-ffi/include/CVardeFM/vardefm.h).
Regenerate after changing the Rust FFI surface (CI checks it with cbindgen 0.29.4):

```sh
cbindgen -c crates/vardefm-ffi/cbindgen.toml -o crates/vardefm-ffi/include/CVardeFM/vardefm.h crates/vardefm-ffi
```

```c
VfmConfig cfg;
vfm_config_default(&cfg);
cfg.decode_threads = 1;   /* decode on the calling thread */

VfmCallbacks cbs = {
    .tx_audio = on_tx_audio,   /* one burst, 48 kHz mono float */
    .tx_flush = on_tx_flush,   /* drop what is queued; it does not count as played */
    .ptt = on_ptt,             /* may block */
    .host_line = on_host_line, /* host protocol lines (docs/PROTOCOL.md) */
    .host_data = on_host_data,
    .log = on_log,             /* optional: bursts, failures, level warnings */
    .tx_report = on_tx_report, /* optional structured TX stats */
    .rx_report = on_rx_report, /* optional structured RX stats */
};

VfmModem *m = vfm_new(&cfg, &cbs, ctx);
vfm_command(m, "MYCALL N0CALL");
vfm_command(m, "LISTEN ON");

/* On the modem thread, for each chunk of received audio: */
vfm_tx_played(m, total_played);       /* TX samples played so far */
vfm_rx_audio(m, samples, n);          /* also advances the sample clock */

/* While keyed with no RX audio from the radio: */
vfm_tx_played(m, total_played);
vfm_advance(m, n);                    /* keep the clock running */

vfm_free(m);
```

### Threads

Do not call the library from a real-time audio callback (a CoreAudio render or
input callback, for example). One `vfm_rx_audio` call can run the FFT and LDPC
decode of a whole burst, allocate memory, start decode threads (`decode_threads = 0`
means one per CPU core) and wait in `ptt(false)`. On an audio thread that means
missed deadlines and dropouts.

Use one ordinary thread for the modem and make every `vfm_*` call from it. The
audio callbacks only move samples:

- The capture callback only copies its samples into a ring buffer. The modem thread
  takes them out and passes them to `vfm_rx_audio`.
- `tx_audio` copies the burst into a second ring buffer. The render callback plays
  from it and adds what it played to an atomic counter.
- The modem thread reads that counter and passes it to `vfm_tx_played` before each
  `vfm_rx_audio` / `vfm_advance`.
- `tx_flush` runs on the modem thread. It empties the TX ring buffer.

### Contract

- **One call at a time.** Callbacks fire on the calling thread before the call returns.
- **Re-entrancy.** `vfm_command`, `vfm_send`, `vfm_tx_played` and `vfm_free` may be
  called from inside any callback. Commands and data are queued and run before the
  outer call returns. `vfm_rx_audio` and `vfm_advance` from inside a callback are
  ignored and logged.
- **Sample clock.** Time advances only by samples given to `vfm_rx_audio` plus
  `vfm_advance`. The PTT tail and watchdog use this clock. The caller must keep it
  running while keyed even when the radio returns no RX audio (common on half-duplex
  network paths). Feeding silence via `vfm_rx_audio` is equivalent; `vfm_advance` is
  the explicit form.
- **`vfm_tx_played`.** Reports how many TX samples have really been played: a running
  total since `vfm_new`, clamped to what was queued. Un-key happens on the next
  `vfm_rx_audio` / `vfm_advance` after the burst has drained plus `txtail_s`.
- **`tx_flush`.** The watchdog (a burst that is not played within its length plus 3 s)
  and `vfm_free` ask the host to drop its queued TX audio. Never add the dropped
  samples to the played total: the modem accounts for them itself. It takes the total
  it holds when `tx_flush` returns as exact, so if the render thread may have played
  more than was last reported, call `vfm_tx_played` from inside `tx_flush`, after
  emptying the queue. Samples played but not reported by then make every later burst
  un-key that much too early.
- **PTT may block.** `ptt(ctx, false)` may wait for the radio to confirm un-key. That
  only pauses the sample clock; it is safe.
- **Commands always get an answer.** A line that is not a valid command, or not valid
  UTF-8, is answered with `WRONG`.
- **`vfm_free`.** Calls `tx_flush` and `ptt(false)`, then frees the modem. From inside
  a callback (on `DISCONNECTED`, for example) it only marks the modem: no other
  callback fires after that, and the flush, the un-key and the free happen when the
  outer call returns. `ctx` must stay valid until then. Do not use the handle after
  `vfm_free`.
- **Panic.** A Rust panic is caught at the boundary: the modem flushes, calls
  `ptt(false)`, logs, and ignores later calls. It never unwinds into the host.

### Constants

`VFM_RATE` is the sample rate of all audio (48000 Hz) and `VFM_LEVEL_COUNT` the number
of speed levels (7). The `kind` field of a TX or RX report is one of `VFM_KIND_CONNECT_REQ`,
`VFM_KIND_CONNECT_ACK`, `VFM_KIND_DATA`, `VFM_KIND_DISC` and `VFM_KIND_DISC_ACK`.

## Apple XCFramework

```sh
./packaging/apple/build-xcframework.sh
```

Produces `packaging/apple/build/VardeFM.xcframework` with slices for
`ios-arm64`, `ios-arm64_x86_64-simulator` and `macos-arm64_x86_64`, plus a zip and
SwiftPM checksum. The local Swift package is [`packaging/apple/Package.swift`](../packaging/apple/Package.swift)
(`CVardeFM` binary target). Build the XCFramework before `swift test` or opening
an Xcode project that depends on it:

```sh
(cd packaging/apple && swift test)
```

Release workflows build the XCFramework, run the Swift tests and upload
`vardefm_<version>_apple_xcframework.zip` alongside the desktop archives. Pushes and
pull requests only check the header, the dependency tree and a C link test.

## Checking the C ABI for undefined behaviour

CI runs the C ABI tests under [Miri](https://github.com/rust-lang/miri) when `vardefm-ffi`
or a crate it depends on changes. To do the same:

```sh
rustup toolchain install nightly --component miri,rust-src
cargo +nightly miri test -p vardefm-ffi --target x86_64-unknown-linux-gnu
```

It takes several minutes. Under Miri the tests that need the receiver are skipped,
the bursts are made as short as possible, and the LDPC codes get a stand-in graph
(`round_robin` in `crates/vardefm-fec/src/ldpc.rs`), because the real construction
takes hours to interpret. Miri checks the pointer handling at the boundary, not the
modem.
