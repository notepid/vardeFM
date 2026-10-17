# VardeFM

[![CI](https://github.com/notepid/vardeFM/actions/workflows/ci.yml/badge.svg)](https://github.com/notepid/vardeFM/actions/workflows/ci.yml)

An open-source, cross-platform high-speed data modem for FM radios. It aims to be a
drop-in alternative to VARA FM for Winlink and similar apps.

The name comes from *varde*, Norwegian for the cairns and beacon fires that once relayed warnings from
mountaintop to mountaintop.

- **Open:** the waveform and protocol are documented in [docs/PROTOCOL.md](docs/PROTOCOL.md).
- **Cross-platform:** written in Rust. Targets Linux, Windows, macOS and Raspberry Pi.
- **App compatible:** VardeFM offers VARA's TCP command/data interface, so VARA clients
  such as Pat and Winlink Express can talk to it. See "Host interface" in the protocol
  document for the differences from VARA.
- **Not on-air compatible with VARA:** both stations need VardeFM.

## Status

Works end to end in simulation. Sound card and PTT support is built (milestone 6), but
not yet tested on the air. See [docs/RADIO.md](docs/RADIO.md) to set up a radio.

| Milestone | State |
|---|---|
| 1. Workspace, FM channel simulator, WAV tools | done (VARA baseline measurement still to do) |
| 2. OFDM PHY (sync, equalization, tracking) | done |
| 3. LDPC FEC + speed levels | done |
| 4. ARQ link layer | done |
| 5. VARA-compatible TCP interface + `sim-pair` | done |
| 6. Sound card, PTT (CM108/CM119, RTS/DTR, rigctld, VOX) | built; on-air testing next |
| 7. Raspberry Pi + packaging | |

## Download

Prebuilt binaries are on the [releases page](https://github.com/notepid/vardeFM/releases):

| File | For |
|---|---|
| `vardefm_<version>_windows_amd64.zip` | Windows 10/11, 64-bit |
| `vardefm_<version>_linux_amd64.tar.gz` | Linux PC, 64-bit (Debian 12 / Ubuntu 22.04 or newer) |
| `vardefm_<version>_linux_arm64.tar.gz` | Raspberry Pi 3/4/5/Zero 2 W with 64-bit Raspberry Pi OS (Bookworm or newer) |
| `vardefm_<version>_linux_armv7.tar.gz` | Raspberry Pi 2/3/4/Zero 2 W with 32-bit Raspberry Pi OS |

The original Pi Zero and Pi 1 (ARMv6) are not supported. On Linux, install the included
`99-vardefm-cm108.rules` for CM108 PTT (see [docs/RADIO.md](docs/RADIO.md)). Check `uname -m`
on a Pi: `aarch64` needs the arm64 build, `armv7l` the armv7 build.

To build from source: install Rust, plus `libasound2-dev` on Linux, then `cargo build --release`.

## Modes

| Mode | Radio connection | Audio band | Net rate (7 speed levels) |
|---|---|---|---|
| narrow | mic/speaker, any FM radio | ~420–2670 Hz | 0.7 – 7.4 kbit/s |
| wide | 9600 data port (flat audio) | ~280–6000 Hz | 1.9 – 18.7 kbit/s |

Net rate includes error correction and burst overhead. The link picks the speed level
from the measured signal quality.

## Workspace

```
crates/vardefm-dsp     OFDM modulator/demodulator, constellations, filters
crates/vardefm-fec     LDPC codes, CRCs, scrambling, interleaving
crates/vardefm-modem   bursts: header, speed levels, coded payload, streaming receiver
crates/vardefm-link    ARQ link layer (connect, reliable byte stream, disconnect)
crates/vardefm-sim     FM radio channel simulator
crates/vardefm-tnc     VARA-compatible TCP interface, station logic, sim-pair, radio engine
crates/vardefm-audio   sound card input/output (cpal)
crates/vardefm-ptt     PTT: CM108/CM119/AIOC GPIO, serial RTS/DTR, Hamlib rigctld
crates/vardefm-cli     `vardefm` command-line tool
```

## On a real radio

```powershell
.	arget
eleaseardefm devices                       # sound cards, CM108 PTT devices, serial ports
.	arget
eleaseardefm ptt-test --ptt cm108          # key the radio for 2 s
.	arget
eleaseardefm tone --ptt cm108              # 1 kHz tone to set deviation
.	arget
eleaseardefm monitor --audio-in "USB"      # receive level meter + burst decoder
.	arget
eleaseardefm modem --audio-in "USB" --audio-out "USB" --ptt cm108
```

The full procedure is in [docs/RADIO.md](docs/RADIO.md). PTT methods: `cm108[:<gpio>][@<path>]`,
`rts:<port>`, `dtr:<port>` (prefix `-` to invert), `rigctld[:<host>:<port>]`, `none`/`vox`.
The CM108 support follows Dire Wolf's implementation by WB2OSZ.

## Virtual radio pair (`sim-pair`)

Two VardeFM modems in one process, joined by the FM channel simulator. Each has its own
VARA TCP ports, so two host applications (Pat, a BBS, your own code) can connect to each
other with no radios.

```powershell
cargo build --release
# Station A on 8300/8301, station B on 8310/8311, narrow mode, 18 dB CNR, 40 ppm clock error
.\target\release\vardefm sim-pair --cnr 18 --ppm -40 --record wavs
```

| Option | Meaning |
|---|---|
| `--mode narrow\|wide` | mic/speaker audio or 9600 data port |
| `--cnr <dB>` | carrier-to-noise ratio (omit for no noise) |
| `--ppm <n>` | sound-card clock error |
| `--keyup <s>`, `--txdelay <s>` | radio key-up time and the modem's leader (must be longer) |
| `--a-port`, `--b-port` | command ports (data = +1); defaults 8300 and 8310 |
| `--speed <x>` | run x times faster than real time (0 = as fast as possible) |
| `--record <dir>` | save each transmission as `NNNN_<A\|B>_tx.wav` and `_rx.wav` |
| `--seed <n>` | noise seed |

It prints one line per transmission and per reception:
```
   20.97s B->A Data       64QAM 2/3 cw  7  1338 B (resent    0) retry 0 air 2.09 s
   23.08s A rx Data       cw  7/7  SNR  17.6 dB
```

## Measuring the modem

```powershell
# Frames through a simulated FM link at 12 dB carrier-to-noise, 80 ppm clock error
.\target\release\vardefm sim --mode narrow --mod qpsk --cnr 12 --ppm 80

# Bit-error table over signal level for every modulation (uncoded)
.\target\release\vardefm sweep --mode wide

# Over real radios: play test.wav into one radio, record the other at 48 kHz, decode
.\target\release\vardefm tx-wav --mode narrow --mod qpsk --out test.wav
.\target\release\vardefm rx-wav --mode narrow --mod qpsk --in recording.wav
```

Useful options: `--txdelay` (leader length; it must exceed the radio's key-up time),
`--tilt=<dB/oct>`, `--rms <level>`. Set `VARDEFM_DUMP=1` to print per-carrier SNR.

## Tests

```sh
cargo test
```

- `crates/vardefm-cli/tests/audio.rs`: generates modem audio and decodes it: direct
  loopback, level and polarity changes, WAV round trips, false alarms, noisy simulated
  links, clock offset, key-up time, many frames in one recording.
- `crates/vardefm-link/src/tests.rs`: the ARQ link over a fake lossy channel.
- `crates/vardefm-tnc/src/radio/tests.rs`: the real-radio engine with fake sound cards
  and PTT. It checks that audio only plays while keyed, the PTT tail, and the watchdog.
- `crates/vardefm-tnc/tests/simpair.rs`: end to end. Two host clients talk over the VARA
  TCP interface through `sim-pair` and transfer 10 kB each way at several signal levels.
  Set `VARDEFM_SIM_VERBOSE=1` to print every transmission.

To keep the audio from the `audio` tests as WAV files (a relative folder is created in the
repo root):

```powershell
# Windows PowerShell
$env:VARDEFM_TEST_WAV_DIR = "wavs"; cargo test --test audio; Remove-Item Env:VARDEFM_TEST_WAV_DIR
```

```sh
# Linux / macOS / Git Bash
VARDEFM_TEST_WAV_DIR=wavs cargo test --test audio
```

Benchmarks (ignored by default; run in release mode):
```sh
cargo test --release -p vardefm-modem level_table -- --ignored --nocapture   # level vs CNR
cargo test --release -p vardefm-fec fer_curve -- --ignored --nocapture       # LDPC curves
```

## Releases (maintainers)

GitHub Actions runs formatting, clippy and the tests on Windows, Linux x86-64 and Linux ARM
for every push and pull request, and builds all release archives (kept 7 days as workflow
artifacts). To publish a release:

1. Set `version` in the root `Cargo.toml` (e.g. `0.2.0`), run `cargo build` to update
   `Cargo.lock`, and commit.
2. Tag and push: `git tag v0.2.0; git push origin master v0.2.0`.

The release workflow checks that the tag matches `Cargo.toml`, runs the tests, builds the
four archives plus `checksums.txt`, and publishes the release with notes generated from the
commits. A tag with a suffix such as `v0.2.0-rc1` becomes a pre-release.

## License

GPL-3.0-or-later; see [LICENSE](LICENSE).
