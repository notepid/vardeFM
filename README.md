# Varde

An open-source, cross-platform high-speed data modem for FM radios. It aims to be a
drop-in alternative to VARA FM for Winlink and similar apps.

*Varde* is Norwegian for the cairns and beacon fires that once relayed warnings from
mountaintop to mountaintop.

- **Open:** the waveform and protocol are documented in [docs/PROTOCOL.md](docs/PROTOCOL.md).
- **Cross-platform:** written in Rust. Targets Linux, Windows, macOS and Raspberry Pi.
- **App compatible (planned):** Varde will offer the same TCP command/data interface as VARA,
  so Winlink Express, Pat and other VARA clients can use it unchanged.
- **Not on-air compatible with VARA:** both stations need Varde.

## Status

Early development. Done so far:

| Milestone | State |
|---|---|
| 1. Workspace, FM channel simulator, WAV tools | done (VARA baseline measurement still to do) |
| 2. OFDM PHY (sync, equalization, tracking) | working in simulation |
| 3. LDPC FEC + speed levels | next |
| 4. ARQ link layer | |
| 5. VARA-compatible TCP API | |
| 6. On-air testing | |
| 7. Raspberry Pi + packaging | |

## Modes

| Mode | Radio connection | Audio band | Raw rate (BPSK → 64QAM) |
|---|---|---|---|
| narrow | mic/speaker, any FM radio | ~420–2670 Hz | 1.6 – 9.5 kbit/s |
| wide | 9600 data port (flat audio) | ~280–6000 Hz | 4.0 – 23.8 kbit/s |

Raw rate is before error correction and protocol overhead.

## Workspace

```
crates/varde-dsp   OFDM modulator/demodulator, constellations, filters
crates/varde-sim   FM radio channel simulator
crates/varde-cli   `varde` test tool
```

## Trying it

```sh
cargo build --release

# Frames through a simulated FM link at 12 dB carrier-to-noise, 80 ppm clock error
./target/release/varde sim --mode narrow --mod qpsk --cnr 12 --ppm 80

# Bit-error table over signal level for every modulation
./target/release/varde sweep --mode wide

# Over real radios: play test.wav into one radio, record the other at 48 kHz, decode
./target/release/varde tx-wav --mode narrow --mod qpsk --out test.wav
./target/release/varde rx-wav --mode narrow --mod qpsk --in recording.wav
```

Useful options: `--txdelay` (leader length; it must exceed the radio's key-up time),
`--tilt=<dB/oct>`, `--rms <level>`. Set `VARDE_DUMP=1` to print per-carrier SNR.

## Tests

```sh
cargo test
```

`crates/varde-cli/tests/audio.rs` generates modem audio and decodes it. It covers:
- direct loopback
- level and polarity changes
- 16-bit WAV round trips
- false alarms on noise and tones
- clean and noisy simulated radio links
- clock offset
- radio key-up time
- many frames in one long recording

To keep the audio from each test as WAV files:

```sh
VARDE_TEST_WAV_DIR=wavs cargo test --test audio
```

## License

GPL-3.0-or-later.
