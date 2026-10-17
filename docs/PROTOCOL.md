# VardeFM Protocol Specification

**Status: draft v0.** Everything here may change until v1.0. Only the physical layer
exists so far. Error correction, framing, ARQ and the host interface will be added as
they are built.

## 1. Physical layer

### 1.1 Common
- Real-valued audio, 48 000 samples/s.
- OFDM with a cyclic prefix. Carrier *k* (0-based) sits at FFT bin `first_bin + k`.
  The time signal is the real part of a Hermitian-symmetric IFFT, scaled by 1/√N.
- The TX applies a spectral tilt: carrier gain = `10^(tilt · log2(f_k / f_0) / 20)`.
- Output is scaled to the mode's RMS level and clipped at ±1.0 (full scale = peak deviation).

| Parameter | narrow | wide |
|---|---|---|
| FFT size N | 1024 | 512 |
| Cyclic prefix | 128 (2.67 ms) | 64 (1.33 ms) |
| Carrier spacing | 46.875 Hz | 93.75 Hz |
| first_bin / carriers | 9 / 49 | 3 / 62 |
| Band | 421.9–2671.9 Hz | 281.3–6000 Hz |
| Pilot spacing | 6 | 6 |
| Pilots / data carriers | 9 / 40 | 12 / 50 |
| Symbol rate | 41.67 /s | 83.33 /s |
| TX tilt | 0 dB/oct | +3 dB/oct |
| TX RMS | 0.25 | 0.30 |

### 1.2 Pseudo-random sequences
All known sequences come from xorshift32 (`x ^= x<<13; x ^= x>>17; x ^= x<<5`). Each
output bit is the most significant bit of the new state; bit 0 maps to +1, bit 1 to −1.

| Use | Seed |
|---|---|
| Pilot table (32 symbols × pilots, row by row) | 0x5EED0001 |
| Preamble A | 0x5EED00A1 |
| Preamble B | 0x5EED00B2 |
| Leader (I then Q per carrier) | 0x5EED00C3 |

### 1.3 Frame
```
[leader × n][preamble A][preamble B][data 0] … [data nsym-1]
```
- **Leader:** `ceil(txdelay / symbol duration)` symbols of pseudo-random QPSK on every
  carrier. It covers the radio's key-up time and is not used by the receiver.
- **Preamble A:** BPSK × √2 on carriers whose FFT bin is even, zero elsewhere. One value
  is drawn per carrier, odd-bin carriers included, so the sequence stays aligned. The time
  signal repeats every N/2 samples, for Schmidl-Cox detection.
- **Preamble B:** BPSK on every carrier. Used for exact timing and the channel estimate.
- **Data symbols:** pilots on carriers where `k mod pilot_spacing == 0` and on the last
  carrier. The pilot value is `table[sym mod 32][pilot index]`. Data fills the remaining
  carriers in ascending order.

### 1.4 Constellations
Gray-coded, unit average power. BPSK uses the I axis only. For square QAM, the first half
of each symbol's bits selects the I level and the second half the Q level, MSB first. The
Gray word *g* maps to level index *i* = gray⁻¹(*g*), amplitude `(2i − (L−1)) · scale`.

### 1.5 Receiver (informative)
The reference receiver:
1. Low-pass filters the audio (wide mode only) and runs a Schmidl-Cox autocorrelation.
2. Confirms the frame by cross-correlating with preamble B.
3. Places the FFT window mid cyclic prefix, which absorbs clock drift of ±ncp/2 samples
   over the frame.
4. Estimates the channel from preamble B, smoothed across frequency.
5. Tracks it with pilots averaged over ±4 symbols and interpolated in magnitude and phase.

Because the window never moves, a frame should stay shorter than
`ncp / 2 / (clock error)`. At 100 ppm that is 13 s (narrow) or 6.7 s (wide).
