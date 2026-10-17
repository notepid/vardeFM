# Running VardeFM on a radio

VardeFM needs a sound card connected to the radio, and a way to key the
transmitter (PTT). `vardefm modem` then offers the VARA-compatible TCP ports
(default 8300/8301) to Pat, Winlink Express, HamBBS or any other VARA client.

## 1. Pick the PTT method

| Interface | `--ptt` |
|---|---|
| CM108/CM119-based USB audio with PTT on a GPIO pin (many DIY and commercial interfaces) | `cm108` (GPIO 3), or `cm108:<gpio>` |
| AIOC (All-In-One Cable) | `cm108` |
| Interface with a USB serial port keying on RTS or DTR (e.g. Digirig) | `rts:COM5` / `dtr:COM5` (`/dev/ttyUSB0` on Linux); prefix `-` to invert |
| Radio keyed over CAT by Hamlib | `rigctld` or `rigctld:<host>:<port>` (start `rigctld` first) |
| VOX (e.g. SignaLink), or the host application keys the radio | `none` |

CM108-family devices are recognised by USB ID: C-Media `0d8c:0008`–`000f`,
`0012`, `0013`, `0139`, `013a`, `013c`; SSS1621/1623 `0c76:1605/1607/160b`; AIOC
`1209:7388`. With more than one connected, choose one with
`--ptt cm108@<path>` (paths come from `vardefm devices`). The method follows
Dire Wolf's `cm108.c` by WB2OSZ.

**Linux:** HID devices are root-only by default. Allow your user (in group
`audio`) to use them, then unplug and re-plug the interface:

```
# /etc/udev/rules.d/99-vardefm-cm108.rules
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0d8c", GROUP="audio", MODE="0660"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0c76", GROUP="audio", MODE="0660"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="7388", GROUP="audio", MODE="0660"
```

## 2. Set up step by step (Windows PowerShell shown)

1. **Find the devices.**
   ```powershell
   .\vardefm devices
   ```
   This lists sound cards, CM108-type PTT devices and serial ports. Sound cards
   are chosen with `--audio-in` / `--audio-out` and any unique part of the name,
   e.g. `--audio-in "USB Audio"`.

2. **Check PTT.** The radio should transmit for 2 seconds.
   ```powershell
   .\vardefm ptt-test --ptt cm108
   ```

3. **Set the transmit level.** Send a full-scale 1 kHz tone into a dummy load or
   a quiet frequency. Adjust the sound card's output level (Windows sound
   settings / `alsamixer`), the radio's data/mic gain or `--tx-gain`, until the
   deviation is about 2.5 kHz (narrow, mic input) or 3 kHz (wide, data port).
   Without a deviation meter, start low and raise it until a second radio's
   `monitor` shows a clean, steady level.
   ```powershell
   .\vardefm tone --audio-out "USB Audio" --ptt cm108 --seconds 10
   ```

4. **Set the receive level.** Let another station transmit (or play
   `tx-wav` audio from a second radio). Adjust the input level so peaks sit
   around −6 dBFS and never reach 0 dBFS. `monitor` also decodes VardeFM bursts
   and shows their SNR.
   ```powershell
   .\vardefm monitor --audio-in "USB Audio"
   ```
   Open the squelch slightly or set it low: VardeFM detects frames by itself.
   Squelch tail noise is fine.

5. **Run the modem.**
   ```powershell
   .\vardefm modem --audio-in "USB Audio" --audio-out "USB Audio" --ptt cm108
   ```
   Then point Pat / HamBBS / Winlink Express at VARA FM on `127.0.0.1:8300`.
   Use `--mode wide` only with the radio's flat 9600-baud data port, on both
   stations. Stop with Ctrl-C; PTT is released on exit.

## 3. Timing options

| Option | Default | Meaning |
|---|---|---|
| `--txdelay` | 0.15 s | Leader before each burst. Must be longer than the radio's key-up time (PTT to full power and receiver unsquelched). Many handhelds need 0.2–0.3 s. |
| `--txtail` | 0.05 s | PTT held after the last sample has left the sound card. |

If the other station's log shows frames missing their start, raise `--txdelay`.
A watchdog unkeys the radio if a transmission runs 3 s past its expected length.

## 4. What is not done yet

- Sound card and CM108 PTT matching is manual when several adapters are
  connected. Dire Wolf matches them automatically on Linux.
- Raspberry Pi GPIO PTT (planned for milestone 7).
- The rate-adaptation thresholds come from the channel simulator; on-air
  measurements will refine them.
