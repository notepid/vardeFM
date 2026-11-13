# Incremental-redundancy HARQ: design note and measurement (issue 5)

**Status: investigation.** Nothing here is part of the protocol. The code it describes
is experimental: `Ldpc::with_extension` in `crates/vardefm-fec/src/ldpc.rs` and the
`ir_sweep` test in `crates/vardefm-modem/src/ir_eval.rs`.

## 1. Problem

When a payload codeword fails today, the receiver drops its soft information. The
sender resends the unacknowledged bytes in new codewords, re-split to fit the current
level (PROTOCOL.md §3.2). A codeword that misses by a fraction of a dB costs a whole
new codeword, and the receiver starts again from nothing.

## 2. Mother code

**Construction.** Keep each payload code (n = 2400, k) exactly as it is and add `X`
extension parity bits, each with its own check:

```
H_mother = [ Hu  Hp   0 ]     (today's code: info, staircase parity)
           [ E        I ]     (X extension checks, one new parity bit each)
```

- Every row of `E` holds `d` variables of today's 2400-bit codeword, information or
  staircase parity. Each variable is drawn from those of the lowest current degree
  that share no check with a variable already in the row: no 4-cycles, and the
  extension goes first to the weakest bits (degree-2 staircase parity, degree-3
  information bits). The draw uses xorshift32 with a per-k seed, as the PEG
  construction does.
- Each extension parity bit is the XOR of its row: encoding is one more pass.
- The first 2400 bits of a mother codeword are today's codeword, bit for bit. The
  first transmission is unchanged, and old stations decode it as now.
- Extension bits not yet received enter the decoder as LLR 0. Their checks then pass
  nothing on, so the decoder gives exactly today's result
  (`erased_extension_decodes_as_the_staircase_code`).

This is the Raptor-like structure of 5G NR's base graphs, grafted onto the existing
IRA codes instead of designed jointly with them.

**Parameters measured.** `X = 2400` (mother rate k/4800: 1/4 to 0.42), row degree
`d` from the AWGN sweep in §5.1.

**Redundancy versions.** Increment *i* carries extension bits `[(i−1)E, iE)`. After
the last one, transmission continues around the whole mother codeword (today's
2400 bits, then the extension again), and the receiver adds the new LLRs to the
stored ones (Chase combining on top of IR). So a codeword never has to restart.

## 3. Protocol changes (sketch)

- **Capability.** A connect-time flag (`"VFM2"` payload magic, or a bit after the call
  signs). IR is used only when both ends set it.
- **Header.** One spare flag bit: *IR*. When set, the burst's payload starts with
  increments, one per codeword that the receiver's last burst marked failed and whose
  sender still holds it, in that order; then new segments. No list is needed: the IR
  burst's `ack_burst` names the receiver's burst whose bitmap the sender acts on, and
  both ends know that bitmap and every codeword's redundancy version. A sender that
  heard no bitmap sends no increments, and plain resends still work.
- **Bitmap.** Bit *i* covers entry *i* of the burst: the increments first (whether the
  codeword they complete now decodes), then the new codewords.
- **Burst layout.** Increments of E = 600 bits fill quarter-codeword slots: four make
  one 2400-bit slot, so bursts stay whole numbers of 2400-bit slots and the burst
  interleaver is unchanged. An odd number of increments leaves padding.
- **Level changes.** An increment goes out at the level of the burst carrying it, so
  its LLRs can come from another constellation than the first transmission. The
  decoder does not care. The sender may still switch level freely; only a codeword's
  bytes are fixed once sent, so data in a pending codeword is not re-split.
- **Giving up.** After a set budget (say 6 codeword-equivalents) the sender drops the
  codeword and resends its bytes as today; the receiver frees the slot when the
  codeword decodes, when it is overtaken by `ack_offset`, or on a new session.

**Receiver memory.** Per pending codeword: 4800 LLRs. As i8 that is 4.7 KB; 24
pending codewords take 113 KB. The decoder's working memory doubles with the
mother code's checks.

**Decoding cost.** A mother-code decode handles up to twice the checks of today's.
Increments decode only the failed codewords.

## 4. Measurement method

`ir_sweep` sends real bursts (4 codewords, fewer where a level's burst holds fewer)
through the modem and the FM channel simulator, demodulates them from the known
position with the header as training, and keeps, per bit slot, the LLR signed by the
sent bit: positive when right. One codeword's values (interleaver undone) are the
channel one transmission sees, clicks and all. Each scheme then sends a mother
codeword through a sequence of these channels, one per transmission, each from a
different burst. The interleaver spreads every codeword over its whole burst, so a
codeword's channel can be reused for another codeword's bits.

Schemes, each charged the coded bits it puts on the air:

| Scheme | After a failure |
|---|---|
| `resend` | today: a new codeword (2400 bits), nothing kept |
| `chase` | the same codeword again, LLRs added (2400 bits) |
| `ir300`, `ir600`, `ir1200` | an increment of 300, 600 or 1200 bits, as in §2 |

Efficiency = data delivered ÷ bits spent (1 = one codeword per 2400 bits); goodput =
efficiency × `Modem::net_rate(level)`. The envelope takes the best level at each CNR
(a genie: the link picks by its SNR thresholds, with margin, and would sit somewhat
lower under every scheme). Not modelled: header and detection losses (common to all
schemes), signalling overhead, and the extra round trips (each increment waits one
turn, as a resend does today).

## 5. Results

### 5.1 Extension degree (AWGN)

Es/N0 (dB, BPSK, AWGN, 100 frames per point) at 10 % FER for today's code plus 0 to
4 increments of 600 extension bits, by row degree (`ir_curve` in vardefm-fec). A dash:
the 0.25 dB scan found no crossing (the curve flattens: an error floor).

| Code | d | +0 | +600 | +1200 | +1800 | +2400 |
|---|---|---|---|---|---|---|
| 1/2 | 4 | −1.80 | −2.47 | −3.03 | −3.55 | −4.05 |
| 1/2 | 6 | −1.80 | −2.28 | −2.57 | −2.87 | −2.99 |
| 1/2 | 8 | −1.80 | −2.10 | −2.20 | – | −2.28 |
| 2/3 | 4 | 0.12 | −0.82 | −1.71 | −2.59 | −3.27 |
| 2/3 | 6 | 0.12 | −0.77 | −1.53 | −2.16 | −2.48 |
| 2/3 | 8 | 0.12 | −0.57 | −1.05 | −1.40 | −1.56 |
| 3/4 | 4 | 1.16 | 0.06 | −0.93 | −1.84 | −2.34 |
| 3/4 | 6 | 1.16 | −0.02 | −1.11 | −2.01 | −2.49 |
| 3/4 | 8 | 1.16 | 0.13 | −0.80 | −1.26 | −1.47 |
| 5/6 | 4 | 2.31 | 1.09 | −0.01 | −0.64 | −0.87 |
| 5/6 | 6 | 2.31 | 0.85 | −0.42 | −1.48 | −2.49 |
| 5/6 | 8 | 2.31 | 0.83 | −0.57 | −1.22 | −1.49 |

Chosen: d = 4 for k ≤ 1600, d = 6 for k ≥ 1800. Dense rows help little and hurt the
low-rate end, where the extension must reach every bit. The first 600-bit increment
(a quarter codeword) buys 0.5 to 1.5 dB; the whole extension 2.2 to 4.8 dB. The rate-1/4
mother code sits about 2.7 dB from BPSK capacity (−6.8 dB Es/N0), part of it
finite length: a jointly designed protograph would do better, but the point here is whether IR pays
at all.

### 5.2 Goodput against CNR

`TRIALS=32 cargo test --release -p vardefm-modem ir_sweep -- --ignored --nocapture`:
32 bursts per point (128 codewords, 64 at narrow level 0), 0.5 dB steps. Envelope over
levels (genie level choice), kbit/s, at whole dB; the best level in brackets for
`resend` and `ir600`.

**Narrow**

| CNR dB | resend | chase | ir300 | ir600 | ir1200 |
|---|---|---|---|---|---|
| 3 | 0.56 (L0) | 0.71 | 0.68 | 0.67 (L0) | 0.64 |
| 4 | 0.70 (L0) | 0.90 | 1.32 | 1.25 (L1) | 1.11 |
| 5 | 1.44 (L1) | 1.45 | 1.83 | 1.73 (L2) | 1.46 |
| 6 | 2.18 (L2) | 2.18 | 2.60 | 2.50 (L3) | 2.23 |
| 7 | 2.91 (L3) | 2.91 | 3.40 | 3.29 (L4) | 2.94 |
| 8 | 3.78 (L4) | 3.89 | 4.33 | 4.26 (L4) | 4.13 |
| 9–10 | 4.40 (L4) | 4.40 | 4.40 | 4.40 (L4) | 4.40 |
| 11 | 4.40 (L4) | 4.40 | 4.61 | 4.48 (L5) | 4.40 |
| 12 | 4.40 (L4) | 4.40 | 5.25 | 4.88 (L6) | 4.85 |
| 13 | 5.75 (L5) | 5.75 | 5.83 | 5.82 (L5) | 5.79 |
| 14 | 5.84 (L5) | 5.84 | 5.89 | 5.84 (L5) | 5.84 |
| 15 | 5.84 (L5) | 5.84 | 6.52 | 5.91 (L6) | 5.84 |
| 16 | 5.90 (L6) | 6.21 | 7.17 | 7.01 (L6) | 6.73 |
| 17+ | 7.33 (L6) | 7.33 | 7.33 | 7.33 (L6) | 7.33 |
| mean 0–18 dB | 3.55 | 3.61 (+2 %) | 3.87 (+9 %) | 3.77 (+6 %) | 3.68 (+4 %) |

**Wide**

| CNR dB | resend | chase | ir300 | ir600 | ir1200 |
|---|---|---|---|---|---|
| 4 | 0.00 (L0) | 1.01 | 1.07 | 1.03 (L0) | 0.97 |
| 5 | 1.85 (L0) | 1.85 | 1.87 | 1.87 (L0) | 1.86 |
| 6 | 2.20 (L1) | 2.72 | 3.53 | 3.42 (L1) | 3.16 |
| 7 | 3.78 (L1) | 3.78 | 4.57 | 4.42 (L2) | 3.81 |
| 8 | 5.19 (L2) | 5.23 | 5.65 | 5.59 (L2) | 5.46 |
| 9 | 5.72 (L2) | 5.72 | 5.72 | 5.72 (L2) | 5.72 |
| 10 | 5.72 (L2) | 5.72 | 6.17 | 5.90 (L3) | 5.72 |
| 11 | 6.97 (L3) | 6.97 | 7.44 | 7.37 (L3) | 7.23 |
| 12 | 7.51 (L3) | 7.51 | 7.69 | 7.54 (L4) | 7.51 |
| 13 | 7.51 (L3) | 7.51 | 8.71 | 8.38 (L4) | 7.58 |
| 14 | 7.51 (L3) | 7.51 | 9.88 | 9.14 (L4) | 7.64 |
| 15 | 10.32 (L4) | 10.40 | 11.24 | 11.11 (L4) | 10.86 |
| 16–18 | 11.37 (L4) | 11.37 | 11.37 | 11.37 (L4) | 11.37 |
| 19 | 11.37 (L4) | 11.37 | 11.58 | 11.42 (L5) | 11.37 |
| 20 | 11.37 (L4) | 11.37 | 13.31 | 12.34 (L5) | 12.11 |
| 21 | 14.36 (L5) | 14.36 | 14.56 | 14.53 (L5) | 14.47 |
| 22 | 14.59 (L5) | 14.59 | 14.65 | 14.59 (L5) | 14.59 |
| 23 | 14.59 (L5) | 14.59 | 16.16 | 14.69 (L6) | 14.59 |
| 24 | 14.59 (L5) | 15.32 | 17.87 | 17.45 (L6) | 16.68 |
| 25+ | 18.31 (L6) | 18.31 | 18.31 | 18.31 (L6) | 18.31 |
| mean 1.5–26 dB | 8.42 | 8.53 (+1 %) | 9.13 (+8 %) | 8.91 (+6 %) | 8.68 (+3 %) |

Peaks between whole dB: narrow 7.5 dB +34 % (ir300) / +24 % (ir600), wide 14.5 dB
+40 % / +31 %. At a few points (narrow 2.5 and 3.5 dB, wide 4.0 and 5.5 dB) the best
level won at the lowest CNR it was measured at, so IR there is if anything understated.

### 5.3 Reading

- **The waterfalls are steep.** Through the FM channel a level goes from 100 % to 0 %
  CER within about 1 dB (narrow level 3: 95 % at 5.75 dB, 0.1 % at 6.5 dB). Just below
  a level's point, a resend at that level is nearly worthless, and today's best choice
  is the level below. So resends lose little where they are used; the loss is in the
  gaps of the ladder, where the next level up does not yet work at all.
- **IR fills the gaps.** Within 2 to 3 dB below a level's point, that level with one
  or two increments beats the level below by 10 to 40 %, and at the bottom (narrow
  4 dB, wide 6 dB) by 55 to 90 %. Deeper in the wide gaps (narrow 9 to 10.5 dB, wide
  16 to 18.5 dB) it gains nothing. Above a level's point all schemes coincide. Averaged over the CNR range
  the gain is +6 % (ir600) to +9 % (ir300), in both modes.
- **Chase combining is no substitute**: doubling a codeword's cost to gain 3 dB rarely
  beats dropping a level (+1 to 2 % on average).
- **Smaller increments pay more**, but each increment waits one turn and holds back
  in-order delivery of everything after the failed codeword. ir600 keeps four per
  2400-bit slot and gets two thirds of ir300's gain.
- **The real link gains more than the genie envelope shows** in one respect: today it
  asks for a level 1 dB above that level's threshold (`SNR_MARGIN_DB`), because a
  failure costs a whole codeword. With IR a near miss costs a quarter codeword, so the
  margin could shrink. It gains less in another: header and detection failures
  (common to all schemes) cap the bottom levels, and increments add turns.
- **Not measured end to end.** No `sim-pair` run: no turn timing, no level choice by
  the real SNR thresholds, no signalling. Those numbers need the prototype in the link.

## 6. Recommendation

IR-HARQ pays, but modestly on average: about +6 % goodput over the whole CNR range
(ir600), +10 to 40 % in the gaps between levels, nothing above a level's point. It
needs a breaking protocol change (capability exchange, a header flag, new bitmap
semantics, a normative extension construction) and LLR storage and bookkeeping in
the receiver.

Suggested decision: **adopt in a future protocol version step, not on its own.**
Bundle it with the next breaking change, as ir600 with the extension of §2, and
before that build the link-layer prototype behind the capability flag and measure it
in `sim-pair` against today's resends, with `SNR_MARGIN_DB` retuned for IR. If no
breaking change is planned, close the issue with these numbers: the average gain
alone does not carry the compatibility break.
