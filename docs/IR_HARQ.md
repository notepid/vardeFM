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
  nothing on. The decoder's stop test takes each extension bit's hard decision from
  its row rather than from its own LLR: an erased bit gets no message once a variable
  of its row is weak, and would otherwise fail its check after the staircase checks
  all hold. So with the extension erased the decoder gives exactly today's result
  (`erased_extension_decodes_as_the_staircase_code`,
  `erased_extension_with_weak_rows_decodes_as_the_staircase_code`).

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
different burst. The decoder treats every codeword alike, so applying a burst's
signed LLRs to another codeword changes only which bits share a QAM symbol. The
test `sign_trick_matches_real_decoding` checks that, on narrow level 3 bursts around
its point, a first transmission this way decodes exactly when the modem's own
decoding of the codeword sent does.

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
turn, as a resend does today). Frame sync is out of scope: the burst position is
known.

## 5. Results

### 5.1 Extension degree (AWGN)

Es/N0 (dB, BPSK, AWGN, 100 frames per point) at 10 % FER for today's code plus 0 to
4 increments of 600 extension bits, by row degree:
`TRIALS=100 DEGREES=4,6,8,10 cargo test --release -p vardefm-fec ir_curve -- --ignored --nocapture`.
Each increment's scan runs from 4 dB below the previous crossing to 1 dB above it.

| Code | d | +0 | +600 | +1200 | +1800 | +2400 |
|---|---|---|---|---|---|---|
| 1/2 | 4 | −1.80 | −2.46 | −3.04 | −3.56 | −4.08 |
| 1/2 | 6 | −1.80 | −2.30 | −2.60 | −2.87 | −2.96 |
| 1/2 | 8 | −1.80 | −2.10 | −2.22 | −2.24 | −2.31 |
| 1/2 | 10 | −1.80 | −1.94 | −1.96 | −1.84 | −1.73 |
| 2/3 | 4 | 0.12 | −0.82 | −1.71 | −2.59 | −3.27 |
| 2/3 | 6 | 0.12 | −0.77 | −1.53 | −2.16 | −2.48 |
| 2/3 | 8 | 0.12 | −0.57 | −1.05 | −1.40 | −1.56 |
| 2/3 | 10 | 0.12 | −0.45 | −0.72 | −0.88 | −0.88 |
| 3/4 | 4 | 1.16 | 0.06 | −0.93 | −1.84 | −2.34 |
| 3/4 | 6 | 1.16 | −0.02 | −1.11 | −2.01 | −2.49 |
| 3/4 | 8 | 1.16 | 0.13 | −0.80 | −1.26 | −1.47 |
| 3/4 | 10 | 1.16 | 0.28 | −0.32 | −0.51 | −0.62 |
| 5/6 | 4 | 2.31 | 1.09 | −0.01 | −0.64 | −0.87 |
| 5/6 | 6 | 2.31 | 0.85 | −0.42 | −1.48 | −2.49 |
| 5/6 | 8 | 2.31 | 0.83 | −0.57 | −1.22 | −1.49 |
| 5/6 | 10 | 2.31 | 0.86 | −0.22 | −0.58 | −0.77 |

Chosen: d = 4 for k ≤ 1600, d = 6 for k ≥ 1800. Dense rows help little and hurt the
low-rate end, where the extension must reach every bit: at d = 10 the rate-1/2 code
gains almost nothing past the first increment. The first 600-bit increment (a
quarter codeword) buys 0.7 to 1.5 dB; the whole extension 2.3 to 4.8 dB. The rate-1/4
mother code sits about 2.7 dB from the Shannon limit at rate 1/4 (−6.8 dB Es/N0;
the BPSK-input limit is somewhat higher, so the gap to it a little smaller), part of
it finite length: a jointly designed protograph would do better, but the point here
is whether IR pays at all.

### 5.2 Goodput against CNR

`TRIALS=48 cargo test --release -p vardefm-modem ir_sweep -- --ignored --nocapture`:
48 bursts per point (192 codewords, 96 at narrow level 0), 0.5 dB steps. With 48
bursts, each of a codeword's up to 41 transmissions comes from a different burst.
Envelope over levels (genie level choice), kbit/s, at whole dB; the best level in
brackets for `resend` and `ir600`.

**Narrow**

| CNR dB | resend | chase | ir300 | ir600 | ir1200 |
|---|---|---|---|---|---|
| 3 | 0.57 (L0) | 0.72 | 0.69 | 0.67 (L0) | 0.64 |
| 4 | 0.70 (L0) | 0.90 | 1.32 | 1.25 (L1) | 1.11 |
| 5 | 1.44 (L1) | 1.45 | 1.84 | 1.74 (L2) | 1.46 |
| 6 | 2.18 (L2) | 2.18 | 2.62 | 2.52 (L3) | 2.26 |
| 7 | 2.91 (L3) | 2.91 | 3.43 | 3.31 (L4) | 2.94 |
| 8 | 3.83 (L4) | 3.90 | 4.33 | 4.27 (L4) | 4.13 |
| 9–10 | 4.40 (L4) | 4.40 | 4.40 | 4.40 (L4) | 4.40 |
| 11 | 4.40 (L4) | 4.40 | 4.60 | 4.47 (L5) | 4.40 |
| 12 | 4.40 (L4) | 4.40 | 5.24 | 4.88 (L6) | 4.85 |
| 13 | 5.75 (L5) | 5.75 | 5.83 | 5.82 (L5) | 5.79 |
| 14 | 5.84 (L5) | 5.84 | 5.90 | 5.84 (L5) | 5.84 |
| 15 | 5.84 (L5) | 5.84 | 6.49 | 5.91 (L6) | 5.84 |
| 16 | 6.15 (L6) | 6.28 | 7.18 | 7.04 (L6) | 6.77 |
| 17+ | 7.33 (L6) | 7.33 | 7.33 | 7.33 (L6) | 7.33 |
| mean 0–18 dB | 3.56 | 3.61 (+1 %) | 3.87 (+9 %) | 3.77 (+6 %) | 3.68 (+3 %) |

**Wide**

| CNR dB | resend | chase | ir300 | ir600 | ir1200 |
|---|---|---|---|---|---|
| 4 | 0.00 (L0) | 0.99 | 1.08 | 1.04 (L0) | 0.97 |
| 5 | 1.86 (L0) | 1.86 | 1.88 | 1.87 (L0) | 1.87 |
| 6 | 2.21 (L1) | 2.71 | 3.54 | 3.42 (L1) | 3.15 |
| 7 | 3.78 (L1) | 3.78 | 4.54 | 4.39 (L2) | 3.81 |
| 8 | 5.15 (L2) | 5.20 | 5.65 | 5.58 (L2) | 5.45 |
| 9 | 5.72 (L2) | 5.72 | 5.72 | 5.72 (L2) | 5.72 |
| 10 | 5.72 (L2) | 5.72 | 6.17 | 5.92 (L3) | 5.72 |
| 11 | 6.97 (L3) | 6.97 | 7.44 | 7.37 (L3) | 7.23 |
| 12 | 7.51 (L3) | 7.51 | 7.68 | 7.53 (L4) | 7.51 |
| 13 | 7.51 (L3) | 7.51 | 8.71 | 8.38 (L4) | 7.58 |
| 14 | 7.51 (L3) | 7.51 | 9.91 | 9.17 (L4) | 7.69 |
| 15 | 10.40 (L4) | 10.40 | 11.24 | 11.11 (L4) | 10.86 |
| 16–18 | 11.37 (L4) | 11.37 | 11.37 | 11.37 (L4) | 11.37 |
| 19 | 11.37 (L4) | 11.37 | 11.60 | 11.40 (L5) | 11.37 |
| 20 | 11.37 (L4) | 11.37 | 13.30 | 12.35 (L5) | 12.08 |
| 21 | 14.44 (L5) | 14.44 | 14.57 | 14.55 (L5) | 14.51 |
| 22 | 14.59 (L5) | 14.59 | 14.65 | 14.59 (L5) | 14.59 |
| 23 | 14.59 (L5) | 14.59 | 16.20 | 14.68 (L6) | 14.59 |
| 24 | 14.83 (L6) | 15.35 | 17.88 | 17.47 (L6) | 16.70 |
| 25+ | 18.31 (L6) | 18.31 | 18.31 | 18.31 (L6) | 18.31 |
| mean 1.5–26 dB | 8.43 | 8.54 (+1 %) | 9.13 (+8 %) | 8.91 (+6 %) | 8.68 (+3 %) |

Peaks between whole dB: narrow 7.5 dB +34 % (ir300) / +23 % (ir600), wide 14.5 dB
+40 % / +30 %. At a few points (narrow 2.5, 3.5 and 12.0 dB, wide 4.0, 5.5 and
20.0 dB) the best level won at the lowest CNR it was measured at, so the scheme
there is if anything understated.

### 5.3 Reading

- **The waterfalls are steep.** Through the FM channel a level goes from 100 % to 0 %
  CER within about 1 dB (narrow level 3: 95 % at 5.75 dB, 0.1 % at 6.5 dB). Just below
  a level's point, a resend at that level is nearly worthless, and today's best choice
  is the level below. So resends lose little where they are used; the loss is in the
  gaps of the ladder, where the next level up does not yet work at all.
- **IR fills the gaps.** In the 2 to 3 dB below a level's point, that level with
  increments gains 0 to 40 % over the level below, most at the half-dB points just
  under the point (narrow 7.5 dB +23 %, 12.5 dB +24 %; wide 14.5 dB +30 %, all ir600)
  and next to nothing where the level below has just reached full rate (narrow 4.5,
  6.5 and 11 dB). At the bottom (narrow 4 dB, wide 6 dB) it gains 54 to 88 %. Deeper in the wide gaps (narrow 9 to 10.5 dB, wide
  16 to 18.5 dB) it gains nothing. Above a level's point all schemes coincide.
  Averaged over the CNR range the gain is +6 % (ir600) to +8–9 % (ir300), in both
  modes.
- **Chase combining is no substitute**: doubling a codeword's cost to gain 3 dB rarely
  beats dropping a level (+1 % on average).
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
(ir600), up to 20 to 30 % just under a level's point and next to nothing elsewhere
in the gaps between levels or above a level's point. It
needs a breaking protocol change (capability exchange, a header flag, new bitmap
semantics, a normative extension construction) and LLR storage and bookkeeping in
the receiver.

Suggested decision: **adopt in a future protocol version step, not on its own.**
Bundle it with the next breaking change, as ir600 with the extension of §2, and
before that build the link-layer prototype behind the capability flag and measure it
in `sim-pair` against today's resends, with `SNR_MARGIN_DB` retuned for IR. If no
breaking change is planned, close the issue with these numbers: the average gain
alone does not carry the compatibility break.
