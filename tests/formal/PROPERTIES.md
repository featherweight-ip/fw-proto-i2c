# I2C kit — invariant properties → checkers

The spec → invariant → checker map for the kit's two formal harnesses:

- `i2c_proto_fv.sv`, the back-to-back harness (design.md §8 issue 2). One
  controller core and one target core share a real open-drain SCL/SDA bus;
  a free environment drives the controller's op stream and the target's
  model decisions.
- `i2c_ctl_sync_fv.sv`, the controller core alone against a free other
  device on SCL and SDA: clock synchronization (see "Clock
  synchronization" below).

```
dfm run fv          # BMC: back-to-back at depth 80, sync at depth 40
dfm run fv-cover    # cover goals of both
```

## Harness shape

The kit's usual formal pattern is `free src --up link--> core --bus--> core
--up link--> free snk`. Here both cores drive and sense both lines (the
target stretches SCL; each side drives SDA in its own phases), so the "bus"
is a wired-AND of the two cores' `*_oe` outputs.

Timing parameters are all 1 clock (`T_LOW=T_HIGH=...=1`). That keeps the
SMT state small, and it is also the fastest bus the target core must follow
(see "Fixed — the target lost bit 7 ..." below).

Data integrity uses direct "last byte" registers, one per direction, not
the anyconst-index tracker: the class API is one blocking op at a time, so
at most one byte is ever in flight.

## Environment assumptions (the caller contract)

Each one scopes the proof to a correct user of the per-byte class API
(design.md §4). Each was added because BMC found the unconstrained
environment breaking it, not because the cores mis-handled a legal caller.

| # | Assumption | Why |
| --- | --- | --- |
| A1 | Reset in the first step, never after | Standard formal reset. |
| A2 | `tx_byte()`/`rx_byte()`/`stop()` only after a `start()` | A bare STOP with no START is a caller error the core can't detect. |
| A3 | `tx_byte()` only in a write (R/W=0), `rx_byte()` only in a read | START(write)+RX_BYTE makes the target clock in 0xFF from the pull-ups, which the integrity check then mis-pairs. Found at depth 68. |
| A4 | The target model never answers in the same cycle the core asks | A real bridge task takes at least one cycle. |
| A5 | After an ACKed read `start()`, or an `rx_byte()` with `ack=1`, the next op is `rx_byte()` (no `stop()`/Sr until a byte is NACKed) | UM10204: the target owns SDA for the next read byte. START(read)+ACK+Sr let the target drive its first data bit, so the Sr edge never appeared and invariant 7 paired the wrong byte. Found at depth 77. Stated in `i2c_controller_if.svh`; the VIP driver rejects a 0-byte read. |

## Invariants

| # | Rule (UM10204) | Class | Checker | Status |
| --- | --- | --- | --- | --- |
| 1 | A STOP only closes an open transaction (never STOP without a prior START) | framing | `assert ($past(in_txn))` on every `stop_edge` | in BMC |
| 2 | ACK sampled on the 9th clock, not before | framing | each address or data byte the controller sent (not lost to arbitration) ends after exactly 9 SCL rises, counted on the bus, and its reported `ack` is the inverse of SDA at the 9th rise | in BMC |
| 3 | The controller's bit index stays in 0..8, and tracks the bus | FSM legality | `f_bit_idx <= 8` always; whenever the controller samples a bit (`BIT_WAIT_HI` with SCL high), `f_bit_idx + 1` equals the bus's SCL rises in this byte | in BMC |
| 4 | Clock-low-wins: the bus never reads SCL high while either core holds it low | handshake | `assert (!$past(scl))` whenever either `*_scl_oe` was set | in BMC |
| 5 | Arbitration is lost only to another controller | mutual exclusion | with one controller, no response ever has `arb_lost=1`: one would mean the target pulled SDA low over a bit the controller drove | in BMC; real arbitration is exercised by the `i2c_arb` sim |
| 6 | Data integrity end to end, write: each `tx_byte()` arrives at the target intact | integrity | `tgt RX_BYTE data == last_tx_byte` | in BMC |
| 7 | Data integrity end to end, read: each byte the target's model supplies reaches the controller's `rx_byte()` intact | integrity | `ctl RX_BYTE rd_data == last_sup_byte` | in BMC |
| 8 | START, Sr and STOP edges are the controller's own | framing | a START edge only while a `start()` op is in progress, with the controller pulling SDA low and the target not; a STOP edge only while a `stop()` op is | in BMC |
| 9 | Address integrity, START and Sr: each address phase the target raises carries the controller's last `start()` address and R/W | integrity | on each new `ADDRESSED` request: a `start()` is pending, and `{addr, read}` match it | in BMC |

Invariants 2 and 3 read the controller's formal-only debug outputs
(`f_state`, `f_bit_idx`; see "Harness pitfalls"). Invariant 2 takes its
bit count and SDA sample from the bus alone, so it doesn't just restate
the core's `bit_idx == 8` write site.

## Cover goals (non-vacuity)

| Goal | First reached |
| --- | --- |
| A data byte reaches the target's up-link (`RX_BYTE` request accepted) | step 63 |
| A byte the target supplied reaches the controller (invariant 7 can fire) | step 68 |
| A real STOP closes a transaction, out of reset | step 42 |
| An address phase opened by an Sr reaches the target (invariants 8 and 9 see a repeated START) | step 66 |

The BMC depth must be above 68, or invariant 7 can never fire (63 for
invariant 6). The earlier depth of 40 "passed" for exactly that reason.
BMC at depth 80 passes in about 24 minutes (12 before invariants 2 and 3
were restored).

## Clock synchronization

UM10204 §3.1.7: SCL is a wired-AND, so its low phase lasts as long as the
longest low period of the controllers driving it, and its high phase as
long as the shortest high period. The first device to end its high phase
pulls SCL low, and every other controller starts its low count from that
fall. The controller core used to count `T_HIGH` without watching SCL.

The back-to-back harness can't check this: nothing in it pulls SCL low
during a high phase. `i2c_ctl_sync_fv.sv` puts the controller core alone
on SCL/SDA with a free other device that can pull either line low on any
clock, and takes a free op stream. `T_LOW = 2` and `T_HIGH = 3`, so a
high phase cut short is distinguishable from a full one.

| # | Rule | Checker | Status |
| --- | --- | --- | --- |
| S1 | The controller counts high time only while SCL reads high | a clock of a high count (`BIT_HOLD_HI`, or the START hold `START_EDGE`) that sees SCL low is its last: next is `BIT_LOW`, or `DONE`/`ARB_ABORT` at the end of an op | in BMC (depth 40) |
| S2 | A high phase never outlasts `T_HIGH` | consecutive `BIT_HOLD_HI` clocks `< T_HIGH` | in BMC |
| S3 | Every low phase the controller drives is a full `T_LOW`, including one started by another device's fall | consecutive `BIT_LOW` clocks `== T_LOW` on exit, with SCL driven low throughout | in BMC |
| S4 | The bit index stays in 0..8 (invariant 3, with free bus inputs) | `f_bit_idx <= 8` | in BMC |

Cover goals, all reached by step 9: a bit's high phase cut short by the
other device; the START hold cut short; a full `T_LOW` after a cut-short
high phase.

Teeth: with the `!scl_i` exit taken back out of `BIT_HOLD_HI`, BMC fails
S1. With the first low after a START counted as `T_HD_STA` again (next
section), it fails S3. Unbounded `mode prove` (k-induction) fails its
induction step until the run counters are tied to the core's `tick_cnt`,
so this harness is BMC only for now.

The sim side is `tests/arb/i2c_arb_sync_tb.sv`: the arbitration collision
with the winner at `T_LOW=3, T_HIGH=16` and the loser at `T_LOW=7,
T_HIGH=3`, addresses differing only in the LSB so the two controllers
share seven bits. The bench checks that each contended low phase lasts the
longer `T_LOW` (up to 2 clocks more) and each contended high phase at most
the shorter `T_HIGH` plus 2. Before the fix, the winner kept counting high
through the loser's low phase and the two lost bit alignment: the winner's
address was not ACKed.

## Fixed — the first low phase after a START, and the last before a STOP

Writing `i2c_arb_sync_tb` showed that the controller held the low phase
after a START, and the one before a STOP, for `T_HD_STA` clocks, not
`T_LOW`. The START parameters were also swapped: `START_EDGE` (SDA low,
SCL high, the hold time t_HD;STA) counted `T_SU_STA`, and the setup time
before an Sr (t_SU;STA) wasn't counted at all. Now `START_HOLD_HI` counts
`T_SU_STA` once SCL reads high, `START_EDGE` counts `T_HD_STA`, and both
of those low phases are `T_LOW`. It didn't show at the default timing,
where `T_HD_STA = 2` is shorter than `T_LOW = 4` and the target follows any
low phase, or in formal, which runs every phase at 1 clock.

## Fixed — the controller could leave reset driving SDA low

In `IDLE` the controller presents the last bit's SDA drive
(`cur_bit_sda_oe`). That is computed from `payload_is_tx`, `byte_reg` and
`ack_in_r`, none of which were reset. So the core could come out of reset
holding SDA low: a spurious START on the bus, then a STOP when it let go.
The cover trace showed exactly that at step 2. Sims start those registers
at X and miss it. Fixed by resetting the three registers to "not driving".

## Fixed — the controller could drive SDA low after a STOP

Same mechanism, different trigger. Accepting a STOP op reset `bit_idx` to
0 but left `payload_is_tx` and `byte_reg` from the last write, so in
`DONE`/`IDLE` after the STOP `cur_bit_sda_oe` followed `byte_reg[7]`. If
the last byte written had MSB 0, the controller pulled SDA low with SCL
high: a spurious START, then a STOP when the next op released it. Found by
the fwvip-i2c UVM bench's bus scoreboard, not by formal; the kit sims only
ever ended a write with an MSB-1 byte. Fixed by clearing `payload_is_tx`
and `ack_in_r` on STOP; `i2c_proto_tb` now ends a write with 0x3C, so its
STOP count catches a regression.

**Formal gap this exposed:** no invariant said that a START or STOP edge
only happens while the controller is making one. Invariant 1 accepts the
spurious pair, since the STOP does close an (also spurious) transaction.
Now invariant 8. Teeth check: with the fix taken back out, BMC fails
invariant 8 at step 42.

## Fixed — the target lost bit 7 after an ACK when SCL low was 1 clock

BMC counterexample (depth 68): after the address ACK, the controller drove
data `0xFE` and the target delivered `0xFD`. The target never sampled bit 7
(the MSB, sent first). It took bits 6..0 as bits 7..1, and the
controller's released ACK slot (a 1) as bit 0.

Cause (`src/i2c_target_xtor_core.sv`): `DRIVE_ACK_WAIT_FALL` saw SCL fall
and moved to `WATCH`, and `WATCH` then tested `!scl_i` as a *level* to start
the next byte. One low phase had to be sampled twice. The controller holds
SCL low for exactly `T_LOW` clocks, so with `T_LOW=1` the second sample saw
SCL high, and the target came in one bit late.

The read path had the same double sample: `SAMPLE_ACK_WAIT_FALL` →
`WATCH` → `UPREQ`, and the address ACK of a read. There the controller read
the released bit 7 as 1, and the target drove every bit one position late,
into the controller's ACK slot. The monitor core had it too (`M_BIT_WAIT_FALL`
→ `M_WATCH`). Invariant 6 checked writes only, so neither showed up in
formal. The sims run at `T_LOW=4`, which hid all three.

Fix: the `*_WAIT_FALL` state that sees the fall ending the ACK bit starts
the next byte itself (`next_byte()` in the target; straight to
`M_BIT_WAIT_RISE` in the monitor). `WATCH`/`M_WATCH` are now only the idle
states. This is the same guess `WATCH` made ("a data bit follows"), made
one cycle earlier, so STOP/Sr handling is unchanged: the global edge
override still corrects it. A 1-clock low or high phase is now enough
everywhere.

Regressions:

- invariant 7 (read integrity) and its cover;
- `tests/i2c_proto_fast_tb.sv`, the back-to-back sim with every timing
  phase at 1 clock. It starts with a read of unwritten bytes, which have
  MSB 0, so a lost bit 7 shows even if writes are broken. Before the fix
  it failed on reads (`0x5A` read as `0xAD`), on writes, and on the
  monitor's byte count.

Adding invariant 7 also turned up a caller-contract gap, now assumption A5:
the harness allowed a STOP or Sr straight after an ACKed read address.

## Repeated START

The target core's header used to list repeated START as a known gap: the
controller drops SCL once before an Sr, the target takes that drop as the
start of a data byte, and the note expected the target to mis-count. In
practice the global START override already handles it. The Sr's SCL rise
clocks in one bit, and the Sr edge then discards it and receives the new
address. The note was written before the 1-clock fix made every
`*_WAIT_FALL` start the next byte directly.

What now covers Sr:

- `i2c_proto_tb` (and `i2c_proto_fast_tb`, at 1-clock timing): write → Sr →
  read, read → Sr → write → Sr → read, NACKed address → Sr → read, with
  data checked and the monitor's START/Sr/STOP/byte counts matched to the
  driver's.
- Invariants 8 and 9, with the Sr cover goal showing the BMC reaches an
  Sr address phase (step 66, inside depth 80). The free environment
  already allowed `repeated=1`; until now nothing showed that it got there.

## Harness pitfalls

**Hierarchical references do not work in this flow.** The harness used to
read `u_ctl.state`, `u_ctl.driving_now` and `u_ctl.intended_bit`. sv2v
passes them through, and yosys then declares each one as an implicit,
undriven wire (`Identifier '\u_ctl.state' is implicitly declared`, then
`setundef` makes it a constant). So the old invariant 5 never checked
anything, and invariant 6's arbitration-loss reset never fired. The
dropped checks for invariants 2 and 3 (below) used the same kind of
reference, which explains their "failures within a few steps". Every check
now uses the cores' ports and the harness's own registers. To check
internal state, bring it out of the core through a port: the controller
core has formal-only outputs `f_state` and `f_bit_idx`, declared under
`` `ifdef FORMAL `` so the sim and lint builds don't see them. `state_e`
has explicit encodings, and each harness decodes `f_state` with
localparams that mirror them.

The harness's own tracking registers (`sclq`, `sdaq`, `in_txn`, ...) reset
*synchronously*, so during the reset step they hold free ("anyinit")
initial values. Any check or cover that looks at them with `$past` must be
gated on `!$past(reset)`. An ungated `cover ($past(stop_edge))` was hit
vacuously at step 2.

Invariants 2 and 3 were dropped for the same reason. They are back, on the
debug outputs.
