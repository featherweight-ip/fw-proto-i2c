# I2C kit — invariant properties → checkers

The spec → invariant → checker map for `i2c_proto_fv.sv`, the back-to-back
formal harness (design.md §8 issue 2). One controller core and one target
core share a real open-drain SCL/SDA bus; a free environment drives the
controller's op stream and the target's model decisions.

```
dfm run fv          # BMC, depth 80
dfm run fv-cover    # cover goals, depth 80
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
| 2 | ACK sampled on the 9th clock, not before | framing | **none.** Relies on the single `bit_idx == 8` write site of `ack_result` (by inspection). A `$past`-based check was dropped; see "Harness pitfalls". | gap |
| 3 | The controller's bit index stays in 0..8 | FSM legality | **none.** A `u_ctl.bit_idx <= 8` check was dropped; see "Harness pitfalls". | gap |
| 4 | Clock-low-wins: the bus never reads SCL high while either core holds it low | handshake | `assert (!$past(scl))` whenever either `*_scl_oe` was set | in BMC |
| 5 | Arbitration loss only after driving 1 and sensing 0 | mutual exclusion | `ARB_ABORT ⇒ arb_cause_seen` | vacuous here (one controller); arbitration is exercised by the `i2c_arb` sim |
| 6 | Data integrity end to end, write: each `tx_byte()` arrives at the target intact | integrity | `tgt RX_BYTE data == last_tx_byte` | in BMC |
| 7 | Data integrity end to end, read: each byte the target's model supplies reaches the controller's `rx_byte()` intact | integrity | `ctl RX_BYTE rd_data == last_sup_byte` | in BMC |

## Cover goals (non-vacuity)

| Goal | First reached |
| --- | --- |
| A data byte reaches the target's up-link (`RX_BYTE` request accepted) | step 63 |
| A byte the target supplied reaches the controller (invariant 7 can fire) | step 68 |
| A real STOP closes a transaction, out of reset | step 42 |

The BMC depth must be above 68, or invariant 7 can never fire (63 for
invariant 6). The earlier depth of 40 "passed" for exactly that reason.
BMC at depth 80 passes in about 14 minutes.

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

**Formal gap this exposes:** no invariant says "a START or STOP edge only
happens while the controller is in a START or STOP state". Invariant 1
accepts the spurious pair, since the STOP does close an (also spurious)
transaction. Worth adding as invariant 8.

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

## Harness pitfalls

The harness's own tracking registers (`sclq`, `sdaq`, `in_txn`, ...) reset
*synchronously*, so during the reset step they hold free ("anyinit")
initial values. Any check or cover that looks at them with `$past` must be
gated on `!$past(reset)`. An ungated `cover ($past(stop_edge))` was hit
vacuously at step 2.

This, or the controller's unreset registers (fixed above), may also explain
why the dropped checks (invariants 2 and 3) "failed" within a few steps.
Retry them with the reset gating before treating them as tool problems.
