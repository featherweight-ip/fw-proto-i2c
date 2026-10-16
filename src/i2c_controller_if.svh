// Controller-side (initiator role) API -- per-byte, per design.md §4. Each
// task moves exactly one byte-sized unit across the bus and returns (one
// task per internal-link op, i2c_xtor_macros.svh's I2C_OP_*).
//
// On arb_lost=1 from ANY task, the other output (ack/data) is undefined --
// arbitration can be lost on any bit including the ACK bit itself, so the
// caller must check arb_lost first and not act on the other outputs.
//
// Caller contract (UM10204): call tx_byte() only after a write start() and
// rx_byte() only after a read start(). After a read start() is ACKed, the
// target owns SDA for the next byte, so the next call must be rx_byte();
// keep calling it until one passes ack=0. Only then may stop() or a
// repeated start() follow. A read therefore moves at least one byte. A
// stop() or Sr issued while the target still owns SDA never appears on the
// bus, and what follows is corrupt. The cores cannot detect a broken
// contract; the formal harness assumes it (tests/formal/PROPERTIES.md).
interface class i2c_controller_if;

    // Drives START (repeated=0) or Sr (repeated=1, this role already owns
    // the bus) followed by the address+R/W byte. addr_10b is accepted but
    // NOT YET IMPLEMENTED by the v1 core (design.md §8 issue 4) -- callers
    // must pass addr_10b=0.
    pure virtual task start(
            input  bit [9:0] addr,
            input  bit       addr_10b,
            input  bit       read,
            input  bit       repeated,
            output bit       ack,
            output bit       arb_lost);

    // One byte in the write direction (this role is sending).
    pure virtual task tx_byte(
            input  bit [7:0] data,
            output bit       ack,
            output bit       arb_lost);

    // One byte in the read direction (this role is receiving). `ack` is an
    // INPUT: the master decides ACK (keep reading) / NACK (this is the last
    // byte wanted) before the byte clocks in, per spec.
    pure virtual task rx_byte(
            input  bit       ack,
            output bit [7:0] data,
            output bit       arb_lost);

    // Terminates the transaction (STOP). No arb_lost output: a loss anywhere
    // aborts at the start()/tx_byte()/rx_byte() call where it happened, so
    // only the eventual bus winner ever reaches stop().
    pure virtual task stop();

endclass
