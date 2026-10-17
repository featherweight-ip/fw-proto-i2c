// Target-side (completer role) API -- per-byte, per design.md §4. The
// bridge calls into a model once per address phase and once per data byte;
// the model drives ACK/NACK and (for reads) data. Clock-stretch delay while
// the model call is in flight is a bridge/core concern (the core holds SCL
// low) -- the class API stays in the protocol's logical vocabulary.
interface class i2c_target_if;

    // Called for every address phase after a START/Sr; ack=1 selects this
    // target. read selects the direction of the tx_byte/rx_byte calls that
    // follow. addr_10b=1: addr is a full 10-bit address (UM10204 §3.1.11),
    // either from a write header whose first byte addressed_10b_hi() ACKed,
    // or, for a read (Sr 11110 A9 A8 R), the address of the last 10-bit
    // write header this target ACKed in the same transaction.
    pure virtual task addressed(
            input  bit [9:0] addr,
            input  bit       addr_10b,
            input  bit       read,
            output bit       ack);

    // Called for the first byte of a 10-bit write header (11110 A9 A8 W):
    // ack=1 if this target answers to a 10-bit address whose upper two bits
    // are addr_hi. On an ACK the second byte (addr[7:0]) follows, and with
    // it an addressed() call carrying the full address. The core answers
    // the first byte of a 10-bit read header itself (see addressed()).
    pure virtual task addressed_10b_hi(
            input  bit [1:0] addr_hi,
            output bit       ack);

    // One byte in the write direction (controller is sending): data is the
    // byte received, the target returns its ACK/NACK.
    //
    // NOTE: design.md §4 (as originally written) declared `data` here as an
    // OUTPUT, which does not type-check against its own description ("data
    // is the byte received") -- the bridge already has the byte from the
    // bus and must tell the model, so `data` must be an INPUT to the
    // model's implementation. Fixed here during kit implementation; flagged
    // for the coordinator to correct in design.md §4 too.
    pure virtual task rx_byte(
            input  bit [7:0] data,
            output bit       ack);

    // One byte in the read direction (controller is receiving): the target
    // supplies data. `ack` is an INPUT carrying the controller's ACK/NACK of
    // the PREVIOUS byte this call sequence sent (ignore its value on the
    // first call of a read -- there is no previous byte yet). Per spec the
    // controller alone decides when a read ends by NACKing the last byte it
    // wants; the bridge calls tx_byte() again only if the sampled ack was 1,
    // so in practice `ack` is always 1 at every call site this task actually
    // receives (there is no final "here is your NACK" call) -- see
    // i2c_target_bridge.svh's run() loop for exactly where that is decided.
    pure virtual task tx_byte(
            output bit [7:0] data,
            input  bit       ack);

endclass
