// Monitor API: receive one reconstructed bus event the passive monitor
// observed. Non-blocking (functions, not tasks) per the kit's monitor rule.
// Implemented by a subscriber; i2c_monitor_xtor_bridge calls these for each
// observed event.
interface class i2c_monitor_if;

    // A START (repeated=0) or repeated-START/Sr (repeated=1) condition.
    pure virtual function void observe_start(input bit repeated);

    // A STOP condition.
    pure virtual function void observe_stop();

    // One observed byte-with-ack: addr_phase=1 for the address+R/W byte
    // (and, after an ACKed 10-bit write header byte 11110 A9 A8 W, for the
    // addr[7:0] byte that follows it), 0 for a data byte. ack is the
    // sampled ACK (1)/NACK (0) for that byte.
    pure virtual function void observe_byte(
            input bit       addr_phase,
            input bit [7:0] data,
            input bit       ack);

    // The monitor inferred that a controller lost arbitration mid-byte.
    // NOTE: not implemented by the v1 monitor core (see
    // i2c_monitor_xtor_core.sv header) -- never called in v1. Kept in the
    // API per design.md §4 so a subscriber's shape does not have to change
    // when that inference is added.
    pure virtual function void observe_arb_lost();

endclass
