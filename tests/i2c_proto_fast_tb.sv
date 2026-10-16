// ======================================================================
// i2c_proto_tb at the fastest bus the cores support: every controller
// timing phase 1 clock, so SCL is low for a single target-core clock.
// Same stimulus and checks as the default-timing case.
//
// The target core once needed SCL low for 2 of its clocks after an ACK
// (it sampled the low level twice, in DRIVE_ACK_WAIT_FALL and then WATCH),
// so at this speed it lost bit 7 of the next byte -- on writes and reads
// alike. Found by formal (tests/formal/PROPERTIES.md); this case is the
// sim regression.
//
// Run:  dfm run i2c-proto-fast
// ======================================================================
module i2c_proto_fast_tb;
    i2c_proto_tb #(
        .T_LOW(1), .T_HIGH(1), .T_HD_STA(1), .T_SU_STA(1), .T_SU_STO(1), .T_BUF(1)
    ) u_tb ();
endmodule
