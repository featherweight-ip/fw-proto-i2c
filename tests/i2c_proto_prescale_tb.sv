// ======================================================================
// i2c_proto_tb with the timing set at runtime: freecores-shaped phases in
// ticks (T_LOW 3, T_HIGH 2, T_HD_STA 2, T_SU_STA 1, T_SU_STO 2, T_BUF 4 --
// the OpenCores i2c core's 5-tick bit) and prescale 5, set through
// set_prescale() after reset. That is 400 kHz from a 12 MHz clock
// (12e6 / (5 * 6) = 400e3), the fw-i2c / iCEBreaker use case. Same
// stimulus and checks as the default case, plus the low-phase length check
// at T_LOW*(prescale+1) clocks.
//
// Run:  dfm run i2c-proto-prescale
// ======================================================================
module i2c_proto_prescale_tb;
    i2c_proto_tb #(
        .T_LOW(3), .T_HIGH(2), .T_HD_STA(2), .T_SU_STA(1), .T_SU_STO(2), .T_BUF(4),
        .PRESCALE(5)
    ) u_tb ();
endmodule
