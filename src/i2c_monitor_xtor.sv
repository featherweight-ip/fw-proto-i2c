`include "i2c_xtor_macros.svh"

// I2C Monitor transactor (integration): read-only tap, interface + core.
module i2c_monitor_xtor (
        input  wire clock,
        input  wire reset,
        `I2C_MONITOR_PORT()
    );

    wire [12:0] mon_dat;
    wire        mon_valid;
    wire        mon_ready;

    i2c_monitor_xtor_if u_if (
        .clock(clock), .reset(reset),
        .mon_dat(mon_dat), .mon_valid(mon_valid), .mon_ready(mon_ready)
    );

    i2c_monitor_xtor_core u_core (
        .clock(clock), .reset(reset),
        .scl_i(scl_i), .sda_i(sda_i),
        .mon_dat(mon_dat), .mon_valid(mon_valid), .mon_ready(mon_ready)
    );

endmodule
