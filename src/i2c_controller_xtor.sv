`include "i2c_xtor_macros.svh"

// ----------------------------------------------------------------------------
// I2C Controller transactor (integration): interface (task API) + core (bus
// FSM), wired by the internal ready/valid link. Public pins are the bare
// open-drain SCL/SDA triples (kit convention) -- see i2c_xtor_macros.svh's
// `I2C_ROLE_PORT.
// ----------------------------------------------------------------------------
module i2c_controller_xtor (
        input  wire clock,
        input  wire reset,
        `I2C_ROLE_PORT()
    );

    wire [23:0] req_dat;
    wire        req_valid;
    wire        req_ready;
    wire [9:0]  rsp_dat;
    wire        rsp_valid;
    wire        rsp_ready;

    i2c_controller_xtor_if u_if (
        .clock(clock), .reset(reset),
        .req_dat(req_dat), .req_valid(req_valid), .req_ready(req_ready),
        .rsp_dat(rsp_dat), .rsp_valid(rsp_valid), .rsp_ready(rsp_ready)
    );

    i2c_controller_xtor_core u_core (
        .clock(clock), .reset(reset),
        .scl_o(scl_o), .scl_oe(scl_oe), .scl_i(scl_i),
        .sda_o(sda_o), .sda_oe(sda_oe), .sda_i(sda_i),
        .req_dat(req_dat), .req_valid(req_valid), .req_ready(req_ready),
        .rsp_dat(rsp_dat), .rsp_valid(rsp_valid), .rsp_ready(rsp_ready)
    );

endmodule
