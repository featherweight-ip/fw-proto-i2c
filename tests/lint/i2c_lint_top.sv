// Lint elaboration top: every synthesizable core of the kit, at its default
// parameters, on one open-drain bus, so Verilator's elaboration-time checks
// (width, latch, unused, undriven) see all of it. Link ports left open on
// purpose.
module i2c_lint_top (
    input wire clock,
    input wire reset
);
    wire ctl_scl_o, ctl_scl_oe, ctl_sda_o, ctl_sda_oe;
    wire tgt_scl_o, tgt_scl_oe, tgt_sda_o, tgt_sda_oe;

    // Wired-AND with pull-ups: a line reads low iff someone drives it low.
    wire scl = !((ctl_scl_oe && !ctl_scl_o) || (tgt_scl_oe && !tgt_scl_o));
    wire sda = !((ctl_sda_oe && !ctl_sda_o) || (tgt_sda_oe && !tgt_sda_o));

    i2c_controller_xtor_core u_ctl (
        .clock, .reset,
        .scl_o(ctl_scl_o), .scl_oe(ctl_scl_oe), .scl_i(scl),
        .sda_o(ctl_sda_o), .sda_oe(ctl_sda_oe), .sda_i(sda),
        .req_dat(), .req_valid(), .req_ready(),
        .rsp_dat(), .rsp_valid(), .rsp_ready());

    i2c_target_xtor_core u_tgt (
        .clock, .reset,
        .scl_o(tgt_scl_o), .scl_oe(tgt_scl_oe), .scl_i(scl),
        .sda_o(tgt_sda_o), .sda_oe(tgt_sda_oe), .sda_i(sda),
        .req_dat(), .req_valid(), .req_ready(),
        .rsp_dat(), .rsp_valid(), .rsp_ready());

    i2c_monitor_xtor_core u_mon (
        .clock, .reset,
        .scl_i(scl), .sda_i(sda),
        .mon_dat(), .mon_valid(), .mon_ready());
endmodule
