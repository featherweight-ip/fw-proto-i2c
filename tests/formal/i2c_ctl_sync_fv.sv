// ======================================================================
// Formal check of the controller core's clock synchronization (UM10204
// §3.1.7), on its own. The back-to-back harness (i2c_proto_fv.sv) has one
// controller and one target, and the target never pulls SCL low during a
// high phase, so a sync check there would be vacuous. Here the controller
// shares SCL and SDA with a free "other device" that may pull either line
// low on any clock -- another controller, or a target misbehaving -- and
// takes a free op stream.
//
// T_LOW and T_HIGH are 2 and 3 clocks, not 1, so that a high phase cut
// short by another device's fall is distinguishable from one that ran its
// full count. See PROPERTIES.md "Clock synchronization" for the property
// list.
// ======================================================================
`ifdef FORMAL

module i2c_ctl_sync_fv (
        input wire clock,
        input wire reset
    );

    localparam int T_LOW  = 2;
    localparam int T_HIGH = 3;

    // f_state encodings (state_e in i2c_controller_xtor_core.sv).
    localparam bit [3:0] S_START_EDGE  = 4'd4;
    localparam bit [3:0] S_BIT_LOW     = 4'd5;
    localparam bit [3:0] S_BIT_HOLD_HI = 4'd7;
    localparam bit [3:0] S_ARB_ABORT   = 4'd12;
    localparam bit [3:0] S_DONE        = 4'd13;

    (* anyseq *) wire [23:0] f_req_dat;
    (* anyseq *) wire        f_req_valid;
    (* anyseq *) wire        f_rsp_ready;
    (* anyseq *) wire        f_other_scl_low;
    (* anyseq *) wire        f_other_sda_low;

    wire       ctl_scl_o, ctl_scl_oe, ctl_sda_o, ctl_sda_oe;
    wire [9:0] ctl_rsp_dat;
    wire       ctl_rsp_valid, ctl_req_ready;
    wire [3:0] f_state, f_bit_idx;

    wire scl = (ctl_scl_oe || f_other_scl_low) ? 1'b0 : 1'b1;
    wire sda = (ctl_sda_oe || f_other_sda_low) ? 1'b0 : 1'b1;

    i2c_controller_xtor_core #(
        .T_LOW(T_LOW), .T_HIGH(T_HIGH),
        .T_HD_STA(1), .T_SU_STA(1), .T_SU_STO(1), .T_BUF(1)
    ) u_ctl (
        .clock(clock), .reset(reset), .prescale(16'd0),
        .scl_o(ctl_scl_o), .scl_oe(ctl_scl_oe), .scl_i(scl),
        .sda_o(ctl_sda_o), .sda_oe(ctl_sda_oe), .sda_i(sda),
        .req_dat(f_req_dat), .req_valid(f_req_valid), .req_ready(ctl_req_ready),
        .rsp_dat(ctl_rsp_dat), .rsp_valid(ctl_rsp_valid), .rsp_ready(f_rsp_ready),
        .f_state(f_state), .f_bit_idx(f_bit_idx)
    );

    reg f_past_valid;
    initial f_past_valid = 1'b0;
    always @(posedge clock) f_past_valid <= 1'b1;
    always @(*)            if (!f_past_valid) assume (reset);
    always @(posedge clock) if (f_past_valid) assume (!reset);

    wire in_hold_hi = (f_state == S_BIT_HOLD_HI);
    wire in_hi_cnt  = in_hold_hi || (f_state == S_START_EDGE);
    wire in_low     = (f_state == S_BIT_LOW);

    // Consecutive clocks spent in BIT_HOLD_HI / BIT_LOW, before this one.
    reg [3:0] hold_run, low_run;
    always @(posedge clock) begin
        if (reset) begin
            hold_run <= 4'd0;
            low_run  <= 4'd0;
        end else begin
            hold_run <= in_hold_hi ? hold_run + 4'd1 : 4'd0;
            low_run  <= in_low     ? low_run  + 4'd1 : 4'd0;
        end
    end

    always @(posedge clock) begin
        if (f_past_valid && !$past(reset)) begin
            // S1: the controller counts high time only while SCL reads
            // high. A clock of a high-phase count (a bit's T_HIGH, or the
            // START hold) that sees SCL low is the last one: the next
            // clock starts the low phase, or ends the op after the ACK bit
            // or a lost arbitration.
            if ($past(in_hi_cnt) && !$past(scl))
                assert (f_state == S_BIT_LOW || f_state == S_DONE ||
                        f_state == S_ARB_ABORT);

            // S2: a high phase never outlasts T_HIGH.
            if (in_hold_hi) assert (hold_run < T_HIGH);

            // S3: every low phase the controller drives is a full T_LOW,
            // including one restarted by another device's early fall.
            if (!in_low && low_run != 0) assert (low_run == T_LOW);
            if (in_low) assert (ctl_scl_oe);
        end
    end

    // S4 (item 3 shares it): the bit index stays in 0..8.
    always @(*) if (!reset) assert (f_bit_idx <= 4'd8);

    // Non-vacuity: another device cuts a bit's high phase short, and the
    // START hold short; and an early-cut bit is followed by a full low.
    always @(posedge clock) begin
        if (f_past_valid && !$past(reset)) begin
            cover ($past(in_hold_hi) && !$past(scl) && $past(hold_run) < T_HIGH - 1 &&
                   f_state == S_BIT_LOW);
            cover ($past(f_state == S_START_EDGE) && !$past(scl) && f_state == S_BIT_LOW);
            cover (!in_low && low_run == T_LOW && $past(low_run, T_LOW + 1) == 0 &&
                   $past(in_hold_hi, T_LOW + 1) && !$past(scl, T_LOW + 1));
        end
    end

endmodule

`endif // FORMAL
