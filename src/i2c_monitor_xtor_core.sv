// ----------------------------------------------------------------------------
// I2C bus monitor core transactor (pure module, signal-level open-drain
// ports, READ-ONLY -- never drives scl_oe/sda_oe).
//
// Mirrors i2c_target_xtor_core's START/Sr/STOP + byte detection (identical
// edge logic), but samples every byte on the bus regardless of direction
// (it does not need to know who is driving to passively read SDA) and never
// participates in clock stretching -- it just watches whatever SCL/SDA do.
//
// NOT IMPLEMENTED in v1 (flagged in the kit implementation report):
// arbitration-loss inference (design.md §5's monitor core notes --
// "sda_i low when expected high with no local drive asserted"). A passive
// third-party tap inferring which bit belongs to which contending master,
// without its own drive-intent state, is a materially harder problem than
// the controller's own first-person arb-check and was not attempted here;
// i2c_monitor_if.svh's observe_arb_lost() exists but is never called.
// ----------------------------------------------------------------------------
module i2c_monitor_xtor_core (
        input  wire        clock,
        input  wire        reset,

        input  wire        scl_i,
        input  wire        sda_i,

        output wire [12:0] mon_dat,
        output wire        mon_valid,
        input  wire        mon_ready
    );

    typedef struct packed {
        bit [1:0] kind;
        bit       repeated;
        bit       addr_phase;
        bit [7:0] data;
        bit       ack;
    } mon_s;
    localparam bit [1:0] KIND_START = 2'd0;
    localparam bit [1:0] KIND_STOP  = 2'd1;
    localparam bit [1:0] KIND_BYTE  = 2'd2;

    typedef enum logic [2:0] {
        M_WATCH, M_ARM, M_BIT_WAIT_RISE, M_BIT_WAIT_FALL
    } state_e;
    state_e state;

    reg        in_frame;
    reg        addr_phase_r;
    reg [3:0]  bit_idx;
    reg [7:0]  shift_reg;
    reg        ack_bit;

    reg scl_i_q, sda_i_q;
    always @(posedge clock or posedge reset) begin
        if (reset) begin scl_i_q <= 1'b1; sda_i_q <= 1'b1; end
        else       begin scl_i_q <= scl_i; sda_i_q <= sda_i; end
    end
    wire start_edge = scl_i && scl_i_q && sda_i_q && !sda_i;
    wire stop_edge  = scl_i && scl_i_q && !sda_i_q && sda_i;

    reg        mon_valid_r;
    mon_s      mon_q;
    assign mon_valid = mon_valid_r;
    assign mon_dat   = mon_q;
    wire mon_fire = mon_valid && mon_ready;

    // Push one event into the single-entry buffer. Events on this bus are
    // separated by many clock cycles (each bit takes a full SCL period), so
    // a 1-deep buffer drained by an always-waiting bridge loop never backs
    // up in practice (same reasoning as wb_monitor_xtor_core).
    task automatic push(input mon_s e);
        mon_q       <= e;
        mon_valid_r <= 1'b1;
    endtask

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            state       <= M_WATCH;
            in_frame    <= 1'b0;
            mon_valid_r <= 1'b0;
        end else begin
            if (mon_fire) mon_valid_r <= 1'b0;

            // Global override, checked ahead of the per-state case below --
            // same reasoning as i2c_target_xtor_core.sv's matching override:
            // a monitor guessing "SCL fell, so another byte begins" can be
            // wrong (a STOP/Sr can follow that same fall), so a real
            // start_edge/stop_edge always wins over whatever partial byte
            // this core had started capturing (discarded, unreported).
            if (start_edge) begin
                push(mon_s'{kind: KIND_START, repeated: in_frame,
                             addr_phase: 1'b0, data: 8'h0, ack: 1'b0});
                in_frame     <= 1'b1;
                addr_phase_r <= 1'b1;
                bit_idx      <= '0;
                state        <= M_ARM;
            end else if (stop_edge) begin
                if (in_frame)
                    push(mon_s'{kind: KIND_STOP, repeated: 1'b0,
                                 addr_phase: 1'b0, data: 8'h0, ack: 1'b0});
                in_frame <= 1'b0;
                state    <= M_WATCH;
            end else begin
            unique case (state)
                // Out of a frame: only a START (the override) leaves.
                M_WATCH: ;

                M_ARM: if (!scl_i) state <= M_BIT_WAIT_RISE;

                M_BIT_WAIT_RISE: begin
                    if (scl_i) begin
                        if (bit_idx < 8) shift_reg[7 - bit_idx] <= sda_i;
                        else             ack_bit <= sda_i;
                        state <= M_BIT_WAIT_FALL;
                    end
                end
                M_BIT_WAIT_FALL: begin
                    if (!scl_i) begin
                        if (bit_idx < 8) begin
                            bit_idx <= bit_idx + 4'd1;
                            state   <= M_BIT_WAIT_RISE;
                        end else begin
                            push(mon_s'{kind: KIND_BYTE, repeated: 1'b0,
                                         addr_phase: addr_phase_r,
                                         data: shift_reg, ack: ~ack_bit});
                            // This fall begins the next byte's first bit
                            // (or a STOP/Sr, which the override catches):
                            // capture from here, without waiting to see
                            // SCL low again, so a 1-clock low is enough.
                            addr_phase_r <= 1'b0;
                            bit_idx      <= '0;
                            state        <= M_BIT_WAIT_RISE;
                        end
                    end
                end

                default: state <= M_WATCH;
            endcase
            end
        end
    end

endmodule
