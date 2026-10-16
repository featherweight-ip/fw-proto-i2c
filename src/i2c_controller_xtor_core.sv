`include "i2c_xtor_macros.svh"

// ----------------------------------------------------------------------------
// I2C Controller core transactor (pure module, signal-level open-drain ports)
//
// Clocked FSM, one op (START/TX_BYTE/RX_BYTE/STOP) at a time from the
// internal link, each op executed as a sequence of real SCL/SDA transitions
// on the bus (design.md §5). Both START's address+R/W byte and TX_BYTE's
// data byte are driven through the SAME 9-bit (8 payload + 1 ack) exchange
// engine (`BIT_LOW`/`BIT_WAIT_HI`/`BIT_HOLD_HI`); RX_BYTE runs the identical
// engine with the payload direction reversed (this core samples instead of
// drives) and the ack bit driven by this core instead of sampled.
//
// Arbitration (design.md §5 "ARB_CHECK"): on every bit this core drives
// (the payload bits of START/TX_BYTE, and the ack bit of RX_BYTE), once SCL
// is sampled high this core compares what it intended to drive against what
// it reads on SDA. Intending '1' (released) but reading '0' means another
// master is driving '0' at the same time -- arbitration lost; the op aborts
// immediately (ARB_ABORT: release both lines) and reports arb_lost=1 (the
// other response field is then undefined, per i2c_controller_if.svh).
//
// Clock stretching / multi-master sync (design.md §5 "CLK_STRETCH_WAIT"):
// after this core releases SCL (scl_oe<=0) to begin a bit's high phase, it
// does not assume SCL is actually high -- it waits in BIT_WAIT_HI for scl_i
// to read 1 before counting the high-phase hold. A target holding SCL low
// to stretch, or another (slower) master still finishing its own low
// period, are the same wait from this core's point of view.
//
// Between-ops bus discipline: after a 9-bit exchange's ack bit, SCL is
// simply left released (high) -- this core does NOT insert an artificial
// low pulse while idle between ops (a REAL master doesn't either). A
// target/monitor core watching for "SCL fell = another bit begins" must
// see a REAL fall, which only happens once the NEXT op actually starts
// (BIT_LOW, a genuine next-bit drop) -- so there is nothing for it to
// misinterpret during the gap while this core is simply waiting for its
// next class-layer call. (An earlier version of this core DID drop SCL
// unconditionally after every ack bit -- "PARK" -- so a target would know
// to release its own ack-bit drive; that pulse turned out to be
// indistinguishable, from the target's side, from a real next bit
// starting, causing it to mis-count by one bit per op boundary. Fixed by
// removing the pulse: the target now simply holds its ack-bit drive,
// unchanged, across the whole gap -- see i2c_target_xtor_core.sv's
// DRIVE_ACK_WAIT_FALL header note.) A repeated START still deliberately
// drops SCL once first (REP_START_DROP) before the Sr edge -- that drop
// is real (this core is actively beginning the next transaction), not a
// filler pulse, so the same reasoning does not apply to it; see that
// state's own comment and i2c_target_xtor_core.sv's matching note on the
// one gap this still leaves (repeated-START, not exercised by this kit's
// required tests).
//
// Timing: T_LOW/T_HIGH/T_HD_STA/T_SU_STA/T_SU_STO/T_BUF are module
// PARAMETERS (clock-cycle counts), not the runtime cfg_* ports design.md §5
// sketches -- a deliberate scope reduction for v1 (documented in the kit
// implementation report); one core instance covers one fixed timing profile.
//
// NOT IMPLEMENTED in v1 (design.md §8, explicitly deferred there):
//   - 10-bit addressing (addr_10b is accepted on the link but ignored --
//     only addr[6:0] is ever driven).
//   - Hs-mode / Ultra-fast mode.
// ----------------------------------------------------------------------------
module i2c_controller_xtor_core #(
        parameter int T_LOW     = 4,
        parameter int T_HIGH    = 4,
        parameter int T_HD_STA  = 2,
        parameter int T_SU_STA  = 2,
        parameter int T_SU_STO  = 2,
        parameter int T_BUF     = 2
    ) (
        input  wire        clock,
        input  wire        reset,

        output reg         scl_o,
        output reg         scl_oe,
        input  wire        scl_i,
        output reg         sda_o,
        output reg         sda_oe,
        input  wire        sda_i,

        input  wire [23:0] req_dat,
        input  wire        req_valid,
        output wire        req_ready,

        output reg  [9:0]  rsp_dat,
        output reg         rsp_valid,
        input  wire        rsp_ready
    );

    typedef `I2C_CTRL_REQ_S req_s;
    typedef `I2C_CTRL_RSP_S rsp_s;

    req_s req_u;
    always_comb req_u = req_s'(req_dat);

    typedef enum logic [3:0] {
        IDLE, WAIT_FREE,
        REP_START_DROP, START_HOLD_HI, START_EDGE,
        BIT_LOW, BIT_WAIT_HI, BIT_HOLD_HI,
        STOP_EDGE_LOW, STOP_WAIT_HI, STOP_HOLD_HI, STOP_RELEASE,
        ARB_ABORT,
        DONE
    } state_e;
    state_e state;

    // Latched op fields
    reg       ack_in_r;

    // Byte engine state
    reg [3:0] bit_idx;        // 0..8 (8 = ack bit)
    reg [7:0] byte_reg;       // driven payload byte (START's addr+rw, or TX data)
    reg [7:0] rx_reg;         // captured payload byte (RX_BYTE)
    reg       payload_is_tx;  // this core drives the 8 payload bits
    reg       ack_result;     // sampled target ack (START/TX_BYTE)
    reg       arb_lost_r;

    reg [7:0] tick_cnt;

    wire ack_is_tx = !payload_is_tx;

    // What this core intends to drive on the CURRENT bit (bit_idx), and
    // whether it is this core's turn to drive at all. Stable across
    // BIT_LOW/BIT_WAIT_HI/BIT_HOLD_HI (depends only on registers that don't
    // change until the bit completes) -- and, deliberately, also still
    // valid once the exchange reaches DONE/IDLE (see the header note): it
    // keeps presenting the LAST bit's drive so SDA does not change while
    // SCL is left high between ops.
    wire payload_bit  = byte_reg[7 - (bit_idx > 7 ? 7 : bit_idx)];
    wire driving_now  = (bit_idx < 8) ? payload_is_tx : ack_is_tx;
    wire intended_bit = (bit_idx < 8) ? payload_bit : ~ack_in_r;

    wire cur_bit_sda_oe = driving_now && (intended_bit == 1'b0);

    assign req_ready = (state == IDLE);

    always_comb begin
        scl_o  = 1'b0;
        sda_o  = 1'b0;
        scl_oe = 1'b0;
        sda_oe = 1'b0;
        unique case (state)
            // Between ops (see the header note): SDA keeps presenting the
            // LAST bit's drive. This matters when THIS core drove the ack
            // bit itself (RX_BYTE, ack_is_tx=1) -- if it released early,
            // SDA would float high while SCL is still high (since nothing
            // else is driving it either), which is electrically a STOP
            // condition. Releasing only happens once a real next bit
            // (BIT_LOW) or an explicit STOP actually changes it.
            IDLE:            begin scl_oe = 1'b0; sda_oe = cur_bit_sda_oe; end
            DONE:            begin scl_oe = 1'b0; sda_oe = cur_bit_sda_oe; end
            WAIT_FREE:       begin scl_oe = 1'b0; sda_oe = 1'b0; end
            REP_START_DROP:  begin scl_oe = 1'b1; sda_oe = 1'b0; end
            START_HOLD_HI:   begin scl_oe = 1'b0; sda_oe = 1'b0; end
            START_EDGE:      begin scl_oe = 1'b0; sda_oe = 1'b1; end
            BIT_LOW:         begin scl_oe = 1'b1; sda_oe = cur_bit_sda_oe; end
            BIT_WAIT_HI:     begin scl_oe = 1'b0; sda_oe = cur_bit_sda_oe; end
            BIT_HOLD_HI:     begin scl_oe = 1'b0; sda_oe = cur_bit_sda_oe; end
            STOP_EDGE_LOW:   begin scl_oe = 1'b1; sda_oe = 1'b1; end
            STOP_WAIT_HI:    begin scl_oe = 1'b0; sda_oe = 1'b1; end
            STOP_HOLD_HI:    begin scl_oe = 1'b0; sda_oe = 1'b1; end
            STOP_RELEASE:    begin scl_oe = 1'b0; sda_oe = 1'b0; end
            ARB_ABORT:       begin scl_oe = 1'b0; sda_oe = 1'b0; end
            default:         begin scl_oe = 1'b0; sda_oe = 1'b0; end
        endcase
    end

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            state       <= IDLE;
            rsp_valid   <= 1'b0;
            rsp_dat     <= '0;
            bit_idx     <= '0;
            arb_lost_r  <= 1'b0;
            tick_cnt    <= '0;
            // IDLE presents the last bit's SDA drive (cur_bit_sda_oe), so
            // these must reset to "not driving": left free, the core could
            // leave reset holding SDA low -- a spurious START, then a STOP
            // when it let go (found by fv-cover).
            payload_is_tx <= 1'b0;
            byte_reg      <= 8'hff;
            ack_in_r      <= 1'b0;
        end else begin
            unique case (state)
                IDLE: begin
                    rsp_valid <= 1'b0;
                    if (req_valid) begin
                        ack_in_r      <= req_u.ack_in;
                        bit_idx       <= '0;
                        arb_lost_r    <= 1'b0;
                        rx_reg        <= '0;
                        unique case (req_u.op)
                            `I2C_OP_START: begin
                                byte_reg      <= {req_u.addr[6:0], req_u.read};
                                payload_is_tx <= 1'b1;
                                if (req_u.repeated) begin
                                    state    <= REP_START_DROP;
                                    tick_cnt <= 8'(T_LOW - 1);
                                end else
                                    state <= WAIT_FREE;
                            end
                            `I2C_OP_TX_BYTE: begin
                                byte_reg      <= req_u.wr_data;
                                payload_is_tx <= 1'b1;
                                state <= BIT_LOW;
                                tick_cnt <= 8'(T_LOW - 1);
                            end
                            `I2C_OP_RX_BYTE: begin
                                payload_is_tx <= 1'b0;
                                state <= BIT_LOW;
                                tick_cnt <= 8'(T_LOW - 1);
                            end
                            `I2C_OP_STOP: begin
                                // DONE/IDLE after the STOP present
                                // cur_bit_sda_oe, and bit_idx is now 0:
                                // left as is, the last write's byte_reg[7]
                                // would pull SDA low after the STOP (a
                                // spurious START, then a STOP).
                                payload_is_tx <= 1'b0;
                                ack_in_r      <= 1'b0;
                                state    <= STOP_EDGE_LOW;
                                tick_cnt <= 8'(T_HD_STA - 1);
                            end
                            default: state <= IDLE;
                        endcase
                    end
                end

                // ---- START / repeated-START preamble --------------------
                WAIT_FREE: begin
                    if (scl_i && sda_i) state <= START_HOLD_HI;
                end
                // A repeated start must look electrically different from a
                // STOP-then-START: drop SCL first (SDA is free to float to
                // 1 here with no protocol meaning, since SCL is low), THEN
                // let SCL rise and pull SDA low for the real Sr edge below.
                REP_START_DROP: begin
                    if (tick_cnt == 0) state <= START_HOLD_HI;
                    else tick_cnt <= tick_cnt - 1;
                end
                START_HOLD_HI: begin
                    // Also serves as the "wait for SCL actually high" step
                    // (honors a stretch/sync wait the same as any other
                    // high phase).
                    if (scl_i) begin
                        state    <= START_EDGE;
                        tick_cnt <= 8'(T_SU_STA - 1);
                    end
                end
                START_EDGE: begin
                    if (tick_cnt == 0) begin
                        state    <= BIT_LOW;
                        tick_cnt <= 8'(T_HD_STA - 1);
                    end else tick_cnt <= tick_cnt - 1;
                end

                // ---- generic 9-bit (8 payload + 1 ack) exchange ---------
                BIT_LOW: begin
                    if (tick_cnt == 0) begin
                        state    <= BIT_WAIT_HI;
                    end else tick_cnt <= tick_cnt - 1;
                end
                BIT_WAIT_HI: begin
                    if (scl_i) begin
                        // Sample at the moment SCL first reads high.
                        if (driving_now && intended_bit && !sda_i) begin
                            arb_lost_r <= 1'b1;
                        end else begin
                            if (bit_idx < 8 && !payload_is_tx)
                                rx_reg[7 - bit_idx] <= sda_i;
                            if (bit_idx == 8 && !ack_is_tx) begin
                                ack_result <= ~sda_i;
                            end
                        end
                        state    <= BIT_HOLD_HI;
                        tick_cnt <= 8'(T_HIGH - 1);
                    end
                end
                BIT_HOLD_HI: begin
                    if (tick_cnt == 0) begin
                        if (arb_lost_r) begin
                            state <= ARB_ABORT;
                        end else if (bit_idx < 8) begin
                            bit_idx  <= bit_idx + 4'd1;
                            state    <= BIT_LOW;
                            tick_cnt <= 8'(T_LOW - 1);
                        end else begin
                            // 9th (ack) bit done: report the result. SCL is
                            // simply left released (high) from here -- see
                            // the header note -- the NEXT op (accepted from
                            // IDLE/DONE) decides what happens next.
                            op_result_latch();
                            state     <= DONE;
                            rsp_valid <= 1'b1;
                        end
                    end else tick_cnt <= tick_cnt - 1;
                end

                ARB_ABORT: begin
                    op_result_latch();
                    state     <= DONE;
                    rsp_valid <= 1'b1;
                end

                // ---- STOP -----------------------------------------------
                STOP_EDGE_LOW: begin
                    if (tick_cnt == 0) state <= STOP_WAIT_HI;
                    else tick_cnt <= tick_cnt - 1;
                end
                STOP_WAIT_HI: begin
                    if (scl_i) begin
                        state    <= STOP_HOLD_HI;
                        tick_cnt <= 8'(T_SU_STO - 1);
                    end
                end
                STOP_HOLD_HI: begin
                    if (tick_cnt == 0) begin
                        state    <= STOP_RELEASE;
                        tick_cnt <= 8'(T_BUF - 1);
                    end else tick_cnt <= tick_cnt - 1;
                end
                STOP_RELEASE: begin
                    if (tick_cnt == 0) begin
                        rsp_dat   <= rsp_s'{ack: 1'b0, arb_lost: 1'b0, rd_data: 8'h0};
                        rsp_valid <= 1'b1;
                        state     <= DONE;
                    end else tick_cnt <= tick_cnt - 1;
                end

                DONE: begin
                    if (rsp_valid && rsp_ready) begin
                        rsp_valid <= 1'b0;
                        state     <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    // Build the response for a completed START/TX_BYTE/RX_BYTE op
    // (BIT_HOLD_HI and ARB_ABORT both funnel here). Implemented as a task
    // (not a function) purely for readability; it only assigns registers
    // already in this always block, so it stays synthesizable.
    task automatic op_result_latch();
        rsp_dat <= rsp_s'{
            ack:      ack_result,
            arb_lost: arb_lost_r,
            rd_data:  rx_reg
        };
    endtask

endmodule
