`include "i2c_xtor_macros.svh"

// ----------------------------------------------------------------------------
// I2C Controller transactor interface (SV, signal-level RV ports)
//
// Thin, FIFO-less bridge from the blocking per-byte task API (HVL side) to the
// core's single-outstanding ready/valid op channel. One task per op
// (start/tx_byte/rx_byte/stop), each a request-then-response round trip --
// mirrors wb_initiator_xtor_if's request()/response() split, collapsed to one
// task per call since each op here is already a single class-layer call.
//
// `prescale` is not an op: it is a level the core reads continuously (see
// i2c_controller_xtor_core.sv), held here and changed by set_prescale().
// ----------------------------------------------------------------------------
interface i2c_controller_xtor_if (
        input  wire clock,
        input  wire reset,

        output reg  [15:0] prescale,

        output reg  [23:0] req_dat,
        output reg         req_valid,
        input  wire        req_ready,

        input  wire [9:0]  rsp_dat,
        input  wire        rsp_valid,
        output reg         rsp_ready
    );

    typedef `I2C_CTRL_REQ_S req_s;
    typedef `I2C_CTRL_RSP_S rsp_s;

    initial prescale = 16'd0;

    function automatic void set_prescale(input bit [15:0] p);
        prescale = p;
    endfunction

    task automatic do_op(input req_s r, output rsp_s p);
        req_dat <= r;
        req_valid <= 1'b1;
        do @(posedge clock); while (!(req_valid && req_ready));
        req_valid <= 1'b0;

        rsp_ready <= 1'b1;
        do @(posedge clock); while (!(rsp_valid && rsp_ready));
        rsp_ready <= 1'b0;
        p = rsp_s'(rsp_dat);
    endtask

    task automatic start(
            input  bit [9:0] addr,
            input  bit       addr_10b,
            input  bit       read,
            input  bit       repeated,
            output bit       ack,
            output bit       arb_lost);
        req_s r;
        rsp_s p;
        r = '{op: `I2C_OP_START, addr: addr, addr_10b: addr_10b, read: read,
              repeated: repeated, wr_data: 8'h0, ack_in: 1'b0};
        do_op(r, p);
        ack      = p.ack;
        arb_lost = p.arb_lost;
    endtask

    task automatic tx_byte(
            input  bit [7:0] data,
            output bit       ack,
            output bit       arb_lost);
        req_s r;
        rsp_s p;
        r = '{op: `I2C_OP_TX_BYTE, addr: 10'h0, addr_10b: 1'b0, read: 1'b0,
              repeated: 1'b0, wr_data: data, ack_in: 1'b0};
        do_op(r, p);
        ack      = p.ack;
        arb_lost = p.arb_lost;
    endtask

    task automatic rx_byte(
            input  bit       ack,
            output bit [7:0] data,
            output bit       arb_lost);
        req_s r;
        rsp_s p;
        r = '{op: `I2C_OP_RX_BYTE, addr: 10'h0, addr_10b: 1'b0, read: 1'b1,
              repeated: 1'b0, wr_data: 8'h0, ack_in: ack};
        do_op(r, p);
        data     = p.rd_data;
        arb_lost = p.arb_lost;
    endtask

    task automatic stop();
        req_s r;
        rsp_s p;
        r = '{op: `I2C_OP_STOP, addr: 10'h0, addr_10b: 1'b0, read: 1'b0,
              repeated: 1'b0, wr_data: 8'h0, ack_in: 1'b0};
        do_op(r, p);
    endtask

endinterface
