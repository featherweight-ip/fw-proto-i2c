`include "i2c_xtor_macros.svh"

// ----------------------------------------------------------------------------
// I2C Target transactor interface (SV, signal-level RV ports)
//
// Mirror of i2c_controller_xtor_if, but the CORE is the requester (one bus
// event -- address phase, a received byte, or "need a byte to send" -- needs
// a class-layer decision) and this interface is the responder. wait_req()
// blocks for the next event; send_rsp() delivers the model's decision. The
// core holds SCL low (clock-stretch) for as long as this round trip takes.
// ----------------------------------------------------------------------------
interface i2c_target_xtor_if (
        input  wire clock,
        input  wire reset,

        input  wire [21:0] req_dat,
        input  wire        req_valid,
        output reg         req_ready,

        output reg  [8:0]  rsp_dat,
        output reg         rsp_valid,
        input  wire        rsp_ready
    );

    // Raw packed vectors (not the `req_s`/`rsp_s` named types below) on the
    // task ports: keeps the actual-argument type a plain bit vector for any
    // caller outside this interface (the target bridge), which packs/unpacks
    // via the same `I2C_TGT_UPREQ_S/`I2C_TGT_UPRSP_S macros on its own side.
    typedef `I2C_TGT_UPREQ_S req_s;
    typedef `I2C_TGT_UPRSP_S rsp_s;

    initial req_ready = 1'b0;
    initial rsp_valid = 1'b0;

    task automatic wait_req(output bit [21:0] r);
        req_ready <= 1'b1;
        do @(posedge clock); while (!(req_valid && req_ready));
        r = req_dat;
        req_ready <= 1'b0;
    endtask

    task automatic send_rsp(input bit [8:0] p);
        rsp_dat   <= p;
        rsp_valid <= 1'b1;
        do @(posedge clock); while (!(rsp_valid && rsp_ready));
        rsp_valid <= 1'b0;
    endtask

endinterface
