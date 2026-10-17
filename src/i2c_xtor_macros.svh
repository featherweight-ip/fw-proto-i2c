
`ifndef INCLUDED_I2C_XTOR_MACROS_SVH
`define INCLUDED_I2C_XTOR_MACROS_SVH

// ----------------------------------------------------------------------
// Packed request/response struct templates for the signal-level I2C
// transactor cores and interfaces, plus the pin/wire/connect macros for the
// open-drain SCL/SDA pair every kit module exposes.
//
// CONTROLLER internal link: one op at a time (design.md §4's per-byte API
// maps 1:1 onto one op each): START | TX_BYTE | RX_BYTE | STOP.
//   addr/read/repeated are only meaningful for op==OP_START; wr_data only
//   for op==OP_TX_BYTE; ack_in only for op==OP_RX_BYTE.
// addr is the full 10 bits; addr_10b selects 10-bit addressing (UM10204
// §3.1.11), where the core sends the two-byte header itself, and for a
// read also the Sr and the read header -- see i2c_controller_xtor_core.sv.
// ----------------------------------------------------------------------

`define I2C_OP_START    2'd0
`define I2C_OP_TX_BYTE  2'd1
`define I2C_OP_RX_BYTE  2'd2
`define I2C_OP_STOP     2'd3

`define I2C_CTRL_REQ_S \
    struct packed { \
        bit [1:0] op; \
        bit [9:0] addr; \
        bit       addr_10b; \
        bit       read; \
        bit       repeated; \
        bit [7:0] wr_data; \
        bit       ack_in; \
    }

`define I2C_CTRL_RSP_S \
    struct packed { \
        bit       ack; \
        bit       arb_lost; \
        bit [7:0] rd_data; \
    }

// TARGET internal (up-)link: the core is the requester (one bus event at a
// time needs a class-layer decision), the interface/bridge is the responder.
// kind==ADDRESSED: addr/read valid, rsp.ack selects ACK/NACK of the address.
// kind==RX_BYTE: data valid (byte received), rsp.ack is this byte's ACK/NACK.
// kind==TX_NEED_DATA: nothing in the req is valid; rsp.data is the byte to
//   drive next (see i2c_target_bridge.svh for the tx_byte() call pattern).
// kind==ADDR10_HI: the first byte of a 10-bit write header; addr[9:8] holds
//   its A9:A8, rsp.ack selects ACK/NACK of that byte.
// addr_10b qualifies ADDRESSED: addr is a full 10-bit address. It is the
// first (most significant) field so the older fields keep their bit
// positions.
`define I2C_TGT_UPREQ_KIND_ADDRESSED    2'd0
`define I2C_TGT_UPREQ_KIND_RX_BYTE      2'd1
`define I2C_TGT_UPREQ_KIND_TX_NEED_DATA 2'd2
`define I2C_TGT_UPREQ_KIND_ADDR10_HI    2'd3

`define I2C_TGT_UPREQ_S \
    struct packed { \
        bit       addr_10b; \
        bit [1:0] kind; \
        bit [9:0] addr; \
        bit       read; \
        bit [7:0] data; \
    }

`define I2C_TGT_UPRSP_S \
    struct packed { \
        bit       ack; \
        bit [7:0] data; \
    }

// ----------------------------------------------------------------------
// Pin set every role module exposes: the open-drain SCL/SDA pad convention
// (design.md §5) -- _o/_oe/_i per line. A testbench wires the wired-AND
// itself (never inside a transactor): scl = (any instance's scl_oe) ? 0 : 1,
// and likewise for sda, each instance's _i fed back from the resolved net.
// ----------------------------------------------------------------------
`define I2C_ROLE_PORT(PREFIX) \
    output PREFIX``scl_o, \
    output PREFIX``scl_oe, \
    input  PREFIX``scl_i, \
    output PREFIX``sda_o, \
    output PREFIX``sda_oe, \
    input  PREFIX``sda_i

`define I2C_MONITOR_PORT(PREFIX) \
    input PREFIX``scl_i, \
    input PREFIX``sda_i

`endif /* INCLUDED_I2C_XTOR_MACROS_SVH */
