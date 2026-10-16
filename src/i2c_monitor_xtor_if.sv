`include "i2c_xtor_macros.svh"

// ----------------------------------------------------------------------------
// I2C Monitor transactor interface (SV, signal-level RV ports)
//
// FIFO-less, like wb_monitor_xtor_if: the core holds one observed event as a
// pipeline register; wait_evt() keeps mon_ready asserted so it consumes one
// event per cycle (a passive monitor cannot backpressure the bus -- see
// i2c_monitor_xtor_core.sv for why this is sized to keep up at this kit's
// bit rate).
// ----------------------------------------------------------------------------
interface i2c_monitor_xtor_if (
        input  wire clock,
        input  wire reset,

        input  wire [12:0] mon_dat,
        input  wire        mon_valid,
        output reg         mon_ready
    );

    // kind: 0=START(repeated valid) 1=STOP 2=BYTE(addr_phase,data,ack valid)
    typedef struct packed {
        bit [1:0] kind;
        bit       repeated;
        bit       addr_phase;
        bit [7:0] data;
        bit       ack;
    } mon_s;

    initial mon_ready = 1'b0;

    // Raw packed vector out (not `mon_s`): keeps the task's actual-argument
    // type a plain bit vector, so a caller outside this interface (the
    // monitor bridge) never has to bind to this interface-local struct type.
    task automatic wait_evt(output bit [12:0] r);
        mon_ready <= 1'b1;
        do @(posedge clock); while (!(mon_valid && mon_ready));
        r = mon_dat;
        mon_ready <= 1'b0;
    endtask

endinterface
