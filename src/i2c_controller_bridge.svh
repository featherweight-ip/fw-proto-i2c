// Controller transactor bridge -- the PROVIDER side. Holds a handle to the
// signal-level transactor interface (i2c_controller_xtor_if) and IMPLEMENTS
// i2c_controller_if directly (mirrors wb_initiator_xtor_bridge; this kit
// does not use the fw-hdl fw_export/`FW_*_IMP macro convention -- see
// fw_proto_i2c_pkg.sv header).
class i2c_controller_xtor_bridge implements i2c_controller_if;

    virtual i2c_controller_xtor_if vif;

    function new(virtual i2c_controller_xtor_if vif);
        this.vif = vif;
    endfunction

    // Call-driven (every method does request-then-response inline); nothing
    // to start in the background.
    task start_bridge();
    endtask

    virtual task start(
            input  bit [9:0] addr,
            input  bit       addr_10b,
            input  bit       read,
            input  bit       repeated,
            output bit       ack,
            output bit       arb_lost);
        vif.start(addr, addr_10b, read, repeated, ack, arb_lost);
    endtask

    virtual task tx_byte(
            input  bit [7:0] data,
            output bit       ack,
            output bit       arb_lost);
        vif.tx_byte(data, ack, arb_lost);
    endtask

    virtual task rx_byte(
            input  bit       ack,
            output bit [7:0] data,
            output bit       arb_lost);
        vif.rx_byte(ack, data, arb_lost);
    endtask

    virtual task stop();
        vif.stop();
    endtask

    virtual function void set_prescale(bit [15:0] prescale);
        vif.set_prescale(prescale);
    endfunction

endclass
