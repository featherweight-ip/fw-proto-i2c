// Monitor transactor bridge -- the CONSUMER side (mirrors
// wb_monitor_xtor_bridge). Holds a handle to the signal-level transactor
// interface (i2c_monitor_xtor_if) AND a handle to the subscriber that
// implements i2c_monitor_if. start_bridge() forks the run() loop: block on
// the transactor's wait_evt() (one observed bus event), then publish it via
// the non-blocking monitor API.
class i2c_monitor_xtor_bridge;

    virtual i2c_monitor_xtor_if vif;
    i2c_monitor_if              monitor_if;

    function new(virtual i2c_monitor_xtor_if vif, i2c_monitor_if monitor_if);
        this.vif        = vif;
        this.monitor_if = monitor_if;
    endfunction

    task start_bridge();
        fork
            run();
        join_none
    endtask

    task run();
        bit [12:0] e;
        bit [1:0]  kind;
        bit        repeated, addr_phase, ack;
        bit [7:0]  data;
        forever begin
            vif.wait_evt(e);
            // Field order must match i2c_monitor_xtor_if's mon_s packing:
            // {kind[1:0], repeated, addr_phase, data[7:0], ack}.
            {kind, repeated, addr_phase, data, ack} = e;
            unique case (kind)
                2'd0: monitor_if.observe_start(repeated);
                2'd1: monitor_if.observe_stop();
                2'd2: monitor_if.observe_byte(addr_phase, data, ack);
                default: ;
            endcase
        end
    endtask

endclass
