// Target transactor bridge -- the CONSUMER side (mirrors
// wb_target_xtor_bridge). Holds a handle to the signal-level transactor
// interface (i2c_target_xtor_if) AND a handle to the model that implements
// i2c_target_if. start_bridge() forks the run() service loop: for each
// captured up-link event, ask the model for its decision, then drive that
// decision back to the core (which may have been holding SCL low the whole
// time -- the clock-stretch wait is entirely on the core side; this loop
// just runs as fast as the model answers).
class i2c_target_xtor_bridge;

    virtual i2c_target_xtor_if vif;
    i2c_target_if              target_if;

    function new(virtual i2c_target_xtor_if vif, i2c_target_if target_if);
        this.vif       = vif;
        this.target_if = target_if;
    endfunction

    task start_bridge();
        fork
            run();
        join_none
    endtask

    task run();
        bit [21:0] req_v;
        bit [8:0]  rsp_v;
        bit [1:0]  kind;
        bit [9:0]  addr;
        bit        addr_10b;
        bit        read;
        bit [7:0]  req_data, data;
        bit        ack;
        forever begin
            vif.wait_req(req_v);
            // Field order must match i2c_xtor_macros.svh's
            // `I2C_TGT_UPREQ_S: {addr_10b, kind[1:0], addr[9:0], read,
            // data[7:0]}.
            {addr_10b, kind, addr, read, req_data} = req_v;
            unique case (kind)
                `I2C_TGT_UPREQ_KIND_ADDRESSED: begin
                    target_if.addressed(addr, addr_10b, read, ack);
                    data = 8'h0;
                end
                `I2C_TGT_UPREQ_KIND_ADDR10_HI: begin
                    target_if.addressed_10b_hi(addr[9:8], ack);
                    data = 8'h0;
                end
                `I2C_TGT_UPREQ_KIND_RX_BYTE: begin
                    target_if.rx_byte(req_data, ack);
                    data = 8'h0;
                end
                `I2C_TGT_UPREQ_KIND_TX_NEED_DATA: begin
                    // ack passed in is always 1 here (see i2c_target_if.svh):
                    // the core only raises TX_NEED_DATA again after sampling
                    // an ACK (continue) on the previous byte.
                    target_if.tx_byte(data, 1'b1);
                    ack = 1'b0;
                end
                default: begin ack = 1'b0; data = 8'h0; end
            endcase
            // Field order must match `I2C_TGT_UPRSP_S: {ack, data[7:0]}.
            rsp_v = {ack, data};
            vif.send_rsp(rsp_v);
        end
    endtask

endclass
