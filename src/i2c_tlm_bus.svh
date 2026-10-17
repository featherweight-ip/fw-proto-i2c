// Transaction-level I2C bus: a single controller connected straight to one
// target model, with any number of passive monitors. It IMPLEMENTS
// i2c_controller_if (the controller side calls it, as it would the
// controller bridge) and CALLS an i2c_target_if and each subscribed
// i2c_monitor_if -- the same three APIs as the signal-level stack, with no
// transactors, pins or clock in between.
//
// What it models is the byte stream, not the controller's request: every
// byte the controller would put on the wire is decoded the way
// i2c_target_xtor_core decodes it, and reported to monitors with the
// addr_phase rule i2c_monitor_xtor_core uses. So a 7-bit start() whose
// address byte is 11110 xx W (software sequencing a 10-bit header one raw
// byte at a time) reaches the target as addressed_10b_hi(), followed by an
// addressed() carrying the full address for the next byte -- exactly what
// the signal-level target does with those bytes. The target-side decode
// state (selected, direction, an ACKed 10-bit write header) lives here and
// follows the core's rules, including that a 10-bit read header with no
// matching ACKed write header is NACKed without asking the model.
//
// Scope: one controller, so arbitration is never lost (arb_lost is always
// 0) and there is no bus-busy wait. A byte nobody ACKs reads back as NACK;
// a read byte from an unselected target reads as 8'hFF (SDA released). The
// caller contract on i2c_controller_if applies unchanged.
//
// Timing: zero time by default. set_timing() with a clock period makes each
// condition and byte consume the time the controller core would take with
// the same T_* parameters (in ticks of prescale+1 clocks), so a TLM run has
// bus-shaped durations. Clock stretching by the target is whatever time the
// model's calls take, as at signal level. The durations are the core's
// nominal phases; they ignore WAIT_FREE and synchronizer latency.
class i2c_tlm_bus implements i2c_controller_if;

    i2c_target_if  target_if;
    i2c_monitor_if monitors[$];

    // Timing (see set_timing()). clk_period 0 means untimed.
    protected time         clk_period;
    protected int unsigned t_low  = 4, t_high   = 4, t_hd_sta = 2;
    protected int unsigned t_su_sta = 2, t_su_sto = 2, t_buf  = 2;
    protected bit [15:0]   prescale;

    // Target-side decode state, after i2c_target_xtor_core.
    typedef enum { PURPOSE_ADDR, PURPOSE_ADDR_LO, PURPOSE_DATA } purpose_e;
    protected purpose_e    purpose;
    protected bit          selected;
    protected bit          dir_read;
    protected bit [9:0]    addr_r;      // address being decoded
    protected bit          addr10_valid;
    protected bit [9:0]    addr10_r;    // last ACKed 10-bit write header

    // Monitor-side addr_phase state, after i2c_monitor_xtor_core.
    protected bit          mon_addr_phase;

    function new(i2c_target_if target_if = null);
        this.target_if = target_if;
    endfunction

    function void add_monitor(i2c_monitor_if m);
        monitors.push_back(m);
    endfunction

    // Make bus events consume time: clk_period is the controller's clock;
    // the T_* values are the controller core's parameters, in ticks.
    function void set_timing(
            time         clk_period,
            int unsigned t_low    = 4,
            int unsigned t_high   = 4,
            int unsigned t_hd_sta = 2,
            int unsigned t_su_sta = 2,
            int unsigned t_su_sto = 2,
            int unsigned t_buf    = 2);
        this.clk_period = clk_period;
        this.t_low      = t_low;
        this.t_high     = t_high;
        this.t_hd_sta   = t_hd_sta;
        this.t_su_sta   = t_su_sta;
        this.t_su_sto   = t_su_sto;
        this.t_buf      = t_buf;
    endfunction

    // ---- i2c_controller_if ------------------------------------------------

    virtual task start(
            input  bit [9:0] addr,
            input  bit       addr_10b,
            input  bit       read,
            input  bit       repeated,
            output bit       ack,
            output bit       arb_lost);
        arb_lost = 1'b0;
        bus_start(repeated);
        if (!addr_10b) begin
            bus_write({addr[6:0], read}, ack);
        end else begin
            // The whole header, each byte only if the one before was ACKed
            // (i2c_controller_if::start()).
            bus_write({5'b11110, addr[9:8], 1'b0}, ack);
            if (ack) bus_write(addr[7:0], ack);
            if (ack && read) begin
                bus_start(1'b1);
                bus_write({5'b11110, addr[9:8], 1'b1}, ack);
            end
        end
    endtask

    virtual task tx_byte(
            input  bit [7:0] data,
            output bit       ack,
            output bit       arb_lost);
        arb_lost = 1'b0;
        bus_write(data, ack);
    endtask

    virtual task rx_byte(
            input  bit       ack,
            output bit [7:0] data,
            output bit       arb_lost);
        arb_lost = 1'b0;
        bus_read(ack, data);
    endtask

    virtual function void set_prescale(bit [15:0] prescale);
        this.prescale = prescale;
    endfunction

    virtual task stop();
        consume(t_low + t_su_sto + t_buf);
        selected     = 1'b0;
        addr10_valid = 1'b0;
        foreach (monitors[i]) monitors[i].observe_stop();
    endtask

    // ---- bus events ------------------------------------------------------

    // START or Sr: the target and monitor both restart address reception.
    protected task bus_start(bit repeated);
        consume((repeated ? t_low : 0) + t_su_sta + t_hd_sta);
        purpose        = PURPOSE_ADDR;
        mon_addr_phase = 1'b1;
        foreach (monitors[i]) monitors[i].observe_start(repeated);
    endtask

    // One byte from the controller, and the ACK bit after it.
    protected task bus_write(input bit [7:0] b, output bit ack);
        bit addr_phase = mon_addr_phase;
        bit addr_first = (purpose == PURPOSE_ADDR);
        consume(8 * (t_low + t_high));
        ack = 1'b0;
        case (purpose)
            PURPOSE_ADDR: begin
                if (b[7:3] != 5'b11110) begin
                    // 7-bit address
                    addr10_valid = 1'b0;
                    addressed({3'b0, b[7:1]}, 1'b0, b[0], ack);
                end else if (!b[0]) begin
                    // 10-bit write header, first byte
                    addr10_valid = 1'b0;
                    selected     = 1'b0;
                    if (target_if != null) target_if.addressed_10b_hi(b[2:1], ack);
                    addr_r  = {b[2:1], 8'h00};
                    purpose = ack ? PURPOSE_ADDR_LO : PURPOSE_DATA;
                end else if (addr10_valid && addr10_r[9:8] == b[2:1]) begin
                    // 10-bit read header for the remembered address
                    addressed(addr10_r, 1'b1, 1'b1, ack);
                end else begin
                    // 10-bit read header, not for us: NACK without asking
                    addr10_valid = 1'b0;
                    selected     = 1'b0;
                    purpose      = PURPOSE_DATA;
                end
            end
            PURPOSE_ADDR_LO: begin
                addressed({addr_r[9:8], b}, 1'b1, 1'b0, ack);
            end
            default: begin
                if (selected && !dir_read && target_if != null)
                    target_if.rx_byte(b, ack);
            end
        endcase
        consume(t_low + t_high);
        // The monitor flags the byte after an ACKed 10-bit write header's
        // first byte as an address byte too.
        mon_addr_phase = addr_first && ack && b[7:3] == 5'b11110 && !b[0];
        foreach (monitors[i]) monitors[i].observe_byte(addr_phase, b, ack);
    endtask

    // One byte from the target, and the controller's ACK/NACK after it.
    protected task bus_read(input bit ack, output bit [7:0] data);
        data = 8'hFF;
        if (selected && dir_read && target_if != null)
            target_if.tx_byte(data, 1'b1);
        consume(9 * (t_low + t_high));
        // A NACK ends the read: the target stops driving and waits for a
        // STOP or Sr.
        if (!ack) selected = 1'b0;
        mon_addr_phase = 1'b0;
        foreach (monitors[i]) monitors[i].observe_byte(1'b0, data, ack);
    endtask

    // An address phase decision: the model's ACK selects the target.
    protected task addressed(bit [9:0] addr, bit addr_10b, bit read, output bit ack);
        ack = 1'b0;
        if (target_if != null) target_if.addressed(addr, addr_10b, read, ack);
        selected = ack;
        dir_read = read;
        purpose  = PURPOSE_DATA;
        if (addr_10b && !read) begin
            addr10_valid = ack;
            addr10_r     = addr;
        end
    endtask

    protected task consume(int unsigned ticks);
        // verilator lint_off ZERODLY
        if (clk_period != 0)
            #(clk_period * ticks * (prescale + 1));
        // verilator lint_on ZERODLY
    endtask

endclass
