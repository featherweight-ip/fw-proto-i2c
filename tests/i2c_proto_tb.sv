// ======================================================================
// REQUIRED back-to-back SIMULATION test for the I2C kit (design.md §8
// issue 1, first of the two required back-to-back sim tests for this kit).
//
// Wires a full CONTROLLER transactor directly to a full TARGET transactor
// over one shared open-drain SCL/SDA bus (plus a MONITOR passively tapping
// it), exercising the complete stack (class API -> bridge -> xtor_if ->
// core -> pins and back). Drives a mixed write/read sequence from a
// component over the controller API against a byte-addressable memory model
// behind the target API, checking true round-trip data integrity (a read
// returns a previously written value), clock stretching (the memory model
// adds simulated latency before answering, forcing the target core to hold
// SCL low), and both ACK and NACK paths.
//
// The controller's timing parameters are this module's, so a wrapper top
// can rerun the same case at another bus speed (i2c_proto_fast_tb.sv: 1
// clock per phase, the target core's minimum).
//
// Run:  dfm run i2c-proto    (expect: [i2c_proto] PASS; any failure exits
//                              nonzero)
// ======================================================================
module i2c_proto_tb #(
        parameter int T_LOW     = 4,
        parameter int T_HIGH    = 4,
        parameter int T_HD_STA  = 2,
        parameter int T_SU_STA  = 2,
        parameter int T_SU_STO  = 2,
        parameter int T_BUF     = 2
    );
    import fw_proto_i2c_pkg::*;

    localparam bit [6:0] TGT_ADDR = 7'h50;

    // --------------------------------------------------------------
    // Driver: holds the controller API handle; runs the stimulus and
    // checks round-trip integrity inline.
    // --------------------------------------------------------------
    class driver;
        i2c_controller_if ctl;
        int unsigned errors;
        int unsigned n_bytes;

        function new(i2c_controller_if ctl);
            this.ctl = ctl;
        endfunction

        task automatic write_bytes(input bit [6:0] addr, input bit [7:0] data[]);
            bit ack, arb_lost;
            ctl.start(addr, 1'b0, 1'b0, 1'b0, ack, arb_lost);
            if (arb_lost || !ack) begin
                $display("FAIL: write addr 0x%02h not ACKed (arb_lost=%0d ack=%0d)",
                          addr, arb_lost, ack);
                errors++;
                ctl.stop();
                return;
            end
            foreach (data[i]) begin
                ctl.tx_byte(data[i], ack, arb_lost);
                n_bytes++;
                if (arb_lost || !ack) begin
                    $display("FAIL: write byte[%0d]=0x%02h not ACKed (arb_lost=%0d ack=%0d)",
                              i, data[i], arb_lost, ack);
                    errors++;
                end
            end
            ctl.stop();
        endtask

        task automatic read_bytes(input bit [6:0] addr, input int n, output bit [7:0] data[]);
            bit ack, arb_lost;
            data = new[n];
            ctl.start(addr, 1'b0, 1'b1, 1'b0, ack, arb_lost);
            if (arb_lost || !ack) begin
                $display("FAIL: read addr 0x%02h not ACKed (arb_lost=%0d ack=%0d)",
                          addr, arb_lost, ack);
                errors++;
                ctl.stop();
                return;
            end
            for (int i = 0; i < n; i++) begin
                bit [7:0] b;
                ctl.rx_byte(i < n-1, b, arb_lost);
                data[i] = b;
                n_bytes++;
                if (arb_lost) begin
                    $display("FAIL: read byte[%0d] arb_lost unexpectedly", i);
                    errors++;
                end
            end
            ctl.stop();
        endtask

        task run();
            bit [7:0] wr[];
            bit [7:0] rd[];
            errors = 0; n_bytes = 0;

            // Read before anything is written: checks the read path on its
            // own, so a broken write cannot hide it. Unwritten bytes have
            // MSB 0 (mem_model::init_byte), so a lost bit 7 shows: the
            // controller would read the released line as 1.
            read_bytes(TGT_ADDR, 4, rd);
            foreach (rd[i]) begin
                if (rd[i] !== mem_model::init_byte(8'(i))) begin
                    $display("FAIL: unwritten[%0d] got 0x%02h exp 0x%02h",
                              i, rd[i], mem_model::init_byte(8'(i)));
                    errors++;
                end
            end
            #200ns;

            // Plain write, then read back and check round-trip integrity.
            wr = new[4];
            wr[0] = 8'hA0; wr[1] = 8'hA1; wr[2] = 8'hA2; wr[3] = 8'hA3;
            write_bytes(TGT_ADDR, wr);
            #200ns;
            read_bytes(TGT_ADDR, 4, rd);
            foreach (wr[i]) begin
                if (rd[i] !== wr[i]) begin
                    $display("FAIL: read-back[%0d] got 0x%02h exp 0x%02h", i, rd[i], wr[i]);
                    errors++;
                end else
                    $display("[driver] read-back[%0d] = 0x%02h OK", i, rd[i]);
            end

            // Second independent write/read pair (exercises the memory
            // model's clock-stretch latency repeatedly, and a different
            // byte count / data pattern).
            wr = new[2];
            // The last byte's MSB is 0 on purpose: the controller once
            // drove byte_reg[7] onto SDA after a STOP, a spurious
            // START/STOP the STOP count below catches.
            wr[0] = 8'h5A; wr[1] = 8'h3C;
            write_bytes(TGT_ADDR, wr);
            #200ns;
            read_bytes(TGT_ADDR, 2, rd);
            foreach (wr[i]) begin
                if (rd[i] !== wr[i]) begin
                    $display("FAIL: read-back2[%0d] got 0x%02h exp 0x%02h", i, rd[i], wr[i]);
                    errors++;
                end
            end

            // A write to an address the target model NACKs (exercise the
            // NACK path end to end).
            begin
                bit ack, arb_lost;
                ctl.start(7'h7F, 1'b0, 1'b0, 1'b0, ack, arb_lost);
                if (ack || arb_lost) begin
                    $display("FAIL: expected address 0x7F to be NACKed");
                    errors++;
                end else
                    $display("[driver] address 0x7F correctly NACKed");
                ctl.stop();
            end
        endtask
    endclass

    // --------------------------------------------------------------
    // Target model: implements i2c_target_if directly. Byte-addressable
    // memory with an implicit auto-incrementing pointer (classic EEPROM-ish
    // model): the FIRST write byte after an address phase sets the pointer,
    // subsequent bytes write-and-increment; reads return from the pointer
    // and increment. Adds simulated latency before every decision to force
    // the target core to clock-stretch.
    // --------------------------------------------------------------
    class mem_model implements i2c_target_if;
        bit [7:0] mem [bit [7:0]];
        bit [7:0] ptr;

        // What an unwritten byte reads as: MSB 0 for every offset used here.
        static function bit [7:0] init_byte(bit [7:0] off);
            return off ^ 8'h5A;
        endfunction

        virtual task addressed(input bit [9:0] addr, input bit addr_10b,
                                input bit read, output bit ack);
            #23ns; // clock-stretch: model latency before the ACK decision
            if (addr[6:0] == TGT_ADDR) begin
                ack = 1'b1;
                // This test has no separate "register pointer" byte
                // convention -- every addressed() call (read or write)
                // simply (re)starts at a fixed base, so a write-then-read
                // pair against the same address round-trips directly.
                ptr = 8'h00;
            end else
                ack = 1'b0;
        endtask

        virtual task rx_byte(input bit [7:0] data, output bit ack);
            #17ns; // clock-stretch
            ack      = 1'b1;
            mem[ptr] = data;
            ptr      = ptr + 8'd1;
        endtask

        virtual task tx_byte(output bit [7:0] data, input bit ack);
            #13ns; // clock-stretch
            data = mem.exists(ptr) ? mem[ptr] : init_byte(ptr);
            ptr  = ptr + 8'd1;
        endtask
    endclass

    // --------------------------------------------------------------
    // Observer: implements i2c_monitor_if directly. Counts every observed
    // byte-with-ack so the test can assert the monitor's stream matches
    // what the driver actually issued.
    // --------------------------------------------------------------
    class observer implements i2c_monitor_if;
        int unsigned n_bytes_seen;
        int unsigned n_starts;
        int unsigned n_stops;

        virtual function void observe_start(input bit repeated);
            n_starts++;
        endfunction
        virtual function void observe_stop();
            n_stops++;
        endfunction
        virtual function void observe_byte(input bit addr_phase, input bit [7:0] data,
                                            input bit ack);
            n_bytes_seen++;
        endfunction
        virtual function void observe_arb_lost();
        endfunction
    endclass

    // --------------------------------------------------------------
    // Signal-level setup: controller + target + monitor transactors on one
    // shared open-drain bus. Wired-AND is modeled explicitly here (never
    // inside a transactor, per design.md §5): each instance's _oe pulls the
    // shared net low; the net is released (1) only when nobody pulls it low.
    // --------------------------------------------------------------
    logic clock = 1'b0;
    logic reset = 1'b1;
    always #5ns clock = ~clock;

    wire ctl_scl_o, ctl_scl_oe, ctl_sda_o, ctl_sda_oe;
    wire tgt_scl_o, tgt_scl_oe, tgt_sda_o, tgt_sda_oe;

    wire scl = (ctl_scl_oe || tgt_scl_oe) ? 1'b0 : 1'b1;
    wire sda = (ctl_sda_oe || tgt_sda_oe) ? 1'b0 : 1'b1;

    i2c_controller_xtor #(
        .T_LOW(T_LOW), .T_HIGH(T_HIGH), .T_HD_STA(T_HD_STA),
        .T_SU_STA(T_SU_STA), .T_SU_STO(T_SU_STO), .T_BUF(T_BUF)
    ) ctl_xtor (
        .clock(clock), .reset(reset),
        .scl_o(ctl_scl_o), .scl_oe(ctl_scl_oe), .scl_i(scl),
        .sda_o(ctl_sda_o), .sda_oe(ctl_sda_oe), .sda_i(sda)
    );
    i2c_target_xtor tgt_xtor (
        .clock(clock), .reset(reset),
        .scl_o(tgt_scl_o), .scl_oe(tgt_scl_oe), .scl_i(scl),
        .sda_o(tgt_sda_o), .sda_oe(tgt_sda_oe), .sda_i(sda)
    );
    i2c_monitor_xtor mon_xtor (
        .clock(clock), .reset(reset),
        .scl_i(scl), .sda_i(sda)
    );

    initial begin
        automatic driver                     drv;
        automatic mem_model                  mem;
        automatic observer                   obs;
        automatic i2c_controller_xtor_bridge cbr;
        automatic i2c_target_xtor_bridge     tbr;
        automatic i2c_monitor_xtor_bridge     mbr;
        automatic int errors = 0;

        reset = 1'b1;
        repeat (4) @(posedge clock);
        reset = 1'b0;
        @(posedge clock);

        cbr = new(ctl_xtor.u_if);
        mem = new();
        obs = new();
        tbr = new(tgt_xtor.u_if, mem);
        mbr = new(mon_xtor.u_if, obs);
        drv = new(cbr);

        tbr.start_bridge();
        mbr.start_bridge();
        drv.run();

        // Drain: let the last observed event propagate to the monitor.
        repeat (50) @(posedge clock);

        errors = drv.errors;
        // The monitor sees one address byte per start() -- 3 reads, 2
        // writes and the NACKed write = 6 -- plus every data byte the
        // driver moved, and one STOP per transaction.
        if (obs.n_bytes_seen != (6 + drv.n_bytes)) begin
            $display("FAIL: monitor saw %0d bytes, expected %0d",
                      obs.n_bytes_seen, 6 + drv.n_bytes);
            errors++;
        end else
            $display("[monitor] observed %0d bytes (%0d address + %0d data) OK",
                      obs.n_bytes_seen, 6, drv.n_bytes);

        if (obs.n_stops != 6) begin
            $display("FAIL: monitor saw %0d STOPs, expected 6", obs.n_stops);
            errors++;
        end

        if (errors != 0) $fatal(1, "[i2c_proto] FAIL (%0d errors)", errors);
        $display("[i2c_proto] PASS");
        $finish;
    end

    // Watchdog so a broken handshake fails fast instead of hanging.
    initial begin
        #200us;
        $fatal(1, "[i2c_proto] TIMEOUT");
    end
endmodule
