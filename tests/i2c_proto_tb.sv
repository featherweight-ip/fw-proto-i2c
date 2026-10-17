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
// SCL low), both ACK and NACK paths, and repeated START (Sr) after a write,
// after a read and after a NACKed address, and 10-bit addressing (write,
// read, read after an Sr, and a NACK at either header byte). The monitor
// must see every START, Sr, STOP and byte the driver put on the bus.
//
// The controller's timing parameters are this module's, so a wrapper top
// can rerun the same case at another bus speed (i2c_proto_fast_tb.sv: 1
// clock per phase, the target core's minimum). PRESCALE is applied at
// runtime through the class API (set_prescale()) before the first op
// (i2c_proto_prescale_tb.sv). Every SCL low phase the controller drives
// is checked to last exactly T_LOW*(PRESCALE+1) clocks.
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
        parameter int T_BUF     = 2,
        parameter int PRESCALE  = 0
    );
    import fw_proto_i2c_pkg::*;

    localparam bit [6:0] TGT_ADDR   = 7'h50;
    localparam bit [9:0] TGT_ADDR10 = 10'h2A5; // the target's 10-bit address

    // --------------------------------------------------------------
    // Driver: holds the controller API handle; runs the stimulus and
    // checks round-trip integrity inline.
    // --------------------------------------------------------------
    class driver;
        i2c_controller_if ctl;
        int unsigned errors;
        int unsigned n_bytes;
        int unsigned n_addr;   // address bytes on the bus
        int unsigned n_starts; // START + Sr conditions on the bus
        int unsigned n_sr;     // of which repeated STARTs
        int unsigned n_stop;

        function new(i2c_controller_if ctl);
            this.ctl = ctl;
        endfunction

        // start() and stop(), counted so the monitor's stream can be
        // checked against what the driver actually put on the bus.
        task automatic do_start(input bit [6:0] addr, input bit read,
                                input bit repeated, output bit ack,
                                output bit arb_lost);
            ctl.start(addr, 1'b0, read, repeated, ack, arb_lost);
            n_addr++;
            n_starts++;
            if (repeated) n_sr++;
        endtask

        // A 10-bit start(). nack_byte is the header byte the target must
        // NACK (0 or 1), or -1 if it must ACK them all; ok says it did as
        // expected. The bus carries the header bytes up to the first NACK,
        // and for a read the controller's own Sr before the read header.
        task automatic do_start10(input bit [9:0] addr, input bit read,
                                  input bit repeated, input int nack_byte,
                                  output bit ok);
            bit ack, arb_lost;
            int unsigned n_hdr;
            ctl.start(addr, 1'b1, read, repeated, ack, arb_lost);
            n_hdr = (nack_byte >= 0) ? nack_byte + 1 : (read ? 3 : 2);
            n_addr   += n_hdr;
            n_starts += (n_hdr == 3) ? 2 : 1;
            n_sr     += ((n_hdr == 3) ? 1 : 0) + (repeated ? 1 : 0);
            ok = !arb_lost && (ack == (nack_byte < 0));
            if (!ok) begin
                $display("FAIL: 10-bit %s %s addr 0x%03h: ack=%0d arb_lost=%0d, expected %s",
                          repeated ? "Sr" : "START", read ? "read" : "write",
                          addr, ack, arb_lost,
                          (nack_byte < 0) ? "ACK" : $sformatf("NACK at header byte %0d", nack_byte));
                errors++;
            end
        endtask

        task automatic do_stop();
            ctl.stop();
            n_stop++;
        endtask

        // The data bytes of a write, after its address phase.
        task automatic tx_bytes(input bit [7:0] data[]);
            bit ack, arb_lost;
            foreach (data[i]) begin
                ctl.tx_byte(data[i], ack, arb_lost);
                n_bytes++;
                if (arb_lost || !ack) begin
                    $display("FAIL: write byte[%0d]=0x%02h not ACKed (arb_lost=%0d ack=%0d)",
                              i, data[i], arb_lost, ack);
                    errors++;
                end
            end
        endtask

        // The data bytes of a read, after its address phase. NACKs the
        // last, which hands SDA back for a STOP or Sr.
        task automatic rx_bytes(input int n, output bit [7:0] data[]);
            bit arb_lost;
            data = new[n];
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
        endtask

        task automatic check_bytes(input string what, input bit [7:0] got[],
                                   input bit [7:0] exp[]);
            if (got.size() != exp.size()) begin
                $display("FAIL: %s: got %0d bytes, exp %0d", what, got.size(), exp.size());
                errors++;
                return;
            end
            foreach (exp[i]) begin
                if (got[i] !== exp[i]) begin
                    $display("FAIL: %s[%0d] got 0x%02h exp 0x%02h", what, i, got[i], exp[i]);
                    errors++;
                end
            end
        endtask

        // An address phase the target must ACK.
        task automatic expect_ack(input bit [6:0] addr, input bit read,
                                  input bit repeated, output bit ok);
            bit ack, arb_lost;
            do_start(addr, read, repeated, ack, arb_lost);
            ok = ack && !arb_lost;
            if (!ok) begin
                $display("FAIL: %s %s addr 0x%02h not ACKed (arb_lost=%0d ack=%0d)",
                          repeated ? "Sr" : "START", read ? "read" : "write",
                          addr, arb_lost, ack);
                errors++;
            end
        endtask

        task automatic write_bytes(input bit [6:0] addr, input bit [7:0] data[]);
            bit ok;
            expect_ack(addr, 1'b0, 1'b0, ok);
            if (ok) tx_bytes(data);
            do_stop();
        endtask

        task automatic read_bytes(input bit [6:0] addr, input int n, output bit [7:0] data[]);
            bit ok;
            data = new[n];
            expect_ack(addr, 1'b1, 1'b0, ok);
            if (ok) rx_bytes(n, data);
            do_stop();
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
                do_start(7'h7F, 1'b0, 1'b0, ack, arb_lost);
                if (ack || arb_lost) begin
                    $display("FAIL: expected address 0x7F to be NACKed");
                    errors++;
                end else
                    $display("[driver] address 0x7F correctly NACKed");
                do_stop();
            end
            #200ns;

            run_repeated_start();
            #200ns;
            run_10bit();
            #200ns;
            run_raw_10bit();
        endtask

        // A 10-bit header sent one raw byte at a time, the way software
        // drives a byte-level controller such as the OpenCores core: a 7-bit
        // start() whose address byte is 11110 A9 A8 R/W, and tx_byte() for
        // addr[7:0]. This breaks the i2c_controller_if contract on purpose
        // (the address is of the reserved form 1111 0xx); the target must
        // still see a 10-bit header, and the monitor must flag addr[7:0] as
        // an address byte.
        task automatic raw_hdr(input bit read, input bit repeated, output bit ok);
            bit ack, arb_lost;
            ctl.start({5'b11110, TGT_ADDR10[9:8]}, 1'b0, read, repeated, ack, arb_lost);
            n_addr++;
            n_starts++;
            if (repeated) n_sr++;
            ok = ack && !arb_lost;
            if (ok && !read) begin
                ctl.tx_byte(TGT_ADDR10[7:0], ack, arb_lost);
                n_addr++;
                ok = ack && !arb_lost;
            end
            if (!ok) begin
                $display("FAIL: raw 10-bit %s header not ACKed", read ? "read" : "write");
                errors++;
            end
        endtask

        task automatic run_raw_10bit();
            bit [7:0] wr[];
            bit [7:0] rd[];
            bit ok;

            // S F4 A5 data P, then S F4 A5 Sr F5 data P.
            wr = '{8'hC3, 8'h3C};
            raw_hdr(1'b0, 1'b0, ok);
            if (ok) tx_bytes(wr);
            do_stop();
            #200ns;
            raw_hdr(1'b0, 1'b0, ok);
            if (ok) raw_hdr(1'b1, 1'b1, ok);
            if (ok) rx_bytes(2, rd);
            do_stop();
            if (ok) check_bytes("raw 10-bit write, read", rd, wr);
        endtask

        // 10-bit addressing (UM10204 §3.1.11). The target answers
        // TGT_ADDR10 as well as TGT_ADDR, with the same memory behind both.
        task automatic run_10bit();
            bit [7:0] wr[];
            bit [7:0] rd[];
            bit ok;

            // Write, STOP, read: S 11110 10 W, A5, data / S 11110 10 W, A5,
            // Sr, 11110 10 R, data.
            wr = '{8'h6B, 8'h17, 8'hE4};
            do_start10(TGT_ADDR10, 1'b0, 1'b0, -1, ok);
            if (ok) tx_bytes(wr);
            do_stop();
            #200ns;
            do_start10(TGT_ADDR10, 1'b1, 1'b0, -1, ok);
            if (ok) rx_bytes(3, rd);
            do_stop();
            if (ok) check_bytes("10-bit write, read", rd, wr);
            #200ns;

            // 7-bit write -> Sr -> 10-bit read: the read's header starts
            // from an Sr.
            wr = '{8'h29, 8'h0F};
            expect_ack(TGT_ADDR, 1'b0, 1'b0, ok);
            tx_bytes(wr);
            do_start10(TGT_ADDR10, 1'b1, 1'b1, -1, ok);
            if (ok) rx_bytes(2, rd);
            do_stop();
            if (ok) check_bytes("7-bit write, Sr, 10-bit read", rd, wr);
            #200ns;

            // 10-bit write -> Sr -> 10-bit read, one transaction.
            wr = '{8'h7E, 8'h01};
            do_start10(TGT_ADDR10, 1'b0, 1'b0, -1, ok);
            if (ok) tx_bytes(wr);
            do_start10(TGT_ADDR10, 1'b1, 1'b1, -1, ok);
            if (ok) rx_bytes(2, rd);
            do_stop();
            if (ok) check_bytes("10-bit write, Sr, 10-bit read", rd, wr);
            #200ns;

            // Wrong A9:A8: NACKed at the first header byte. Wrong
            // addr[7:0]: NACKed at the second, for a write and a read.
            do_start10(10'h1A5, 1'b0, 1'b0, 0, ok);
            do_stop();
            #200ns;
            do_start10(10'h2A4, 1'b0, 1'b0, 1, ok);
            do_stop();
            #200ns;
            do_start10(10'h2A4, 1'b1, 1'b0, 1, ok);
            do_stop();
            #200ns;

            // After all that, 7-bit addressing still works.
            expect_ack(TGT_ADDR, 1'b1, 1'b0, ok);
            rx_bytes(1, rd);
            do_stop();
            check_bytes("7-bit read after 10-bit", rd, '{8'h7E});
        endtask

        // Repeated START (Sr): the controller keeps the bus between
        // segments. mem_model resets its pointer on every address phase,
        // Sr included, so each read below returns from offset 0.
        task automatic run_repeated_start();
            bit [7:0] wr[];
            bit [7:0] rd[];
            bit ok, ack, arb_lost;

            // write -> Sr -> read: the usual register read. The target
            // ACKed the last write byte, so it is mid-write when the Sr
            // arrives.
            wr = '{8'hC0, 8'h41, 8'hC2};
            expect_ack(TGT_ADDR, 1'b0, 1'b0, ok);
            tx_bytes(wr);
            expect_ack(TGT_ADDR, 1'b1, 1'b1, ok);
            rx_bytes(3, rd);
            do_stop();
            check_bytes("write-Sr-read", rd, wr);
            #200ns;

            // read -> Sr -> write -> Sr -> read: Sr after a NACKed read
            // byte, and Sr after a write whose last byte has MSB 0 (the
            // controller presents no SDA drive across that ack).
            expect_ack(TGT_ADDR, 1'b1, 1'b0, ok);
            rx_bytes(2, rd);
            check_bytes("read-Sr", rd, '{8'hC0, 8'h41});
            wr = '{8'h3D, 8'h1E};
            expect_ack(TGT_ADDR, 1'b0, 1'b1, ok);
            tx_bytes(wr);
            expect_ack(TGT_ADDR, 1'b1, 1'b1, ok);
            rx_bytes(3, rd);
            do_stop();
            check_bytes("Sr-write-Sr-read", rd, '{8'h3D, 8'h1E, 8'hC2});
            #200ns;

            // NACKed address -> Sr -> read: the target is not selected
            // when the Sr arrives.
            do_start(7'h7F, 1'b0, 1'b0, ack, arb_lost);
            if (ack || arb_lost) begin
                $display("FAIL: expected address 0x7F to be NACKed (before Sr)");
                errors++;
            end
            expect_ack(TGT_ADDR, 1'b1, 1'b1, ok);
            rx_bytes(1, rd);
            do_stop();
            check_bytes("NACK-Sr-read", rd, '{8'h3D});
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
            if (addr_10b ? addr == TGT_ADDR10 : addr[6:0] == TGT_ADDR) begin
                ack = 1'b1;
                // This test has no separate "register pointer" byte
                // convention -- every addressed() call (read or write)
                // simply (re)starts at a fixed base, so a write-then-read
                // pair against the same address round-trips directly.
                ptr = 8'h00;
            end else
                ack = 1'b0;
        endtask

        virtual task addressed_10b_hi(input bit [1:0] addr_hi, output bit ack);
            #19ns; // clock-stretch
            ack = (addr_hi == TGT_ADDR10[9:8]);
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
        int unsigned n_sr;
        int unsigned n_stops;
        int unsigned n_addr_bytes;
        string       events[$]; // every event, in order, for the TLM compare

        virtual function void observe_start(input bit repeated);
            n_starts++;
            if (repeated) n_sr++;
            events.push_back(repeated ? "Sr" : "S");
        endfunction
        virtual function void observe_stop();
            n_stops++;
            events.push_back("P");
        endfunction
        virtual function void observe_byte(input bit addr_phase, input bit [7:0] data,
                                            input bit ack);
            n_bytes_seen++;
            if (addr_phase) n_addr_bytes++;
            events.push_back($sformatf("%s%02h%s", addr_phase ? "A" : "D", data,
                                       ack ? "+" : "-"));
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

    // Prescaler check: each SCL low phase the controller drives (BIT_LOW,
    // REP_START_DROP, STOP_EDGE_LOW -- all T_LOW ticks) lasts exactly
    // T_LOW*(PRESCALE+1) clocks.
    int unsigned low_run = 0;
    int unsigned n_low_phases = 0;
    int unsigned low_errors = 0;
    always @(posedge clock) begin
        if (reset) low_run <= 0;
        else if (ctl_scl_oe) low_run <= low_run + 1;
        else if (low_run != 0) begin
            n_low_phases <= n_low_phases + 1;
            if (low_run != T_LOW * (PRESCALE + 1)) begin
                $display("FAIL: controller SCL low for %0d clocks, expected %0d",
                          low_run, T_LOW * (PRESCALE + 1));
                low_errors <= low_errors + 1;
            end
            low_run <= 0;
        end
    end

    initial begin
        automatic driver                     drv;
        automatic mem_model                  mem;
        automatic observer                   obs;
        automatic i2c_controller_xtor_bridge cbr;
        automatic i2c_target_xtor_bridge     tbr;
        automatic i2c_monitor_xtor_bridge     mbr;
        automatic int errors = 0;
        automatic realtime t_sig, t_tlm;

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
        cbr.set_prescale(16'(PRESCALE));

        tbr.start_bridge();
        mbr.start_bridge();
        t_sig = $realtime;
        drv.run();
        t_sig = $realtime - t_sig;

        // Drain: let the last observed event propagate to the monitor.
        repeat (50) @(posedge clock);

        errors = drv.errors + low_errors;
        if (n_low_phases == 0) begin
            $display("FAIL: no controller SCL low phases observed");
            errors++;
        end else
            $display("[timing] %0d SCL low phases of %0d clocks (T_LOW=%0d, prescale=%0d) OK",
                      n_low_phases, T_LOW * (PRESCALE + 1), T_LOW, PRESCALE);
        // The monitor sees every address byte (one per 7-bit start(), up
        // to three per 10-bit one) and data byte the driver moved, every
        // START and Sr, with the Sr flagged as repeated, and one STOP per
        // stop().
        if (obs.n_bytes_seen != (drv.n_addr + drv.n_bytes)) begin
            $display("FAIL: monitor saw %0d bytes, expected %0d",
                      obs.n_bytes_seen, drv.n_addr + drv.n_bytes);
            errors++;
        end else
            $display("[monitor] observed %0d bytes (%0d address + %0d data) OK",
                      obs.n_bytes_seen, drv.n_addr, drv.n_bytes);

        if (obs.n_addr_bytes != drv.n_addr) begin
            $display("FAIL: monitor saw %0d address bytes, expected %0d",
                      obs.n_addr_bytes, drv.n_addr);
            errors++;
        end
        if (obs.n_starts != drv.n_starts || obs.n_sr != drv.n_sr) begin
            $display("FAIL: monitor saw %0d STARTs (%0d Sr), expected %0d (%0d Sr)",
                      obs.n_starts, obs.n_sr, drv.n_starts, drv.n_sr);
            errors++;
        end
        if (obs.n_stops != drv.n_stop) begin
            $display("FAIL: monitor saw %0d STOPs, expected %0d", obs.n_stops, drv.n_stop);
            errors++;
        end

        // The same stimulus over the transaction-level bus (i2c_tlm_bus),
        // with a fresh target model: the driver's own checks must pass, and
        // the monitor must see exactly the event stream the signal-level
        // monitor saw.
        begin
            automatic driver      tdrv;
            automatic mem_model   tmem = new();
            automatic observer    tobs = new();
            automatic i2c_tlm_bus bus  = new(tmem);
            bus.add_monitor(tobs);
            bus.set_prescale(16'(PRESCALE));
            bus.set_timing(10ns, T_LOW, T_HIGH, T_HD_STA, T_SU_STA, T_SU_STO, T_BUF);
            tdrv = new(bus);
            t_tlm = $realtime;
            tdrv.run();
            t_tlm = $realtime - t_tlm;
            $display("[tlm] timed bus: %0t simulated vs %0t at signal level", t_tlm, t_sig);
            errors += tdrv.errors;
            if (tobs.events.size() != obs.events.size()) begin
                $display("FAIL: TLM bus produced %0d monitor events, signal level %0d",
                          tobs.events.size(), obs.events.size());
                errors++;
            end
            foreach (obs.events[i]) begin
                if (i >= tobs.events.size()) break;
                if (tobs.events[i] != obs.events[i]) begin
                    $display("FAIL: monitor event %0d: TLM bus %s, signal level %s",
                              i, tobs.events[i], obs.events[i]);
                    errors++;
                    break;
                end
            end
            $display("[tlm] %0d monitor events, same as signal level: %s",
                      tobs.events.size(),
                      (tobs.events == obs.events) ? "yes" : "NO");
        end

        if (errors != 0) $fatal(1, "[i2c_proto] FAIL (%0d errors)", errors);
        $display("[i2c_proto] PASS");
        $finish;
    end

    // Watchdog so a broken handshake fails fast instead of hanging.
    initial begin
        #(200us * (PRESCALE + 1));
        $fatal(1, "[i2c_proto] TIMEOUT");
    end
endmodule
