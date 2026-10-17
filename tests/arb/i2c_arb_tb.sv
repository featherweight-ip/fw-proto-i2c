// ======================================================================
// REQUIRED back-to-back SIMULATION test #2 of 2 for the I2C kit
// (design.md §8 issue 1): two CONTROLLER transactors contend for one
// shared open-drain bus against a single TARGET, exercising arbitration --
// something no other kit's mandatory back-to-back test needs, since no
// other protocol in this family is multi-master.
//
// Both controllers issue start() at the same simulated time against
// addresses whose MSB differs (0x10 vs 0x60): per UM10204 arbitration
// (wired-AND, lowest address wins), the controller driving the HIGHER
// address must sense its own released-high bit pulled low by the other
// controller and abandon the transaction, reporting arb_lost=1. The lower-
// address controller must be completely unaffected -- it does not even
// see a collision from its own point of view (design.md §5 ARB_CHECK only
// ever fires on the loser, by construction: the winner always drives what
// it reads back).
//
// The addresses and each controller's T_LOW/T_HIGH are this module's
// parameters, so a wrapper top can rerun the collision with the two
// controllers clocking at different rates (i2c_arb_sync_tb.sv: clock
// synchronization, UM10204 §3.1.7). While both controllers drive SCL, every
// low phase the bus shows must last as long as the longer T_LOW and no
// longer, and every high phase no longer than the shorter T_HIGH -- each
// within the two clocks a core takes to see SCL change and react. The bench
// checks all three.
//
// Run:  dfm run i2c-arb    (expect: [i2c_arb] PASS; any failure exits
//                            nonzero)
// ======================================================================
module i2c_arb_tb #(
        parameter bit [6:0] WINNER_ADDR = 7'h10, // 001_0000 -- MSB 0
        parameter bit [6:0] LOSER_ADDR  = 7'h60, // 110_0000 -- MSB 1
        parameter int A_T_LOW  = 4,
        parameter int A_T_HIGH = 4,
        parameter int B_T_LOW  = 4,
        parameter int B_T_HIGH = 4
    );
    import fw_proto_i2c_pkg::*;

    class driver;
        i2c_controller_if ctl;
        string             name;
        bit [6:0]           addr;
        int unsigned        errors;
        bit                 done;
        bit                 got_arb_lost;
        bit                 got_ack;

        function new(i2c_controller_if ctl, string name, bit [6:0] addr);
            this.ctl  = ctl;
            this.name = name;
            this.addr = addr;
        endfunction

        task automatic run();
            bit       ack, arb_lost;
            bit [7:0] data;
            ctl.start(addr, 1'b0, 1'b0, 1'b0, ack, arb_lost);
            got_arb_lost = arb_lost;
            got_ack      = ack;
            if (!arb_lost) begin
                ctl.tx_byte(8'hC5, ack, arb_lost);
                if (arb_lost || !ack) begin
                    $display("FAIL(%s): data byte not ACKed (arb_lost=%0d ack=%0d)",
                              name, arb_lost, ack);
                    errors++;
                end
                ctl.stop();
            end
            done = 1'b1;
        endtask
    endclass

    class mem_model implements i2c_target_if;
        bit [7:0] got_data;
        bit       got_write;

        virtual task addressed(input bit [9:0] addr, input bit addr_10b,
                                input bit read, output bit ack);
            #11ns;
            ack = (addr[6:0] == WINNER_ADDR);
        endtask
        virtual task addressed_10b_hi(input bit [1:0] addr_hi, output bit ack);
            #7ns;
            ack = 1'b0;
        endtask
        virtual task rx_byte(input bit [7:0] data, output bit ack);
            #7ns;
            ack       = 1'b1;
            got_data  = data;
            got_write = 1'b1;
        endtask
        virtual task tx_byte(output bit [7:0] data, input bit ack);
            #7ns;
            data = 8'h00;
        endtask
    endclass

    logic clock = 1'b0;
    logic reset = 1'b1;
    always #5ns clock = ~clock;

    wire a_scl_o, a_scl_oe, a_sda_o, a_sda_oe;
    wire b_scl_o, b_scl_oe, b_sda_o, b_sda_oe;
    wire t_scl_o, t_scl_oe, t_sda_o, t_sda_oe;

    wire scl = (a_scl_oe || b_scl_oe || t_scl_oe) ? 1'b0 : 1'b1;
    wire sda = (a_sda_oe || b_sda_oe || t_sda_oe) ? 1'b0 : 1'b1;

    i2c_controller_xtor #(.T_LOW(A_T_LOW), .T_HIGH(A_T_HIGH)) ctl_a (
        .clock(clock), .reset(reset),
        .scl_o(a_scl_o), .scl_oe(a_scl_oe), .scl_i(scl),
        .sda_o(a_sda_o), .sda_oe(a_sda_oe), .sda_i(sda)
    );
    i2c_controller_xtor #(.T_LOW(B_T_LOW), .T_HIGH(B_T_HIGH)) ctl_b (
        .clock(clock), .reset(reset),
        .scl_o(b_scl_o), .scl_oe(b_scl_oe), .scl_i(scl),
        .sda_o(b_sda_o), .sda_oe(b_sda_oe), .sda_i(sda)
    );
    i2c_target_xtor tgt (
        .clock(clock), .reset(reset),
        .scl_o(t_scl_o), .scl_oe(t_scl_oe), .scl_i(scl),
        .sda_o(t_sda_o), .sda_oe(t_sda_oe), .sda_i(sda)
    );

    // --------------------------------------------------------------
    // Clock synchronization check. A low phase is contended when both
    // controllers pulled SCL low during it; a high phase is contended when
    // the low phases on both sides of it are. Phase lengths are in clocks.
    // --------------------------------------------------------------
    localparam int MAX_T_LOW  = (A_T_LOW  > B_T_LOW)  ? A_T_LOW  : B_T_LOW;
    localparam int MIN_T_HIGH = (A_T_HIGH < B_T_HIGH) ? A_T_HIGH : B_T_HIGH;

    int unsigned sync_errors;
    int unsigned n_contended_hi;
    int unsigned phase_len;
    int unsigned hi_len;       // length of the last high phase
    bit          scl_q = 1'b1;
    bit          a_low, b_low; // each controller pulled SCL low this phase
    bit          prev_low_contended;

    always @(posedge clock) begin
        if (reset) begin
            scl_q = 1'b1; phase_len = 0;
            a_low = 1'b0; b_low = 1'b0; prev_low_contended = 1'b0;
        end else begin
            if (scl != scl_q) begin
                if (!scl_q) begin
                    // A low phase just ended.
                    if (a_low && b_low &&
                        (phase_len < MAX_T_LOW || phase_len > MAX_T_LOW + 2)) begin
                        $display("FAIL: contended SCL low lasted %0d clocks, not %0d..%0d",
                                  phase_len, MAX_T_LOW, MAX_T_LOW + 2);
                        sync_errors++;
                    end
                    if (a_low && b_low && prev_low_contended) begin
                        n_contended_hi++;
                        if (hi_len > MIN_T_HIGH + 2) begin
                            $display("FAIL: contended SCL high lasted %0d clocks, > %0d",
                                      hi_len, MIN_T_HIGH + 2);
                            sync_errors++;
                        end
                    end
                    prev_low_contended = a_low && b_low;
                    a_low = 1'b0; b_low = 1'b0;
                end else
                    hi_len = phase_len;
                phase_len = 0;
            end
            phase_len++;
            if (a_scl_oe) a_low = 1'b1;
            if (b_scl_oe) b_low = 1'b1;
            scl_q = scl;
        end
    end

    initial begin
        automatic driver                     drv_a, drv_b;
        automatic mem_model                  mem;
        automatic i2c_controller_xtor_bridge  cbr_a, cbr_b;
        automatic i2c_target_xtor_bridge      tbr;
        automatic int errors = 0;

        reset = 1'b1;
        repeat (4) @(posedge clock);
        reset = 1'b0;
        @(posedge clock);

        cbr_a = new(ctl_a.u_if);
        cbr_b = new(ctl_b.u_if);
        mem   = new();
        tbr   = new(tgt.u_if, mem);
        drv_a = new(cbr_a, "A(winner)", WINNER_ADDR);
        drv_b = new(cbr_b, "B(loser)",  LOSER_ADDR);

        tbr.start_bridge();

        // Both controllers issue start() at the SAME simulated time --
        // the actual multi-master collision this test exists to exercise.
        fork
            drv_a.run();
            drv_b.run();
        join

        repeat (20) @(posedge clock);

        errors = drv_a.errors + drv_b.errors + sync_errors;
        $display("[sync] %0d contended SCL high phases checked", n_contended_hi);

        if (drv_a.got_arb_lost) begin
            $display("FAIL: winner (addr 0x%02h) unexpectedly lost arbitration", WINNER_ADDR);
            errors++;
        end else if (!drv_a.got_ack) begin
            $display("FAIL: winner's address was not ACKed"); errors++;
        end else
            $display("[driver] winner completed its transaction uncorrupted");

        if (!drv_b.got_arb_lost) begin
            $display("FAIL: loser (addr 0x%02h) did not report arb_lost", LOSER_ADDR);
            errors++;
        end else
            $display("[driver] loser correctly reported arb_lost");

        if (!mem.got_write || mem.got_data !== 8'hC5) begin
            $display("FAIL: target did not receive the winner's byte intact (got_write=%0d data=0x%02h)",
                      mem.got_write, mem.got_data);
            errors++;
        end else
            $display("[target] received 0x%02h from the winner OK", mem.got_data);

        if (errors != 0) $fatal(1, "[i2c_arb] FAIL (%0d errors)", errors);
        $display("[i2c_arb] PASS");
        $finish;
    end

    initial begin
        #200us;
        $fatal(1, "[i2c_arb] TIMEOUT");
    end
endmodule
