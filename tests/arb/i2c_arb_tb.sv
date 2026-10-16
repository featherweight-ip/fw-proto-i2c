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
// Run:  dfm run i2c-arb    (expect: [i2c_arb] PASS; any failure exits
//                            nonzero)
// ======================================================================
module i2c_arb_tb;
    import fw_proto_i2c_pkg::*;

    localparam bit [6:0] WINNER_ADDR = 7'h10; // 001_0000 -- MSB 0
    localparam bit [6:0] LOSER_ADDR  = 7'h60; // 110_0000 -- MSB 1

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

    i2c_controller_xtor ctl_a (
        .clock(clock), .reset(reset),
        .scl_o(a_scl_o), .scl_oe(a_scl_oe), .scl_i(scl),
        .sda_o(a_sda_o), .sda_oe(a_sda_oe), .sda_i(sda)
    );
    i2c_controller_xtor ctl_b (
        .clock(clock), .reset(reset),
        .scl_o(b_scl_o), .scl_oe(b_scl_oe), .scl_i(scl),
        .sda_o(b_sda_o), .sda_oe(b_sda_oe), .sda_i(sda)
    );
    i2c_target_xtor tgt (
        .clock(clock), .reset(reset),
        .scl_o(t_scl_o), .scl_oe(t_scl_oe), .scl_i(scl),
        .sda_o(t_sda_o), .sda_oe(t_sda_oe), .sda_i(sda)
    );

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

        errors = drv_a.errors + drv_b.errors;

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
