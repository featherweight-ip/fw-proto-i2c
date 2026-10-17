// ======================================================================
// REQUIRED back-to-back FORMAL verification component for the I2C kit
// (design.md §8 issue 2 / the fw-proto-kit SKILL's "Back-to-back formal
// verification component"). Wires the two synthesizable CORES (controller
// <-> target) back-to-back over the real open-drain SCL/SDA bus; a free
// environment drives the controller's op stream and the target's model
// decisions, so the solver explores every legal interleaving.
//
// See PROPERTIES.md for the spec -> invariant -> checker mapping this file
// implements.
//
// Adapted shape vs. the kit's usual single-producer/single-consumer formal
// pattern (SKILL.md "Back-to-back formal verification component"): I2C has
// no pipelining at the class-layer API (one blocking op at a time, design
// rule confirmed in design.md §4), so the classic anyconst-index tracker
// (built for proving order/integrity across an OVERLAPPING stream) is
// overkill here -- a direct "last transferred byte" register suffices and
// is simpler to get right, since there is never more than one byte in
// flight between the two cores. See PROPERTIES.md item 5.
// ======================================================================
`ifdef FORMAL

module i2c_proto_fv (
        input wire clock,
        input wire reset
    );

    // ---- free environment: controller's op stream -----------------
    (* anyseq *) wire [23:0] f_req_dat;
    (* anyseq *) wire        f_req_valid;
    (* anyseq *) wire        f_rsp_ready;

    wire [9:0] ctl_rsp_dat;
    wire       ctl_rsp_valid;
    wire       ctl_req_ready;

    wire ctl_scl_o, ctl_scl_oe, ctl_sda_o, ctl_sda_oe;
    wire tgt_scl_o, tgt_scl_oe, tgt_sda_o, tgt_sda_oe;

    // Two-core open-drain bus (no third party, so a simple OR-reduction
    // suffices -- the same wired-AND convention the sim testbenches use).
    wire scl = (ctl_scl_oe || tgt_scl_oe) ? 1'b0 : 1'b1;
    wire sda = (ctl_sda_oe || tgt_sda_oe) ? 1'b0 : 1'b1;

    // Minimal timing parameters for formal: keeps the SMT state small and
    // lets a full op (and the cover goals below) complete within a modest
    // BMC depth. The cores' logic paths exercised are identical regardless
    // of T_LOW/T_HIGH's actual values -- only the hold-count durations
    // shrink, matching the kit's own "keep the SMT state small" guidance.
    i2c_controller_xtor_core #(
        .T_LOW(1), .T_HIGH(1), .T_HD_STA(1), .T_SU_STA(1), .T_SU_STO(1), .T_BUF(1)
    ) u_ctl (
        .clock(clock), .reset(reset),
        .scl_o(ctl_scl_o), .scl_oe(ctl_scl_oe), .scl_i(scl),
        .sda_o(ctl_sda_o), .sda_oe(ctl_sda_oe), .sda_i(sda),
        .req_dat(f_req_dat), .req_valid(f_req_valid), .req_ready(ctl_req_ready),
        .rsp_dat(ctl_rsp_dat), .rsp_valid(ctl_rsp_valid), .rsp_ready(f_rsp_ready),
        .f_state(ctl_f_state), .f_bit_idx(ctl_f_bit_idx)
    );
    // The controller's formal-only debug view (see its port list).
    wire [3:0] ctl_f_state, ctl_f_bit_idx;
    localparam bit [3:0] S_BIT_WAIT_HI = 4'd6; // state_e in the core

    // ---- free environment: target's model decisions ----------------
    wire [21:0] tgt_req_dat;   // {addr_10b, kind, addr[9:0], read, data}
    wire        tgt_req_valid;
    (* anyseq *) wire        f_tgt_req_ready;
    (* anyseq *) wire [8:0]  f_tgt_rsp_dat;
    (* anyseq *) wire        f_tgt_rsp_valid;

    i2c_target_xtor_core u_tgt (
        .clock(clock), .reset(reset),
        .scl_o(tgt_scl_o), .scl_oe(tgt_scl_oe), .scl_i(scl),
        .sda_o(tgt_sda_o), .sda_oe(tgt_sda_oe), .sda_i(sda),
        .req_dat(tgt_req_dat), .req_valid(tgt_req_valid), .req_ready(f_tgt_req_ready),
        .rsp_dat(f_tgt_rsp_dat), .rsp_valid(f_tgt_rsp_valid), .rsp_ready(tgt_rsp_ready_unused)
    );
    wire tgt_rsp_ready_unused;

    // =================================================================
    // Checkers (PROPERTIES.md maps each of these to its spec clause)
    // =================================================================
    reg f_past_valid;
    initial f_past_valid = 1'b0;
    always @(posedge clock) f_past_valid <= 1'b1;
    always @(*)            if (!f_past_valid) assume (reset);
    always @(posedge clock) if (f_past_valid) assume (!reset);

    // The free environment models a CORRECT caller of the per-byte class
    // API (design.md §4): start() always precedes tx_byte()/rx_byte()/
    // stop(), exactly like a real driver sequence. The core itself has no
    // way to detect "stop() called without a prior start()" (that
    // precondition is the caller's contract, same as the class-layer API
    // docs state), so an unconstrained free environment can legally pick
    // op==STOP as its very first op -- which the RTL then (correctly, per
    // its own narrow contract) executes as a bare STOP condition with no
    // preceding START, something a real I2C driver would never do. BMC
    // found exactly that before this assumption was added (see the kit
    // implementation report); this assumption scopes the proof to actual
    // callers, matching how this core is contractually used.
    reg env_owns_bus;
    always @(posedge clock) begin
        if (reset) env_owns_bus <= 1'b0;
        else if (f_req_valid && ctl_req_ready) begin
            if (f_req_dat[23:22] == 2'd0 /* I2C_OP_START */) env_owns_bus <= 1'b1;
            if (f_req_dat[23:22] == 2'd3 /* I2C_OP_STOP */)  env_owns_bus <= 1'b0;
        end
    end
    always @(*) begin
        if (f_req_valid && f_req_dat[23:22] != 2'd0 /* anything but START */)
            assume (env_owns_bus);
    end

    // Likewise a correct caller moves data in the direction its START's R/W
    // bit selected: tx_byte() only in a write, rx_byte() only in a read.
    // Without this, BMC (depth 68) found START(write) followed by RX_BYTE:
    // the controller releases SDA, the write-mode target clocks in 0xFF from
    // the pull-ups, and the data-integrity check below pairs that byte with
    // the NEXT tx_byte()'s data -- a caller error, not a core defect.
    reg env_read;
    always @(posedge clock) begin
        if (reset) env_read <= 1'b0;
        else if (f_req_valid && ctl_req_ready && f_req_dat[23:22] == 2'd0 /* START */)
            env_read <= f_req_dat[10]; // read field, see i2c_xtor_macros.svh
    end
    always @(*) begin
        if (f_req_valid && f_req_dat[23:22] == 2'd1 /* TX_BYTE */) assume (!env_read);
        if (f_req_valid && f_req_dat[23:22] == 2'd2 /* RX_BYTE */) assume (env_read);
    end

    // And a correct caller finishes a read before it ends it (UM10204: a
    // target ACKed for a read owns SDA for the next byte, so only a NACKed
    // byte hands the bus back): after a read address is ACKed, or after an
    // rx_byte() that ACKed, the next op is rx_byte() -- never stop() or a
    // repeated start(). Without this, BMC (depth 77) found START(read),
    // ACK, then an immediate Sr: the target drove its first data bit, the
    // Sr edge never appeared, and invariant 7 paired the wrong byte.
    reg [1:0] ctl_op;            // op the controller last accepted
    reg       ctl_rsp_valid_q;
    wire      ctl_rsp_rise = ctl_rsp_valid && !ctl_rsp_valid_q;
    reg       env_owes_rx;
    always @(posedge clock) begin
        if (reset) env_owes_rx <= 1'b0;
        else if (f_req_valid && ctl_req_ready && f_req_dat[23:22] == 2'd2 /* RX_BYTE */)
            env_owes_rx <= f_req_dat[0]; // ack_in: 1 asks for another byte
        else if (ctl_rsp_rise && ctl_op == 2'd0 /* START */)
            env_owes_rx <= env_read && ctl_rsp_dat[9] /* ack */ && !ctl_rsp_dat[8] /* arb_lost */;
        else if (ctl_rsp_rise && ctl_rsp_dat[8] /* arb_lost: bus released */)
            env_owes_rx <= 1'b0;
    end
    always @(*) if (f_req_valid && env_owes_rx) assume (f_req_dat[23:22] == 2'd2 /* RX_BYTE */);

    // A correct caller does not send a 7-bit address of the form 1111 0xx:
    // UM10204 reserves it for the 10-bit prefix, and its address byte IS a
    // 10-bit header on the bus. Without this, BMC (depth 31) found a 7-bit
    // start() to 0x7A raise the target's ADDR10_HI request.
    always @(*)
        if (f_req_valid && f_req_dat[23:22] == 2'd0 /* START */ && !f_req_dat[11] /* addr_10b */)
            assume (f_req_dat[18:14] != 5'b11110); // addr[6:2]

    // The target's model-decision response cannot arrive in the SAME
    // cycle the core raises its request (a real bridge calling a real
    // task always takes at least one delta/cycle to run) -- without this,
    // the free environment can answer instantaneously every time, which
    // is not a real bridge and (per the kit implementation report) was
    // observed to expose handshake assumptions the RTL does not actually
    // need to satisfy. Modeled the same way for both the controller's own
    // response consumer and the target's model-decision responder.
    reg tgt_upreq_prev;
    always @(posedge clock) begin
        if (reset) tgt_upreq_prev <= 1'b0;
        else       tgt_upreq_prev <= tgt_req_valid;
    end
    always @(*) if (tgt_req_valid && !tgt_upreq_prev) assume (!f_tgt_rsp_valid);

    // ---- #1/#2 START/STOP legality (framing) ------------------------
    reg sclq, sdaq;
    always @(posedge clock) begin
        if (reset) begin sclq <= 1'b1; sdaq <= 1'b1; end
        else       begin sclq <= scl;  sdaq <= sda;  end
    end
    wire start_edge = scl && sclq && sdaq && !sda;
    wire stop_edge  = scl && sclq && !sdaq && sda;

    reg in_txn;
    always @(posedge clock) begin
        if (reset) in_txn <= 1'b0;
        else if (start_edge) in_txn <= 1'b1;
        else if (stop_edge)  in_txn <= 1'b0;
    end

    always @(posedge clock) begin
        if (f_past_valid && !$past(reset)) begin
            // A STOP is never observed on the bus unless a START (and no
            // intervening STOP) already opened a transaction.
            if ($past(stop_edge)) assert ($past(in_txn));
        end
    end

    // ---- #2/#3 bit counting ------------------------------------------
    // Count the SCL rises of the byte in flight: from the START (or Sr)
    // edge for an address byte, from the op's acceptance for a data byte.
    // A data op is accepted with SCL high and starts with a fall, so no
    // rise is counted early. The count wraps after 9: the second byte of a
    // 10-bit header follows its first with no START edge between.
    wire      scl_rise = scl && !sclq;
    reg [3:0] byte_rises;          // rises before this clock
    wire [3:0] rises_next = (byte_rises == 4'd9) ? 4'd1 : byte_rises + 4'd1;
    wire [3:0] rises_now  = scl_rise ? rises_next : byte_rises;
    reg       sda_at_9th;
    always @(posedge clock) begin
        if (reset) byte_rises <= 4'd0;
        else if (start_edge ||
                 (f_req_valid && ctl_req_ready &&
                  (f_req_dat[23:22] == 2'd1 || f_req_dat[23:22] == 2'd2) /* TX/RX_BYTE */))
            byte_rises <= 4'd0;
        else if (scl_rise)
            byte_rises <= rises_next;
        if (!reset && scl_rise && rises_now == 4'd9) sda_at_9th <= sda;
    end

    always @(posedge clock) begin
        if (f_past_valid && !$past(reset)) begin
            // #3: the controller's bit index stays in 0..8, and each time
            // it samples a bit (SCL first reads high) it is on the bit the
            // bus is on: index n at the (n+1)th rise of the byte.
            assert (ctl_f_bit_idx <= 4'd8);
            if (ctl_f_state == S_BIT_WAIT_HI && scl)
                assert (rises_now == ctl_f_bit_idx + 4'd1);

            // #2: an address or data byte the controller sent ends after
            // exactly 9 clocks, and the ack it reports is what SDA carried
            // at the 9th rise: the ACK is sampled on the 9th clock. (For a
            // 10-bit start(), the last header byte sent.)
            if (ctl_rsp_rise && (ctl_op == 2'd0 || ctl_op == 2'd1) /* START/TX */ &&
                !ctl_rsp_dat[8] /* arb_lost */) begin
                assert (byte_rises == 4'd9);
                assert (ctl_rsp_dat[9] /* ack */ == !sda_at_9th);
            end
        end
    end

    // ---- #4 clock-low-wins synchronization --------------------------
    // Whenever EITHER core still holds its own line low (scl_oe), the bus
    // reads low -- i.e. the resolved bus can never read high while a
    // party that has not yet released is still asserting low. This is
    // what makes "wait for scl_i to actually read high" (both cores) a
    // correct stretch/sync mechanism rather than a race.
    always @(posedge clock) begin
        if (f_past_valid && !$past(reset)) begin
            if ($past(ctl_scl_oe) || $past(tgt_scl_oe))
                assert (!$past(scl));
        end
    end

    // ---- #5 no arbitration loss without a second controller ---------
    // UM10204 arbitration is lost only to another controller driving 0
    // while this one drives 1. This harness has one controller, so every
    // arb_lost=1 would mean the TARGET pulled SDA low over a bit the
    // controller was driving: out of turn, e.g. one bit misaligned. Real
    // arbitration is exercised by the i2c_arb sim.
    //
    // (This used to assert ARB_ABORT => an arbitration cause, read through
    // u_ctl.* hierarchical references. The sv2v + yosys flow does not
    // resolve those: yosys declares each as an implicit, undriven wire, so
    // the check was a constant. Checks here use ports and harness state
    // only -- PROPERTIES.md "Harness pitfalls".)
    always @(posedge clock) begin
        if (f_past_valid && !$past(reset)) begin
            if (ctl_rsp_rise) assert (!ctl_rsp_dat[8] /* arb_lost */);
        end
    end

    // ---- #6 data integrity, end to end (adapted method, see header) --
    // Capture the byte on every TX_BYTE op the controller accepts; assert
    // the target's next RX_BYTE up-request (if arbitration was not lost
    // in between) carries that same byte. Single-outstanding link, so a
    // direct capture-and-match is equivalent in rigor to the usual
    // anyconst-index tracker here (see header note / PROPERTIES.md #5).
    reg [7:0] last_tx_byte;
    reg       last_tx_valid;
    reg       lost_since;
    always @(posedge clock) begin
        if (reset) begin
            last_tx_valid <= 1'b0;
            lost_since    <= 1'b0;
        end else begin
            if (f_req_valid && ctl_req_ready &&
                f_req_dat[23:22] == 2'd1 /* I2C_OP_TX_BYTE */) begin
                last_tx_byte  <= f_req_dat[8:1]; // wr_data field, see i2c_xtor_macros.svh
                last_tx_valid <= 1'b1;
                lost_since    <= 1'b0;
            end
            if (ctl_rsp_rise && ctl_rsp_dat[8] /* arb_lost */) lost_since <= 1'b1;
            if (tgt_req_valid && f_tgt_req_ready && tgt_req_dat[20:19] == 2'd1 /* RX_BYTE */) begin
                if (last_tx_valid && !lost_since)
                    assert (tgt_req_dat[7:0] == last_tx_byte);
                last_tx_valid <= 1'b0;
            end
        end
    end

    // ---- #7 data integrity, read direction ---------------------------
    // The mirror of #6: capture the byte the target's model supplies (the
    // response to a TX_NEED_DATA up-request); the controller's next RX_BYTE
    // result (arbitration not lost) must carry it. #6 alone left the read
    // path unproven, and the target once mis-timed reads the same way it
    // mis-timed writes.
    reg [7:0] last_sup_byte;
    reg       last_sup_valid;
    wire      rx_result   = ctl_rsp_rise && ctl_op == 2'd2 /* RX_BYTE */ &&
                             !ctl_rsp_dat[8] /* arb_lost */;
    always @(posedge clock) begin
        if (reset) begin
            ctl_op          <= 2'd0;
            ctl_rsp_valid_q <= 1'b0;
            last_sup_valid  <= 1'b0;
        end else begin
            ctl_rsp_valid_q <= ctl_rsp_valid;
            if (f_req_valid && ctl_req_ready) ctl_op <= f_req_dat[23:22];
            if (tgt_req_valid && f_tgt_rsp_valid && tgt_req_dat[20:19] == 2'd2 /* TX_NEED_DATA */) begin
                last_sup_byte  <= f_tgt_rsp_dat[7:0]; // data field, see i2c_xtor_macros.svh
                last_sup_valid <= 1'b1;
            end
            if (ctl_rsp_rise && ctl_op == 2'd2) begin
                if (rx_result && last_sup_valid)
                    assert (ctl_rsp_dat[7:0] == last_sup_byte);
                last_sup_valid <= 1'b0;
            end
        end
    end

    // ---- #8 START/STOP edges are the controller's own ----------------
    // Every START (or Sr) edge on the bus falls while the controller is
    // executing a start() op and is itself pulling SDA low, with the
    // target not driving SDA; every STOP edge rises while it is executing
    // a stop(). Catches a core making a spurious edge: the controller
    // presenting a stale SDA drive between ops (both fixed bugs in
    // PROPERTIES.md), or the target still holding its ACK, or driving a
    // data bit, when SCL rises for an Sr. An op is in progress from its
    // acceptance until the controller raises rsp_valid, which it does only
    // after the edge (both edges at T_SU_STA = T_BUF = 1).
    reg ctl_busy;
    always @(posedge clock) begin
        if (reset) ctl_busy <= 1'b0;
        else if (f_req_valid && ctl_req_ready) ctl_busy <= 1'b1;
        else if (ctl_rsp_valid) ctl_busy <= 1'b0;
    end
    always @(posedge clock) begin
        if (f_past_valid && !$past(reset)) begin
            if (start_edge)
                assert (ctl_busy && ctl_op == 2'd0 /* START */ && ctl_sda_oe && !tgt_sda_oe);
            if (stop_edge)
                assert (ctl_busy && ctl_op == 2'd3 /* STOP */);
        end
    end

    // ---- #9 address integrity, START and Sr, 7- and 10-bit -----------
    // Each address phase the target raises carries the address and R/W of
    // the controller's last start() -- whether a START or an Sr opened it --
    // and the target raises none the controller did not send. A request is
    // new on the rise of req_valid (the core leaves UPREQ before raising
    // the next).
    //
    // A 10-bit start() raises up to three: ADDR10_HI with its A9:A8, then
    // ADDRESSED with the full address and R/W=0 (the write header), and
    // for a read, after the controller's own Sr, ADDRESSED again with R/W=1.
    // The read one comes only after the write one.
    reg [9:0] last_addr;
    reg       last_read, last_10b, last_addr_valid, last_hdr_wr_seen;
    wire      tgt_upreq_new = tgt_req_valid && !tgt_upreq_prev;
    wire      tgt_addressed = tgt_upreq_new && tgt_req_dat[20:19] == 2'd0 /* ADDRESSED */;
    wire      tgt_addr10_hi = tgt_upreq_new && tgt_req_dat[20:19] == 2'd3 /* ADDR10_HI */;
    wire       tgt_10b  = tgt_req_dat[21];
    wire [9:0] tgt_addr = tgt_req_dat[18:9];
    wire       tgt_read = tgt_req_dat[8];
    always @(posedge clock) begin
        if (reset) last_addr_valid <= 1'b0;
        else begin
            if (f_req_valid && ctl_req_ready && f_req_dat[23:22] == 2'd0 /* START */) begin
                // addr, addr_10b and read fields, see i2c_xtor_macros.svh
                last_addr        <= f_req_dat[21:12];
                last_10b         <= f_req_dat[11];
                last_read        <= f_req_dat[10];
                last_addr_valid  <= 1'b1;
                last_hdr_wr_seen <= 1'b0;
            end
            if (tgt_addr10_hi) begin
                assert (last_addr_valid && last_10b);
                if (last_addr_valid && last_10b)
                    assert (tgt_addr[9:8] == last_addr[9:8]);
            end
            if (tgt_addressed) begin
                assert (last_addr_valid);
                if (last_addr_valid && !last_10b) begin
                    assert (!tgt_10b && {tgt_addr[6:0], tgt_read} == {last_addr[6:0], last_read});
                    last_addr_valid <= 1'b0;
                end
                if (last_addr_valid && last_10b) begin
                    assert (tgt_10b && tgt_addr == last_addr);
                    if (!tgt_read) begin
                        assert (!last_hdr_wr_seen);
                        last_hdr_wr_seen <= 1'b1;
                        if (!last_read) last_addr_valid <= 1'b0;
                    end else begin
                        assert (last_read && last_hdr_wr_seen);
                        last_addr_valid <= 1'b0;
                    end
                end
            end
        end
    end

    // An Sr has been seen on the bus: a START edge inside a transaction.
    reg sr_seen;
    always @(posedge clock) begin
        if (reset) sr_seen <= 1'b0;
        else if (start_edge && in_txn) sr_seen <= 1'b1;
    end

    // ---- cover: the proof is not vacuous -----------------------------
    always @(posedge clock) begin
        // An address phase opened by an Sr reaches the target (#8 and #9
        // see a repeated START).
        cover (f_past_valid && !reset && sr_seen && tgt_addressed);
        // A 10-bit write header, and a 10-bit read header, reach the
        // target's model (#9 sees both 10-bit forms).
        cover (f_past_valid && !reset && tgt_addressed && tgt_10b && !tgt_read);
        cover (f_past_valid && !reset && tgt_addressed && tgt_10b && tgt_read);
        cover (tgt_req_valid && f_tgt_req_ready && tgt_req_dat[20:19] == 2'd1);
        // A target-supplied byte reaches the controller (#7 can fire).
        cover (!reset && rx_result && last_sup_valid);
        // A real STOP closing a transaction, out of reset. The harness's own
        // registers (sclq/sdaq) reset synchronously, so during the reset
        // step they hold free initial values and can fake a stop_edge --
        // an ungated `$past(stop_edge)` was hit vacuously at step 2.
        cover (f_past_valid && !$past(reset) && $past(stop_edge) && $past(in_txn));
        // NOTE: no `cover` goal for ARB_ABORT here -- arbitration needs a
        // SECOND controller contending for the bus, which this harness
        // (one controller, one target, per the kit's usual formal shape)
        // cannot produce; the assert above stays (vacuously true in this
        // harness, which is fine -- it is a real check the moment a second
        // controller is ever added to a formal harness). Arbitration
        // itself is proven by the dedicated i2c_arb_tb.sv back-to-back SIM
        // test instead (design.md §8 issue 1).
    end

endmodule

`endif // FORMAL
