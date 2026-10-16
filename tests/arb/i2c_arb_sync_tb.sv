// ======================================================================
// i2c_arb_tb with the two controllers clocking at different rates: clock
// synchronization (UM10204 §3.1.7). The winner has the long high phase and
// the short low phase, the loser the reverse, so the bus clock both of
// them follow comes from the loser: its low phase stretches the winner's,
// and the winner must cut its own high phase short when the loser pulls
// SCL low. The addresses differ only in their LSB, so the two controllers
// share the bus for seven bits before the loser drops out, and the winner
// then finishes the transaction on its own clock.
//
// The controller once counted T_HIGH without watching SCL. Here the
// winner kept counting high through the loser's low phase, and was still
// counting when the loser released SCL again, so the loser clocked a bit
// the winner did not.
//
// Run:  dfm run i2c-arb-sync
// ======================================================================
module i2c_arb_sync_tb;
    i2c_arb_tb #(
        .WINNER_ADDR(7'h54), .LOSER_ADDR(7'h55),
        .A_T_LOW(3), .A_T_HIGH(16),
        .B_T_LOW(7), .B_T_HIGH(3)
    ) u_tb ();
endmodule
