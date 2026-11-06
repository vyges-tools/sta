// A clock buffer, then an inverter: r1 and r3 on the buffer, r2 on the inverter. r1 -> r3 meets
// at the buffer output with one transition, r1 -> r2 with the two (rise launch, fall at the
// buffer for r2's rising capture).
module top (clk, rst_n, d, q2, q3);
  input clk, rst_n, d;
  output q2, q3;
  wire c1, c2, n1;
  sky130_fd_sc_hd__buf_2 b0 (.A(clk), .X(c1));
  sky130_fd_sc_hd__inv_2 i0 (.A(c1), .Y(c2));
  sky130_fd_sc_hd__dfrtp_2 r1 (.CLK(c1), .D(d), .RESET_B(rst_n), .Q(n1));
  sky130_fd_sc_hd__dfrtp_2 r2 (.CLK(c2), .D(n1), .RESET_B(rst_n), .Q(q2));
  sky130_fd_sc_hd__dfrtp_2 r3 (.CLK(c1), .D(n1), .RESET_B(rst_n), .Q(q3));
endmodule
