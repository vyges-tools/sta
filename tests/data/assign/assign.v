// assign aliases at the top, each form the loader merges as the reference's mergeAssignNet does
module assign_top (clk, rst_n, a, b, y, z, w, f, k);
  input clk, rst_n, a, b;
  output y, z, w, f, k;
  wire n1, n2, n3, n4, n5, n6, q1;
  sky130_fd_sc_hd__inv_2 u0 (.A(a), .Y(n1));
  sky130_fd_sc_hd__dfrtp_2 r1 (.CLK(clk), .D(n1), .RESET_B(rst_n), .Q(q1));
  // an output port named by the net that drives it: the port's net merges into q1
  assign y = q1;
  sky130_fd_sc_hd__nand2_2 u1 (.A(q1), .B(n4), .Y(n2));
  // a wire that is another name for an input port: n4 merges into b
  assign n4 = b;
  sky130_fd_sc_hd__buf_2 u2 (.A(n2), .X(n3));
  // two output ports on one net
  assign z = n3, w = n3;
  sky130_fd_sc_hd__buf_2 u3 (.A(n3), .X(n6));
  // loads on both names of one internal net, before and after the assign
  sky130_fd_sc_hd__inv_2 u4 (.A(n5), .Y());
  sky130_fd_sc_hd__inv_2 u7 (.A(n5), .Y());
  assign n5 = n6;
  sky130_fd_sc_hd__inv_2 u5 (.A(n5), .Y());
  sky130_fd_sc_hd__dfrtp_2 r2 (.CLK(clk), .D(n6), .RESET_B(rst_n), .Q());
  sky130_fd_sc_hd__dfrtp_2 r3 (.CLK(clk), .D(n5), .RESET_B(rst_n), .Q());
  // a port fed straight through, and a port tied low
  assign f = a;
  assign k = 1'b0;
endmodule
