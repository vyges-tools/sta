// ICGs with constant enables: g1 ENA=1 SE=0 (always on), g2 ENA=0 SE=0 (always off).
module top (clk, d1, d2, q1, q2);
  input clk, d1, d2;
  output q1, q2;
  wire hi, lo, gclk1, gclk2;
  TIEHI th (.H(hi));
  TIELO tl (.L(lo));
  ICG g1 (.CLK(clk), .ENA(hi), .SE(lo), .GCLK(gclk1));
  ICG g2 (.CLK(clk), .ENA(lo), .SE(lo), .GCLK(gclk2));
  DFF r1 (.CLK(gclk1), .D(d1), .Q(q1));
  DFF r2 (.CLK(gclk2), .D(d2), .Q(q2));
endmodule
