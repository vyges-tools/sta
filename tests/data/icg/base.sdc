create_clock -name clk -period 10 [get_ports clk]
set_input_delay 0 -clock clk [get_ports {d1 d2}]
