create_clock -name clk -period 2 [get_ports clk]
set_input_delay 0.3 -clock clk [get_ports {rst_n a b}]
set_output_delay 0.4 -clock clk [get_ports {y z w f k}]
set_load 0.01 [get_ports {y z w f}]
