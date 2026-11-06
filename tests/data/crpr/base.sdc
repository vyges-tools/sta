create_clock -name clk -period 2 [get_ports clk]
set_propagated_clock [get_clocks clk]
set_input_delay 0 -clock clk [get_ports {d rst_n}]
set_output_delay 0 -clock clk [get_ports {q2 q3}]
set_timing_derate -early 0.9
set_timing_derate -late 1.1
