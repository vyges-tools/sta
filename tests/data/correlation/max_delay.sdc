create_clock -name clk_i -period 3 [get_ports {clk_i}]
set_input_delay 0.5 -clock clk_i [all_inputs]
set_output_delay 0.5 -clock clk_i [all_outputs]
set_max_delay -ignore_clock_latency 1.5 -from [get_pins {_1432_/Y}] -to [get_pins {_1433_/A}]
