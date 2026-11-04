create_clock -name clk_i -period 3 [get_ports {clk_i}]
set_input_delay 0.5 -clock clk_i [all_inputs]
set_output_delay 0.5 -clock clk_i [all_outputs]
set_max_delay -ignore_clock_latency 0.2 -from [get_pins {_1434_/Y}] -through [get_pins {_2000_/A1}] -to [get_pins {_2000_/X}]
