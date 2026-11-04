create_clock -name clk_i -period 50 [get_ports {clk_i}]
set_input_delay 10.0 -clock clk_i [all_inputs]
set_output_delay 10.0 -clock clk_i [all_outputs]
set_max_fanout 10 [current_design]
