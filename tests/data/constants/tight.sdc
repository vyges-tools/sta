create_clock -name clk_i -period 2.5 [get_ports {clk_i}]
set_input_delay 0.4 -clock clk_i [all_inputs]
set_output_delay 0.6 -clock clk_i [all_outputs]
