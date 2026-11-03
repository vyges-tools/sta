create_clock -name clk_i -period 2.5 [get_ports {clk_i}]
create_clock -name vclk -period 2.5
set_clock_transition 0.15 [all_clocks]
set_clock_transition -rise -max 0.25 [get_clocks clk_i]
set_clock_transition -fall -min 0.05 [get_clocks clk_i]
set_input_delay 0.4 -clock clk_i [all_inputs]
set_output_delay 0.6 -clock clk_i [all_outputs]
