create_clock -name clk_i -period 2.5 [get_ports {clk_i}]
create_clock -name vclk -period 2.5
set_clock_uncertainty 0.3 [all_clocks]
set_clock_uncertainty -setup 0.125 [get_clocks clk_i]
set_clock_uncertainty -hold 0.03 [all_clocks]
set_clock_uncertainty -setup 0.2 -from [get_clocks clk_i] -to [get_clocks vclk]
set_input_delay 0.4 -clock clk_i [all_inputs]
set_output_delay 0.6 -clock vclk [all_outputs]
