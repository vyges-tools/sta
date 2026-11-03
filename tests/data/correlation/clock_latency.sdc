create_clock -name clk_i -period 2.5 [get_ports {clk_i}]
create_clock -name vclk -period 2.5
set_clock_latency 0.3 [get_clocks clk_i]
set_clock_latency -max -rise 0.45 [get_clocks clk_i]
set_clock_latency -min 0.1 [all_clocks]
set_clock_latency 0.2 [get_pins {_3116_/CLK}]
set_input_delay 0.4 -clock vclk [all_inputs]
set_input_delay 0.5 -clock clk_i [get_ports {tl_i[*]}] -add_delay
set_output_delay 0.6 -clock vclk [all_outputs]
set_output_delay 0.3 -clock clk_i [get_ports irq_o] -add_delay
