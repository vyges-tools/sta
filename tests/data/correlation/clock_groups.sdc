create_clock -name clk_i -period 2.5 [get_ports {clk_i}]
create_clock -name vclk -period 2.5
create_clock -name vclk2 -period 3
create_clock -name vclk3 -period 4
set_input_delay 0.4 -clock clk_i [all_inputs]
set_input_delay 0.3 -clock vclk2 [get_ports {tl_i[*]}] -add_delay
set_input_delay 0.2 -clock vclk3 [get_ports {intr_src_i[*]}] -add_delay
set_output_delay 0.6 -clock vclk [all_outputs]
set_output_delay 0.5 -clock clk_i [get_ports irq_o] -add_delay
set_output_delay 0.7 -clock vclk3 [get_ports {tl_o[*]}] -add_delay
set_clock_groups -asynchronous -group {clk_i} -group {vclk no_such_clock}
set_clock_groups -logically_exclusive -group [get_clocks vclk2]
set_clock_groups -name g -physically_exclusive -group {clk_i} -group {vclk3}
set_clock_groups -name g -asynchronous -allow_paths -group {clk_i} -group {vclk3}
set_clock_groups -asynchronous -group {vclk} -group {nope}
