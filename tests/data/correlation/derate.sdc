create_clock -name clk_i -period 2.5 [get_ports {clk_i}]
set_input_delay 0.4 -clock clk_i [all_inputs]
set_output_delay 0.6 -clock clk_i [all_outputs]
set_timing_derate -early 0.95
set_timing_derate -late 1.05
set_timing_derate -late -cell_delay -rise 1.08
set_timing_derate -early -net_delay 0.9
set_timing_derate -late -cell_check 1.1
set_timing_derate -early -cell_check -fall 0.8
set_timing_derate -late -data -fall 1.03
set_timing_derate -late -clock 1.2
