vlib work
vlog cpu_lzh.sv
#vlog rf_tb.sv
#vlog datapath_tb.sv
#vlog controlpath_tb.sv
vlog tb.sv
vsim -novopt tb
add wave /tb/*
add wave /tb/dut/*
add wave /tb/dut/DP/*
run -all


