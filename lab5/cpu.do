vlib work
vlog cpu.sv
#vlog rf_tb.sv
#vlog datapath_tb.sv
#vlog controlpath_tb.sv
vlog cpu_tb.sv
vsim -novopt tb
add wave /tb/*
add wave /tb/DUT/*
run -all