vlib work
vlog cpu.sv
vlog tb.sv
vsim -novopt tb

add wave /tb/dut/*
run -all


