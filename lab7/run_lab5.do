vlib work
vlog cpu.sv
vlog tb.sv
vsim -novopt tb
add wave /tb/*
add wave /tb/dut/*
add wave /tb/dut/stage_3/*
run -all


