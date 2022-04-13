vlib work
vlog part1.sv
vlog part1tb.sv
vsim -novopt tb
add wave /tb/*
add wave /tb/uut/*
add wave /tb/uut/cpu_chip/CP/current_state
add wave /tb/uut/cpu_chip/CP/ldst_data_in
run -all