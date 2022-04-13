vlib work
vlog cpu_1.sv
vlog bpbtb.sv
vsim -novopt tb
add wave /tb/*
add wave /tb/dut/*
add wave /tb/dut/bpb_pc
add wave /tb/dut/bpb_target_pc
add wave /tb/dut/bpb_state
add wave /tb/dut/bpb_valid
run -all
