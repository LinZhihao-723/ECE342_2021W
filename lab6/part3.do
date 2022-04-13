vlib work
vlog part3.sv
vlog part3tb.sv
vsim -L work -L altera_mf_ver -L lpm_ver tb
add wave /tb/*
add wave /tb/uut/*
add wave /tb/uut/cpu_chip/CP/current_state
add wave /tb/uut/cpu_chip/o_tb_regs
run -all