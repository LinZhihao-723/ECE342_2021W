vlib work
vlog cpu_1.sv
vlog part2tb.sv
vsim -novopt tb
add wave /tb/*
add wave /tb/dut/*

#add wave /tb/dut/stage_2/i_ex_inform.pc
#add wave /tb/dut/stage_2/i_ex_inform.ins_type
#add wave /tb/dut/stage_2/o_pc_imm
#add wave /tb/dut/stage_2/o_ex_flush

#add wave /tb/dut/stage_2/op1
#add wave /tb/dut/stage_2/op2
#add wave /tb/dut/stage_2/alu_result 
#add wave /tb/dut/stage_2/EQ 
#add wave /tb/dut/stage_2/LT 
#add wave /tb/dut/stage_2/LTU 

#add wave /tb/dut/stage_3/*

run -all
