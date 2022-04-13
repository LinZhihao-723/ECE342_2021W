vlib work
vlog cpu_1.sv
vlog tb.sv
vsim -novopt tb

add wave /tb/dut/*

#add wave /tb/dut/stage_2/i_ex_inform.pc
#add wave /tb/dut/stage_2/i_ex_inform.ra_addr
#add wave /tb/dut/stage_2/i_ex_inform.rb_addr
#add wave /tb/dut/stage_2/ra_addr
#add wave /tb/dut/stage_2/rb_addr
#add wave /tb/dut/stage_2/ra_data
#add wave /tb/dut/stage_2/rb_data
#add wave /tb/dut/stage_2/reg_1
#add wave /tb/dut/stage_2/reg_2
#add wave /tb/dut/stage_2/op1
#add wave /tb/dut/stage_2/op2
#add wave /tb/dut/stage_2/alu_result

run -all


