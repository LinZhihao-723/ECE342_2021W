vlib work
vlog part3.sv
vlog part3tb.sv

vsim -novopt tb_wallace_tree_multiplier_8_bit
add wave /tb_wallace_tree_multiplier_8_bit/DUT/*
#vsim -novopt tb_wallace_tree_row
run -all

