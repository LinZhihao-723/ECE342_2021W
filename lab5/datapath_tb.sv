
// Used to test the multiplier row, which is actually an adder
module tb();
	logic clk;
	logic reset;
	logic rf_we;
	logic rf_in_sel;
	logic [4:0] addr_a;
	logic [4:0] addr_b;
	logic [4:0] addr_c;
	logic load_ra;
	logic load_rb;
	logic op1_sel;
	logic op2_sel;
	logic [2:0] alu_sel;
	logic load_rr;
	logic [1:0] mem_data_out_sel;
	logic load_rm;
	logic [1:0] ry_sel;
	logic load_ry;
	logic pc_sel;
	logic inc_sel;
	logic load_pc;
	logic load_pc_temp;
	logic [31:0] mem_data_in;
	logic [31:0] imm_rf;
	logic [31:0] imm_op2;
	logic [31:0] imm_pc;
	logic [31:0] imm_ry;
	logic  [31:0] rf_list_out [0:31];
	logic  [31:0] pc_addr_out;
	logic  [31:0] ldst_addr_out;
	logic  [31:0] mem_data_out;
	logic  Z;
	logic  N;
	logic  C;
	logic  V;
	

	data_path DUT(
		clk,
		reset,
		rf_we,
		rf_in_sel,
		addr_a,
		addr_b,
		addr_c,
		load_ra,
		load_rb,
		op1_sel,
		op2_sel,
		alu_sel,
		load_rr,
		mem_data_out_sel,
		load_rm,
		ry_sel,
		load_ry,
		pc_sel,
		inc_sel,
		load_pc,
		load_pc_temp,
		mem_data_in,
		imm_rf,
		imm_op2,
	    imm_pc,
	    imm_ry,
		rf_list_out,
	    pc_addr_out,
	    ldst_addr_out,
		mem_data_out,
		Z,
		N,
		C,
		V	
	);

	always begin
        #1 clk = !clk;
    end

	initial begin
		clk = 0;
		reset = 1;
		rf_we = 0;
		rf_in_sel = 0;
		addr_a = 0;
		addr_b = 0;
		addr_c = 0;
		load_ra = 0;
		load_rb = 0;
		op1_sel = 0;
		op2_sel = 0;
		alu_sel = 0;
		load_rr = 0;
		load_rm = 0;
		ry_sel = 0;
		load_ry = 0;
		pc_sel = 0;
		inc_sel = 0;
		load_pc = 0;
		load_pc_temp = 0;
		mem_data_in = 0;
		imm_rf = 0;
		imm_op2 = 0;
	    imm_pc = 0;
	    imm_ry = 0;
	    mem_data_out_sel = 0;
		#2;
		reset = 0;
		#2;

		/* Create a register file */
		for (integer i = 0; i < 32; i += 1) begin
			imm_rf = i;
			addr_c = i;
			rf_in_sel = 1;
			rf_we = 1;
			#2;
		end
		rf_we = 0;

		// # Value of tb: 00000000         0
		// # Value of tb: 00000001         1
		// # Value of tb: 00000002         2
		// # Value of tb: 00000003         3
		// # Value of tb: 00000004         4
		// # Value of tb: 00000005         5
		// # Value of tb: 00000006         6
		// # Value of tb: 00000007         7
		// # Value of tb: 00000008         8
		// # Value of tb: 00000009         9
		// # Value of tb: 0000000a        10
		// # Value of tb: 0000000b        11
		// # Value of tb: 0000000c        12
		// # Value of tb: 0000000d        13
		// # Value of tb: 0000000e        14
		// # Value of tb: 0000000f        15
		// # Value of tb: 00000010        16
		// # Value of tb: 00000011        17
		// # Value of tb: 00000012        18
		// # Value of tb: 00000013        19
		// # Value of tb: 00000014        20
		// # Value of tb: 00000015        21
		// # Value of tb: 00000016        22
		// # Value of tb: 00000017        23
		// # Value of tb: 00000018        24
		// # Value of tb: 00000019        25
		// # Value of tb: 0000001a        26
		// # Value of tb: 0000001b        27
		// # Value of tb: 0000001c        28
		// # Value of tb: 0000001d        29
		// # Value of tb: 0000001e        30
		// # Value of tb: 0000001f        31

		/* Print the rf content */
		for (integer i = 0; i < 32; i += 1) begin
			$display("Value of %i: %x", i, rf_list_out[i]);
		end

		/* ALU Test: start */
		// imm_rf = 32'h00000001;    // Test op1
		// addr_c = 1;
		// rf_in_sel = 1;
		// rf_we = 1;
		// #2;
		// imm_rf = 32'hffffffff;    // Test op2
		// addr_c = 2;
		// rf_in_sel = 1;
		// rf_we = 1;
		// #2;
		// addr_a = 1;
		// addr_b = 2;
		// load_ra = 1;
		// load_rb = 1;
		// #2
		// load_ra = 0;
		// load_rb = 0;
		// op1_sel = 0;
		// op2_sel = 0;
		// #2
		// alu_sel = 1;
		// load_rr = 1;
		// #2
		/* ALU Test: finish */

		/* RY ALU: start */
		// rf_we = 0;
		// addr_a = 1;
		// addr_b = 2;
		// load_ra = 1;
		// load_rb = 1;
		// #2;
		// load_ra = 0;
		// load_rb = 0;

		// op1_sel = 0;
		// op2_sel = 0;
		// alu_sel = 0;
		// load_rr = 1;
		// #2;
		// load_rr = 0;

		// ry_sel = 0;
		// load_ry = 1;
		// #2;
		// load_ry = 0;

		// rf_in_sel = 0;
		// addr_c = 31;
		// rf_we = 1;
		// #2;
		// $display("Reg31: %x", rf_list_out[31]);

		// rf_we = 0;
		// addr_a = 31;
		// addr_b = 1;
		// load_ra = 1;
		// load_rb = 1;
		// #2;
		// load_ra = 0;
		// load_rb = 0;

		// op1_sel = 0;
		// op2_sel = 0;
		// alu_sel = 1;
		// load_rr = 1;
		// #2;
		// load_rr = 0;

		// ry_sel = 0;
		// load_ry = 1;
		// #2;
		// load_ry = 0;

		// rf_in_sel = 0;
		// addr_c = 31;
		// rf_we = 1;
		// #2;
		// $display("Reg31: %x", rf_list_out[31]);
		/* RY ALU: finish */

		/* PC test: begin */
		/*Case 1*/
		// inc_sel = 0;
		// pc_sel = 1;
		// load_pc = 1;
		// #2;
		// load_pc_temp = 1;
		// #2;
		// load_ry = 1;
		// ry_sel = 2;
		// #2;
		// rf_in_sel = 0;
		// addr_c = 31;
		// rf_we = 1;
		// #2;
		// $display("Reg31: %x", rf_list_out[31]);
		/*Case 2*/
		// rf_we = 0;
		// addr_a = 28;
		// addr_b = 2;
		// load_ra = 1;
		// load_rb = 1;
		// #2;
		// pc_sel = 0;
		// load_pc = 1;
		// #2;

		/* PC test: finish */

		/* Memory out test: start */
		// imm_rf = 32'hffffffff; 
		// addr_c = 2;
		// rf_in_sel = 1;
		// rf_we = 1;
		// #2;
		// addr_a = 1;
		// addr_b = 2;
		// load_ra = 1;
		// load_rb = 1;
		// #2
		// load_rm = 1;
		// #2
		// mem_data_out_sel = 2;
		// #2
		// $display("Memory out: %x", mem_data_out);
		/* Memory out test: finish */
		
		$stop();
	end
endmodule

module data_path_prototype(
	input clk,
	input reset,

	/* Control input signals */
	input rf_we,
	input rf_in_sel,
	input [4:0] addr_a,
	input [4:0] addr_b,
	input [4:0] addr_c,

	input load_ra,
	input load_rb,

	input op1_sel,
	input op2_sel,

	input [2:0] alu_sel,

	input load_rr,

	input [1:0] mem_data_out_sel,
	input load_rm,

	input [1:0] ry_sel,
	input load_ry,
	
	input pc_sel,
	input inc_sel,
	input load_pc,
	input load_pc_temp,

	/* Memory input */
	input [31:0] mem_data_in,

	/* Control input data */
	input [31:0] imm_rf,
	input [31:0] imm_op2,
	input [31:0] imm_pc,
	input [31:0] imm_ry,

	/* Output */
	output logic [31:0][31:0] rf_list_out,
	output logic [31:0] pc_addr_out,
	output logic [31:0] ldst_addr_out,
	output logic [31:0] mem_data_out,
	output logic Z,
	output logic N,
	output logic C,
	output logic V
);

endmodule