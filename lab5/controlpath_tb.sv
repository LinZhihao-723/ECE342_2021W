
// Used to test the multiplier row, which is actually an adder
module tb();
	logic clk;
	logic reset;
	logic Z;
	logic N;
	logic C;
	logic V;	
	logic [31:0] pc_data_in;
	logic [31:0] ldst_data_in;
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
	logic pc_rd;
	logic [3:0] pc_byte_en;
	logic ldst_rd;
	logic ldst_wr;
	logic [3:0] ldst_byte_en;
	

	control_path DUT(
	 clk,
	 reset,

	/* Control inputs */
	 Z,
	 N,
	 C,
	 V,
	 pc_data_in,
	 ldst_data_in,

	/* Control output signals for data path */
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

	/* Output data to the data path */
	mem_data_in,
	imm_rf,
	imm_op2,
	imm_pc,
	imm_ry,

	/* Output to the memory interface */
	pc_rd,
	pc_byte_en,
	ldst_rd,
	ldst_wr,
	ldst_byte_en
	);

	always begin
        #1 clk = !clk;
    end

	initial begin
		clk = 0;
		Z = 0;
	 	N = 0;
	 	C = 0;
	 	V = 0;
	 	reset = 0;
	 	ldst_data_in = 0;


	 	/* Basic arithematic test: begin */
	 	// 	@00000000
		// 33 8e ee 01 	// 0x00000000: add t3, t4, t5
		// 33 89 49 41 	// 0x00000004: sub s2, s3, s4
		// 33 c9 49 01 	// 0x00000008: xor s2, s3, s4
		// 33 e9 49 01 	// 0x0000000c: or s2, s3, s4
		// 33 f9 49 01 	// 0x00000010: and s2, s3, s4
		// 33 99 49 01 	// 0x00000014: sll s2, s3, s4
		// 33 d9 49 01 	// 0x00000018: srl s2, s3, s4
		// 33 d9 49 41 	// 0x0000001c: sra s2, s3, s4
		// 33 a9 49 01 	// 0x00000020: slt s2, s3, s4
		// 33 b9 49 01 	// 0x00000024: sltu s2, s3, s4
	 	
	 	pc_data_in = 32'h01ee8e33;
		#10;
		pc_data_in = 32'h41498933;
		#10;
		pc_data_in = 32'h0149c933;
		#10;
		pc_data_in = 32'h0149e933;
		#10;
		pc_data_in = 32'h0149f933;
		#10;
		pc_data_in = 32'h01499933;
		#10;
		pc_data_in = 32'h0149d933;
		#10;
		pc_data_in = 32'h4149d933;
		#10;
		pc_data_in = 32'h4149a933;
		#6;
		N = 1;
		#2;
		N = 0;
		#2;
		pc_data_in = 32'h4149a933;
		#10;
		pc_data_in = 32'h4149b933;
		#6;
		C = 1;
		#2;
		C = 0;
		#2;
		pc_data_in = 32'h4149b933;
		#10;
		/* Basic arithematic test: end */
		
		$stop();
	end
endmodule

module control_path_prototype(
	input clk,
	input reset,

	/* Control inputs */
	input Z,
	input N,
	input C,
	input V,
	input [31:0] pc_data_in,
	input [31:0] ldst_data_in,

	/* Control output signals for data path */
	output logic rf_we,
	output logic rf_in_sel,
	output logic [4:0] addr_a,
	output logic [4:0] addr_b,
	output logic [4:0] addr_c,
	output logic load_ra,
	output logic load_rb,
	output logic op1_sel,
	output logic op2_sel,
	output logic [2:0] alu_sel,
	output logic load_rr,
	output logic [1:0] mem_data_out_sel,
	output logic load_rm,
	output logic [1:0] ry_sel,
	output logic load_ry,
	output logic pc_sel,
	output logic inc_sel,
	output logic load_pc,
	output logic load_pc_temp,

	/* Output data to the data path */
	output logic [31:0] mem_data_in,
	output logic [31:0] imm_rf,
	output logic [31:0] imm_op2,
	output logic [31:0] imm_pc,
	output logic [31:0] imm_ry,

	/* Output to the memory interface */
	output logic pc_rd,
	output logic [3:0] pc_byte_en,
	output logic ldst_rd,
	output logic ldst_wr,
	output logic [3:0] ldst_byte_en
);
	localparam	INS_ALU_R = 7'b0110011;
	localparam	INS_ALU_IMM = 7'b0010011;
	localparam	INS_LD = 7'b0000011;
	localparam	INS_ST = 7'b0100011;
	localparam	INS_LD_UPPER_IMM = 7'b0110111;
	localparam	INS_ADD_UPPER_IMM_PC = 7'b0010111;
	localparam	INS_BRANCH = 7'b1100011;
	localparam	INS_JP_LK = 7'b1101111;
	localparam	INS_JP_LK_R = 7'b1100111;

endmodule

	// case(current_state)
 //    		STAGE_0: begin
 //    			// Load from PC
 //    			pc_rd = 1;
 //    			pc_byte_en = WORD_ACCESS;
 //    			load_ir = 1;
 //    			next_state = STAGE_1;
 //    		end
 //    		STAGE_1: begin
 //    			case(opcode)
 //    				INS_ALU_R: begin
 //    					addr_a = rs1;
 //    					addr_b = rs2;
 //    					load_ra = 1;
 //    					load_rb = 1;
 //    					next_state = STAGE_2;
 //    				end
 //    				INS_ALU_IMM: begin
 //    					addr_a = rs1;
 //    					load_ra = 1;
 //    					next_state = STAGE_2;
 //    				end
 //    				INS_LD: begin
 //    					addr_a = rs1;
 //    					load_ra = 1;
 //    					next_state = STAGE_2;
 //    				end
 //    				INS_ST: begin
 //    					addr_a = rs1;
 //    					addr_b = rs2;
 //    					load_ra = 1;
 //    					load_rb = 1;
 //    					next_state = STAGE_2;
 //    				end
 //    				INS_LD_UPPER_IMM: begin
 //    					addr_c = rd;
 //    					rf_in_sel = 1;
 //    					imm_rf = immediate_data;
 //    					rf_we = 1;
 //    					next_state = STAGE_0;
 //    				end
 //    				INS_ADD_UPPER_IMM_PC: begin
 //    					imm_pc = immediate_data;
 //    					inc_sel = 1;
 //    					pc_sel = 1;
 //    					load_pc = 1;
 //    					next_state = STAGE_0;
 //    				end
 //    				INS_BRANCH: begin
 //    					addr_a = rs1;
 //    					addr_b = rs2;
 //    					load_ra = 1;
 //    					load_rb = 1;
 //    					next_state = STAGE_2;
 //    				end
 //    				INS_JP_LK: begin
 //    					op1_sel = 1;
 //    					op2_sel = 1;
 //    					imm_op2 = 32'd4;
 //    					alu_sel = 3'd0;
 //    					load_rr = 1;
 //    					next_state = STAGE_2;
 //    				end
 //    				INS_JP_LK_R: begina
 //    					addr_a = r1;
 //    					load_ra = 1;
 //    					inc_sel = 0;
 //    					pc_sel = 1;
 //    					load_pc = 1;
 //    					next_state = STAGE_2;
 //    				end
 //    			endcase
 //    		end
 //    	endcase

// 3'd0: {alu_flags[2], alu_result} = op1 +   op2;       // Add (Checked)
// 3'd1: {alu_flags[2], alu_result} = op1 -   op2;       // Sub (Checked)
// 3'd2: {alu_flags[2], alu_result} = op1 ^   op2;       // Xor (Checked)
// 3'd3: {alu_flags[2], alu_result} = op1 |   op2;       // Or  (Checked)
// 3'd4: {alu_flags[2], alu_result} = op1 &   op2;       // And (Checked)
// 3'd5: {alu_flags[2], alu_result} = op1 <<  op2[4:0];  // Sll (Checked)
// 3'd6: {alu_flags[2], alu_result} = op1 >>  op2[4:0];  // Srl (Checked)
// 3'd7: {alu_flags[2], alu_result} = $signed(op1) >>> (op2[4:0]);  // Sra