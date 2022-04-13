package cpu_pkg;
	typedef enum int unsigned {
		OP_NULL,

		// Arithmetic (R)
		OP_ADD,
		OP_SUB,
		OP_XOR,
		OP_OR,
		OP_AND,
		OP_SLL,
		OP_SRL,
		OP_SRA,
		OP_SLT,
		OP_SLTU,

		// Arithmetic with IMM (I)
		OP_ADDI,
		OP_XORI,
		OP_ORI,
		OP_ANDI,
		OP_SLLI,
		OP_SRLI,
		OP_SRAI,
		OP_SLTI,
		OP_SLTIU,

		// Load (I)
		OP_LB,
		OP_LH,
		OP_LW,
		OP_LBU,
		OP_LHU,

		// Store (S)
		OP_SB,
		OP_SH,
		OP_SW,

		// Upper IMM (U)
		OP_LUI,
		OP_AUIPC,

		// Branch (B)
		OP_BEQ,
		OP_BNE,
		OP_BLT,
		OP_BGE,
		OP_BLTU,
		OP_BGEU,

		// Jump (J)
		OP_JAL,
		OP_JALR
	} e_instr_type;

	typedef enum int unsigned {
		ALU_ADD,
		ALU_SUB,
		ALU_XOR,
		ALU_OR,
		ALU_AND,
		ALU_SLL,
		ALU_SRL,
		ALU_SRA,
		ALU_LT,
		ALU_LTU,
		ALU_EQ
	} e_alu_type;

	typedef enum bit[6:0] {
		R = 7'b0110011,
		I = 7'b0010011,
		L = 7'b0000011,
		S = 7'b0100011,
		LUI = 7'b0110111,
		AUIPC = 7'b0010111,
		B = 7'b1100011,
		J = 7'b1101111,
		JALR = 7'b1100111
	} e_op_code;    
endpackage

import cpu_pkg::*;

// Top-level Module
module cpu # (
	parameter IW = 32, // instr width
	parameter REGS = 32 // number of registers
)(
	input clk,						// clock
	input reset,					// active-high reset
	
	// read only port
	output [IW-1:0] o_pc_addr,  	// Expose PC address
	output o_pc_rd,					// Fetch instruction at PC addr
	input [IW-1:0] i_pc_rddata,		// Instruction data
	output [3:0] o_pc_byte_en,		// Word byte selection (4'b1111 for reading the whole word)
	
	// read/write port
	output [IW-1:0] o_ldst_addr,	// Load/Store Address
	output o_ldst_rd,				// Load/Store Read Enable
	output o_ldst_wr,				// Load/Store Write Enable
	input [IW-1:0] i_ldst_rddata,	// Data to read
	output [IW-1:0] o_ldst_wrdata,	// Data to write
	output [3:0] o_ldst_byte_en,	// Word byte selection (4'b1111 for reading the whole word)
	
	output [IW-1:0] o_tb_regs [0:REGS-1]	// All registers expose interface
);
	
	// Registers
	logic [31:0] stage1_pc, stage2_pc, stage3_pc, stage3_jump_pc; // Program counter

	// Proceed signals
	logic stage1_proceed, stage2_proceed, stage4_proceed, stage5_proceed;
	logic stage3_proceed_load, stage3_proceed_jump, stage3_proceed_memop;

	// Propagate RD and ALU regs
	logic [4:0] rs1, rs2, stage2_rd, stage3_rd, stage4_rd, stage5_rd; 	
	logic [31:0] imm;

	// Propagate Instruction types
	e_instr_type stage2_instr_type, stage3_instr_type, stage4_instr_type;
	e_op_code stage2_opcode;

	logic [31:0] rY, rY_PC, mem_rs2_reg;					// ALU outputs
	logic [4:0] mem_rs2;									// Code of stored register 

	logic stage4_stall_op;									// Stall operations in stage 1, 2, 3

	logic [31:0] stage5_rd_data;							// Write back value

	// Stage 1: Instruction Fetch
	stage1_instrfetch S1(
		.clk            (clk),
		.reset          (reset),
		.i_proceed_jump (stage3_proceed_jump),
		.i_jump_pc      (stage3_jump_pc),
		.i_stall_op		(stage4_stall_op),
		.o_pc           (stage1_pc),
		.o_pc_byte_en	(o_pc_byte_en),
		.o_proceed   	(stage1_proceed)
	);
	assign o_pc_addr = stage1_pc;
	assign o_pc_rd = stage1_proceed;

	// Stage 2: Decoder
	stage2_decoder S2(
		.clk            (clk),
		.reset          (reset | stage3_proceed_jump),
		.i_proceed		(stage1_proceed),
		.i_stall_op		(stage4_stall_op),
		.o_proceed      (stage2_proceed),

		.i_pc           (stage1_pc),
		.i_instr        (i_pc_rddata),
		.o_pc           (stage2_pc),
		.o_rs1			(rs1),
		.o_rs2			(rs2),
		.o_rd           (stage2_rd),
		.o_imm          (imm),
		.o_instr_type   (stage2_instr_type),
		.o_opcode    	(stage2_opcode)
	);

	// Stage 3: ALU
	stage3_alu S3(
		.clk       				(clk),
		.reset  				(reset),
		.i_proceed  			(stage2_proceed),
		.i_stall_op				(stage4_stall_op),
		.o_proceed_load   		(stage3_proceed_load),
		.o_proceed_jump  		(stage3_proceed_jump),
		.o_proceed_memop 		(stage3_proceed_memop),

		.i_pc 					(stage2_pc),
		.i_rs1 					(rs1),
		.i_rs2 					(rs2),
		.i_rd_prop 				(stage2_rd),
		.i_rd_load 				(stage5_rd),
		.i_rd_load_data 		(stage5_rd_data),
		.i_stage5_proceed 		(stage5_proceed),
		.i_imm 					(imm),
		.i_instr_type 			(stage2_instr_type),
		.i_opcode 				(stage2_opcode),
		.i_tb_regs 				(o_tb_regs),

		.o_pc 					(stage3_pc),
		.o_jump_pc       		(stage3_jump_pc),
		.o_rY 					(rY),
		.o_rY_PC 				(rY_PC),
		.o_rs2                  (mem_rs2),
		.o_rs2_reg				(mem_rs2_reg),
		.o_rd_prop 				(stage3_rd),
		.o_instr_type 			(stage3_instr_type)
	);

	// Stage 4: Load/Store Operations
	stage4_ldst S4(
		.clk 					(clk),
		.reset 					(reset),
		.i_proceed 				(stage3_proceed_memop),
		.o_proceed_store 		(o_ldst_wr),
		.o_proceed_load 		(o_ldst_rd),
		.o_stall_op 			(stage4_stall_op),

		.i_mem_addr 			(rY),
		.i_st_data_reg 			(mem_rs2),
		.i_st_data 				(mem_rs2_reg),
		.i_instr_type 			(stage3_instr_type),
		.i_rd_prop				(stage3_rd),
		.i_rd_load 				(stage5_rd),
		.i_rd_load_data 		(stage5_rd_data),
		.i_stage5_proceed 		(stage5_proceed),

		.o_mem_addr 			(o_ldst_addr),
		.o_instr_type 			(stage4_instr_type),
		.o_st_data 				(o_ldst_wrdata),
		.o_ldst_byte_en 		(o_ldst_byte_en),
		.o_rd_prop 				(stage4_rd)
	);

	// Stage 5: Write Back
	stage5_wb S5(
		.clk(clk),
		.reset(reset),
		.i_stage3_proceed(stage3_proceed_load),
		.i_stage4_proceed(o_ldst_rd),
		.o_proceed(stage5_proceed),

		.i_stage3_pc(stage3_pc),
		.i_rY(rY),
		.i_rY_PC(rY_PC),
		.i_load_data(i_ldst_rddata),
		.i_stage3_rd(stage3_rd),
		.i_stage4_rd(stage4_rd),
		.i_stage3_instr_type(stage3_instr_type),
		.i_stage4_instr_type(stage4_instr_type),

		.o_rd(stage5_rd),
		.o_rd_data(stage5_rd_data),
		.o_tb_regs(o_tb_regs)
	);

endmodule

module stage4_ldst (
	input logic clk, reset,

	input logic i_proceed,
	output logic o_proceed_store,
	output logic o_proceed_load,
	output logic o_stall_op,			// Stall the opeartions in Stage 1, 2, 3

	input logic [31:0] i_mem_addr,
	input logic [4:0] i_st_data_reg,	// The register code that stores the data to be stored
	input logic [31:0] i_st_data,		// The "old" value of the register data
	input e_instr_type i_instr_type,
	input logic [4:0] i_rd_prop,		// RD value being propagated
	input logic [4:0] i_rd_load,
	input logic [31:0] i_rd_load_data,
	input logic i_stage5_proceed,
	
	output logic [31:0] o_mem_addr,
	output e_instr_type o_instr_type,
	output logic [31:0] o_st_data,
	output logic [3:0] o_ldst_byte_en,
	output logic [4:0] o_rd_prop
);

	logic valid;
	logic [31:0] temp_st_data;
	logic [4:0] temp_st_data_reg;
	assign o_stall_op = valid & (o_proceed_store | o_proceed_load);

	always_ff @ (posedge clk) begin
		if (~reset & i_proceed) begin
			valid <= 1'b1;
			o_mem_addr <= i_mem_addr;
			o_instr_type <= i_instr_type;
			o_rd_prop <= i_rd_prop;
			temp_st_data <= i_st_data;
			temp_st_data_reg <= i_st_data_reg;
		end
		else begin
			valid <= 1'b0;
			o_mem_addr <= '0;
			o_instr_type <= OP_NULL;
			o_rd_prop <= '0;
			temp_st_data <= '0;
			temp_st_data_reg <= '0;
		end
	end

	/* Check if the stored value is being updated in the next cycle */
	always_comb begin
		if (i_stage5_proceed && (i_rd_load == temp_st_data_reg))
			o_st_data = i_rd_load_data;
		else
			o_st_data = temp_st_data;
	end

	always_comb begin
		case (o_instr_type)
			OP_LB, OP_LH, OP_LW, OP_LBU, OP_LHU: begin
				o_proceed_store = 1'b0;
				o_proceed_load = 1'b1;
			end
			OP_SB, OP_SH, OP_SW: begin
				o_proceed_store = 1'b1;
				o_proceed_load = 1'b0;
			end
			default: begin
				o_proceed_store = 1'b0;
				o_proceed_load = 1'b0;
			end
		endcase
	end

	always_comb begin
		case (o_instr_type)
			OP_LB, OP_LBU, OP_SB: o_ldst_byte_en = 4'b0001;
			OP_LH, OP_LHU, OP_SH: o_ldst_byte_en = 4'b0011;
			OP_LW, OP_SW: o_ldst_byte_en = 4'b1111;
			default: o_ldst_byte_en = 4'b1111;
		endcase
	end

endmodule

module stage3_alu (
	input logic clk, reset,

	input logic i_proceed,
	input logic i_stall_op,				// Stall operation
	output logic o_proceed_load,		// Load result register
	output logic o_proceed_jump,		// Change PC value and revert changes
	output logic o_proceed_memop,		// Load/store operation

	input logic [31:0] i_pc,			// Current instruction PC
	input logic [4:0] i_rs1, i_rs2,		// Decoded register codes
	input logic [4:0] i_rd_prop,		// Rd being propagated
	input logic [4:0] i_rd_load,		// Rd being loaded currently (ALU shortcut)
	input logic [31:0] i_rd_load_data,  // Data being loaded currently (ALU shortcut)
	input logic i_stage5_proceed,		// Stage 5 is active
	input logic [31:0] i_imm,			// Sign-extended immediate data
	input e_instr_type i_instr_type,	// Current instruction type
	input e_op_code i_opcode,			// Current instruction opcode
	input logic [31:0] i_tb_regs [0:31],// All registers interface

	output logic [31:0] o_pc,			// PC propagate
	output logic [31:0] o_jump_pc,		// New PC value to jump to
	output logic [31:0] o_rY,			// ALU output
	output logic [31:0] o_rY_PC,		// ALU PC + IMM output
	output logic [4:0] o_rs2,			// RS2
	output logic [31:0] o_rs2_reg,		// RS2 selected reg
	output logic [4:0] o_rd_prop,		// Rd being propagated
	output e_instr_type o_instr_type	// Propagate instruction type
);

	// Registers to store decode values
	logic [31:0] imm;
	logic [4:0] rs1, rs2;
	e_op_code opcode;
	logic valid;
	
	always_ff @ (posedge clk) begin
		if (reset | o_proceed_jump) begin
			imm <= '0;
			rs1 <= '0;
			rs2 <= '0;
			opcode <= R;
			o_pc <= '0;
			o_rd_prop <= '0;
			o_instr_type <= OP_NULL;
			valid <= 1'b0;
		end
		else if (i_proceed & ~i_stall_op) begin
			imm <= i_imm;
			rs1 <= i_rs1;
			rs2 <= i_rs2;
			opcode <= i_opcode;
			o_pc <= i_pc;
			o_rd_prop <= i_rd_prop;
			o_instr_type <= i_instr_type;
			valid <= 1'b1;
		end
		else begin // keep data
			imm <= imm;
			rs1 <= rs1;
			rs2 <= rs2;
			opcode <= opcode;
			o_pc <= o_pc;
			o_rd_prop <= o_rd_prop;
			o_instr_type <= o_instr_type;
			valid <= valid;					
		end
	end

	// ALU registers
	logic [31:0] ra, rb, rout, rs1_reg, rs2_reg;
	e_alu_type alu_type;

	// Instantiations
	cpu_alu A(ra, rb, alu_type, rout);
	reg_decoder R1(rs1, i_tb_regs, rs1_reg);
	reg_decoder R2(rs2, i_tb_regs, rs2_reg);

	// Determine ALU inputs
	always_comb begin
		if (reset)
			ra = '0;
		else if (i_stage5_proceed && (rs1 == i_rd_load))
			ra = i_rd_load_data;
		else
			ra = rs1_reg;
	end
	
	always_comb begin
		if (reset)
			rb = '0;
		else if (opcode == I || opcode == L || opcode == S || opcode == JALR)
			rb = imm;
		else if (i_stage5_proceed && (rs2 == i_rd_load))
			rb = i_rd_load_data;
		else
			rb = rs2_reg;
	end

	// Determine ALU op type
	always_comb begin
		alu_type = ALU_ADD;

		case (o_instr_type)
			OP_SUB: alu_type = ALU_SUB;
			OP_XOR, OP_XORI: alu_type = ALU_XOR;
			OP_OR, OP_ORI: alu_type = ALU_OR;
			OP_AND, OP_ANDI: alu_type = ALU_AND;
			OP_SLL, OP_SLLI: alu_type = ALU_SLL;
			OP_SRL, OP_SRLI: alu_type = ALU_SRL;
			OP_SRA, OP_SRAI: alu_type = ALU_SRA;
			OP_SLT, OP_SLTI, OP_BLT, OP_BGE: alu_type = ALU_LT;
			OP_SLTU, OP_SLTIU, OP_BLTU, OP_BGEU: alu_type = ALU_LTU;
			OP_BEQ, OP_BNE: alu_type = ALU_EQ;
		endcase
	end

	// Output adjustment
	assign o_rY_PC = o_pc + imm;
	always_comb begin
		case (o_instr_type)
			OP_LUI: o_rY = imm;
			OP_AUIPC: o_rY = o_rY_PC;
			OP_BGE, OP_BGEU, OP_BNE: o_rY = rout ? '0 : 32'd1; // invert
			OP_JAL, OP_JALR: o_rY = o_pc + 4;
			default: o_rY = rout;
		endcase			
	end
	
	// Proceed enable signals
	logic proceed_en;
	assign proceed_en = valid & ~i_stall_op;
	always_comb begin
		case (opcode)
			R, I, LUI, AUIPC: begin
				o_proceed_load = proceed_en;
				o_proceed_memop = 1'b0;
				o_proceed_jump = 1'b0;
				o_jump_pc = '0;
			end
			L, S: begin
				o_proceed_load = 1'b0;
				o_proceed_memop = proceed_en;
				o_proceed_jump = 1'b0;
				o_jump_pc = '0;
			end
			B: begin
				o_proceed_load = 1'b0;
				o_proceed_memop = 1'b0;
				o_proceed_jump = o_rY & proceed_en;
				o_jump_pc = o_rY_PC;
			end
			J: begin
				o_proceed_load = proceed_en;
				o_proceed_memop = 1'b0;
				o_proceed_jump = proceed_en;
				o_jump_pc = o_rY_PC;
			end
			JALR: begin
				o_proceed_load = proceed_en;
				o_proceed_memop = 1'b0;
				o_proceed_jump = proceed_en;
				o_jump_pc = rout;
			end
			default: begin
				o_proceed_memop = 1'b0;
				o_proceed_load = 1'b0;
				o_proceed_jump = 1'b0;
				o_jump_pc = o_rY_PC;
			end
		endcase
	end

	// Determine Stored Data
	assign o_rs2 = rs2;
	always_comb begin
		if (reset)
			o_rs2_reg = '0;
		else if (i_stage5_proceed && (rs2 == i_rd_load))
			o_rs2_reg = i_rd_load_data;
		else
			o_rs2_reg = rs2_reg;
	end

endmodule

module stage1_instrfetch (
	input logic clk, reset,

	input logic i_proceed_jump,				// Do a branch at the given address
	input logic [31:0] i_jump_pc,			// PC branch address
	input logic i_stall_op,					// Stall operation

	output logic [31:0] o_pc, 			    // PC address to be fetched
	output logic [3:0] o_pc_byte_en,		// Word byte selection (4'b1111 for reading the whole word)

	output logic o_proceed					// Fetch instruction?
);

	assign o_pc_byte_en = 4'b1111; 			// Always fetch the full instruction

	// If load/store is busy, do not fetch memory
	assign o_proceed = ~i_stall_op;

	always_ff @ (posedge clk) begin
		if (reset)
			o_pc <= '0;
		else if (i_proceed_jump)
			o_pc <= i_jump_pc;
		else if (~i_stall_op)
			o_pc <= o_pc + 4;
		else
			o_pc <= o_pc;
	end
	
endmodule


module stage2_decoder (
	input logic clk, reset,

	input logic i_proceed,						// Use the new instr data?
	input logic i_stall_op,						// Stall operation
	output logic o_proceed,						// Load ALU regs in the next state ?

	input logic [31:0] i_pc,					// PC propagate in
	input logic [31:0] i_instr,					// Instruction fetched from memory
	output logic [31:0] o_pc,					// PC propagate out
	output logic [4:0] o_rs1, o_rs2, o_rd,		// Register code
	output logic [31:0] o_imm,					// Sign-extended immediate data
	output e_instr_type o_instr_type,			// Decoded instruction type
	output e_op_code o_opcode					// Decoded opcode type
);

	// TODO: change this to account for mem
	logic valid, o_skip_op, last_was_stall, last_was_proceed;
	assign o_proceed = valid & ~i_stall_op;

	// Instruction register (in case that pipelining is stalled)
	logic [31:0] ir;
	logic [31:0] actual_instr;			// Actual instruction that is used
	assign actual_instr = last_was_stall ? ir : i_instr;
	always_ff @ (posedge clk) begin
		if (reset) begin
			ir <= '0;
			o_pc <= '0;
			valid <= 1'b0;
		end
		else if (i_proceed) begin
			o_pc <= i_pc;				// Current instruction PC
			valid <= 1'b1;
		end
		else begin // Keep data
			o_pc <= o_pc;
			valid <= valid;
		end
	end

	always_ff @ (posedge clk) begin
		if (reset) begin
			last_was_stall <= 1'b0;
			last_was_proceed <= 1'b0;
		end
		else begin
			last_was_stall <= i_stall_op;
			last_was_proceed <= i_proceed;
		end
	end

	always_ff @ (posedge clk) begin
		ir <= last_was_proceed ? i_instr : ir;				// Last instruction data
	end

	// Decoding wires
	logic [14:12] funct3;
	logic [31:25] funct7;
	logic imm_sign;

	always_comb begin
		case (actual_instr[6:0])
			7'b0110011: o_opcode = R;
			7'b0010011: o_opcode = I;
			7'b0000011: o_opcode = L;
			7'b0100011: o_opcode = S;
			7'b0110111: o_opcode = LUI;
			7'b0010111: o_opcode = AUIPC;
			7'b1100011: o_opcode = B;
			7'b1101111: o_opcode = J;
			7'b1100111: o_opcode = JALR;
		endcase
	end

	assign funct3 = actual_instr[14:12];
	assign funct7 = actual_instr[31:25];
	assign imm_sign = actual_instr[31];
	
	// Instruction decode patterns
	always_comb begin
		// defaults
		o_rs1 = '0;
		o_rs2 = '0;
		o_rd = '0;
		o_imm = '0;

		case (o_opcode)
			// R-type
			R: begin
				o_rs1 = actual_instr[19:15];
				o_rs2 = actual_instr[24:20];
				o_rd = actual_instr[11:7];
			end

			// I-type
			I, L, JALR: begin
				o_rs1 = actual_instr[19:15];
				o_rs2 = actual_instr[24:20];
				o_rd = actual_instr[11:7];
				o_imm = {{20{imm_sign}}, actual_instr[31:20]};
			end

			// S-type
			S: begin
				o_rs1 = actual_instr[19:15];
				o_rs2 = actual_instr[24:20];
				o_imm = {{20{imm_sign}}, actual_instr[31:25], actual_instr[11:7]};
			end

			// B-type
			B: begin
				o_rs1 = actual_instr[19:15];
				o_rs2 = actual_instr[24:20];
				o_imm = {{20{imm_sign}}, actual_instr[7], actual_instr[30:25], actual_instr[11:8], 1'b0};
			end

			// U-type
			LUI, AUIPC: begin
				o_rd = actual_instr[11:7];
				o_imm = {actual_instr[31:12], 12'd0};
			end

			// J-type
			J: begin
				o_rd = actual_instr[11:7];
				o_imm = {{12{imm_sign}}, actual_instr[19:12], actual_instr[20], actual_instr[30:21], 1'b0};
			end
		endcase
	end

	// Determine the next operation
	always_comb begin
		// defaults
		o_instr_type = OP_NULL;
		o_skip_op = 1'b0; 

		case (o_opcode)
			// Arithmetic Ops
			R: begin
				if (funct3 == 3'h0 && funct7 == 7'h00)
					o_instr_type = OP_ADD;
				else if (funct3 == 3'h0 && funct7 == 7'h20)
					o_instr_type = OP_SUB;
				else if (funct3 == 3'h4 && funct7 == 7'h00)
					o_instr_type = OP_XOR;
				else if (funct3 == 3'h6 && funct7 == 7'h00)
					o_instr_type = OP_OR;
				else if (funct3 == 3'h7 && funct7 == 7'h00)
					o_instr_type = OP_AND;
				else if (funct3 == 3'h1 && funct7 == 7'h00)
					o_instr_type = OP_SLL;
				else if (funct3 == 3'h5 && funct7 == 7'h00)
					o_instr_type = OP_SRL;
				else if (funct3 == 3'h5 && funct7 == 7'h20)
					o_instr_type = OP_SRA;
				else if (funct3 == 3'h2 && funct7 == 7'h00)
					o_instr_type = OP_SLT;
				else if (funct3 == 3'h3 && funct7 == 7'h00)
					o_instr_type = OP_SLTU;
				else
					o_skip_op = 1'b1; // Unknown operation
			end

			// Arithmetic Ops with IMM
			I: begin
				if (funct3 == 3'h0)
					o_instr_type = OP_ADDI;
				else if (funct3 == 3'h4)
					o_instr_type = OP_XORI;
				else if (funct3 == 3'h6)
					o_instr_type = OP_ORI;
				else if (funct3 == 3'h7)
					o_instr_type = OP_ANDI;
				else if (funct3 == 3'h1 && funct7 == 7'h00)
					o_instr_type = OP_SLLI;
				else if (funct3 == 3'h5 && funct7 == 7'h00)
					o_instr_type = OP_SRLI;
				else if (funct3 == 3'h5 && funct7 == 7'h20)
					o_instr_type = OP_SRAI;
				else if (funct3 == 3'h2)
					o_instr_type = OP_SLTI;
				else if (funct3 == 3'h3)
					o_instr_type = OP_SLTIU;
				else
					o_skip_op = 1'b1; // Unknown operation
			end

			// Load OPs
			L: begin
				if (funct3 == 3'h0)
					o_instr_type = OP_LB;
				else if (funct3 == 3'h1)
					o_instr_type = OP_LH;
				else if (funct3 == 3'h2)
					o_instr_type = OP_LW;
				else if (funct3 == 3'h4)
					o_instr_type = OP_LBU;
				else if (funct3 == 3'h5)
					o_instr_type = OP_LHU;
				else
					o_skip_op = 1'b1; // Unknown operation
			end

			// Store OPs
			S: begin
				if (funct3 == 3'h0)
					o_instr_type = OP_SB;
				else if (funct3 == 3'h1)
					o_instr_type = OP_SH;
				else if (funct3 == 3'h2)
					o_instr_type = OP_SW;
				else
					o_skip_op = 1'b1; // Unknown operation
			end

			// Upper Immediate Ops
			LUI: o_instr_type = OP_LUI;
			AUIPC: o_instr_type = OP_AUIPC;

			// Branch Ops
			B: begin
				if (funct3 == 3'h0)
					o_instr_type = OP_BEQ;
				else if (funct3 == 3'h1)
					o_instr_type = OP_BNE;
				else if (funct3 == 3'h4)
					o_instr_type = OP_BLT;
				else if (funct3 == 3'h5)
					o_instr_type = OP_BGE;
				else if (funct3 == 3'h6)
					o_instr_type = OP_BLTU;
				else if (funct3 == 3'h7)
					o_instr_type = OP_BGEU;
				else
					o_skip_op = 1'b1; // Unknown operation
			end

			// Jump Ops
			J: o_instr_type = OP_JAL;
			JALR: o_instr_type = OP_JALR;

			default:
				o_skip_op = 1'b1; // Unknown operation

		endcase
	end

endmodule

module stage5_wb(
	input logic clk, reset,

	input logic i_stage3_proceed,
	input logic i_stage4_proceed,
	output logic o_proceed,

	input logic [31:0] i_stage3_pc,			// Stage3 instruction PC
	input logic [31:0] i_rY,				// ALU output
	input logic [31:0] i_rY_PC,				// ALU PC + IMM output
	input logic [31:0] i_load_data,			// Load data
	input logic [4:0] i_stage3_rd,			// Stage 3 rd
	input logic [4:0] i_stage4_rd,			// Stage 4 rd
	input e_instr_type i_stage3_instr_type,	// Stage 3 instruction
	input e_instr_type i_stage4_instr_type,	// Stage 4 instruction

	output logic [4:0] o_rd,				// Register code being loaded
	output logic [31:0] o_rd_data,			// Value written back
	output logic [31:0] o_tb_regs [0:31]	// All registers interface
);

	logic stage3_proceed, stage4_proceed;
	logic [31:0] rY, rY_PC;
	e_instr_type instr_type;

	assign o_proceed = stage3_proceed | stage4_proceed;

	always_ff @ (posedge clk) begin
		if (~reset & (i_stage3_proceed | i_stage4_proceed)) begin
			o_rd <= i_stage3_proceed ? i_stage3_rd : i_stage4_rd;
			instr_type = i_stage3_proceed ? i_stage3_instr_type : i_stage4_instr_type;
			rY <= i_rY;
			rY_PC <= i_rY_PC;
			stage3_proceed <= i_stage3_proceed;
			stage4_proceed <= i_stage4_proceed;
		end
		else begin
			o_rd <= '0;
			instr_type <= OP_NULL;
			rY <= '0;
			rY_PC <= '0;
			stage3_proceed <= 1'b0;
			stage4_proceed <= 1'b0;
		end
	end

	// Determine data
	always_comb begin
		if (reset)
			o_rd_data = '0;
		else if (stage3_proceed)
			o_rd_data = rY;
		else 
			case (instr_type)
				OP_LB: o_rd_data = {{24{i_load_data[7]}}, i_load_data[7:0]};
				OP_LH: o_rd_data = {{16{i_load_data[15]}}, i_load_data[15:0]};
				OP_LW: o_rd_data = i_load_data;
				OP_LBU: o_rd_data = {24'd0, i_load_data[7:0]};
				OP_LHU: o_rd_data = {16'd0, i_load_data[15:0]};
				default: o_rd_data = i_load_data;
			endcase
	end

	// Registers
	integer i;
	always_ff @ (posedge clk) begin
		if (reset) begin
			for (i = 0; i < 32; i++)
				o_tb_regs[i] <= '0;
		end
		else if (o_proceed) begin
			o_tb_regs[0] <= '0; // Zero register
			for (i = 1; i < 32; i++)
				if (o_rd == i)
					o_tb_regs[i] <= o_rd_data;
		end
	end

endmodule

module reg_decoder (
	input logic [4:0] select,
	input logic [31:0] i_tb_regs [0:31],
	output logic [31:0] out_reg
);
	integer i;
	always_comb begin
		out_reg = 0;
		for (i = 0; i < 32; i++)
			if (select == i)
				out_reg = i_tb_regs[i];
	end

endmodule

module cpu_alu (
	input logic [31:0] ra, rb,
	input e_alu_type alu_type,
	output logic [31:0] rout
);
	always_comb begin
		case (alu_type)
			ALU_ADD: rout = ra + rb;
			ALU_SUB: rout = ra - rb;
			ALU_XOR: rout = ra ^ rb;
			ALU_OR: rout = ra | rb;
			ALU_AND: rout = ra & rb;
			ALU_SLL: rout = ra << rb[4:0];
			ALU_SRL: rout = ra >> rb[4:0];
			ALU_SRA: rout = $signed(ra) >>> rb[4:0];
			ALU_LT: rout = ($signed(ra) < $signed(rb)) ? 32'd1 : 32'd0;
			ALU_LTU: rout = (ra < rb) ? 32'd1 : 32'd0;
			ALU_EQ: rout = (ra == rb) ? 32'd1 : 32'd0;
			default: rout = '0;
		endcase
	end
endmodule





	