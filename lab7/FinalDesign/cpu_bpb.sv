/* Define package */

package cpu_encoding;
	typedef enum int unsigned {
		NONE,
		// R
		ADD, SUB, XOR, OR, AND, SLL, SRL, SRA, SLT, SLTU,
		// I
		ADDI, XORI, ORI, ANDI, SLLI, SRLI, SRAI, SLTI, SLTUI,
		// LOAD
		LB, LH, LW, LBU, LHU,
		// STORE
		SB, SH, SW,
		// U
		LUI, AUIPC,
		// B
		BEQ, BNE, BLT, BGE, BLTU, BGEU,
		// JUMP
		JAL, JALR
	} ins_encoding;

	typedef struct {
		logic [31:0] pc;
		logic flush;
		logic branch_taken;
	} if_id_reg;

	typedef struct {
		ins_encoding ins_type;
		logic [4:0] rd;
		logic [31:0] reg_a;
		logic [31:0] reg_b;
		logic [4:0] ra_addr;
		logic [4:0] rb_addr;
		logic [31:0] imm_data;
		logic [31:0] pc;
		logic branch_taken;
	} id_ex_reg;

    typedef struct {
		ins_encoding ins_type;
		logic [4:0] rd;
		logic [31:0] alu_result;
		logic [31:0] pc;
	} ex_wb_reg;

	typedef struct {
		logic [4:0] rd;
		logic [31:0] wb_data;
	} rf_forwarding;

	typedef struct {
		logic [31:0] branch_target_pc;
		logic [31:0] pc;
		logic valid;
		logic branch_taken;     /* 0: not taken; 1: taken;*/
		logic jump;             /* 100% taken */
	} bpb_inform;

	typedef struct {
		logic [31:0] target_pc;
		logic branch_taken;
		logic valid;
	} bpb_prediction;
endpackage

import cpu_encoding::*;

/************************************ BPB Implementation *************************************/

module Branch_Prediction_Buffer #(
	parameter NUM_ENTRY = 32
)(
	input clk,
	input reset,
	input [31:0] pc_stage_0,
	input bpb_inform i_updates,
	output bpb_prediction o_prediction
);
	logic [$clog2(NUM_ENTRY)-1:0] index;
	logic [$clog2(NUM_ENTRY)-1:0] update_index;
	// logic bpb_entry cache [0:63]; /* 65-34: pc; 33-2: pc_predict; 1-0: state */
	logic [31:0] bpb_pc [0:NUM_ENTRY-1];
	logic [31:0] bpb_target_pc [0:NUM_ENTRY-1];
	logic [1:0] bpb_state [0:NUM_ENTRY-1];
	logic bpb_valid [0:NUM_ENTRY-1];
	logic found;

	localparam not_taken_1 = 2'b00;
	localparam not_taken_2 = 2'b01;
	localparam taken_1 = 2'b10;
	localparam taken_2 = 2'b11;

	/* Locate the index */
	integer j;
	always@(*) begin
		found = 0;
		// update_index = 0;
		o_prediction.valid = 0;
		// o_prediction.target_pc = '0;
		// o_prediction.branch_taken = 0; 
		for(j = 0; j < NUM_ENTRY; j ++) begin
			/* Give a prediction */
			if(bpb_valid[j] && bpb_pc[j] == pc_stage_0) begin
				o_prediction.valid = 1;
				if(bpb_state[j] == taken_1 || bpb_state[j] == taken_2) begin
					o_prediction.target_pc = bpb_target_pc[j];
					o_prediction.branch_taken = 1;
				end else begin
					o_prediction.target_pc = pc_stage_0 + 4;
					o_prediction.branch_taken = 0; 
				end
			end

			/* Find if we have the slot */
			if(i_updates.valid && (bpb_pc[j] == i_updates.pc)) begin
				found = 1;
				update_index = j;
			end
		end
	end

	/* Update state */
	integer i;
	always_ff @(posedge clk) begin
		if(reset) begin
			/* Initialize */
			index <= '0;
			for(i = 0; i < NUM_ENTRY; i ++) begin
				bpb_pc[i] <= 0;
				bpb_target_pc[i] <= 0;
				bpb_state[i] <= 0;
				bpb_valid[i] <= 0;
			end 
		end else if(i_updates.valid) begin
			if(found) begin
				case(bpb_state[update_index])
					not_taken_1: bpb_state[update_index] <= (i_updates.branch_taken) ? not_taken_2 : not_taken_1;
					not_taken_2: bpb_state[update_index] <= (i_updates.branch_taken) ? taken_1 : not_taken_1;
					taken_1: bpb_state[update_index] <= (i_updates.branch_taken) ? taken_1 : taken_2;
					taken_2: bpb_state[update_index] <= (i_updates.branch_taken) ? taken_1 : not_taken_1;
					default: bpb_state[update_index] <= '0;
				endcase
			end else begin
				bpb_pc[index] <= i_updates.pc;
				bpb_target_pc[index] <= i_updates.branch_target_pc;
				bpb_valid[index] <= 1;
				if(i_updates.jump) begin
					bpb_state[index] <= taken_1;
				end else if(i_updates.branch_taken) begin
					bpb_state[index] <= not_taken_2;
				end else begin
					bpb_state[index] <= not_taken_1;
				end
				index <= index + 1;
			end
		end
	end
endmodule

/************************************ CPU Implementation *************************************/

module cpu # (
	parameter IW = 32, // instr width
	parameter REGS = 32 // number of registers
)(
	input clk,
	input reset,
	
	// read only port
	output [IW-1:0] o_pc_addr,
	output o_pc_rd,
	input [IW-1:0] i_pc_rddata,
	output [3:0] o_pc_byte_en,
	
	// read/write port
	output [IW-1:0] o_ldst_addr,
	output o_ldst_rd,
	output o_ldst_wr,
	input [IW-1:0] i_ldst_rddata,
	output [IW-1:0] o_ldst_wrdata,
	output [3:0] o_ldst_byte_en,
	
	output [IW-1:0] o_tb_regs [0:REGS-1]
);
	
	logic [31:0] pc_imm;
	logic ex_flush; // This signal should be send by stage 3.

	if_id_reg id_inform;

	bpb_inform bpb_updates;
	bpb_prediction bpb_predict;

	Branch_Prediction_Buffer bpb(
		.clk(clk),
		.reset(reset),
		.pc_stage_0(o_pc_addr),
		.i_updates(bpb_updates),
		.o_prediction(bpb_predict)
	);

	stage_0_IF stage_0 (
		.clk(clk),
		.reset(reset),
		.i_pc_imm(pc_imm),
		.i_flush(ex_flush),
		.i_bpb_predict(bpb_predict),
		.o_id_inform(id_inform),
		.o_pc_addr(o_pc_addr),
		.o_pc_rd(o_pc_rd)
	);

	assign o_pc_byte_en = 4'b1111;

	logic [4:0] address_a_in;
	logic [4:0] address_b_in;
	logic [4:0] address_c_in;
	logic [31:0] rf_data_in;
	logic rf_write_in;
	logic [31:0] reg_a_data;
	logic [31:0] reg_b_data;

	register_file rf(
		.clk(clk),
		.reset(reset),
		.address_a_in(address_a_in),
		.address_b_in(address_b_in),
		.address_c_in(address_c_in),
		.data_in(rf_data_in),
		.rf_write_in(rf_write_in),
		.rf_list_out(o_tb_regs),
		.reg_a_out(reg_a_data),
		.reg_b_out(reg_b_data)
	);

	id_ex_reg ex_inform;

	stage_1_ID stage_1 (
		.clk(clk),
		.reset(reset),
		.i_pc_rddata(i_pc_rddata),
		.i_id_inform(id_inform),
		.i_flush(ex_flush),
		.ra_data(reg_a_data),
		.rb_data(reg_b_data),
		.ra_addr(address_a_in),
		.rb_addr(address_b_in),
		.o_ex_inform(ex_inform)
	);

	ex_wb_reg wb_inform;
	rf_forwarding forwarding_control;

	stage_2_EX stage_2 (
		.clk(clk),
		.reset(reset),
		.i_ex_inform(ex_inform),
		.i_forwarding_control(forwarding_control),
		.o_ldst_addr(o_ldst_addr),
		.o_ldst_rd(o_ldst_rd),
		.o_ldst_wr(o_ldst_wr),
		.o_ldst_wrdata(o_ldst_wrdata),
		.o_ldst_byte_en(o_ldst_byte_en),
		.o_pc_imm(pc_imm),
		.o_ex_flush(ex_flush),
		.o_wb_inform(wb_inform),
		.o_bpb_updates(bpb_updates)
	);

	stage_3_WB stage_3 (
		.i_wb_inform(wb_inform),
		.i_ldst_rddata(i_ldst_rddata),
		.o_rf_write_data(rf_data_in),
		.o_rf_write(rf_write_in),
		.rc_addr(address_c_in),
		.o_forwarding_control(forwarding_control)
	);
endmodule

/* The register file. */
/*
	Note that this register file will write in the first half of cycle,
	and reading data should be sampled at the second half of cycle.
	The acutual writing happens at the negtive clock edge.
 */
module register_file(
	input clk,
	input reset,

	// Reading address
	input [4:0] address_a_in,
	input [4:0] address_b_in,

	// Writing address
	input [4:0] address_c_in,
	input [31:0] data_in,
	input rf_write_in,

	// Outputs
	output logic [31:0] rf_list_out [0:31],
	output logic [31:0] reg_a_out,
	output logic [31:0] reg_b_out
);
	assign reg_a_out = rf_list_out[address_a_in][31:0];
	assign reg_b_out = rf_list_out[address_b_in][31:0];

	integer i;
	always_ff @(negedge clk) begin
		if(reset) begin
			rf_list_out[0]  <= 0;
			rf_list_out[1]  <= 0;
			rf_list_out[2]  <= 0;
			rf_list_out[3]  <= 0;
			rf_list_out[4]  <= 0;
			rf_list_out[5]  <= 0;
			rf_list_out[6]  <= 0;
			rf_list_out[7]  <= 0;
			rf_list_out[8]  <= 0;
			rf_list_out[9]  <= 0;
			rf_list_out[10] <= 0;
			rf_list_out[11] <= 0;
			rf_list_out[12] <= 0;
			rf_list_out[13] <= 0;
			rf_list_out[14] <= 0;
			rf_list_out[15] <= 0;
			rf_list_out[16] <= 0;
			rf_list_out[17] <= 0;
			rf_list_out[18] <= 0;
			rf_list_out[19] <= 0;
			rf_list_out[20] <= 0;
			rf_list_out[21] <= 0;
			rf_list_out[22] <= 0;
			rf_list_out[23] <= 0;
			rf_list_out[24] <= 0;
			rf_list_out[25] <= 0;
			rf_list_out[26] <= 0;
			rf_list_out[27] <= 0;
			rf_list_out[28] <= 0;
			rf_list_out[29] <= 0;
			rf_list_out[30] <= 0;
			rf_list_out[31] <= 0;
		end else begin
			if(address_c_in && rf_write_in) rf_list_out[address_c_in][31:0] <= data_in;
		end
	end
endmodule

/* Instruction Fetch stage. */
/*
	At this stage, PC will be updated, and reading signal will be sent to the memory
	to fetch instructions. 
	Note that when a flush is happening, nothing will be done but update the PC value.
	It should also send a signal to flush the next stage.
*/
module stage_0_IF (
	input clk,
	input reset,

	/* Information from ex (2) stage. */
	input [31:0] i_pc_imm,
	input i_flush,

	/* Information from bpb */
	input bpb_prediction i_bpb_predict,

	/* Output to the next stage. */
	output if_id_reg o_id_inform,
	output logic [31:0] o_pc_addr,
	output logic o_pc_rd
);
	logic [31:0] pc; //PC register

	assign o_pc_addr = pc;
	assign o_pc_rd = ~i_flush;

	always_ff @(posedge clk) begin 
		if(reset) begin
			o_id_inform.pc <= '0;
			o_id_inform.flush <= 0;
			o_id_inform.branch_taken <= 0;
			pc <= '0;
		end else begin
			if(i_flush) begin 
				/* Sending a flush signal and update PC by immediate data. */
				o_id_inform.pc <= 0;
				o_id_inform.branch_taken <= 0;
				o_id_inform.flush <= 1;
				pc <= i_pc_imm;
			end else if(i_bpb_predict.valid) begin
				pc <= i_bpb_predict.target_pc;
				o_id_inform.pc <= pc;
				o_id_inform.branch_taken <= i_bpb_predict.branch_taken;
				o_id_inform.flush <= 0;
			end else begin
				/* No flush happening. Record PC value. */
				o_id_inform.pc <= pc;
				o_id_inform.branch_taken <= 0;
				o_id_inform.flush <= 0;
				pc <= pc + 32'd4;
			end
		end
	end
endmodule

module stage_1_ID (
	input clk,
	input reset,

	/* Information from PC */
	input [31:0] i_pc_rddata,

	/* Information from last stage */
	input if_id_reg i_id_inform,

	/* Information for flushing */
	input i_flush,

	/* Information from register file */
	input [31:0] ra_data,
	input [31:0] rb_data,

	/* Output to the register file */
	output logic [4:0] ra_addr,
	output logic [4:0] rb_addr,

	/* Output to the next stage */
	output id_ex_reg o_ex_inform
);
	/* Opcode */
	localparam	INS_ALU_R = 7'b0110011;
	localparam	INS_ALU_IMM = 7'b0010011;
	localparam	INS_LD = 7'b0000011;
	localparam	INS_ST = 7'b0100011;
	localparam	INS_LD_UPPER_IMM = 7'b0110111;
	localparam	INS_ADD_UPPER_IMM_PC = 7'b0010111;
	localparam	INS_BRANCH = 7'b1100011;
	localparam	INS_JP_LK = 7'b1101111;
	localparam	INS_JP_LK_R = 7'b1100111;
	localparam  INS_NONE = 7'b0000000;

	/* Handling instructions */
	logic [31:0] instruction;
	assign instruction = i_pc_rddata;

	ins_encoding instruction_encoding;
	logic [7:0] opcode;
	logic [2:0] func3;
	logic [6:0] func7;
	logic [4:0] rd;
	logic [31:0] immediate;

	assign opcode = instruction[6:0];
	assign func3 = instruction[14:12];
	assign func7 = instruction[31:25];
	assign rd = instruction[11:7];
	assign ra_addr = instruction[19:15];
	assign rb_addr = instruction[24:20];

	/* Instruction encoding */
	always@(*) begin
		case(opcode)
			INS_ALU_R: begin
				case(func3)
					3'd0: instruction_encoding = (func7 == 0) ? ADD : SUB;
					3'd1: instruction_encoding = SLL;
					3'd2: instruction_encoding = SLT;
					3'd3: instruction_encoding = SLTU;
					3'd4: instruction_encoding = XOR;
					3'd5: instruction_encoding = (func7 == 0) ? SRL : SRA;
					3'd6: instruction_encoding = OR;
					3'd7: instruction_encoding = AND;
				endcase
				immediate = '0;
			end
			INS_ALU_IMM: begin
				case(func3)
					3'd0: instruction_encoding = ADDI;
					3'd1: instruction_encoding = SLLI;
					3'd2: instruction_encoding = SLTI;
					3'd3: instruction_encoding = SLTUI;
					3'd4: instruction_encoding = XORI;
					3'd5: instruction_encoding = (func7 == 0) ? SRLI : SRAI;
					3'd6: instruction_encoding = ORI;
					3'd7: instruction_encoding = ANDI;
				endcase
				immediate = {{20{instruction[31]}}, instruction[31:20]};
			end
			INS_LD: begin 
				case(func3)
					3'd0: instruction_encoding = LB;
					3'd1: instruction_encoding = LH;
					3'd2: instruction_encoding = LW;
					3'd4: instruction_encoding = LBU;
					3'd5: instruction_encoding = LHU;
					default: instruction_encoding = NONE;
				endcase
				immediate = {{20{instruction[31]}}, instruction[31:20]};
			end
			INS_ST: begin
				case(func3)
					3'd0: instruction_encoding = SB;
					3'd1: instruction_encoding = SH;
					3'd2: instruction_encoding = SW;
					default: instruction_encoding = NONE;
				endcase
				immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
			end
			INS_LD_UPPER_IMM: begin
				instruction_encoding = LUI;
				immediate =  {instruction[31:12], {12{1'b0}}};
			end
			INS_ADD_UPPER_IMM_PC: begin
				instruction_encoding = AUIPC;
				immediate =  {instruction[31:12], {12{1'b0}}};
			end
			INS_BRANCH: begin
				case(func3)
					3'd0: instruction_encoding = BEQ;
					3'd1: instruction_encoding = BNE;
					3'd4: instruction_encoding = BLT;
					3'd5: instruction_encoding = BGE;
					3'd6: instruction_encoding = BLTU;
					3'd7: instruction_encoding = BGEU;
					default: instruction_encoding = NONE;
				endcase
				immediate = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
			end
			INS_JP_LK: begin
				instruction_encoding = JAL;
				immediate = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
			end
			INS_JP_LK_R: begin
				instruction_encoding = JALR;
				immediate = {{20{instruction[31]}}, instruction[31:20]};
			end
			default: begin
				// instruction_encoding = NONE;
				// immediate = 0;
			end
		endcase
	end

	/* Store information for the next stage */
	always_ff @(posedge clk) begin
		if(reset || i_flush || i_id_inform.flush) begin
			o_ex_inform.ins_type <= NONE;
			o_ex_inform.rd <= 0;
			o_ex_inform.reg_a <= '0;
			o_ex_inform.reg_b <= '0;
			o_ex_inform.ra_addr <= '0;
			o_ex_inform.rb_addr <= '0;
			o_ex_inform.imm_data <= '0;
			o_ex_inform.pc <= '0;
			o_ex_inform.branch_taken <= 0;
		end else begin
			o_ex_inform.ins_type <= instruction_encoding;
			o_ex_inform.rd <= rd;
			o_ex_inform.reg_a <= ra_data;
			o_ex_inform.reg_b <= rb_data;
			o_ex_inform.ra_addr <= ra_addr;
			o_ex_inform.rb_addr <= rb_addr;
			o_ex_inform.imm_data <= immediate;
			o_ex_inform.pc <= i_id_inform.pc;
			o_ex_inform.branch_taken <= i_id_inform.branch_taken;
		end
	end
endmodule

module arithematic_logic_unit (
	input [31:0] op1,
	input [31:0] op2,
	input [2:0] alu_sel,
	output logic [3:0] alu_flags, // {V, C, N, Z}
	output logic [31:0] alu_result
);
	always@(*) begin
		case(alu_sel)
			3'd0: {alu_flags[2], alu_result} = op1 +   op2;       			 // Add 
			3'd1: {alu_flags[2], alu_result} = op1 -   op2;       			 // Sub 
			3'd2: {alu_flags[2], alu_result} = op1 ^   op2;       			 // Xor 
			3'd3: {alu_flags[2], alu_result} = op1 |   op2;       			 // Or  
			3'd4: {alu_flags[2], alu_result} = op1 &   op2;       			 // And 
 			3'd5: {alu_flags[2], alu_result} = op1 <<  op2[4:0];  			 // Sll 
			3'd6: {alu_flags[2], alu_result} = op1 >>  op2[4:0];  			 // Srl 
			3'd7: {alu_flags[2], alu_result} = $signed(op1) >>> (op2[4:0]);  // Sra
			default: ; // {alu_flags[2], alu_result} = 0;
		endcase
		alu_flags[0] = ~| alu_result;
		alu_flags[1] = alu_result[31];
		alu_flags[3] = op1[31] ^ op2[31] ^ alu_result[31] ^ alu_flags[2];
	end
endmodule

module forwarding_decoder(
	input id_ex_reg i_ex_inform,
	input rf_forwarding i_forwarding_control,
	output logic [31:0] reg_1,
	output logic [31:0] reg_2
);
	always_comb begin
		/* Determine the actual value of reg1 */
		if(i_forwarding_control.rd != 0 && i_forwarding_control.rd == i_ex_inform.ra_addr) begin
			reg_1 = i_forwarding_control.wb_data;
		end else begin
			reg_1 = i_ex_inform.reg_a;
		end
		/* Determine the actual value of reg2 */
		if(i_forwarding_control.rd != 0 && i_forwarding_control.rd == i_ex_inform.rb_addr) begin
			reg_2 = i_forwarding_control.wb_data;
		end else begin
			reg_2 = i_ex_inform.reg_b;
		end
	end
endmodule

module stage_2_EX (
	input clk,
	input reset,

	/* Input from the previous stage and the next stage. */
	input id_ex_reg i_ex_inform,
	input rf_forwarding i_forwarding_control,

	/* Output to to the memory */
	output logic [31:0] o_ldst_addr,
	output logic o_ldst_rd,
	output logic o_ldst_wr,
	output logic [31:0] o_ldst_wrdata,
	output logic [3:0] o_ldst_byte_en,

	/* Output to PC and flush */
	output logic [31:0] o_pc_imm,
	output logic o_ex_flush,

	/* Output to the next stage */
	output ex_wb_reg o_wb_inform,

	/* Output to bpb */
	output bpb_inform o_bpb_updates
);
	localparam ALU_ADD = 3'd0;
	localparam ALU_SUB = 3'd1;
	localparam ALU_XOR = 3'd2;
	localparam ALU_OR  = 3'd3;
	localparam ALU_AND = 3'd4;
	localparam ALU_SLL = 3'd5;
	localparam ALU_SRL = 3'd6;
	localparam ALU_SRA = 3'd7;

	logic [31:0] op1;
	logic [31:0] op2;
	logic [2:0] alu_sel;
	logic V, C, N, Z;
	logic [31:0] alu_result;

	arithematic_logic_unit alu(
		.op1(op1),
		.op2(op2),
		.alu_sel(alu_sel),
		.alu_flags({V, C, N, Z}),
		.alu_result(alu_result)
	);

	/* Condition bits */
	logic EQ, LT, LTU;
	assign EQ = Z;
	assign LT = N ^ V;
	assign LTU = C;

	logic [31:0] reg_1;
	logic [31:0] reg_2;
	forwarding_decoder fd(
		.i_ex_inform(i_ex_inform),
		.i_forwarding_control(i_forwarding_control),
		.reg_1(reg_1),
		.reg_2(reg_2)
	);

	/* Handle the ALU. */
	always@(*) begin
		case(i_ex_inform.ins_type)
			/* ARI R */
			ADD: begin
				op1 = reg_1;
				op2 = reg_2;
				alu_sel = ALU_ADD;
			end
			SUB, SLT, SLTU: begin
				op1 = reg_1;
				op2 = reg_2;
				alu_sel = ALU_SUB;
			end
			XOR: begin
				op1 = reg_1;
				op2 = reg_2;
				alu_sel = ALU_XOR;
			end
			OR: begin
				op1 = reg_1;
				op2 = reg_2;
				alu_sel = ALU_OR;
			end
			AND: begin
				op1 = reg_1;
				op2 = reg_2;
				alu_sel = ALU_AND;
			end
			SLL: begin
				op1 = reg_1;
				op2 = reg_2;
				alu_sel = ALU_SLL;
			end
			SRL: begin
				op1 = reg_1;
				op2 = reg_2;
				alu_sel = ALU_SRL;
			end
			SRA: begin
				op1 = reg_1;
				op2 = reg_2;
				alu_sel = ALU_SRA;
			end

			/* ARI I */
			ADDI: begin
				op1 = reg_1;
				op2 = i_ex_inform.imm_data;
				alu_sel = ALU_ADD;
			end
			SLTI, SLTUI: begin
				op1 = reg_1;
				op2 = i_ex_inform.imm_data;
				alu_sel = ALU_SUB;
			end
			XORI: begin
				op1 = reg_1;
				op2 = i_ex_inform.imm_data;
				alu_sel = ALU_XOR;
			end
			ORI: begin
				op1 = reg_1;
				op2 = i_ex_inform.imm_data;
				alu_sel = ALU_OR;
			end
			ANDI: begin
				op1 = reg_1;
				op2 = i_ex_inform.imm_data;
				alu_sel = ALU_AND;
			end
			SLLI: begin
				op1 = reg_1;
				op2 = i_ex_inform.imm_data;
				alu_sel = ALU_SLL;
			end
			SRLI: begin
				op1 = reg_1;
				op2 = i_ex_inform.imm_data;
				alu_sel = ALU_SRL;
			end
			SRAI: begin
				op1 = reg_1;
				op2 = i_ex_inform.imm_data;
				alu_sel = ALU_SRA;
			end

			/* Load and Store */
			LB, LH, LW, LBU, LHU, SB, SH, SW: begin
				op1 = reg_1;
				op2 = i_ex_inform.imm_data;
				alu_sel = ALU_ADD;
			end

			/* Branching */
			BEQ, BNE, BLT, BGE, BLTU, BGEU: begin
				op1 = reg_1;
				op2 = reg_2;
				alu_sel = ALU_SUB;
			end

			/* LUI */
			LUI: begin
				op1 = 0;
				op2 = i_ex_inform.imm_data;
				alu_sel = ALU_ADD;
			end

			AUIPC: begin
				op1 = i_ex_inform.pc;
				op2 = i_ex_inform.imm_data;
				alu_sel = ALU_ADD;
			end

			/* Jump */
			JAL, JALR: begin
				op1 = i_ex_inform.pc;
				op2 = 4;
				alu_sel = ALU_ADD;
			end

			default: begin
				// op1 = 0;
				// op2 = 0;
				// alu_sel = ALU_ADD;
			end
		endcase
	end

	/* Jump PC */
	logic [31:0] pc_branch_target;
	always@(*) begin
		case(i_ex_inform.ins_type)
			BEQ, BNE, BLT, BGE, BLTU, BGEU, JAL: pc_branch_target = i_ex_inform.pc + i_ex_inform.imm_data;
			JALR: pc_branch_target = i_ex_inform.imm_data + reg_1;
			default: pc_branch_target = '0;
		endcase
	end

	/* Handle PC immediate value. */
	always_comb begin
		if(i_ex_inform.branch_taken == 0) begin
			o_pc_imm = pc_branch_target;
		end else begin
			o_pc_imm = i_ex_inform.pc + 4;
		end
	end

	/* Handle PC flush. */
	always_comb begin
		case(i_ex_inform.ins_type)
			// BEQ:   o_ex_flush = (EQ & (~i_ex_inform.branch_taken)) | (~EQ & i_ex_inform.branch_taken);
			// BNE:   o_ex_flush = (~EQ & (~i_ex_inform.branch_taken)) | (EQ & i_ex_inform.branch_taken);
			// BLT:   o_ex_flush = (LT & (~i_ex_inform.branch_taken)) | (~LT & i_ex_inform.branch_taken);
			// BGE:   o_ex_flush = (~LT & (~i_ex_inform.branch_taken)) | (LT & i_ex_inform.branch_taken);
			// BLTU:  o_ex_flush = (LTU & (~i_ex_inform.branch_taken)) | (~LTU & i_ex_inform.branch_taken);
			// BGEU:  o_ex_flush = (~LTU & (~i_ex_inform.branch_taken)) | (LTU & i_ex_inform.branch_taken);
			BEQ:   o_ex_flush = EQ ^ i_ex_inform.branch_taken;
			BNE:   o_ex_flush = ~EQ ^ i_ex_inform.branch_taken;
			BLT:   o_ex_flush = LT ^ i_ex_inform.branch_taken;
			BGE:   o_ex_flush = ~LT ^ i_ex_inform.branch_taken;
			BLTU:  o_ex_flush = LTU ^ i_ex_inform.branch_taken;
			BGEU:  o_ex_flush = ~LTU ^ i_ex_inform.branch_taken;
			JAL:   o_ex_flush = 1 & (~i_ex_inform.branch_taken);
			JALR:  o_ex_flush = 1;
			default: o_ex_flush = 0;
		endcase
	end

	/* Handle bpb updates */
	always_comb begin
		
		case(i_ex_inform.ins_type)
			BEQ: begin
				o_bpb_updates.valid = 1;
				o_bpb_updates.pc = i_ex_inform.pc;
				o_bpb_updates.branch_target_pc = pc_branch_target;
				o_bpb_updates.branch_taken = EQ;
				o_bpb_updates.jump = 0;
			end
			BNE: begin
				o_bpb_updates.valid = 1;
				o_bpb_updates.pc = i_ex_inform.pc;
				o_bpb_updates.branch_target_pc = pc_branch_target;
				o_bpb_updates.branch_taken = ~EQ;
				o_bpb_updates.jump = 0;
			end
			BLT: begin
				o_bpb_updates.valid = 1;
				o_bpb_updates.pc = i_ex_inform.pc;
				o_bpb_updates.branch_target_pc = pc_branch_target;
				o_bpb_updates.branch_taken = LT;
				o_bpb_updates.jump = 0;
			end
			BGE: begin
				o_bpb_updates.valid = 1;
				o_bpb_updates.pc = i_ex_inform.pc;
				o_bpb_updates.branch_target_pc = pc_branch_target;
				o_bpb_updates.branch_taken = ~LT;
				o_bpb_updates.jump = 0;
			end
			BLTU: begin
				o_bpb_updates.valid = 1;
				o_bpb_updates.pc = i_ex_inform.pc;
				o_bpb_updates.branch_target_pc = pc_branch_target;
				o_bpb_updates.branch_taken = LTU;
				o_bpb_updates.jump = 0;
			end
			BGEU: begin
				o_bpb_updates.valid = 1;
				o_bpb_updates.pc = i_ex_inform.pc;
				o_bpb_updates.branch_target_pc = pc_branch_target;
				o_bpb_updates.branch_taken = ~LTU;
				o_bpb_updates.jump = 0;
			end
			JAL: begin
				o_bpb_updates.valid = 1;
				o_bpb_updates.pc = i_ex_inform.pc;
				o_bpb_updates.branch_target_pc = pc_branch_target;
				o_bpb_updates.branch_taken = 0;
				o_bpb_updates.jump = 1;
			end
			default: begin
				o_bpb_updates.valid = 0;
			end
		endcase
	end

	/* Handle Load & Store */
	always_comb begin
		o_ldst_addr = 0;
		o_ldst_rd = 0;
		o_ldst_wr = 0;
		o_ldst_byte_en = '0; 
		o_ldst_wrdata = '0;
		case(i_ex_inform.ins_type)
			LB, LBU: begin
				o_ldst_addr = alu_result;
				o_ldst_rd = 1;
				o_ldst_byte_en = 4'b0001; 
			end
			LH, LHU: begin
				o_ldst_addr = alu_result;
				o_ldst_rd = 1;
				o_ldst_byte_en = 4'b0011; 
			end
			LW: begin
				o_ldst_addr = alu_result;
				o_ldst_rd = 1;
				o_ldst_byte_en = 4'b1111; 
			end
			SB: begin
				o_ldst_addr = alu_result;
				o_ldst_wr = 1;
				o_ldst_byte_en = 4'b0001; 
				o_ldst_wrdata = {{24{1'b0}}, reg_2[7:0]};
			end
			SH: begin
				o_ldst_addr = alu_result;
				o_ldst_wr = 1;
				o_ldst_byte_en = 4'b0011; 
				o_ldst_wrdata = {{16{1'b0}}, reg_2[15:0]};
			end
			SW: begin
				o_ldst_addr = alu_result;
				o_ldst_wr = 1;
				o_ldst_byte_en = 4'b1111; 
				o_ldst_wrdata = reg_2;
			end
			default: begin
				/* Doing nothing. */
			end
		endcase
	end

	/* Record information for wb stage. */
	always_ff @(posedge clk) begin
		if(reset) begin
			o_wb_inform.ins_type <= NONE;
			o_wb_inform.rd <= '0;
			o_wb_inform.alu_result <= '0;
			o_wb_inform.pc <= 0;
		end else begin
			o_wb_inform.ins_type <= i_ex_inform.ins_type;
			o_wb_inform.rd <= i_ex_inform.rd;
			o_wb_inform.pc <= i_ex_inform.pc;
			if(i_ex_inform.ins_type == SLT || i_ex_inform.ins_type == SLTI) begin
				o_wb_inform.alu_result <= LT ? 32'd1 : 0;
			end else if(i_ex_inform.ins_type == SLTU || i_ex_inform.ins_type == SLTUI) begin
				o_wb_inform.alu_result <= LTU ? 32'd1 : 0;
			end else begin
				o_wb_inform.alu_result <= alu_result;
			end
		end
	end
endmodule

/* This stage does not need a clock. */
module stage_3_WB (
	/* Input from the previous stage */
	input ex_wb_reg i_wb_inform,

	/* Input from memory */
	input [31:0] i_ldst_rddata,

	/* Output to the register file */
	output logic [31:0] o_rf_write_data,
	output logic o_rf_write,
	output logic [4:0] rc_addr,

	/* Output to the forwarding */ 
	output rf_forwarding o_forwarding_control
);
	logic [31:0] pc;
	assign pc = i_wb_inform.pc;

	/* Selecting write data */
	always@(*) begin
		case(i_wb_inform.ins_type)
			LB: o_rf_write_data = {{24{i_ldst_rddata[7]}}, i_ldst_rddata[7:0]};
			LH: o_rf_write_data = {{16{i_ldst_rddata[15]}}, i_ldst_rddata[15:0]};
			LW: o_rf_write_data = i_ldst_rddata;
			LBU: o_rf_write_data = {{24{1'b0}}, i_ldst_rddata[7:0]};
			LHU: o_rf_write_data = {{16{1'b0}}, i_ldst_rddata[15:0]};
			NONE: o_rf_write_data = '0;
			default: ;// o_rf_write_data = i_wb_inform.alu_result;
		endcase
	end

	/* Write back */
	always@(*) begin
		case(i_wb_inform.ins_type)
			SB, SH, SW, BEQ, BNE, BLT, BGE, BLTU, BGEU, NONE: begin
				rc_addr = 0;
				o_rf_write = 0;
				o_forwarding_control.rd = 0;
				o_forwarding_control.wb_data = 0;
			end
			default: begin
				// rc_addr = i_wb_inform.rd;
				// o_rf_write = 1;
				// o_forwarding_control.rd = i_wb_inform.rd;
				// o_forwarding_control.wb_data = o_rf_write_data;
			end
		endcase
	end
endmodule
