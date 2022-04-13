module mem # (
	parameter WIDTH = 32,
	parameter DEPTH = 8192,
	parameter HEX_FILE = "part1.hex"
)(
	input clk,
	input reset,

	// Read/Write port
	input [$clog2(DEPTH)-1:0] p1_addr,
	input p1_write,
	input p1_read,
	input [(WIDTH/8)-1:0] p1_byteenable,
	input [WIDTH-1:0] p1_writedata,
	output logic [WIDTH-1:0] p1_readdata,

	// Read only port
	input [$clog2(DEPTH)-1:0] p2_addr,
	input p2_read,
	input [(WIDTH/8)-1:0] p2_byteenable,
	output logic [WIDTH-1:0] p2_readdata
);

integer i;
logic [WIDTH-1:0] ram [0:DEPTH-1];
logic [7:0] temp_ram [0:(WIDTH/8)*DEPTH-1];

initial begin
	$readmemh(HEX_FILE, temp_ram);
end

always_comb begin
	p1_readdata = 32'h0;
	p2_readdata = 32'h0;
	if (p1_read) begin
		if (p1_byteenable[0]) begin
			p1_readdata[7:0] = ram[p1_addr][7:0];
		end
		if (p1_byteenable[1]) begin
			p1_readdata[15:8] = ram[p1_addr][15:8];
		end
		if (p1_byteenable[2]) begin
			p1_readdata[23:16] = ram[p1_addr][23:16];
		end
		if (p1_byteenable[3]) begin
			p1_readdata[31:24] = ram[p1_addr][31:24];
		end
	end
	if (p2_read) begin
		if (p2_byteenable[0]) begin
			p2_readdata[7:0] = ram[p2_addr][7:0];
		end
		if (p2_byteenable[1]) begin
			p2_readdata[15:8] = ram[p2_addr][15:8];
		end
		if (p2_byteenable[2]) begin
			p2_readdata[23:16] = ram[p2_addr][23:16];
		end
		if (p2_byteenable[3]) begin
			p2_readdata[31:24] = ram[p2_addr][31:24];
		end
	end
end

always_ff @(posedge clk) begin
	if (reset) begin
		for (i = 0; i < DEPTH; i++)
			ram[i] = {temp_ram[4*i+3], temp_ram[4*i+2], temp_ram[4*i+1], temp_ram[4*i]};
	end else begin
		if (p1_write) begin
			if (p1_byteenable[0]) begin
				ram[p1_addr][7:0] = p1_writedata[7:0];
			end
			if (p1_byteenable[1]) begin
				ram[p1_addr][15:8] = p1_writedata[15:8];
			end
			if (p1_byteenable[2]) begin
				ram[p1_addr][23:16] = p1_writedata[23:16];
			end
			if (p1_byteenable[3]) begin
				ram[p1_addr][31:24] = p1_writedata[31:24];
			end
		end
	end
end

endmodule


	/* CPU I/O map */
	/*
		output [IW-1:0] o_pc_addr,              //D path
		output o_pc_rd,    						//C path 
		input [IW-1:0] i_pc_rddata,				//C path
		output [3:0] o_pc_byte_en,				//C path
		output [IW-1:0] o_ldst_addr,			//D path
		output o_ldst_rd,						//C path
		output o_ldst_wr,						//C path
		input [IW-1:0] i_ldst_rddata,			//C path
		output [IW-1:0] o_ldst_wrdata,			//D path
		output [3:0] o_ldst_byte_en,			//C path
		output [IW-1:0] o_tb_regs [0:REGS-1]	//D path
	*/


module cpu # (
	parameter IW = 32, // instr width
	parameter REGS = 32 // number of registers
)(
	input clk,

	// Assume this is an active high reset
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
	/* Define inputs */
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
	logic [31:0] mem_data_out;
	logic Z, N, C, V;

	data_path DP(
		.clk(clk),
		.reset(reset),
		.rf_we(rf_we),
		.rf_in_sel(rf_in_sel),
		.addr_a(addr_a),
		.addr_b(addr_b),
		.addr_c(addr_c),
		.load_ra(load_ra),
		.load_rb(load_rb),
		.op1_sel(op1_sel),
		.op2_sel(op2_sel),
		.alu_sel(alu_sel),
		.load_rr(load_rr),
		.mem_data_out_sel(mem_data_out_sel),
		.load_rm(load_rm),
		.ry_sel(ry_sel),
		.load_ry(load_ry),
		.pc_sel(pc_sel),
		.inc_sel(inc_sel),
		.load_pc(load_pc),
		.load_pc_temp(load_pc_temp),
		.mem_data_in(mem_data_in),
		.imm_rf(imm_rf),
		.imm_op2(imm_op2),
		.imm_pc(imm_pc),
		.imm_ry(imm_ry),
		.rf_list_out(o_tb_regs),
		.pc_addr_out(o_pc_addr),
		.ldst_addr_out(o_ldst_addr),
		.mem_data_out(o_ldst_wrdata),
		.Z(Z),
		.N(N),
		.C(C),
		.V(V)
	);

	control_path CP(
		.clk(clk),
		.reset(reset),
		.Z(Z),
		.N(N),
		.C(C),
		.V(V),
		.pc_data_in(i_pc_rddata),
		.ldst_data_in(i_ldst_rddata),
		.rf_we(rf_we),
		.rf_in_sel(rf_in_sel),
		.addr_a(addr_a),
		.addr_b(addr_b),
		.addr_c(addr_c),
		.load_ra(load_ra),
		.load_rb(load_rb),
		.op1_sel(op1_sel),
		.op2_sel(op2_sel),
		.alu_sel(alu_sel),
		.load_rr(load_rr),
		.mem_data_out_sel(mem_data_out_sel),
		.load_rm(load_rm),
		.ry_sel(ry_sel),
		.load_ry(load_ry),
		.pc_sel(pc_sel),
		.inc_sel(inc_sel),
		.load_pc(load_pc),
		.load_pc_temp(load_pc_temp),
		.mem_data_in(mem_data_in),
		.imm_rf(imm_rf),
		.imm_op2(imm_op2),
		.imm_pc(imm_pc),
		.imm_ry(imm_ry),
		.pc_rd(o_pc_rd),
		.pc_byte_en(o_pc_byte_en),
		.ldst_rd(o_ldst_rd),
		.ldst_wr(o_ldst_wr),
		.ldst_byte_en(o_ldst_byte_en)
	); 

endmodule

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

	always_ff @(posedge clk) begin
		if(reset) begin
			rf_list_out[0] <= 0;
			rf_list_out[1] <= 0;
			rf_list_out[2] <= 0;
			rf_list_out[3] <= 0;
			rf_list_out[4] <= 0;
			rf_list_out[5] <= 0;
			rf_list_out[6] <= 0;
			rf_list_out[7] <= 0;
			rf_list_out[8] <= 0;
			rf_list_out[9] <= 0;
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

module data_path(
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
	output logic [31:0] rf_list_out [0:31],
	output logic [31:0] pc_addr_out,
	output logic [31:0] ldst_addr_out,
	output logic [31:0] mem_data_out,
	output logic Z,
	output logic N,
	output logic C,
	output logic V
);
	logic [31:0] ra; //Register A
	logic [31:0] rb; //Register B
	logic [31:0] rm; //Register M
	logic [31:0] rr; //Register R, the first four bits are flags {Z, N, C, V}
	logic [31:0] ry; //Register Y
	logic [31:0] pc; //Register PC
	logic [31:0] pc_temp; //Register PC_temp
	
	/* The following code implements datapath from the top to the bottom, from left to right */

	/* File Register */
	logic [31:0] rf_out_a;
	logic [31:0] rf_out_b;
	logic [31:0] rf_data_in;
	always_comb begin
		case(rf_in_sel)
			1'b0: rf_data_in = ry;
			1'b1: rf_data_in = imm_rf;
			default: rf_data_in = 0;
		endcase
	end
	register_file rf(
		.clk(clk),
		.reset(reset),
		.address_a_in(addr_a),
		.address_b_in(addr_b),
		.address_c_in(addr_c),
		.data_in(rf_data_in),
		.rf_write_in(rf_we),
		.rf_list_out(rf_list_out),
		.reg_a_out(rf_out_a),
		.reg_b_out(rf_out_b)
	);

	/* RA */
	always_ff @(posedge clk) begin 
		if(reset) begin
			ra <= 0;
		end else if(load_ra) begin
			ra <= rf_out_a;
		end
	end

	/* RB */
	always_ff @(posedge clk) begin 
		if(reset) begin
			rb <= 0;
		end else if(load_ra) begin
			rb <= rf_out_b;
		end
	end

	/* ALU */
	logic [31:0] op1;
	logic [31:0] op2;
	always_comb begin
		case(op1_sel)
			1'b0: op1 = ra;
			1'b1: op1 = pc;
			default: op1 = 0;
		endcase

		case(op2_sel)
			1'b0: op2 = rb;
			1'b1: op2 = imm_op2;
			default: op2 = 0;
		endcase
	end
	logic [31:0] alu_result;
	logic [3:0] alu_flags; // {V, C, N, Z}
	always_comb begin
		case(alu_sel)
			3'd0: {alu_flags[2], alu_result} = op1 +   op2;       // Add (Checked)
			3'd1: {alu_flags[2], alu_result} = op1 -   op2;       // Sub (Checked)
			3'd2: {alu_flags[2], alu_result} = op1 ^   op2;       // Xor (Checked)
			3'd3: {alu_flags[2], alu_result} = op1 |   op2;       // Or  (Checked)
			3'd4: {alu_flags[2], alu_result} = op1 &   op2;       // And (Checked)
 			3'd5: {alu_flags[2], alu_result} = op1 <<  op2[4:0];  // Sll (Checked)
			3'd6: {alu_flags[2], alu_result} = op1 >>  op2[4:0];  // Srl (Checked)
			3'd7: {alu_flags[2], alu_result} = $signed(op1) >>> (op2[4:0]);  // Sra
			default: {alu_flags[2], alu_result} = 0;
		endcase
		alu_flags[0] = ~| alu_result;
		alu_flags[1] = alu_result[31];
		alu_flags[3] = op1[31] ^ op2[31] ^ alu_result[31] ^ alu_flags[2];
	end

	/* RR */
	always_ff @(posedge clk) begin 
		if(reset) begin
			rr <= 0;
			{V, C, N, Z} <= 0;
		end else if(load_rr) begin
			rr <= {alu_flags, alu_result};
			{V, C, N, Z} <= alu_flags;
		end
	end

	/* RM */
	always_ff @(posedge clk) begin 
		if(reset) begin
			rm <= 0;
		end else if(load_rm) begin
			rm <= rb;
		end
	end
	

	/* Memory output */
	always_comb begin
		case(mem_data_out_sel) 
			2'b00: mem_data_out = {{24{1'b0}}, rm[7:0]};
			2'b01: mem_data_out = {{16{1'b0}}, rm[15:0]};
			2'b10: mem_data_out = rm;
			default: mem_data_out = 0;
		endcase
	end

	/* RY */ 
	always_ff @(posedge clk) begin 
		if(reset) begin
			ry <= 0;
		end else if(load_ry) begin
			case(ry_sel)
				2'd0: ry <= rr;
				2'd1: ry <= mem_data_in;
				2'd2: ry <= pc_temp;
				2'd3: ry <= imm_ry;
				default: ry <= 0;
			endcase
		end
	end

	/* PC unit */
	/* PC adder */
	logic [31:0] pc_inc_result;
	always_comb begin
		case(inc_sel) 
			1'b0: pc_inc_result = pc + 32'd4;
			1'b1: pc_inc_result = pc + imm_pc;
			default: pc_inc_result = pc + 32'd4;
		endcase
	end
	/* PC register */
	always_ff @(posedge clk) begin 
		if(reset) begin
			pc <= 0;
		end else if(load_pc) begin
			case(pc_sel)
				1'b0: pc <= ra;
				1'b1: pc <= pc_inc_result;
				default: pc <= pc_inc_result;
			endcase
		end
	end
	/* PC temp register */
	always_ff @(posedge clk) begin 
		if(reset) begin
			pc_temp <= 0;
		end else if(load_pc_temp) begin
			pc_temp <= pc;
		end
	end

	/* Address select */
	assign pc_addr_out = pc;
	assign ldst_addr_out = rr;
endmodule

module instruction_register(
	input clk,
	input reset,
	input [31:0] mem_data_in, 
	input load_ir,
	output logic [31:0] instruction
);
	always_ff @(negedge clk) begin 
		if(reset) begin
			instruction <= 0;
		end else if(load_ir) begin
			instruction <= mem_data_in;
		end
	end
endmodule 

module control_path(
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
	enum int unsigned
    {
    	STAGE_RESET,
        STAGE_0,
        STAGE_1,
        STAGE_2,
        STAGE_3,
        STAGE_4
    } current_state, next_state;

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

	/* Access */
	localparam  WORD_ACCESS = 4'b1111;
	localparam  HAFL_WORD_ACCESS = 4'b0011;
	localparam  BYTE_ACCESS = 4'b0001;

	/* Instruction decoding */
	logic load_decoding;
	logic [6:0] func7;
	logic [2:0] func3;
	logic [4:0] rs1;
	logic [4:0] rs2;
	logic [4:0] rd;
	logic [6:0] opcode_reg;
	logic [6:0] opcode;
	logic [31:0] immediate_data;

	assign rs1 = pc_data_in[19:15];  
	assign rs2 = pc_data_in[24:20];  

	always_comb begin
		if(current_state == STAGE_1) opcode = pc_data_in[6:0];
		else opcode = opcode_reg; 
	end

	always_ff @ (posedge clk) begin
        if (reset) begin
        	func7 <= 0;
        	func3 <= 0;
        	rd <= 0;
        	opcode_reg <= 0;
        	immediate_data <= 0;
        end else if(load_decoding) begin
        	func7 <= pc_data_in[31:25];
        	func3 <= pc_data_in[14:12];
        	rd    <= pc_data_in[11:7];
        	opcode_reg <= pc_data_in[6:0];
        	if(opcode == INS_ALU_IMM || opcode == INS_LD || opcode == INS_JP_LK_R) begin
        		// I type
				immediate_data <= {{20{pc_data_in[31]}}, pc_data_in[31:20]};
        	end else if(opcode == INS_ST) begin
        		// S type
        		immediate_data <= {{20{pc_data_in[31]}}, pc_data_in[31:25], pc_data_in[11:7]};
        	end else if(opcode == INS_BRANCH) begin
        		// B type
        		immediate_data <= {{20{pc_data_in[31]}}, pc_data_in[7], pc_data_in[30:25], pc_data_in[11:8], 1'b0};
        	end else if(opcode == INS_JP_LK) begin
        		// J type
        		immediate_data <= {{12{pc_data_in[31]}}, pc_data_in[19:12], pc_data_in[20], pc_data_in[30:21], 1'b0};
        	end
        end
    end 

	/* Condition bits */
	logic EQ, LT, LTU;
	assign EQ = Z;
	assign LT = N ^ V;
	assign LTU = C;

	/*************************************** FSM ***************************************/

    /* State transition */
    always_ff @ (posedge clk) begin
        if (reset) current_state <= STAGE_RESET;
        else current_state <= next_state;
    end

    always_comb begin

    	/* Default value */
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
    	mem_data_out_sel = 0;
    	load_rm = 0;
    	ry_sel = 0;
    	load_ry = 0;
    	pc_sel = 0;
    	inc_sel = 0;
    	load_pc = 0;
    	load_pc_temp = 0;
    	load_decoding = 0;
    	mem_data_in = 0;
    	imm_rf = 0;
    	imm_op2 = 0;
    	imm_pc = 0;
    	imm_ry = 0;
    	pc_rd = 0;
    	pc_byte_en = 0;
    	ldst_rd = 0;
    	ldst_wr = 0;
    	ldst_byte_en = 0;

    	if(current_state == STAGE_RESET) begin
    		next_state = STAGE_0;
    	end

    	if(current_state == STAGE_0) begin
			//Load from PC
    		pc_rd = 1;
    		pc_byte_en = WORD_ACCESS;
    		next_state = STAGE_1;
    	end

    	case(opcode)
    		INS_ALU_R: begin
    			case(current_state)
    				STAGE_1: begin
    					load_decoding = 1;
    					addr_a = rs1;
    					addr_b = rs2;
    					load_ra = 1;
    					load_rb = 1;
    					next_state = STAGE_2;
    				end
    				STAGE_2: begin
    					op1_sel = 0;
    					op2_sel = 0;
    					load_rr = 1;
    					case(func3)
    						3'd0: begin
    							if(func7 == 0) alu_sel = 3'd0; //Add
    							else if(func7 == 7'b0100000) alu_sel = 3'd1; //Sub
    						end 
    						3'd1: alu_sel = 3'd5; //sll
    						3'd2: alu_sel = 3'd1; //set less than 
    						3'd3: alu_sel = 3'd1; //set less than (unsigned)
    						3'd4: alu_sel = 3'd2; //xor
    						3'd5: begin
    							if(func7 == 0) alu_sel = 3'd6; //srl
    							else if(func7 == 7'b0100000) alu_sel = 3'd7; //sra 
    						end
    						3'd6: alu_sel = 3'd3; //or
    						3'd7: alu_sel = 3'd4; //and
    					endcase
    					next_state = STAGE_3;
    				end
    				STAGE_3: begin
    					if(func3 == 3'd2) begin
    						// set less than
    						imm_ry = LT;
    						ry_sel = 2'd3;
    					end else if(func3 == 3'd3) begin
    						// set less than unsigned
    						imm_ry = LTU;
    						ry_sel = 2'd3;
    					end else begin
    						ry_sel = 0;
    					end
    					load_ry = 1;
    					next_state = STAGE_4;
    				end
    				STAGE_4: begin
    					rf_in_sel = 0;
    					addr_c = rd;
    					rf_we = 1;
    					inc_sel = 0;
    					pc_sel = 1;
    					load_pc = 1;
    					next_state = STAGE_0;
    				end
    			endcase
    		end
    		INS_ALU_IMM: begin
				case(current_state)
					STAGE_1: begin
						load_decoding = 1;
						addr_a = rs1;
    					load_ra = 1;
    					next_state = STAGE_2;
    				end
    				STAGE_2: begin
    					op1_sel = 0;
    					op2_sel = 1;
    					imm_op2 = immediate_data;
    					load_rr = 1;
    					case(func3)
    						3'd0: alu_sel = 3'd0; //add
    						3'd1: alu_sel = 3'd5; //sll
    						3'd2: alu_sel = 3'd1; //set less than 
    						3'd3: alu_sel = 3'd1; //set less than (unsigned)
    						3'd4: alu_sel = 3'd2; //xor
    						3'd5: begin
    							if(immediate_data[11:5] == 0) alu_sel = 3'd6; //srl
    							else if(immediate_data[11:5] == 7'b0100000) alu_sel = 3'd7; //sra 
    						end
    						3'd6: alu_sel = 3'd3; //or
    						3'd7: alu_sel = 3'd4; //and
    					endcase
    					next_state = STAGE_3;
    				end
    				STAGE_3: begin
    					if(func3 == 3'd2) begin
    						// set less than
    						imm_ry = LT;
    						ry_sel = 2'd3;
    					end else if(func3 == 3'd3) begin
    						// set less than unsigned
    						imm_ry = LTU;
    						ry_sel = 2'd3;
    					end else begin
    						ry_sel = 0;
    					end
    					load_ry = 1;
    					next_state = STAGE_4;
    				end
    				STAGE_4: begin
    					rf_in_sel = 0;
    					addr_c = rd;
    					rf_we = 1;
    					inc_sel = 0;
    					pc_sel = 1;
    					load_pc = 1;
    					next_state = STAGE_0;
    				end
    			endcase
    		end
    		INS_LD: begin
				case(current_state)
					STAGE_1: begin
						load_decoding = 1;
						addr_a = rs1;
    					load_ra = 1;
    					next_state = STAGE_2;
    				end
    				STAGE_2: begin
    					op1_sel = 0;
    					op2_sel = 1;
    					imm_op2 = immediate_data;
    					alu_sel = 3'd0;
    					load_rr = 1;
    					next_state = STAGE_3;
    				end
    				STAGE_3: begin
    					ldst_rd = 1;
    					case(func3)
    						3'd0: ldst_byte_en = BYTE_ACCESS;
    						3'd1: ldst_byte_en = HAFL_WORD_ACCESS;
    						3'd2: ldst_byte_en = WORD_ACCESS;
    						3'd4: ldst_byte_en = BYTE_ACCESS;
    						3'd5: ldst_byte_en = HAFL_WORD_ACCESS;
    					endcase
    					next_state = STAGE_4;
    				end
    				STAGE_4: begin
    					rf_in_sel = 1;
    					addr_c = rd;
    					rf_we = 1;
    					case(func3)
    						3'd0: imm_rf = {{24{ldst_data_in[7]}}, ldst_data_in[7:0]};
    						3'd1: imm_rf = {{16{ldst_data_in[15]}}, ldst_data_in[15:0]};
    						3'd2: imm_rf = ldst_data_in;
    						3'd4: imm_rf = {{24{1'b0}}, ldst_data_in[7:0]};
    						3'd5: imm_rf = {{16{1'b0}}, ldst_data_in[15:0]};
    					endcase
    					/* Increment PC */
    					inc_sel = 0;
    					pc_sel = 1;
    					load_pc = 1;
    					next_state = STAGE_0;
    				end
    			endcase
    		end
    		INS_ST: begin
				case(current_state)
					STAGE_1: begin
						load_decoding = 1;
						addr_a = rs1;
    					addr_b = rs2;
    					load_ra = 1;
    					load_rb = 1;
    					next_state = STAGE_2;
    				end
    				STAGE_2: begin
    					op1_sel = 0;
    					op2_sel = 1;
    					imm_op2 = immediate_data;
    					alu_sel = 3'd0;
    					load_rr = 1;
    					load_rm = 1;
    					next_state = STAGE_3;
    				end
    				STAGE_3: begin
    					ldst_wr = 1;
    					case(func3)
    						3'd0: begin
    							ldst_byte_en = BYTE_ACCESS;
    							mem_data_out_sel = 2'd0;
    						end
    						3'd1: begin
    							ldst_byte_en = HAFL_WORD_ACCESS;
    							mem_data_out_sel = 2'd1;
    						end
    						3'd2: begin
    							ldst_byte_en = WORD_ACCESS;
    							mem_data_out_sel = 2'd2;
    						end
    					endcase
    					/* Increment PC */
    					inc_sel = 0;
    					pc_sel = 1;
    					load_pc = 1;
    					next_state = STAGE_0;
    				end
    			endcase
    		end
    		INS_LD_UPPER_IMM: begin
				case(current_state)
					STAGE_1: begin
						addr_c = pc_data_in[11:7];
    					rf_in_sel = 1;
    					imm_rf =  {pc_data_in[31:12], {12{1'b0}}};
    					rf_we = 1;
    					next_state = STAGE_0;
    					/* Increment PC */
    					inc_sel = 0;
    					pc_sel = 1;
    					load_pc = 1;
    				end
    			endcase
    		end
    		INS_ADD_UPPER_IMM_PC: begin
				case(current_state)
					STAGE_1: begin
						load_decoding = 1;
						op1_sel = 1;
						op2_sel = 1;
						imm_op2 = {pc_data_in[31:12], {12{1'b0}}};
						alu_sel = 0;
						load_rr = 1;
						next_state = STAGE_2;
    				end
    				STAGE_2: begin
						ry_sel = 0;
						load_ry = 1;
						next_state = STAGE_3;
    				end
    				STAGE_3: begin
    					rf_in_sel = 0;
    					addr_c = rd;
    					rf_we = 1;
    					inc_sel = 0;
    					pc_sel = 1;
    					load_pc = 1;
    					next_state = STAGE_0;
    				end
    			endcase
    		end
    		INS_BRANCH: begin
				case(current_state)
					STAGE_1: begin
						load_decoding = 1;
						addr_a = rs1;
    					addr_b = rs2;
    					load_ra = 1;
    					load_rb = 1;
    					next_state = STAGE_2;
    				end
    				STAGE_2: begin
    					op1_sel = 0;
    					op2_sel = 0;
    					alu_sel = 3'd1;
    					load_rr = 1;
    					next_state = STAGE_3;
    				end
    				STAGE_3: begin
    					imm_pc = immediate_data;
    					pc_sel = 1;
    					load_pc = 1;
    					case(func3)
    						3'd0: inc_sel = EQ;
    						3'd1: inc_sel = ~EQ;
    						3'd4: inc_sel = LT;
    						3'd5: inc_sel = ~LT;
    						3'd6: inc_sel = LTU;
    						3'd7: inc_sel = ~LTU;
    					endcase
    					next_state = STAGE_0;
    				end
    			endcase
    		end
    		INS_JP_LK: begin
				case(current_state)
					STAGE_1: begin
						load_decoding = 1;
						op1_sel = 1;
    					op2_sel = 1;
    					imm_op2 = 32'd4;
    					alu_sel = 3'd0;
    					load_rr = 1;
    					next_state = STAGE_2;
    				end
    				STAGE_2: begin
    					load_ry = 1;
    					ry_sel = 0;
    					next_state = STAGE_3;
    				end
    				STAGE_3: begin
    					rf_in_sel = 0;
    					addr_c = rd;
    					rf_we = 1;
    					imm_pc = immediate_data;
    					inc_sel = 1;
    					pc_sel = 1;
    					load_pc = 1;
    					next_state = STAGE_0;
    				end
    			endcase
    		end
    		INS_JP_LK_R: begin
    			case(current_state)
					STAGE_1: begin
						load_decoding = 1;
						addr_a = rs1;
    					load_ra = 1;
    					inc_sel = 0;
    					pc_sel = 1;
    					load_pc = 1;
    					next_state = STAGE_2;
    				end
    				STAGE_2: begin
    					load_pc_temp = 1;
    					pc_sel = 0;
    					load_pc = 1;
    					next_state = STAGE_3;
    				end
    				STAGE_3: begin
    					ry_sel = 2'd2;
    					load_ry = 1;
    					next_state = STAGE_4;
    				end
    				STAGE_4: begin
    					rf_in_sel = 0;
    					addr_c = rd;
    					rf_we = 1;
    					imm_pc = immediate_data;
    					inc_sel = 1;
    					pc_sel = 1;
    					load_pc = 1;
    					next_state = STAGE_0;
    				end
    			endcase
    		end
    	endcase
    end
endmodule

