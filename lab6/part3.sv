module part3
(
    input                       clk,
    input				        reset
);
	logic [31:0] o_pc_addr;
	logic o_pc_rd;
	logic [31:0] i_pc_rddata;
	logic [3:0] o_pc_byte_en;
	logic [31:0] o_ldst_addr;
	logic o_ldst_rd;
	logic o_ldst_wr;
	logic [31:0] i_ldst_rddata;
	logic [31:0] o_ldst_wrdata;
	logic [3:0] o_ldst_byte_en;
	logic i_ldst_waitrequest;
	logic [31:0] o_tb_regs [0:31];

	cpu cpu_chip (
		.clk(clk), 
		.reset(reset),
		.o_pc_addr(o_pc_addr),
		.o_pc_rd(o_pc_rd),
		.i_pc_rddata(i_pc_rddata),
		.o_pc_byte_en(o_pc_byte_en),
		.o_ldst_addr(o_ldst_addr),
		.o_ldst_rd(o_ldst_rd),
		.o_ldst_wr(o_ldst_wr),
		.i_ldst_rddata(i_ldst_rddata),
		.o_ldst_wrdata(o_ldst_wrdata),
		.o_ldst_byte_en(o_ldst_byte_en),
		.i_ldst_waitrequest(i_ldst_waitrequest),
		.o_tb_regs(o_tb_regs)
	);

	logic [12:0] p1_addr;
	logic p1_read;      
	logic p1_write;     
	logic [3:0] p1_byteenable;
	logic [31:0] p1_writedata; 
	logic [31:0] p1_readdata;   	/* Output */ 
	logic [12:0] p2_addr;      		
	logic p2_read;      
	logic [3:0] p2_byteenable;
	logic [31:0] p2_readdata;  		/* Output */

	mem mem_inst (
		.clk          (clk),
		.reset        (reset),
		.p1_addr      (p1_addr),
		.p1_read      (p1_read),
		.p1_write     (p1_write),
		.p1_byteenable(p1_byteenable),
		.p1_writedata (p1_writedata),
		.p1_readdata  (p1_readdata),
		.p2_addr      (p2_addr),
		.p2_read      (p2_read),
		.p2_byteenable(p2_byteenable),
		.p2_readdata  (p2_readdata)
	);

	logic [2:0] avs_s1_address; 
	logic avs_s1_read;
	logic avs_s1_write;
	logic [31:0] avs_s1_writedata;
	logic [31:0] avs_s1_readdata;
	logic avs_s1_waitrequest;

	avalon_fp_mult fp(
		.clk               (clk),
		.reset             (reset),
		.avs_s1_address    (avs_s1_address),
		.avs_s1_read       (avs_s1_read),
		.avs_s1_write      (avs_s1_write),
		.avs_s1_writedata  (avs_s1_writedata),
		.avs_s1_readdata   (avs_s1_readdata),
		.avs_s1_waitrequest(avs_s1_waitrequest)
	);

	logic [1:0] device;
	always_comb begin
		/* Device */
		if(o_ldst_addr < 32'h8000) device = 0;
		else if(o_ldst_addr >= 32'hA000 && o_ldst_addr <= 32'hA020) device = 1;
		else device = 3;
	end

	/* Fp */
	always_comb begin
		if(device == 1) begin
			avs_s1_address = o_ldst_addr[4:2];
			avs_s1_read = o_ldst_rd;
			avs_s1_write = o_ldst_wr;
			avs_s1_writedata = o_ldst_wrdata;
		end else begin
			avs_s1_address = 0;
			avs_s1_read = 0;
			avs_s1_write = 0;
			avs_s1_writedata = 0;
		end
	end

	/* Wait signal */
	always_comb begin
		if(device == 1) i_ldst_waitrequest = avs_s1_waitrequest;
		else i_ldst_waitrequest = 0;
	end

	/* Connect memory */
	assign p2_addr = o_pc_addr[14:2];
	assign p2_read = o_pc_rd;
	assign p2_byteenable = o_pc_byte_en;
	assign p1_addr = o_ldst_addr[14:2];
	assign p1_read = o_ldst_rd;
	assign p1_write = o_ldst_wr & ~device[0] & ~device[1];
	assign p1_byteenable = o_ldst_byte_en;
	assign p1_writedata = o_ldst_wrdata;

	/* PC read */
	always_ff @(posedge clk) begin
		if(reset) begin
			i_pc_rddata <= 0;
		end else if(o_pc_rd) begin
			i_pc_rddata <= p2_readdata;
		end else begin
			i_pc_rddata <= 0;
		end
	end

	/* Ldst read */
	always_ff @(posedge clk) begin
		if(reset) begin
			i_ldst_rddata <= 0;
		end else if(o_ldst_rd) begin
			if(device == 0) i_ldst_rddata <= p1_readdata;
			else if(device == 1) i_ldst_rddata <= avs_s1_readdata;
			else i_ldst_rddata <= 0;
		end else begin
			i_ldst_rddata <= 0;
		end
	end
endmodule

module mem # (
	parameter WIDTH = 32,
	parameter DEPTH = 8192,
	parameter HEX_FILE = "part3.hex"
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
	input  i_ldst_waitrequest,
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
		.ldst_byte_en(o_ldst_byte_en),
		.ldst_waitrequest(i_ldst_waitrequest)
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
	input ldst_waitrequest,

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
        else if(ldst_waitrequest == 0) current_state <= next_state;
        else current_state <= current_state;
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

module avalon_fp_mult
(
	input clk,
	input reset,
	
	// Avalon Slave
	input [2:0] avs_s1_address,
	input avs_s1_read,
	input avs_s1_write,
	input [31:0] avs_s1_writedata,
	output logic [31:0] avs_s1_readdata,
	output logic avs_s1_waitrequest
);
	logic [3:0] status_connection;
	logic [31:0] result_connection;
	logic [31:0] input_1_connection;
	logic [31:0] input_2_connection;
	logic mult_start_connection;

	avalon_slave avs
	(
		.clk(clk),
		.reset(reset),
		.avs_s1_address(avs_s1_address),
		.avs_s1_read(avs_s1_read),
		.avs_s1_write(avs_s1_write),
		.avs_s1_writedata(avs_s1_writedata),
		.avs_s1_readdata(avs_s1_readdata),
		.avs_s1_waitrequest(avs_s1_waitrequest),
		.status(status_connection),
		.result(result_connection),
		.input_1(input_1_connection),
		.input_2(input_2_connection),
		.mult_start(mult_start_connection)
	);


	fp_mult fpm
	(
        .aclr(reset),
        .clk_en(mult_start_connection),
        .clock(clk),
        .dataa(input_1_connection),
        .datab(input_2_connection),
        .nan(status_connection[0]),
        .zero(status_connection[1]),
        .underflow(status_connection[2]),
        .overflow(status_connection[3]),
        .result(result_connection)
	);

endmodule

module avalon_slave
(
	input clk,
	input reset,
	
	// Avalon Slave
	input [2:0] avs_s1_address,
	input avs_s1_read,
	input avs_s1_write,
	input [31:0] avs_s1_writedata,
	output logic [31:0] avs_s1_readdata,
	output logic avs_s1_waitrequest,

	input [3:0] status,
	input [31:0] result,
	output logic [31:0] input_1,
	output logic [31:0] input_2,
	output logic mult_start
);

	// Mem-mapped regs
	// Reg0-32:			A
	// Reg1-32:			B
	// Reg2-04:			Start/Busy
	// Reg3-32:			Result
	// Reg4-04:			Status (Flags)
	reg [31:0] mul_op_1;
	reg [31:0] mul_op_2;
	reg [31:0] mul_start;
	reg [31:0] mul_result;
	reg [31:0] mul_status;
	reg [7:0] counter;

	enum int unsigned
    {
        S_NORMAL,
        S_WAIT
    } current_state, next_state;

	assign mult_start = mul_start[0];
	assign input_1 = mul_op_1;
	assign input_2 = mul_op_2;

	always_comb begin
		case (avs_s1_address)
			3'b000: avs_s1_readdata = mul_op_1;
			3'b001: avs_s1_readdata = mul_op_2;
			3'b010: avs_s1_readdata = mul_start;
			3'b011: avs_s1_readdata = mul_result;
			3'b100: avs_s1_readdata = mul_status;
			default: avs_s1_readdata = 0;
		endcase
	end

	always_ff @(posedge clk) begin
		if(reset) begin
			mul_op_1 <= 0;
			mul_op_2 <= 0;
			mul_start <= 0;
			mul_result <= 0;
			mul_status <= 0;
			counter <= 11;
		end else begin
			if(!mult_start) begin 
				if(avs_s1_write) begin
					case (avs_s1_address)
						3'b000: mul_op_1 <= avs_s1_writedata;
						3'b001: mul_op_2 <= avs_s1_writedata;
						3'b010: mul_start <= avs_s1_writedata;
						3'b011: mul_result <= avs_s1_writedata;
						3'b100: mul_status <= avs_s1_writedata;
					endcase
				end
			end

			if(current_state == S_NORMAL) begin
				counter <= 11;
			end else if(current_state == S_WAIT) begin
				counter <= counter - 1;
			end

			if(counter == 0) begin 
				/* It ends */
				mul_start <= 0;
				mul_status <= status;
				mul_result <= result;
			end
		end
	end

	always_ff @(posedge clk) begin
		if(reset) current_state <= S_NORMAL;
		else current_state <= next_state;
	end

	always_comb begin
		case(current_state)
			S_NORMAL: begin
				next_state = mult_start ? S_WAIT : S_NORMAL;
				avs_s1_waitrequest = 1'b0;
			end 
			S_WAIT: begin
				next_state = (counter == 0) ? S_NORMAL : S_WAIT;
				avs_s1_waitrequest = 1'b1;
			end
		endcase
	end

endmodule

// megafunction wizard: %ALTFP_MULT%
// GENERATION: STANDARD
// VERSION: WM1.0
// MODULE: ALTFP_MULT 

// ============================================================
// File Name: fp_mult.v
// Megafunction Name(s):
// 			ALTFP_MULT
//
// Simulation Library Files(s):
// 			lpm
// ============================================================
// ************************************************************
// THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
//
// 18.0.0 Build 614 04/24/2018 SJ Lite Edition
// ************************************************************


//Copyright (C) 2018  Intel Corporation. All rights reserved.
//Your use of Intel Corporation's design tools, logic functions 
//and other software and tools, and its AMPP partner logic 
//functions, and any output files from any of the foregoing 
//(including device programming or simulation files), and any 
//associated documentation or information are expressly subject 
//to the terms and conditions of the Intel Program License 
//Subscription Agreement, the Intel Quartus Prime License Agreement,
//the Intel FPGA IP License Agreement, or other applicable license
//agreement, including, without limitation, that your use is for
//the sole purpose of programming logic devices manufactured by
//Intel and sold by Intel or its authorized distributors.  Please
//refer to the applicable agreement for further details.


//altfp_mult CBX_AUTO_BLACKBOX="ALL" DEDICATED_MULTIPLIER_CIRCUITRY="YES" DENORMAL_SUPPORT="NO" DEVICE_FAMILY="Cyclone V" EXCEPTION_HANDLING="NO" PIPELINE=11 REDUCED_FUNCTIONALITY="NO" ROUNDING="TO_NEAREST" WIDTH_EXP=8 WIDTH_MAN=23 aclr clk_en clock dataa datab nan overflow result underflow zero
//VERSION_BEGIN 18.0 cbx_alt_ded_mult_y 2018:04:24:18:04:18:SJ cbx_altbarrel_shift 2018:04:24:18:04:18:SJ cbx_altera_mult_add 2018:04:24:18:04:18:SJ cbx_altera_mult_add_rtl 2018:04:24:18:04:18:SJ cbx_altfp_mult 2018:04:24:18:04:18:SJ cbx_altmult_add 2018:04:24:18:04:18:SJ cbx_cycloneii 2018:04:24:18:04:18:SJ cbx_lpm_add_sub 2018:04:24:18:04:18:SJ cbx_lpm_compare 2018:04:24:18:04:18:SJ cbx_lpm_mult 2018:04:24:18:04:18:SJ cbx_mgl 2018:04:24:18:08:49:SJ cbx_nadder 2018:04:24:18:04:18:SJ cbx_padd 2018:04:24:18:04:18:SJ cbx_parallel_add 2018:04:24:18:04:18:SJ cbx_stratix 2018:04:24:18:04:18:SJ cbx_stratixii 2018:04:24:18:04:18:SJ cbx_util_mgl 2018:04:24:18:04:18:SJ  VERSION_END
// synthesis VERILOG_INPUT_VERSION VERILOG_2001
// altera message_off 10463


//synthesis_resources = lpm_add_sub 4 lpm_mult 1 reg 297 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fp_mult_altfp_mult_her
	( 
	aclr,
	clk_en,
	clock,
	dataa,
	datab,
	nan,
	overflow,
	result,
	underflow,
	zero) ;
	input   aclr;
	input   clk_en;
	input   clock;
	input   [31:0]  dataa;
	input   [31:0]  datab;
	output   nan;
	output   overflow;
	output   [31:0]  result;
	output   underflow;
	output   zero;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
	tri0   aclr;
	tri1   clk_en;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

	reg	dataa_exp_all_one_ff_p1;
	reg	dataa_exp_not_zero_ff_p1;
	reg	dataa_man_not_zero_ff_p1;
	reg	dataa_man_not_zero_ff_p2;
	reg	datab_exp_all_one_ff_p1;
	reg	datab_exp_not_zero_ff_p1;
	reg	datab_man_not_zero_ff_p1;
	reg	datab_man_not_zero_ff_p2;
	reg	[9:0]	delay_exp2_bias;
	reg	[9:0]	delay_exp3_bias;
	reg	[9:0]	delay_exp_bias;
	reg	delay_man_product_msb;
	reg	delay_man_product_msb2;
	reg	delay_man_product_msb_p0;
	reg	delay_man_product_msb_p1;
	reg	[23:0]	delay_round;
	reg	[8:0]	exp_add_p1;
	reg	[9:0]	exp_adj_p1;
	reg	[9:0]	exp_adj_p2;
	reg	[8:0]	exp_bias_p1;
	reg	[8:0]	exp_bias_p2;
	reg	[8:0]	exp_bias_p3;
	reg	[7:0]	exp_result_ff;
	reg	input_is_infinity_dffe_0;
	reg	input_is_infinity_dffe_1;
	reg	input_is_infinity_dffe_2;
	reg	input_is_infinity_dffe_3;
	reg	input_is_infinity_ff1;
	reg	input_is_infinity_ff2;
	reg	input_is_infinity_ff3;
	reg	input_is_infinity_ff4;
	reg	input_is_infinity_ff5;
	reg	input_is_nan_dffe_0;
	reg	input_is_nan_dffe_1;
	reg	input_is_nan_dffe_2;
	reg	input_is_nan_dffe_3;
	reg	input_is_nan_ff1;
	reg	input_is_nan_ff2;
	reg	input_is_nan_ff3;
	reg	input_is_nan_ff4;
	reg	input_is_nan_ff5;
	reg	input_not_zero_dffe_0;
	reg	input_not_zero_dffe_1;
	reg	input_not_zero_dffe_2;
	reg	input_not_zero_dffe_3;
	reg	input_not_zero_ff1;
	reg	input_not_zero_ff2;
	reg	input_not_zero_ff3;
	reg	input_not_zero_ff4;
	reg	input_not_zero_ff5;
	reg	lsb_dffe;
	reg	[22:0]	man_result_ff;
	reg	man_round_carry;
	reg	man_round_carry_p0;
	reg	[23:0]	man_round_p;
	reg	[23:0]	man_round_p0;
	reg	[23:0]	man_round_p1;
	reg	[24:0]	man_round_p2;
	reg	nan_ff;
	reg	overflow_ff;
	reg	round_dffe;
	reg	[0:0]	sign_node_ff0;
	reg	[0:0]	sign_node_ff1;
	reg	[0:0]	sign_node_ff2;
	reg	[0:0]	sign_node_ff3;
	reg	[0:0]	sign_node_ff4;
	reg	[0:0]	sign_node_ff5;
	reg	[0:0]	sign_node_ff6;
	reg	[0:0]	sign_node_ff7;
	reg	[0:0]	sign_node_ff8;
	reg	[0:0]	sign_node_ff9;
	reg	[0:0]	sign_node_ff10;
	reg	sticky_dffe;
	reg	underflow_ff;
	reg	zero_ff;
	wire  [8:0]   wire_exp_add_adder_result;
	wire  [9:0]   wire_exp_adj_adder_result;
	wire  [9:0]   wire_exp_bias_subtr_result;
	wire  [24:0]   wire_man_round_adder_result;
	wire  [47:0]   wire_man_product2_mult_result;
	wire  [9:0]  bias;
	wire  [7:0]  dataa_exp_all_one;
	wire  [7:0]  dataa_exp_not_zero;
	wire  [22:0]  dataa_man_not_zero;
	wire  [7:0]  datab_exp_all_one;
	wire  [7:0]  datab_exp_not_zero;
	wire  [22:0]  datab_man_not_zero;
	wire  exp_is_inf;
	wire  exp_is_zero;
	wire  [9:0]  expmod;
	wire  [7:0]  inf_num;
	wire  lsb_bit;
	wire  [24:0]  man_shift_full;
	wire  [7:0]  result_exp_all_one;
	wire  [8:0]  result_exp_not_zero;
	wire  round_bit;
	wire  round_carry;
	wire  [22:0]  sticky_bit;

	// synopsys translate_off
	initial
		dataa_exp_all_one_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_exp_all_one_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_exp_all_one_ff_p1 <= dataa_exp_all_one[7];
	// synopsys translate_off
	initial
		dataa_exp_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_exp_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_exp_not_zero_ff_p1 <= dataa_exp_not_zero[7];
	// synopsys translate_off
	initial
		dataa_man_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_man_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_man_not_zero_ff_p1 <= dataa_man_not_zero[10];
	// synopsys translate_off
	initial
		dataa_man_not_zero_ff_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_man_not_zero_ff_p2 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_man_not_zero_ff_p2 <= dataa_man_not_zero[22];
	// synopsys translate_off
	initial
		datab_exp_all_one_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_exp_all_one_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_exp_all_one_ff_p1 <= datab_exp_all_one[7];
	// synopsys translate_off
	initial
		datab_exp_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_exp_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_exp_not_zero_ff_p1 <= datab_exp_not_zero[7];
	// synopsys translate_off
	initial
		datab_man_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_man_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_man_not_zero_ff_p1 <= datab_man_not_zero[10];
	// synopsys translate_off
	initial
		datab_man_not_zero_ff_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_man_not_zero_ff_p2 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_man_not_zero_ff_p2 <= datab_man_not_zero[22];
	// synopsys translate_off
	initial
		delay_exp2_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp2_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp2_bias <= delay_exp_bias;
	// synopsys translate_off
	initial
		delay_exp3_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp3_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp3_bias <= delay_exp2_bias;
	// synopsys translate_off
	initial
		delay_exp_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp_bias <= wire_exp_bias_subtr_result;
	// synopsys translate_off
	initial
		delay_man_product_msb = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb <= delay_man_product_msb_p1;
	// synopsys translate_off
	initial
		delay_man_product_msb2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb2 <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb2 <= delay_man_product_msb;
	// synopsys translate_off
	initial
		delay_man_product_msb_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb_p0 <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb_p0 <= wire_man_product2_mult_result[47];
	// synopsys translate_off
	initial
		delay_man_product_msb_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb_p1 <= delay_man_product_msb_p0;
	// synopsys translate_off
	initial
		delay_round = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_round <= 24'b0;
		else if  (clk_en == 1'b1)   delay_round <= ((man_round_p2[23:0] & {24{(~ man_round_p2[24])}}) | (man_round_p2[24:1] & {24{man_round_p2[24]}}));
	// synopsys translate_off
	initial
		exp_add_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_add_p1 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_add_p1 <= wire_exp_add_adder_result;
	// synopsys translate_off
	initial
		exp_adj_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_adj_p1 <= 10'b0;
		else if  (clk_en == 1'b1)   exp_adj_p1 <= delay_exp3_bias;
	// synopsys translate_off
	initial
		exp_adj_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_adj_p2 <= 10'b0;
		else if  (clk_en == 1'b1)   exp_adj_p2 <= wire_exp_adj_adder_result;
	// synopsys translate_off
	initial
		exp_bias_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_bias_p1 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_bias_p1 <= exp_add_p1[8:0];
	// synopsys translate_off
	initial
		exp_bias_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_bias_p2 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_bias_p2 <= exp_bias_p1;
	// synopsys translate_off
	initial
		exp_bias_p3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_bias_p3 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_bias_p3 <= exp_bias_p2;
	// synopsys translate_off
	initial
		exp_result_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_result_ff <= 8'b0;
		else if  (clk_en == 1'b1)   exp_result_ff <= ((inf_num & {8{((exp_is_inf | input_is_infinity_ff5) | input_is_nan_ff5)}}) | ((exp_adj_p2[7:0] & {8{(~ exp_is_zero)}}) & {8{input_not_zero_ff5}}));
	// synopsys translate_off
	initial
		input_is_infinity_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_0 <= ((dataa_exp_all_one_ff_p1 & (~ (dataa_man_not_zero_ff_p1 | dataa_man_not_zero_ff_p2))) | (datab_exp_all_one_ff_p1 & (~ (datab_man_not_zero_ff_p1 | datab_man_not_zero_ff_p2))));
	// synopsys translate_off
	initial
		input_is_infinity_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_1 <= input_is_infinity_dffe_0;
	// synopsys translate_off
	initial
		input_is_infinity_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_2 <= input_is_infinity_dffe_1;
	// synopsys translate_off
	initial
		input_is_infinity_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_3 <= input_is_infinity_dffe_2;
	// synopsys translate_off
	initial
		input_is_infinity_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff1 <= input_is_infinity_dffe_3;
	// synopsys translate_off
	initial
		input_is_infinity_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff2 <= input_is_infinity_ff1;
	// synopsys translate_off
	initial
		input_is_infinity_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff3 <= input_is_infinity_ff2;
	// synopsys translate_off
	initial
		input_is_infinity_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff4 <= input_is_infinity_ff3;
	// synopsys translate_off
	initial
		input_is_infinity_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff5 <= input_is_infinity_ff4;
	// synopsys translate_off
	initial
		input_is_nan_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_0 <= ((dataa_exp_all_one_ff_p1 & (dataa_man_not_zero_ff_p1 | dataa_man_not_zero_ff_p2)) | (datab_exp_all_one_ff_p1 & (datab_man_not_zero_ff_p1 | datab_man_not_zero_ff_p2)));
	// synopsys translate_off
	initial
		input_is_nan_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_1 <= input_is_nan_dffe_0;
	// synopsys translate_off
	initial
		input_is_nan_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_2 <= input_is_nan_dffe_1;
	// synopsys translate_off
	initial
		input_is_nan_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_3 <= input_is_nan_dffe_2;
	// synopsys translate_off
	initial
		input_is_nan_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff1 <= input_is_nan_dffe_3;
	// synopsys translate_off
	initial
		input_is_nan_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff2 <= input_is_nan_ff1;
	// synopsys translate_off
	initial
		input_is_nan_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff3 <= input_is_nan_ff2;
	// synopsys translate_off
	initial
		input_is_nan_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff4 <= input_is_nan_ff3;
	// synopsys translate_off
	initial
		input_is_nan_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff5 <= input_is_nan_ff4;
	// synopsys translate_off
	initial
		input_not_zero_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_0 <= (dataa_exp_not_zero_ff_p1 & datab_exp_not_zero_ff_p1);
	// synopsys translate_off
	initial
		input_not_zero_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_1 <= input_not_zero_dffe_0;
	// synopsys translate_off
	initial
		input_not_zero_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_2 <= input_not_zero_dffe_1;
	// synopsys translate_off
	initial
		input_not_zero_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_3 <= input_not_zero_dffe_2;
	// synopsys translate_off
	initial
		input_not_zero_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff1 <= input_not_zero_dffe_3;
	// synopsys translate_off
	initial
		input_not_zero_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff2 <= input_not_zero_ff1;
	// synopsys translate_off
	initial
		input_not_zero_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff3 <= input_not_zero_ff2;
	// synopsys translate_off
	initial
		input_not_zero_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff4 <= input_not_zero_ff3;
	// synopsys translate_off
	initial
		input_not_zero_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff5 <= input_not_zero_ff4;
	// synopsys translate_off
	initial
		lsb_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) lsb_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   lsb_dffe <= lsb_bit;
	// synopsys translate_off
	initial
		man_result_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_result_ff <= 23'b0;
		else if  (clk_en == 1'b1)   man_result_ff <= {((((((delay_round[22] & input_not_zero_ff5) & (~ input_is_infinity_ff5)) & (~ exp_is_inf)) & (~ exp_is_zero)) | (input_is_infinity_ff5 & (~ input_not_zero_ff5))) | input_is_nan_ff5), (((((delay_round[21:0] & {22{input_not_zero_ff5}}) & {22{(~ input_is_infinity_ff5)}}) & {22{(~ exp_is_inf)}}) & {22{(~ exp_is_zero)}}) & {22{(~ input_is_nan_ff5)}})};
	// synopsys translate_off
	initial
		man_round_carry = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_carry <= 1'b0;
		else if  (clk_en == 1'b1)   man_round_carry <= man_round_carry_p0;
	// synopsys translate_off
	initial
		man_round_carry_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_carry_p0 <= 1'b0;
		else if  (clk_en == 1'b1)   man_round_carry_p0 <= round_carry;
	// synopsys translate_off
	initial
		man_round_p = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p <= 24'b0;
		else if  (clk_en == 1'b1)   man_round_p <= man_shift_full[24:1];
	// synopsys translate_off
	initial
		man_round_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p0 <= 24'b0;
		else if  (clk_en == 1'b1)   man_round_p0 <= man_round_p;
	// synopsys translate_off
	initial
		man_round_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p1 <= 24'b0;
		else if  (clk_en == 1'b1)   man_round_p1 <= man_round_p0;
	// synopsys translate_off
	initial
		man_round_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p2 <= 25'b0;
		else if  (clk_en == 1'b1)   man_round_p2 <= wire_man_round_adder_result;
	// synopsys translate_off
	initial
		nan_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) nan_ff <= 1'b0;
		else if  (clk_en == 1'b1)   nan_ff <= (input_is_nan_ff5 | (input_is_infinity_ff5 & (~ input_not_zero_ff5)));
	// synopsys translate_off
	initial
		overflow_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) overflow_ff <= 1'b0;
		else if  (clk_en == 1'b1)   overflow_ff <= (((exp_is_inf | input_is_infinity_ff5) & (~ input_is_nan_ff5)) & (~ (input_is_infinity_ff5 & (~ input_not_zero_ff5))));
	// synopsys translate_off
	initial
		round_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) round_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   round_dffe <= round_bit;
	// synopsys translate_off
	initial
		sign_node_ff0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff0 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff0 <= (dataa[31] ^ datab[31]);
	// synopsys translate_off
	initial
		sign_node_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff1 <= sign_node_ff0[0:0];
	// synopsys translate_off
	initial
		sign_node_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff2 <= sign_node_ff1[0:0];
	// synopsys translate_off
	initial
		sign_node_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff3 <= sign_node_ff2[0:0];
	// synopsys translate_off
	initial
		sign_node_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff4 <= sign_node_ff3[0:0];
	// synopsys translate_off
	initial
		sign_node_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff5 <= sign_node_ff4[0:0];
	// synopsys translate_off
	initial
		sign_node_ff6 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff6 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff6 <= sign_node_ff5[0:0];
	// synopsys translate_off
	initial
		sign_node_ff7 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff7 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff7 <= sign_node_ff6[0:0];
	// synopsys translate_off
	initial
		sign_node_ff8 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff8 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff8 <= sign_node_ff7[0:0];
	// synopsys translate_off
	initial
		sign_node_ff9 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff9 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff9 <= sign_node_ff8[0:0];
	// synopsys translate_off
	initial
		sign_node_ff10 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff10 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff10 <= sign_node_ff9[0:0];
	// synopsys translate_off
	initial
		sticky_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sticky_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   sticky_dffe <= sticky_bit[22];
	// synopsys translate_off
	initial
		underflow_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) underflow_ff <= 1'b0;
		else if  (clk_en == 1'b1)   underflow_ff <= (((exp_is_zero & input_not_zero_ff5) & (~ input_is_nan_ff5)) & (~ (input_is_infinity_ff5 & (~ input_not_zero_ff5))));
	// synopsys translate_off
	initial
		zero_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) zero_ff <= 1'b0;
		else if  (clk_en == 1'b1)   zero_ff <= (((exp_is_zero | (~ input_not_zero_ff5)) & (~ input_is_nan_ff5)) & (~ input_is_infinity_ff5));
	lpm_add_sub   exp_add_adder
	( 
	.aclr(aclr),
	.cin(1'b0),
	.clken(clk_en),
	.clock(clock),
	.cout(),
	.dataa({1'b0, dataa[30:23]}),
	.datab({1'b0, datab[30:23]}),
	.overflow(),
	.result(wire_exp_add_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.add_sub(1'b1)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_add_adder.lpm_pipeline = 1,
		exp_add_adder.lpm_width = 9,
		exp_add_adder.lpm_type = "lpm_add_sub";
	lpm_add_sub   exp_adj_adder
	( 
	.cin(1'b0),
	.cout(),
	.dataa(exp_adj_p1),
	.datab({expmod[9:0]}),
	.overflow(),
	.result(wire_exp_adj_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_adj_adder.lpm_pipeline = 0,
		exp_adj_adder.lpm_width = 10,
		exp_adj_adder.lpm_type = "lpm_add_sub";
	lpm_add_sub   exp_bias_subtr
	( 
	.cout(),
	.dataa({1'b0, exp_bias_p3}),
	.datab({bias[9:0]}),
	.overflow(),
	.result(wire_exp_bias_subtr_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_bias_subtr.lpm_direction = "SUB",
		exp_bias_subtr.lpm_pipeline = 0,
		exp_bias_subtr.lpm_representation = "UNSIGNED",
		exp_bias_subtr.lpm_width = 10,
		exp_bias_subtr.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_round_adder
	( 
	.cout(),
	.dataa({1'b0, man_round_p1}),
	.datab({{24{1'b0}}, man_round_carry}),
	.overflow(),
	.result(wire_man_round_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		man_round_adder.lpm_pipeline = 0,
		man_round_adder.lpm_width = 25,
		man_round_adder.lpm_type = "lpm_add_sub";
	lpm_mult   man_product2_mult
	( 
	.aclr(aclr),
	.clken(clk_en),
	.clock(clock),
	.dataa({1'b1, dataa[22:0]}),
	.datab({1'b1, datab[22:0]}),
	.result(wire_man_product2_mult_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.sclr(1'b0),
	.sum({1{1'b0}})
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		man_product2_mult.lpm_pipeline = 5,
		man_product2_mult.lpm_representation = "UNSIGNED",
		man_product2_mult.lpm_widtha = 24,
		man_product2_mult.lpm_widthb = 24,
		man_product2_mult.lpm_widthp = 48,
		man_product2_mult.lpm_widths = 1,
		man_product2_mult.lpm_type = "lpm_mult",
		man_product2_mult.lpm_hint = "DEDICATED_MULTIPLIER_CIRCUITRY=YES";
	assign
		bias = {{3{1'b0}}, {7{1'b1}}},
		dataa_exp_all_one = {(dataa[30] & dataa_exp_all_one[6]), (dataa[29] & dataa_exp_all_one[5]), (dataa[28] & dataa_exp_all_one[4]), (dataa[27] & dataa_exp_all_one[3]), (dataa[26] & dataa_exp_all_one[2]), (dataa[25] & dataa_exp_all_one[1]), (dataa[24] & dataa_exp_all_one[0]), dataa[23]},
		dataa_exp_not_zero = {(dataa[30] | dataa_exp_not_zero[6]), (dataa[29] | dataa_exp_not_zero[5]), (dataa[28] | dataa_exp_not_zero[4]), (dataa[27] | dataa_exp_not_zero[3]), (dataa[26] | dataa_exp_not_zero[2]), (dataa[25] | dataa_exp_not_zero[1]), (dataa[24] | dataa_exp_not_zero[0]), dataa[23]},
		dataa_man_not_zero = {(dataa[22] | dataa_man_not_zero[21]), (dataa[21] | dataa_man_not_zero[20]), (dataa[20] | dataa_man_not_zero[19]), (dataa[19] | dataa_man_not_zero[18]), (dataa[18] | dataa_man_not_zero[17]), (dataa[17] | dataa_man_not_zero[16]), (dataa[16] | dataa_man_not_zero[15]), (dataa[15] | dataa_man_not_zero[14]), (dataa[14] | dataa_man_not_zero[13]), (dataa[13] | dataa_man_not_zero[12]), (dataa[12] | dataa_man_not_zero[11]), dataa[11], (dataa[10] | dataa_man_not_zero[9]), (dataa[9] | dataa_man_not_zero[8]), (dataa[8] | dataa_man_not_zero[7]), (dataa[7] | dataa_man_not_zero[6]), (dataa[6] | dataa_man_not_zero[5]), (dataa[5] | dataa_man_not_zero[4]), (dataa[4] | dataa_man_not_zero[3]), (dataa[3] | dataa_man_not_zero[2]), (dataa[2] | dataa_man_not_zero[1]), (dataa[1] | dataa_man_not_zero[0]), dataa[0]},
		datab_exp_all_one = {(datab[30] & datab_exp_all_one[6]), (datab[29] & datab_exp_all_one[5]), (datab[28] & datab_exp_all_one[4]), (datab[27] & datab_exp_all_one[3]), (datab[26] & datab_exp_all_one[2]), (datab[25] & datab_exp_all_one[1]), (datab[24] & datab_exp_all_one[0]), datab[23]},
		datab_exp_not_zero = {(datab[30] | datab_exp_not_zero[6]), (datab[29] | datab_exp_not_zero[5]), (datab[28] | datab_exp_not_zero[4]), (datab[27] | datab_exp_not_zero[3]), (datab[26] | datab_exp_not_zero[2]), (datab[25] | datab_exp_not_zero[1]), (datab[24] | datab_exp_not_zero[0]), datab[23]},
		datab_man_not_zero = {(datab[22] | datab_man_not_zero[21]), (datab[21] | datab_man_not_zero[20]), (datab[20] | datab_man_not_zero[19]), (datab[19] | datab_man_not_zero[18]), (datab[18] | datab_man_not_zero[17]), (datab[17] | datab_man_not_zero[16]), (datab[16] | datab_man_not_zero[15]), (datab[15] | datab_man_not_zero[14]), (datab[14] | datab_man_not_zero[13]), (datab[13] | datab_man_not_zero[12]), (datab[12] | datab_man_not_zero[11]), datab[11], (datab[10] | datab_man_not_zero[9]), (datab[9] | datab_man_not_zero[8]), (datab[8] | datab_man_not_zero[7]), (datab[7] | datab_man_not_zero[6]), (datab[6] | datab_man_not_zero[5]), (datab[5] | datab_man_not_zero[4]), (datab[4] | datab_man_not_zero[3]), (datab[3] | datab_man_not_zero[2]), (datab[2] | datab_man_not_zero[1]), (datab[1] | datab_man_not_zero[0]), datab[0]},
		exp_is_inf = (((~ exp_adj_p2[9]) & exp_adj_p2[8]) | ((~ exp_adj_p2[8]) & result_exp_all_one[7])),
		exp_is_zero = (exp_adj_p2[9] | (~ result_exp_not_zero[8])),
		expmod = {{8{1'b0}}, (delay_man_product_msb2 & man_round_p2[24]), (delay_man_product_msb2 ^ man_round_p2[24])},
		inf_num = {8{1'b1}},
		lsb_bit = man_shift_full[1],
		man_shift_full = ((wire_man_product2_mult_result[46:22] & {25{(~ wire_man_product2_mult_result[47])}}) | (wire_man_product2_mult_result[47:23] & {25{wire_man_product2_mult_result[47]}})),
		nan = nan_ff,
		overflow = overflow_ff,
		result = {sign_node_ff10[0:0], exp_result_ff[7:0], man_result_ff[22:0]},
		result_exp_all_one = {(result_exp_all_one[6] & exp_adj_p2[7]), (result_exp_all_one[5] & exp_adj_p2[6]), (result_exp_all_one[4] & exp_adj_p2[5]), (result_exp_all_one[3] & exp_adj_p2[4]), (result_exp_all_one[2] & exp_adj_p2[3]), (result_exp_all_one[1] & exp_adj_p2[2]), (result_exp_all_one[0] & exp_adj_p2[1]), exp_adj_p2[0]},
		result_exp_not_zero = {(result_exp_not_zero[7] | exp_adj_p2[8]), (result_exp_not_zero[6] | exp_adj_p2[7]), (result_exp_not_zero[5] | exp_adj_p2[6]), (result_exp_not_zero[4] | exp_adj_p2[5]), (result_exp_not_zero[3] | exp_adj_p2[4]), (result_exp_not_zero[2] | exp_adj_p2[3]), (result_exp_not_zero[1] | exp_adj_p2[2]), (result_exp_not_zero[0] | exp_adj_p2[1]), exp_adj_p2[0]},
		round_bit = man_shift_full[0],
		round_carry = (round_dffe & (lsb_dffe | sticky_dffe)),
		sticky_bit = {(sticky_bit[21] | (wire_man_product2_mult_result[47] & wire_man_product2_mult_result[22])), (sticky_bit[20] | wire_man_product2_mult_result[21]), (sticky_bit[19] | wire_man_product2_mult_result[20]), (sticky_bit[18] | wire_man_product2_mult_result[19]), (sticky_bit[17] | wire_man_product2_mult_result[18]), (sticky_bit[16] | wire_man_product2_mult_result[17]), (sticky_bit[15] | wire_man_product2_mult_result[16]), (sticky_bit[14] | wire_man_product2_mult_result[15]), (sticky_bit[13] | wire_man_product2_mult_result[14]), (sticky_bit[12] | wire_man_product2_mult_result[13]), (sticky_bit[11] | wire_man_product2_mult_result[12]), (sticky_bit[10] | wire_man_product2_mult_result[11]), (sticky_bit[9] | wire_man_product2_mult_result[10]), (sticky_bit[8] | wire_man_product2_mult_result[9]), (sticky_bit[7] | wire_man_product2_mult_result[8]), (sticky_bit[6] | wire_man_product2_mult_result[7]), (sticky_bit[5] | wire_man_product2_mult_result[6]), (sticky_bit[4] | wire_man_product2_mult_result[5]), (sticky_bit[3] | wire_man_product2_mult_result[4]), (sticky_bit[2] | wire_man_product2_mult_result[3]), (sticky_bit[1] | wire_man_product2_mult_result[2]), (sticky_bit[0] | wire_man_product2_mult_result[1]), wire_man_product2_mult_result[0]},
		underflow = underflow_ff,
		zero = zero_ff;
endmodule //fp_mult_altfp_mult_her
//VALID FILE


// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module fp_mult (
	aclr,
	clk_en,
	clock,
	dataa,
	datab,
	nan,
	overflow,
	result,
	underflow,
	zero);

	input	  aclr;
	input	  clk_en;
	input	  clock;
	input	[31:0]  dataa;
	input	[31:0]  datab;
	output	  nan;
	output	  overflow;
	output	[31:0]  result;
	output	  underflow;
	output	  zero;

	wire  sub_wire0;
	wire  sub_wire1;
	wire [31:0] sub_wire2;
	wire  sub_wire3;
	wire  sub_wire4;
	wire  nan = sub_wire0;
	wire  overflow = sub_wire1;
	wire [31:0] result = sub_wire2[31:0];
	wire  underflow = sub_wire3;
	wire  zero = sub_wire4;

	fp_mult_altfp_mult_her	fp_mult_altfp_mult_her_component (
				.aclr (aclr),
				.clk_en (clk_en),
				.clock (clock),
				.dataa (dataa),
				.datab (datab),
				.nan (sub_wire0),
				.overflow (sub_wire1),
				.result (sub_wire2),
				.underflow (sub_wire3),
				.zero (sub_wire4));

endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: PRIVATE: FPM_FORMAT STRING "Single"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone V"
// Retrieval info: CONSTANT: DEDICATED_MULTIPLIER_CIRCUITRY STRING "YES"
// Retrieval info: CONSTANT: DENORMAL_SUPPORT STRING "NO"
// Retrieval info: CONSTANT: EXCEPTION_HANDLING STRING "NO"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "UNUSED"
// Retrieval info: CONSTANT: LPM_HINT STRING "UNUSED"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altfp_mult"
// Retrieval info: CONSTANT: PIPELINE NUMERIC "11"
// Retrieval info: CONSTANT: REDUCED_FUNCTIONALITY STRING "NO"
// Retrieval info: CONSTANT: ROUNDING STRING "TO_NEAREST"
// Retrieval info: CONSTANT: WIDTH_EXP NUMERIC "8"
// Retrieval info: CONSTANT: WIDTH_MAN NUMERIC "23"
// Retrieval info: USED_PORT: aclr 0 0 0 0 INPUT NODEFVAL "aclr"
// Retrieval info: CONNECT: @aclr 0 0 0 0 aclr 0 0 0 0
// Retrieval info: USED_PORT: clk_en 0 0 0 0 INPUT NODEFVAL "clk_en"
// Retrieval info: CONNECT: @clk_en 0 0 0 0 clk_en 0 0 0 0
// Retrieval info: USED_PORT: clock 0 0 0 0 INPUT NODEFVAL "clock"
// Retrieval info: CONNECT: @clock 0 0 0 0 clock 0 0 0 0
// Retrieval info: USED_PORT: dataa 0 0 32 0 INPUT NODEFVAL "dataa[31..0]"
// Retrieval info: CONNECT: @dataa 0 0 32 0 dataa 0 0 32 0
// Retrieval info: USED_PORT: datab 0 0 32 0 INPUT NODEFVAL "datab[31..0]"
// Retrieval info: CONNECT: @datab 0 0 32 0 datab 0 0 32 0
// Retrieval info: USED_PORT: nan 0 0 0 0 OUTPUT NODEFVAL "nan"
// Retrieval info: CONNECT: nan 0 0 0 0 @nan 0 0 0 0
// Retrieval info: USED_PORT: overflow 0 0 0 0 OUTPUT NODEFVAL "overflow"
// Retrieval info: CONNECT: overflow 0 0 0 0 @overflow 0 0 0 0
// Retrieval info: USED_PORT: result 0 0 32 0 OUTPUT NODEFVAL "result[31..0]"
// Retrieval info: CONNECT: result 0 0 32 0 @result 0 0 32 0
// Retrieval info: USED_PORT: underflow 0 0 0 0 OUTPUT NODEFVAL "underflow"
// Retrieval info: CONNECT: underflow 0 0 0 0 @underflow 0 0 0 0
// Retrieval info: USED_PORT: zero 0 0 0 0 OUTPUT NODEFVAL "zero"
// Retrieval info: CONNECT: zero 0 0 0 0 @zero 0 0 0 0
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.v TRUE FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.qip TRUE FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.bsf TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult_inst.v FALSE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult_bb.v FALSE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.inc FALSE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.cmp FALSE TRUE
// Retrieval info: LIB_FILE: lpm
