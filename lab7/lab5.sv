module ALU (
	input [31:0] RA,
	input [31:0] RB,
	input [4:0] rs2,
	input [4:0] rs1,
	input [4:0] rd,
	//logic [31:0] imm;
	input [6:0] opcode,
	input [6:0] funct7,
	input [2:0] funct3,
	output logic [31:0] RZ
	);
	

	
	logic signed [31:0] a, b, s;
	logic z, n, v, c;
	assign a = RA;
	assign b = ~RB;
	assign {c, s} = a + b;
	assign z = ~|s;
	assign n = s[31];
	assign v = a[31]^b[31]^s[31]^c;
	
	
	always_comb begin
		case(opcode) 
			7'b0110011, 7'b0010011: //1,2 arithmetic
			case(funct3)
				3'h0: 
					if (opcode == 7'b0110011 && funct7 == 7'h20)	RZ = RA - RB;
					else RZ = RA + RB;
				3'h1:
					RZ = RA << RB[4:0]; //sll, slli
				3'h2: //slt
					RZ = ($signed(RA) < $signed(RB)) ? 32'b1 : 32'b0;//slt slti
				3'h3: //sltu 
					RZ = (RA < RB) ? 32'b1 : 32'b0; //sltu sltiu
				3'h4:
					RZ = RA ^ RB; //XOR
				3'h5:
					begin
					case(funct7) 
						7'h00: RZ = RA >> RB[4:0];//srl . srli
						7'h20: RZ = $signed(RA) >>> RB[4:0];//sra, srai
					endcase
					end
				3'h6:
					RZ = RA | RB;
				3'h7:
					RZ = RA & RB;
				default: RZ = 0;
			endcase
			7'b0000011, 7'b0100011, 7'b0110111, 7'b0010111: //ld, st, U, U
				RZ = RA + RB;
			7'b1100011: //B
			case(funct3)
				3'h0:
					RZ = {31'b0, (RA == RB)};
				3'h1:
					RZ = {31'b0, (RA != RB)};
				3'h4:
					RZ = ($signed(RA) < $signed(RB)) ? 32'b1 : 32'b0;
				3'h5:
					RZ = ($signed(RA) < $signed(RB)) ? 32'b0 : 32'b1;
				3'h6:
					RZ = (RA < RB) ? 32'b1 : 32'b0;
				3'h7:
					RZ = (RA < RB) ? 32'b0 : 32'b1;
				default: RZ = 0;
			endcase
			7'b1101111: //J may be deleted // jal
				RZ = RA + RB;
			7'b1100111: //jalr
				RZ = RA + RB;
			default: RZ = 0;	
		endcase	
	end
endmodule	


module cpu # (
	parameter IW = 32, // instr width
	parameter REGS = 32 // number of registers
)(
	input clk,
	input reset,
	
	// read only port
	output logic [IW-1:0] o_pc_addr,
	output logic o_pc_rd,
	input [IW-1:0] i_pc_rddata,
	output logic[3:0] o_pc_byte_en,
	
	// read/write port
	output logic [IW-1:0] o_ldst_addr,
	output logic o_ldst_rd,
	output logic o_ldst_wr,
	input [IW-1:0] i_ldst_rddata,
	output logic [IW-1:0] o_ldst_wrdata,
	output logic [3:0] o_ldst_byte_en,
	
	output logic [IW-1:0] o_tb_regs [0:REGS-1]	
);

	
	//instruction decoded results:
	typedef struct packed {
		logic [6:0] opcode;
		logic [2:0] funct3;
		logic [6:0] funct7;
		logic [4:0] rs1;
		logic [4:0] rs2;
		logic [4:0] rd;
		logic [31:0] imm;
		logic [31:0] pc_addr;
	} PPL_ID;

	//
	typedef struct packed {
		logic [4:0] rd;
		logic [31:0] RZ;  //result from ALU
		logic [31:0] RY;  // for jal and jalr
		logic [6:0] opcode; 
		logic [2:0] funct3;
		//logic pc_change;
		//logic pc_addr;
		
	} PPL_EX;
	
	logic [31:0] m_pc_addr;
	logic [31:0] m_pc_addr_prev;
	logic m_pc_change;
	logic m_pc_change_d;
	logic [31:0] m_instruction;
	logic [4:0] m_rs1;
	logic [4:0] m_rs2;
	logic [6:0] m_opcode;
	logic [31:0] m_imm;
	logic [31:0] m_RZ; 
	logic [31:0] m_pc_addr_branch;  //branch purpose
	logic [31:0] m_alu_RA;
	logic [31:0] m_alu_RB;
	logic abcdef;
	logic [31:0] m_pc_rddata;
	
	
	//PPL_IR IR_reg;
	PPL_ID ID_reg;
	PPL_EX EX_reg;
	
	
	always_comb
	begin
		if(reset) 
		begin
			o_tb_regs[0] = 32'b0;
			o_tb_regs[1] = 32'b0;
			o_tb_regs[2] = 32'b0;
			o_tb_regs[3] = 32'b0;
			o_tb_regs[4] = 32'b0;
			o_tb_regs[5] = 32'b0;
			o_tb_regs[6] = 32'b0;
			o_tb_regs[7] = 32'b0;
			o_tb_regs[8] = 32'b0;
			o_tb_regs[9] = 32'b0;
			o_tb_regs[10] = 32'b0;
			o_tb_regs[11] = 32'b0;
			o_tb_regs[12] = 32'b0;
			o_tb_regs[13] = 32'b0;
			o_tb_regs[14] = 32'b0;
			o_tb_regs[15] = 32'b0;
			o_tb_regs[16] = 32'b0;
			o_tb_regs[17] = 32'b0;
			o_tb_regs[18] = 32'b0;
			o_tb_regs[19] = 32'b0;
			o_tb_regs[20] = 32'b0;
			o_tb_regs[21] = 32'b0;
			o_tb_regs[22] = 32'b0;
			o_tb_regs[23] = 32'b0;
			o_tb_regs[24] = 32'b0;
			o_tb_regs[25] = 32'b0;
			o_tb_regs[26] = 32'b0;
			o_tb_regs[27] = 32'b0;
			o_tb_regs[28] = 32'b0;
			o_tb_regs[29] = 32'b0;
			o_tb_regs[30] = 32'b0;
			o_tb_regs[31] = 32'b0;
			
			m_instruction = 32'bx;;
			m_rs1 = 5'bx;
			m_rs2 = 5'bx;
			m_opcode = 7'bx;
			m_imm = 32'bx;
			
			m_pc_addr_branch = 32'bx;
			m_alu_RA = 32'bx;
			m_alu_RB = 32'bx;
			m_pc_change = 0;
			abcdef = 0;
		end
	
		else 
		begin
		
			m_instruction = m_pc_rddata;
			//o_pc_rd = 0;
			//instruction = i_pc_rddata;
			m_rs2 = m_instruction[24:20];
			m_rs1 = m_instruction[19:15];
			//m_rd = instruction[11:7];
			m_opcode = m_instruction[6:0];
			//funct3 = instruction[14:12];

		
			case(m_opcode) 
				7'b0110011: //R
					begin
					m_imm = 32'b0;
					end
				7'b0010011, 7'b0000011: //I 
					begin
					m_imm = {{20{m_instruction[31]}}, m_instruction[31:20]};
					end
				7'b0100011: //S
					begin
					m_imm = {{20{m_instruction[31]}}, m_instruction[31:25], m_instruction[11:7]};
					end
				7'b1100011: //B
					begin
					m_imm =  {{19{m_instruction[31]}}, m_instruction[31], m_instruction[7], m_instruction[30:25], m_instruction[11:8], 1'b0};
					end
				7'b0110111, 7'b0010111: //U
					begin
					m_imm = {12'b0, m_instruction[31:12]} << 12;
					end
				7'b1101111: //J: jal
					begin
					m_imm = {{11{m_instruction[31]}}, m_instruction[31], m_instruction[19:12], m_instruction[20], m_instruction[30:21], 1'b0};
					end
				7'b1100111: //jalr
					begin
					m_imm = {{20{m_instruction[31]}}, m_instruction[31:20]};
					end
				default: m_imm = 0;	
			endcase
			
			
			
			//write process
			case(EX_reg.opcode) 
				7'b0110011, 7'b0010011: //arithmetic
				begin
					if (EX_reg.rd != 5'b0)
						o_tb_regs[EX_reg.rd] = EX_reg.RZ;
					else o_tb_regs[EX_reg.rd] = o_tb_regs[EX_reg.rd];
				end
				7'b0000011: //load unfinished!..finished
				begin
					case(EX_reg.funct3)
					3'h0: 
						o_tb_regs[EX_reg.rd] = {{24{i_ldst_rddata[7]}}, i_ldst_rddata[7:0]};  //data load to rd
					3'h1:
						o_tb_regs[EX_reg.rd] = {{16{i_ldst_rddata[7]}}, i_ldst_rddata[15:0]};  //data load to rd
					3'h2:
						o_tb_regs[EX_reg.rd] = i_ldst_rddata;  //data load to rd
					3'h4:
						o_tb_regs[EX_reg.rd] = {24'b0, i_ldst_rddata[7:0]}; 
					3'h5:
						o_tb_regs[EX_reg.rd] = {16'b0, i_ldst_rddata[15:0]};
					default: o_tb_regs[EX_reg.rd] = o_tb_regs[EX_reg.rd];
					endcase
				end
				7'b0100011: //store 
				begin
					//done!
					o_ldst_wr = 1'b0;
				end
				7'b0110111, 7'b0010111: //U
				begin 
					if (EX_reg.rd != 5'b0)
						o_tb_regs[EX_reg.rd] = EX_reg.RZ;
					else o_tb_regs[EX_reg.rd] = o_tb_regs[EX_reg.rd];
				end
	
				default: o_tb_regs[EX_reg.rd] = o_tb_regs[EX_reg.rd];
				endcase
		
			
			
			//for fetching alu_RA late, so that if the current ALU input requires the previous instruction result
			if (ID_reg.opcode == 7'b0110111) m_alu_RA = 32'b0; //lui
			else if (ID_reg.opcode == 7'b0010111) m_alu_RA = ID_reg.pc_addr; //auipc
			else if (ID_reg.opcode == 7'b1101111) m_alu_RA = ID_reg.pc_addr; //jal
			else if (ID_reg.opcode == 7'b1100111) m_alu_RA = ID_reg.pc_addr; //jalr
			else m_alu_RA = o_tb_regs[ID_reg.rs1];
			
			case(ID_reg.opcode) 
				7'b0110011: //R
					begin
					m_alu_RB = o_tb_regs[ID_reg.rs2];
					end
				7'b0010011, 7'b0000011: //I 
					begin
					m_alu_RB = ID_reg.imm;
					end
				7'b0100011: //S
					begin
					m_alu_RB = ID_reg.imm;
					end
				7'b1100011: //B
					begin
					m_alu_RB = o_tb_regs[ID_reg.rs2];
					end
				7'b0110111, 7'b0010111: //U
					begin
					m_alu_RB = ID_reg.imm;
					end
				7'b1101111: //J: jal
					begin
					m_alu_RB = 32'd4;
					end
				7'b1100111: //jalr
					begin
					m_alu_RB = 32'd4;
					end
				default: m_alu_RB = 0;	
			endcase	
			
			
			case(ID_reg.opcode)
			
			7'b0000011: //load
			begin
				m_pc_change = 0;
				o_ldst_rd = 1;
				o_ldst_addr = m_RZ;
				case(ID_reg.funct3)
				3'h0, 3'h4: 
					o_ldst_byte_en = 4'b0001;
				3'h1, 3'h5:
					o_ldst_byte_en = 4'b0011;
				3'h2:
					o_ldst_byte_en = 4'b1111;
				default: ;
				endcase
			end
			7'b0100011: //store
			begin
				m_pc_change = 0;
				o_ldst_wr = 1;
				o_ldst_wrdata = o_tb_regs[ID_reg.rs2];/////////////need to change!!!
				o_ldst_addr = m_RZ;
				case(ID_reg.funct3)
				3'h0: 
					o_ldst_byte_en = 4'b0001;
				3'h1:
					o_ldst_byte_en = 4'b0011;
				3'h2:
					o_ldst_byte_en = 4'b1111;
				default: ;
				endcase
			end

			7'b1100011: //branch B
			begin
				//done!
				if (m_RZ[0] == 1) 
				begin
					m_pc_change = 1;
					m_pc_addr_branch = ID_reg.pc_addr + ID_reg.imm;
				end
				else m_pc_change = 0;
			end
			7'b1101111: //jal
			begin
				//done!
				if (ID_reg.rd != 5'b0) 	
					o_tb_regs[ID_reg.rd] = m_RZ; //the PC + 4 is stored in RZ
				m_pc_change = 1;
				m_pc_addr_branch = ID_reg.pc_addr + ID_reg.imm;
			end
			7'b1100111: //jalr
			begin
				//done!
				if (ID_reg.rd != 5'b0)
					o_tb_regs[ID_reg.rd] = m_RZ; //the PC + 4 is stored in RZ
				m_pc_change = 1;
				m_pc_addr_branch =o_tb_regs[ID_reg.rs1] + ID_reg.imm;
				//next_state = 3'b000;
			end
			default: 
			begin
				m_pc_change = 0;
				o_ldst_wr = 0;
				o_ldst_rd = 0;
			end
			endcase
		
		end
		
	end
	
	
	ALU a1(
	//.RA(ID_reg.RA),
	//.RB(ID_reg.RB),
	.RA(m_alu_RA),
	.RB(m_alu_RB),
	.rs2(ID_reg.rs2),
	.rs1(ID_reg.rs1),
	.rd(ID_reg.rd),
	.opcode(ID_reg.opcode),
	.funct3(ID_reg.funct3),
	.funct7(ID_reg.funct7),
	.RZ(m_RZ)
	);
	
	always_comb
	begin
		if (reset)
			m_pc_rddata = 32'bx;
		else if (!m_pc_change_d)
			m_pc_rddata = i_pc_rddata;
		else 
			m_pc_rddata = 32'bx;
	end

	
	always@ (posedge clk) 
	begin
	if (reset) begin
		m_pc_addr <= 0;
		o_pc_rd <= 0;
		o_pc_byte_en <= 4'b0;
		//IR_reg.instruction <= 32'bx;
		//IR_reg.pc_addr <= 32'bx;
	
		ID_reg.opcode <= 7'bx;
		ID_reg.funct3 <= 3'bx;
		ID_reg.funct7 <= 7'bx;
		ID_reg.rs1 <= 5'bx;
		ID_reg.rs2 <= 5'bx;
		ID_reg.rd <= 5'bx;
		ID_reg.pc_addr <= 32'bx;
		ID_reg.imm <= 32'bx;
		
		EX_reg.rd <= 5'bx;
		EX_reg.opcode <= 7'bx;
		EX_reg.funct3 <= 3'bx;
		//EX_reg.pc_change <= 1'bx;
		EX_reg.RZ <= 32'bx;
		EX_reg.RY <= 32'bx;
		m_pc_change_d <= m_pc_change;
		end
	
	else begin
		m_pc_change_d <= m_pc_change;
		//PC fetches instruction
		if (!m_pc_change) begin
			o_pc_rd <= 1;
			o_pc_byte_en <= 4'b1111;
			o_pc_addr <= m_pc_addr;
			
			m_pc_addr <= m_pc_addr + 4; //increment at the next cycle
			m_pc_addr_prev <= o_pc_addr;  //save the previous pc_addr
			
		   end
		else begin
			//branch exceptions
			o_pc_rd <= 1;
			o_pc_byte_en <= 4'b1111;
			o_pc_addr <= m_pc_addr_branch;
			m_pc_addr <= m_pc_addr_branch + 4; //increment at the next cycle
			m_pc_addr_prev <= o_pc_addr;  //save the previous pc_addr	
			//m_pc_rddata = 32'bx;
			end
			
		//store the instruction into IR_reg
		//IR_reg.instruction <= m_pc_change ? 32'bx : m_pc_rddata;
		//IR_reg.pc_addr <= m_pc_change ? 32'bx : m_pc_addr_prev;
		 
		//decode process
		ID_reg.opcode <= m_pc_change ? 7'bx :m_opcode;
		ID_reg.funct3 <= m_pc_change ? 3'bx :m_instruction[14:12];
		ID_reg.funct7 <= m_pc_change ? 7'bx :m_instruction[31:25];
		ID_reg.rs1 <= m_pc_change ? 5'bx :m_instruction[19:15];
		ID_reg.rs2 <= m_pc_change ? 5'bx :m_instruction[24:20];
		ID_reg.rd <= m_pc_change ? 5'bx :m_instruction[11:7];
		ID_reg.pc_addr <= m_pc_change ? 32'bx : m_pc_addr_prev;

		ID_reg.imm <= m_imm;
		
		//execution process		
			
		if( m_pc_change == 1'b1)  begin
			EX_reg.RZ <= 32'bx;
			EX_reg.RZ <= 32'bx;
		end
		else begin
		
		case(ID_reg.opcode)
			7'b0110011, 7'b0010011: //arithmetic
			begin
				EX_reg.RY <= 32'bx;
				EX_reg.RZ <= m_RZ;     //////its correct!@@@@@@_@@@@@@@
			end
			7'b0000011: //load
			begin
				EX_reg.RY <= 32'bx;
				EX_reg.RZ <= 32'bx;
			end
			7'b0100011: //store
			begin
				EX_reg.RY <= 32'bx;
				EX_reg.RZ <= 32'bx;
			end
			7'b0110111, 7'b0010111: //U  lui, auipc
			begin
				EX_reg.RY <= 32'bx;
				EX_reg.RZ <= m_RZ;
			end
			7'b1100011: //branch B
			begin
				EX_reg.RZ <= m_RZ;
				if (m_RZ[0] == 1'b0) EX_reg.RY <= ID_reg.pc_addr + {32'd4};
				else EX_reg.RY <= ID_reg.pc_addr + ID_reg.imm;
			end
			7'b1101111: //jal
			begin
				EX_reg.RY <= ID_reg.pc_addr + ID_reg.imm; //pc+imm
				EX_reg.RZ <= m_RZ;
			end
				
			7'b1100111: //jalr
			begin
				EX_reg.RY <= o_tb_regs[ID_reg.rs1] + ID_reg.imm; //rs1 + imm
				EX_reg.RZ <= m_RZ;
			end
			default: 
			begin
				EX_reg.RY <= 32'bx;
				EX_reg.RZ <= 32'bx;
			end
			endcase
		end
		
		EX_reg.rd <= m_pc_change ? 5'bx :ID_reg.rd;
		EX_reg.opcode <= m_pc_change ? 7'bx :ID_reg.opcode;
		EX_reg.funct3 <= m_pc_change ? 3'bx :ID_reg.funct3;
		//EX_reg.pc_addr <= ID_reg.pc_addr;
		//EX_reg.pc_change <= m_pc_change ? 0:1;
	
		end
		
	end
		

endmodule












