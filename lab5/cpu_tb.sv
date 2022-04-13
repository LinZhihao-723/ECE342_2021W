
// Used to test the multiplier row, which is actually an adder
module tb();
	logic clk;
	logic reset;
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
	logic [31:0] o_tb_regs [0:31];
	
	cpu DUT(
	 clk,
	 reset,
	 o_pc_addr,
	 o_pc_rd,
	 i_pc_rddata,
	 o_pc_byte_en,
	 o_ldst_addr,
	 o_ldst_rd,
	 o_ldst_wr,
	 i_ldst_rddata,
	 o_ldst_wrdata,
	 o_ldst_byte_en,
	 o_tb_regs
	);

	always begin
        #1 clk = !clk;
    end

	initial begin
		clk = 0;
	 	reset = 0;
	 	#2;
		
		$stop();
	end
endmodule
