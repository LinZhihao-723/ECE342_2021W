
// Used to test the multiplier row, which is actually an adder
module tb();
	logic clk;
	logic reset;
	logic [4:0] address_a_in, address_b_in, address_c_in;
	logic [31:0] data_in;
	logic rf_write_in;
	logic [31:0] rf_list_out [0:31];
	logic [31:0] reg_a_out, reg_b_out;

	register_file DUT(
		clk,
		reset,
		address_a_in,
		address_b_in,
		address_c_in,
		data_in,
		rf_write_in,
		rf_list_out,
		reg_a_out,
		reg_b_out
	);
	always begin
        #1 clk = !clk;
    end

	initial begin
		clk = 0;
		reset = 1;
		#2;
		reset = 0;
		data_in = 32'hffffffaa;
		rf_write_in = 1'b1;
		address_c_in = 5'd10;
		#2;
		data_in = 32'hacf2255b;
		rf_write_in = 1'b1;
		address_c_in = 5'd17;
		#2;
		address_a_in = 5'd10;
		address_b_in = 5'd17;
		#2;
		$display("Value of A: %x", reg_a_out);
		$display("Value of B: %x", reg_b_out);
		#2;
		data_in = 32'hcccccccc;
		rf_write_in = 1'b1;
		address_c_in = 5'd17;
		#2;
		address_a_in = 5'd10;
		address_b_in = 5'd17;
		#2;
		$display("Value of A: %x", reg_a_out);
		$display("Value of B: %x", reg_b_out);
		#2;
		address_a_in = 5'd10;
		address_b_in = 5'd08;
		#2;
		$display("Value of A: %x", reg_a_out);
		$display("Value of B: %x", reg_b_out);
		$stop();
	end
endmodule