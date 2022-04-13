
module tb_wallace_tree_row();
	logic [7:0] row_1;
	logic [7:0] row_2;
	logic [7:0] row_3;
	logic [7:0] sum;
	logic [7:0] carry_out;
	wallace_tree_row #(.N(8)) DUT(row_1, row_2, row_3, sum, carry_out);
	initial begin
		row_1 = 8'b10011100;
		row_2 = 8'b01010110;
		row_3 = 8'b11000001;
		#5
		if(sum == 8'b00001011 && carry_out == 8'b10101000)
			$display("Passed!");
		else 
			$display("Fail!");
		$stop();
	end
endmodule


// Used to test the multiplier row, which is actually an adder
module tb_wallace_tree_multiplier_8_bit();
	logic [7:0] X_in;
	logic [7:0] Y_in;
	logic [15:0] Z_out;
	logic C_out;
	wallace_tree_multiplier_8_bit DUT(X_in, Y_in, Z_out);
	initial begin
		// X_in = -8'd74;
		// Y_in = -8'd99;
		// #100
		// $display("Z: %d, X: %d, Y: %d", $signed(Z_out), $signed(X_in), $signed(Y_in));

		// if($signed(Z_out) == $signed(X_in) * $signed(Y_in))
		// //if(Z_out == X_in * Y_in)
		// 	$display("Passed!");
		// else 
		// 	$display("Fail!");
		// $stop();

		for(integer i = -128; i < 128; i ++) begin
			for(integer j = -128; j < 128; j ++) begin
				X_in = i[7:0];
				Y_in = j[7:0];
				#1
				if($signed(Z_out) != i * j) begin
					$display("Error Occured!");
					$display("Z: %d, X: %d, Y: %d", $signed(Z_out), $signed(X_in), $signed(Y_in));
					$stop();
				end
			end
		end
		$display("All tests passed!");
		$stop();
		
	end
endmodule