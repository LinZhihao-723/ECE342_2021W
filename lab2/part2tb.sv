
module tb_multiplier_row();
	logic [7:0] x_in;
	logic [7:0] y_in;
	logic [7:0] s_out;
	logic c_out;
	carry_lookahead_adder #(.N(8)) DUT(x_in, y_in, {1'b0}, s_out, c_out);
	initial begin
		x_in = -8'd48;
		y_in = -8'd75;
		#5
		if(s_out == x_in + y_in)
			$display("Passed!");
		else 
			$display("Fail!");
		$stop();
	end
endmodule


// Used to test the multiplier row, which is actually an adder
module tb_carry_saved_multiplier();
	logic [7:0] X_in;
	logic [7:0] Y_in;
	logic [15:0] Z_out;
	logic C_out;
	carry_saved_multiplier #(.N(8)) DUT(X_in, Y_in, Z_out);
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