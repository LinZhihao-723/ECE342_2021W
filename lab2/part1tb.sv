
// Used to test the multiplier row, which is actually an adder
module tb_multiplier_row();
	logic [7:0] P_in;
	logic [7:0] M_in;
	logic [7:0] Q_in;
	logic [7:0] P_out;
	logic C_out;
	multiplier_row #(.N(8)) DUT(P_in, M_in, Q_in, P_out, C_out);
	initial begin
		P_in = -8'd48;
		M_in = -8'd75;
		Q_in = 2'b00;
		#5
		if(P_out == P_in)
			$display("Passed!");
		else 
			$display("Fail!");
		#5;
		Q_in = 2'b01;
		#5;
		if(P_out == P_in + M_in)
			$display("Passed!");
		else 
			$display("Fail!");
		#5;
		Q_in = 2'b10;
		#5;
		if(P_out == P_in - M_in)
			$display("Passed!");
		else 
			$display("Fail!");
		$stop();
	end
endmodule

// Used to test the multiplier row, which is actually an adder
module tb_array_multiplier();
	logic [7:0] X_in;
	logic [7:0] Y_in;
	logic [15:0] Z_out;
	logic C_out;
	array_multiplier #(.X(8), .Y(8)) DUT(X_in, Y_in, Z_out);
	initial begin
		// X_in = -8'd48;
		// Y_in = 8'd75;
		// #50
		// $display("Z: %d, X: %d, Y: %d", Z_out, $signed(X_in), $signed(Y_in));

		// if($signed(Z_out) == $signed(X_in) * $signed(Y_in))
		// //if(Z_out == X_in * Y_in)
		// 	$display("Passed!");
		// else 
		// 	$display("Fail!");
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