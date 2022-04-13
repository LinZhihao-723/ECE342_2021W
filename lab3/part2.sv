//bias = 8'd127

module part2
	(
		input [31:0] X,
		input [31:0] Y,
		output reg inf, nan, zero, overflow, underflow,
		output reg [31:0] result
);
	logic sign_bit;
	logic [47:0] mantissa_multiplication;
	logic [23:0] X_fraction;
	logic [23:0] Y_fraction;

	assign sign_bit = X[31] ^ Y[31];
	assign X_fraction = {1'b1, X[22:0]};
	assign Y_fraction = {1'b1, Y[22:0]};
	assign mantissa_multiplication = X_fraction * Y_fraction;

	logic [8:0] temp_exponents;
	logic [22:0] temp_mantissa;
	logic [7:0] real_exponents;
	assign real_exponents = temp_exponents - 8'd127;

	always_comb begin
		if(mantissa_multiplication[47] == 1'b1) begin
			temp_exponents = X[30:23] + Y[30:23] + 1'b1;
			temp_mantissa = mantissa_multiplication[46:24];
		end else begin				
			temp_exponents = X[30:23] + Y[30:23];	
			temp_mantissa = mantissa_multiplication[45:23];
		end

		if( (X[30:23] == 8'd255 && X[22:0] != 0) || (Y[30:23] == 8'd255 && Y[22:0] != 0) ) begin
			// NaN * sth
			inf = 0;
			nan = 1;
			overflow = 0;
			zero = 0;
			underflow = 0;
			result = {1'b0, {8{1'b1}}, {23{1'b0}}};
		end else if ( (X[30:23] == 0 && X[22:0] == 0) || (Y[30:23] == 0 && Y[22:0] == 0) ) begin
			// Zero * sth
			inf = 0;
			nan = 0;
			overflow = 0;
			zero = 1;
			underflow = 0;
			result = 0;
		end else if ( (X[30:23] == 8'd255 && X[22:0] == 0) || (Y[30:23] == 8'd255 && Y[22:0] == 0) ) begin
			// Inf * sth
			inf = 1;
			nan = 0;
			overflow = 0;
			zero = 0;
			underflow = 0;
			result = {1'b0, {8{1'b1}}, {23{1'b0}}};
		end else begin 
			if(temp_exponents <= 9'd127) begin
				result = 0;
				inf = 0;
				nan = 0;
				overflow = 0;
				zero = (temp_exponents == 9'd127) & (temp_mantissa == 0);
				underflow = temp_exponents < 9'd127;
			end else if(temp_exponents >= 9'd382) begin
				result = {1'b0, 8'b11111111, 23'b0};
				inf = (temp_exponents == 9'd382) & (temp_mantissa == 0);
				nan = (temp_exponents == 9'd382) & (temp_mantissa != 0);
				overflow = (temp_exponents > 9'd382);
				zero = 0;
				underflow = 0;
			end else begin
				inf = 0;
				nan = 0;
				overflow = 0;
				zero = 0;
				underflow = 0;
				result = {sign_bit, real_exponents, temp_mantissa};
			end
		end 
 	end

endmodule
