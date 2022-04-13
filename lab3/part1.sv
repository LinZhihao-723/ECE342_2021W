

module part1
	(
		input [31:0] X,
		input [31:0] Y,
		output reg [31:0] result
);

// Design your 32-bit Floating Point unit here. 
	logic sign_bit;
	logic [8:0] exponents_addtion;
	logic [47:0] mantissa_multiplication;
	logic [23:0] X_fraction;
	logic [23:0] Y_fraction;

	assign exponents_addtion = X[30:23] + Y[30:23] - 8'd127;
	assign sign_bit = X[31] ^ Y[31];
	assign X_fraction = {1'b1, X[22:0]};
	assign Y_fraction = {1'b1, Y[22:0]};
	assign mantissa_multiplication = X_fraction * Y_fraction;

	always_comb begin
		result[31] = sign_bit;
		if(mantissa_multiplication[47] == 1'b1) begin
			result[30:23] = exponents_addtion + 1'b1;
			result[22:0] = mantissa_multiplication[46:24];
		end else begin
			result[30:23] = exponents_addtion;
			result[22:0] = mantissa_multiplication[45:23];
		end
 	end
endmodule
