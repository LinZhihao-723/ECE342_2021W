// This module uses parameterized instantiation. If values are not set in the testbench the default values specified here are used. 
// The values for EXP and MAN set here are for a IEEE-754 32-bit Floating point representation. 
// TO DO: Edit BITS and BIAS based on the EXP and MAN parameters. 

module part3
	#(parameter EXP = 8,			// Number of bits in the Exponent
	  parameter MAN = 23, 			// Number of bits in the Mantissa
	  parameter BITS = EXP + MAN + 1,	// Total number of bits in the floating point number
	  parameter BIAS = (1'b1 << (EXP - 1)) - 1	// Value of the bias, based on the exponent. 
	  )
	(
		input [BITS - 1:0] X,
		input [BITS - 1:0] Y,
		output reg inf, nan, zero, overflow, underflow,
		output reg[BITS - 1:0] result
);

	logic sign_bit;
	logic [MAN*2+1:0] mantissa_multiplication;
	logic [MAN:0] X_fraction;
	logic [MAN:0] Y_fraction;

	assign sign_bit = X[BITS-1] ^ Y[BITS-1];
	assign X_fraction = {1'b1, X[MAN-1:0]};
	assign Y_fraction = {1'b1, Y[MAN-1:0]};
	assign mantissa_multiplication = X_fraction * Y_fraction;

	logic [EXP:0] temp_exponents;
	logic [MAN-1:0] temp_mantissa;
	logic [EXP-1:0] real_exponents;
	assign real_exponents = temp_exponents - BIAS;

	always_comb begin
		if(mantissa_multiplication[MAN*2+1] == 1'b1) begin
			temp_exponents = X[MAN+EXP-1:MAN] + Y[MAN+EXP-1:MAN] + 1'b1;
			temp_mantissa = mantissa_multiplication[MAN*2:MAN*2-MAN+1];
		end else begin				
			temp_exponents = X[MAN+EXP-1:MAN] + Y[MAN+EXP-1:MAN] ;	
			temp_mantissa = mantissa_multiplication[MAN*2-1:MAN];
		end

		if( (X[MAN+EXP-1:MAN] == {EXP{1'b1}} && X[MAN-1:0] != 0) || (Y[MAN+EXP-1:MAN] == {EXP{1'b1}} && Y[MAN-1:0] != 0) ) begin
			// NaN * sth
			inf = 0;
			nan = 1;
			overflow = 0;
			zero = 0;
			underflow = 0;
			result = {1'b0, {EXP{1'b1}}, {MAN{1'b0}}};
		end else if ( (X[MAN+EXP-1:0] == 0) || (Y[MAN+EXP-1:0] == 0) ) begin
			// Zero * sth
			inf = 0;
			nan = 0;
			overflow = 0;
			zero = 1;
			underflow = 0;
			result = 0;
		end else if ( (X[MAN+EXP-1:MAN] == {EXP{1'b1}} && X[MAN-1:0] == 0) || (Y[MAN+EXP-1:MAN] == {EXP{1'b1}} && Y[MAN-1:0] == 0) ) begin
			// Inf * sth
			inf = 1;
			nan = 0;
			overflow = 0;
			zero = 0;
			underflow = 0;
			result = {1'b0, {EXP{1'b1}}, {MAN{1'b0}}};
		end else begin 
			if(temp_exponents <= BIAS) begin
				result = 0;
				inf = 0;
				nan = 0;
				overflow = 0;
				zero = (temp_exponents == BIAS) & (temp_mantissa == 0);
				underflow = temp_exponents < BIAS;
			end else if(temp_exponents >= (BIAS + {EXP{1'b1}})) begin
				result = {1'b0, {EXP{1'b1}}, {MAN{1'b0}}};
				inf = (temp_exponents == (BIAS + {EXP{1'b1}})) & (temp_mantissa == 0);
				nan = (temp_exponents == (BIAS + {EXP{1'b1}})) & (temp_mantissa != 0);
				overflow = (temp_exponents > (BIAS + {EXP{1'b1}}));
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
