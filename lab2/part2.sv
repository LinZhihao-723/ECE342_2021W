// Lin Zhihao
// 1005071299
// ECE342 Lab2

module full_adder
(
	input a,
	input b,
	input c_in,
	output s,
	output c_out
);
	assign s = c_in ^ a ^ b;
	assign c_out = (a & b) | (a & c_in) | (b & c_in);
endmodule

module multiplier_cell
(
	input plus,
	input minus,

	input P_in,
	input m_in,
	input c_in,

	output sign_ext,
	output P_out,
	output c_out
);
	assign sign_ext = (plus & m_in) | (minus & ~m_in);
	full_adder FA(P_in, sign_ext, c_in, P_out, c_out);
endmodule

module carry_row #
(
	parameter N = 16
)
(
	input [N-1:0] P_in,
	input [N-1:0] M_in,
	input [N-1:0] carry_in,
	input plus,
	input minus,

	output [N-1:0] P_out,
	output [N-1:0] carry_out
);
	// Generate multiplier cells
	genvar i;
	generate
		for(i = 0; i < N; i ++) begin : multiplier_cells_generator
			multiplier_cell mc_ins
			(
				.plus(plus),
				.minus(minus),
				.P_in(P_in[i]),
				.m_in(M_in[i]),
				.c_in(carry_in[i]),
				.P_out(P_out[i]),
				.c_out(carry_out[i])
			);
		end
	endgenerate
endmodule

module booth_encoder #
(
	parameter N = 8
)
(
	input [N-1:0] m,
	output [N-1:0] Plus,
	output [N-1:0] Minus
);
	logic [N:0] _m;
	assign _m = {m, {1'b0}};
	genvar i;
	generate
		for(i = 0; i < N; i ++) begin	
			assign Plus[i] = ~_m[i+1] & _m[i];
			assign Minus[i] = _m[i+1] & ~_m[i];
		end
	endgenerate
endmodule

module carry_lookahead_adder #
(
	parameter N = 8
)
(
	input [N-1:0] x,
	input [N-1:0] y,
	input c_in,
	output [N-1:0] s,
	output c_out
);
	logic [N-1:0] g;
	logic [N-1:0] p;
	logic [N:0] c;
	assign c[0] = c_in;
	assign c_out = c[N];
	assign g = x & y;
	assign p = x | y;
	genvar i;
	generate
		for(i = 0; i < N; i ++) begin
			assign c[i + 1] = g[i] | (p[i] & c[i]);
			assign s[i] = x[i] ^ y[i] ^ c[i];
		end
	endgenerate
endmodule

module carry_saved_multiplier #
(
	parameter N = 8
)
(
	input [N-1:0] i_X,
	input [N-1:0] i_Y,
	output [2*N-1:0] o_Z
);
	/* This module implement the following relation: X*Y=Z */

	// First, create plus and minus
	// Note Y is shifted out as the multiplier
	logic [N-1:0] plus;
	logic [N-1:0] minus;
	booth_encoder #(.N(N)) ins_bd(
		.m(i_Y),
		.Plus(plus),
		.Minus(minus)
	);

	logic [N:0][2*N-1:0] carry;
	logic [N-1:0][2*N-1:0] m;
	logic [N:0][2*N-1:0] p;
    assign carry[0] = {{N{1'b0}}, minus};
    assign p[0] = 0;

    genvar i, j;
    generate
    	for(i = 0; i < N; i ++) begin
    		for(j = 0; j < 2 * N; j ++) begin
    			if(j < i) begin
    				assign p[i+1][j] = p[i][j];
    				assign m[i][j] = 1'b0;
    			end
    			else if(j < i + N) assign m[i][j] = i_X[j - i];
    			else assign m[i][j] = i_X[N-1];

    			if(i > 0 && j > 2*N - 1 - i) assign carry[i+1][j] = 2'b0;
    		end
    		carry_row #(.N(2 * N - i)) mr_ins
				(
					.P_in(p[i][2*N-1:i]),
					.M_in(m[i][2*N-1:i]),
					.carry_in(carry[i][2*N-1-i:0]),
					.plus(plus[i]),
					.minus(minus[i]),
					.P_out(p[i+1][2*N-1:i]),
					.carry_out(carry[i+1][2*N-1-i:0])
				);
    	end
    endgenerate

    logic [N-1:0] z_low, z_high;
    assign z_low = p[N][N-1:0];
    carry_lookahead_adder #(.N(N)) ins_cla
    	(
    		.x(p[N][2*N-1:N]),
    		.y(carry[N][N-1:0]),
    		.c_in({1'b0}),
    		.s(z_high)
    	);

    assign o_Z = {z_high, z_low};
endmodule