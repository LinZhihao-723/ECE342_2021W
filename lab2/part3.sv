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

module wallace_tree_row #
(
	parameter N = 8
)
(
	input [N-1:0] row_1_in,
	input [N-1:0] row_2_in,
	input [N-1:0] row_3_in,
	output [N-1:0] sum,
	output [N-1:0] carry_out
);
	assign sum = row_1_in ^ row_2_in ^ row_3_in;
	assign carry_out[0] = 1'b0;
	genvar i;
	generate
		for(i = 1; i < N; i ++) begin
			assign carry_out[i] = (row_1_in[i - 1] & row_2_in[i - 1])  | 
								  (row_2_in[i - 1] & row_3_in[i - 1])  |
								  (row_1_in[i - 1] & row_3_in[i - 1]);
		end
	endgenerate
endmodule

module m_value_producer #
(
	parameter N = 8
)
(
	input [N-1:0] X_in,
	input [N-1:0] plus,
	input [N-1:0] minus,
	output [N-1:0][2*N-1:0] m_row
);
	logic [N-1:0][N-1:0] m_referece_array;
	genvar i, j;
	generate
		for(i = 0; i < N; i ++) begin
			m_decoder ins_md(X_in, plus[i], minus[i], m_referece_array[i]);
			for(j = 0; j < 2 * N; j ++) begin
				if(j < i) assign m_row[i][j] = 1'b0;
				else if(j < i + N) begin
					assign m_row[i][j] = m_referece_array[i][j - i];
				end else assign m_row[i][j] = m_row[i][j - 1];
			end
		end
	endgenerate
endmodule

module m_decoder #
(
	parameter N = 8
)
(
	input [N-1:0] X_in,
	input plus,
	input minus,
	output [N-1:0] m_referece
);
	genvar i;
	generate
		for(i = 0; i < N; i ++) begin
			assign m_referece[i] = (X_in[i] & plus) | (~X_in[i] & minus);
		end
	endgenerate
endmodule

module wallace_tree_multiplier_8_bit
( 
	input [7:0] X_in,
	input [7:0] Y_in,
	output [15:0] Z_out
);
	// Setup booth values
	logic [7:0] plus;
	logic [7:0] minus;
	booth_encoder #(.N(8)) ins_bd(
		.m(Y_in),
		.Plus(plus),
		.Minus(minus)
	);

	logic [7:0][15:0] m_row;
	m_value_producer #(.N(8)) ins_mvp(
		.X_in(X_in),
		.plus(plus),
		.minus(minus),
		.m_row(m_row)
	);

	logic [8:0][15:0] stage_1_array;
	logic [5:0][15:0] stage_2_array;
	logic [3:0][15:0] stage_3_array;
	logic [2:0][15:0] stage_4_array;
	logic [1:0][15:0] final_addition;

	assign stage_1_array = {{8'b00000000, minus}, m_row};
	
	genvar i;
	generate
		for(i = 0; i < 3; i ++) begin
			wallace_tree_row #(.N(16)) ins_wtr(
				.row_1_in(stage_1_array[i*3 + 0]),
				.row_2_in(stage_1_array[i*3 + 1]),
				.row_3_in(stage_1_array[i*3 + 2]),
				.sum(stage_2_array[i*2 + 0]),
				.carry_out(stage_2_array[i*2 + 1])
			);
		end
		for(i = 0; i < 2; i ++) begin
			wallace_tree_row #(.N(16)) ins_wtr(
				.row_1_in(stage_2_array[i*3 + 0]),
				.row_2_in(stage_2_array[i*3 + 1]),
				.row_3_in(stage_2_array[i*3 + 2]),
				.sum(stage_3_array[i*2 + 0]),
				.carry_out(stage_3_array[i*2 + 1])
			);
		end
		for(i = 0; i < 1; i ++) begin
			wallace_tree_row #(.N(16)) ins_wtr(
				.row_1_in(stage_3_array[i*3 + 0]),
				.row_2_in(stage_3_array[i*3 + 1]),
				.row_3_in(stage_3_array[i*3 + 2]),
				.sum(stage_4_array[i*2 + 0]),
				.carry_out(stage_4_array[i*2 + 1])
			);
		end
		assign stage_4_array[2] = stage_3_array[3];
		wallace_tree_row #(.N(16)) ins_wtr(
			.row_1_in(stage_4_array[0]),
			.row_2_in(stage_4_array[1]),
			.row_3_in(stage_4_array[2]),
			.sum(final_addition[0]),
			.carry_out(final_addition[1])
		);
	endgenerate

	carry_lookahead_adder #(.N(16)) ins_cla(
		.x(final_addition[0]),
		.y(final_addition[1]),
		.c_in({1'b0}),
		.s(Z_out)
	);
endmodule