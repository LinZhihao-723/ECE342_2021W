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

module multiplier_row #
(
	parameter N = 16
)
(
	input [N-1:0] P_in,
	input [N-1:0] M_in,
	input [1:0] Q_in,

	output [N-1:0] P_out,
	output C_out
);
	// Variables for plus and minus
	logic plus;
	logic minus;
	assign plus = Q_in[0] & ~Q_in[1];
	assign minus = Q_in[1] & ~Q_in[0];

	// Variables for carrier bits
	logic [N:0] carrier;
	assign carrier[0] = minus;

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
				.c_in(carrier[i]),
				.P_out(P_out[i]),
				.c_out(carrier[i+1])
			);
		end
	endgenerate

	// Assign the carry our bit
	assign C_out = carrier[N];
endmodule

module array_multiplier #
(
	parameter X = 8,  /* Size of the first input */
	parameter Y = 8   /* Size of the second input (multiplier) */
)
(
	input [X-1:0] i_X,
	input [Y-1:0] i_Y,
	output [X+Y-1:0] o_Z
);
	/* This module implement the following relation: X*Y=Z */

	// Q is used to determine plus/minus
	logic [Y:0] Q_connections;
	assign Q_connections[0] = 0;
	assign Q_connections[Y:1] = i_Y;

	// P_connection is used as the interconnection of P
	logic [Y:0][X+Y-1:0] P_connections;
	assign P_connections[0][X+Y-1:0] = 0;
	assign o_Z = P_connections[Y][X+Y-1:0];

	// The following is used as the interconnection of M
	logic [Y-1:0][X+Y-1:0] M_connections;

	genvar i, j;
	generate
		for(i = 0; i < Y; i ++) begin : multiplier_rows_generator
			for(j = 0; j < X + Y; j ++) begin: M_connections_generator
				if(j < i) begin
					assign M_connections[i][j] = 1'b0;
					assign P_connections[i+1][j] = P_connections[i][j];
				end
				else if(j < i + X) assign M_connections[i][j] = i_X[j-i];
				else assign M_connections[i][j] = i_X[X-1];
			end
			multiplier_row #(.N(X+Y-i)) mr_ins
			(
				.P_in(P_connections[i][X+Y-1:i]),
				.M_in(M_connections[i][X+Y-1:i]),
				.Q_in(Q_connections[i+1:i]),
				.P_out(P_connections[i+1][X+Y-1:i])
			);
		end
	endgenerate

endmodule