// Lin Zhihao
// 1005071299

//Top module for game
module top 
(
    // Clock pins
    input                     clk,
    input                     reset,
    input                     enter,
    input               [7:0] guess,
    output                    dp_over,
    output                    dp_under,
    output                    dp_equal,
    output              [7:0] actual,
    output              [3:0] dp_tries
);

	logic over;
	logic under;
	logic equal;

	logic [3:0] tries;
	assign dp_tries = tries;

	// Datapath
	logic dp_inc_actual;
	logic dec_tries;
	datapath the_datapath
	(
		.clk(clk),
		.reset(reset),
		.i_guess(guess),
		.i_inc_actual(dp_inc_actual),
		.o_over(over),
		.o_under(under),
		.o_equal(equal),
        .actual(actual),
        .i_dec_tries(dec_tries),
        .tries(tries)
	);
	
	// State Machine
	logic ctrl_update_leds;
	control the_control
	(
		.clk(clk),
		.reset(reset),
		.i_enter(enter),
		.o_inc_actual(dp_inc_actual),
		.i_over(over),
		.i_under(under),
		.i_equal(equal),
		.o_update_leds(ctrl_update_leds),
		.o_dec_tries(dec_tries),
		.i_tries(tries)
	);
	
	// LED controllers
	led_ctrl ledc_under(clk, reset, under, ctrl_update_leds, dp_under);
	led_ctrl ledc_over(clk, reset, over, ctrl_update_leds, dp_over);
	led_ctrl ledc_equal(clk, reset, equal, ctrl_update_leds, dp_equal);
	
endmodule

/*******************************************************/
/********************Control module********************/
/*****************************************************/
module control
(
	input clk,
	input reset,
	
	// Button input
	input i_enter,
	
	// Datapath
	output logic o_inc_actual,
	output logic o_dec_tries,
	input i_over,
	input i_under,
	input i_equal,
	input [3:0] i_tries,
	
	// LED Control
	output logic o_update_leds
);

    // Declare two objects, 'state' and 'nextstate'
    // that are of enum type.
    enum int unsigned
    {
        S_GEN_RAND,
        S_CHECK,
        S_WAIT_NOENTER,
        S_WAIT_ENTER,
        S_END
    } state, nextstate;

    // Clocked always block for making state registers
    always_ff @ (posedge clk or posedge reset) begin
        if (reset) state <= S_GEN_RAND;
        else state <= nextstate;
    end

    // always_comb replaces always @* and gives compile-time errors instead of warnings
    // if you accidentally infer a latch
    always_comb begin
        nextstate = state;
        o_inc_actual = 1'b0;
        o_update_leds = 1'b0;
        o_dec_tries = 1'b0;

        case (state)
            S_GEN_RAND: begin
                nextstate = i_enter ? S_CHECK : S_GEN_RAND;
                o_inc_actual = 1'b1;
            end
            S_CHECK: begin
                nextstate = i_equal ? S_END : S_WAIT_NOENTER;
                o_update_leds = 1'b1;
                o_dec_tries = 1'b1;
            end
            S_WAIT_NOENTER: begin
                if(i_tries > 0) nextstate = i_enter ? S_WAIT_NOENTER : S_WAIT_ENTER;
                else nextstate = S_END;
            end
            S_WAIT_ENTER: begin
                nextstate = i_enter ? S_CHECK : S_WAIT_ENTER;
            end
            S_END: begin
                nextstate = S_END;
            end
        endcase
    end
endmodule


/*******************************************************/
/********************Datapath module*******************/
/*****************************************************/
module datapath
(
	input clk,
	input reset,
	
	// Number entry
	input [7:0] i_guess,
	
	// Increment actual
	input i_inc_actual,
	
	// Comparison result
	output o_over,
	output o_under,
	output o_equal,
	output logic [7:0] actual,

	// Number of tries
	output logic [3:0] tries,

	// Decrement tries
	input i_dec_tries
);

    // Update the 'actual' register based on control signals
    always_ff @ (posedge clk or posedge reset) begin
        if (reset) begin
            actual <= '0;
            tries <= 4'd7;
        end
        else begin
            if (i_inc_actual) actual <= actual + 8'd1;
            if (i_dec_tries) tries <= tries - 4'd1;
        end
    end

    // Generate comparisons
    assign o_over = i_guess > actual;
    assign o_equal = i_guess == actual;
    assign o_under = i_guess < actual;
endmodule

/*******************************************************/
/********************LED control module****************/
/*****************************************************/
module led_ctrl
(
	input clk,
	input reset,
	
	input i_val,
	input i_enable,
	output logic o_out
);

    always_ff @ (posedge clk or posedge reset) begin
        if (reset) o_out <= '0;
        else if (i_enable) o_out <= i_val;
    end
endmodule
