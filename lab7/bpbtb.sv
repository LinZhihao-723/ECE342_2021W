import cpu_encoding::*;

module tb();
    logic clk;
    logic reset;
    logic [31:0] pc_stage_0;
    bpb_inform i_updates;
    bpb_prediction o_prediction;

    Branch_Prediction_Buffer dut(.*);

    always begin
        #1 clk = !clk;
    end

    integer round;
    integer i;

    initial begin
        clk = 0;
        reset = 1;
        i_updates.valid = 0;
        #2;

        reset = 0;
        pc_stage_0 = 28;
        round = 0;
        #2;

        /* First time taken a branch */
        i_updates.valid = 1;
        i_updates.branch_target_pc = 36;
        i_updates.pc = 28;
        i_updates.branch_taken = 1;
        i_updates.jump = 0;
        round = 1;
        #2;

        /* Insert a PC */
        i_updates.valid = 0;
        pc_stage_0 = 28;
        round = 2;
        #2;

        /* Second time taken a branch */
        i_updates.valid = 1;
        i_updates.branch_target_pc = 36;
        i_updates.pc = 28;
        i_updates.branch_taken = 1;
        i_updates.jump = 0;
        round = 3;
        #2;

        i_updates.valid = 0;
        pc_stage_0 = 28;
        round = 4;
        #2;

        /* Second time taken a branch */
        i_updates.valid = 1;
        i_updates.branch_target_pc = 36;
        i_updates.pc = 28;
        i_updates.branch_taken = 0;
        i_updates.jump = 0;
        round = 5;
        #2;

        i_updates.valid = 0;
        pc_stage_0 = 28;
        round = 6;
        #2;

        /* Second time taken a branch */
        i_updates.valid = 1;
        i_updates.branch_target_pc = 36;
        i_updates.pc = 28;
        i_updates.branch_taken = 1;
        i_updates.jump = 0;
        round = 7;
        #2;

        i_updates.valid = 0;
        pc_stage_0 = 28;
        round = 8;
        #2;

        pc_stage_0 = 0;
        round = 9;
        #2;

        // for(i = 0; i < 70; i ++) begin
        //     i_updates.valid = 1;
        //     i_updates.branch_target_pc = 32'h00ECE342;
        //     i_updates.pc = i * 4;
        //     i_updates.branch_taken = 0;
        //     i_updates.jump = 0;
        //     #2;
        // end

        $stop();
    end
endmodule
