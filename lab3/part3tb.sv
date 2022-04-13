`timescale 1ns/1ns
module tb();

	logic [6:0] dut_x7, dut_y7;
	logic [6:0] dut_out7;
	wire  inf7, nan7, zero7, overflow7, underflow7;
	part3 DUT7(
		.X             (dut_x7),
		.Y             (dut_y7),
      .result      (dut_out7),
		.inf		        (inf7),
		.nan	           (nan7),
		.zero		       (zero7),
		.overflow   (overflow7),
		.underflow (underflow7));
		defparam DUT7.EXP = 2;
		defparam DUT7.MAN = 4;


	initial begin
		dut_y7 = 7'b0010010;
		dut_x7 = 7'b0100010;
		#50;

		dut_y7 = 7'b0011111;
		dut_x7 = 7'b0101111;
		#50;

		dut_y7 = 7'b0110000;
		dut_x7 = 7'b0101111;
		#50;

		dut_y7 = 7'b0110000;
		dut_x7 = 7'b0110100;
		#50;

		dut_y7 = 7'b0000000;
		dut_x7 = 7'b0110100;
		#50;

		dut_y7 = 7'b0000000;
		dut_x7 = 7'b0110000;
		#50;
	end
	

	/*
	initial begin
		input_x32 = new[11];
		input_y32 = new[11];
		input_x32 = '{32'hba57711a, 32'h34082401, 32'h5c75da81, 32'h07f00000, 32'h00000000, 32'h7f800ff0, 32'hfCFA3E28};
		input_y32 = '{32'hee1818c5, 32'hb328cd45, 32'h2f642a39, 32'h00800000, 32'hb3edcd45, 32'h74749112, 32'hfCFA3E28};
		
		foreach (input_x32[i]) begin
		
			dut_x32 = input_x32[i];
			dut_y32 = input_y32[i];
			x = $bitstoshortreal(dut_x32);
			y = $bitstoshortreal(dut_y32);
			result = x * y;
			#5;
			out = $bitstoshortreal(dut_out32);
			
			result_abs = result > 0 ? result : -result;
			out_abs = out > 0 ? out : -out;
			largest = result_abs > out_abs ? result_abs : out_abs;
			
			if ((((result_abs - out_abs) >  largest * epsilon ) ||inf32 || nan32 || zero32  || overflow32 || underflow32)&& i < 3 ) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %g", x, y, out,  result);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end else if (i < 3) begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x, y, out, result);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end
			if ((out != 0 || inf32 || nan32 || zero32  || overflow32 || !underflow32)&& (i == 3 ||i == 7)) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %d", x, y, out, 0);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end else if ( i == 3 || i == 7)begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x, y, out, 0);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end
			if ((out != 0 || inf32 || nan32 || !zero32  || overflow32 || underflow32) && i == 4 ) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %d", x, y, out, 0);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end else if ( i == 4)begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x, y, out, 0);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end
			if ((dut_out32 != 32'h7f800000 || inf32 || !nan32 || zero32  || overflow32 || underflow32) && i == 5) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %d", x, y, out, 32'h7f800000);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end else if ( i == 5)begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x, y, out, 32'h7f800000);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end
			if ((dut_out32 != 32'h7f800000 || inf32 || nan32 || zero32 || !overflow32 || underflow32) && i == 6) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %d", x, y, out, 32'h7f800000);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end else if ( i == 6)begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x, y, out, 32'h7f800000);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end
		end
		
		
		input_x64 = new[8];
		input_y64 = new[8];
		input_x64 = '{64'h40BA858E56041893, 64'h3478374839238348, 64'hbcdaeffebcda3832, 64'h0000000000000000, 64'h7FF0000000000000};
		input_y64 = '{64'hee1818c5144030e0, 64'h384739e834793238, 64'hf3c2e4d1a8230fe3, 64'hee1818c5144030e0, 64'hee1818c5144030e0};
		
		foreach (input_x64[i]) begin
			dut_x64 = input_x64[i];
			dut_y64 = input_y64[i];
			x64 = $bitstoreal(dut_x64);
			y64 = $bitstoreal(dut_y64);
			result64 = x64 * y64;
			#5;
			out64 = $bitstoreal(dut_out64);
			
			result_abs64 = result > 0 ? result64 : -result64;
			out_abs64 = out64 > 0 ? out64 : -out64;
			largest64 = result_abs64 > out_abs64 ? result_abs64 : out_abs64;
			
			if ((((result_abs64 - out_abs64) >  largest64 * epsilon64 ) ||inf64 || nan64 || zero64  || overflow64 || underflow64)&& i < 3 ) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64,  result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end else if (i < 3) begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64, result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end
			
			if ((out64 != 0 ||inf64 || nan64 || zero64  || overflow64 || !underflow64)&& i == 3 ) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64,  result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end else if (i == 3) begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64, result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end
			if ((dut_out64 != 64'h7ff0000000000000 ||!inf64 || nan64 || zero64  || overflow64 || underflow64)&& i == 4 ) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64,  result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end else if (i == 4) begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64, result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end
		end

	dut_x4 = 4'b0100;
	dut_y4 = 4'b0100;
	#5;
	if(!inf4 && nan4 && !zero4  && !overflow4 && !underflow4) begin
		$display("4 Bit PASS!");
		$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf4, nan4, zero4,  overflow4, underflow4);
		$display("%x", dut_out4);
	end else begin
		$display("4 Bit FAIL!");
		$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf4, nan4, zero4,  overflow4, underflow4);
		$display("%x", dut_out4);
	end


	$stop();
	end
	*/
          
endmodule
