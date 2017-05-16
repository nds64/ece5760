`timescale 1ns/1ns

module testbench();
	
	reg clk_50, clk_25, reset;
	reg 			  [31:0] 	index;
	wire signed [31:0]  testbench_out;
	wire signed 		  	done;
	
	//Initialize clocks and index
	initial begin
		clk_50 = 1'b0;
		clk_25 = 1'b0;
		index  = 32'd0;
		//testbench_out = 15'd0 ;
	end
	
	//Toggle the clocks
	always begin
		#10
		clk_50  = !clk_50;
	end
	
	always begin
		#20
		clk_25  = !clk_25;
	end
	
	//Intialize and drive signals
	initial begin
		reset  = 1'b0;
		#10 
		reset  = 1'b1;
		#30
		reset  = 1'b0;
	end
	
	//Increment index
	always @ (posedge clk_50) begin
		index  <= index + 32'd1;
	end

	//Instantiation of Device Under Test
comp_pp DUT  (
            .clk(clk_50), 
            .reset(reset),
            .Cre(27'h01039b0),
            .Cim(27'h03973f9),
            .max_count(1000),
            .iter(testbench_out),
            .done(done)
          );
endmodule

module comp_pp(clk, reset, Cre, Cim, max_count, rgb_data, done);
	input 						    clk, reset;
	input 			  [9:0] 	max_count;
	input signed 	[26:0] 	Cre, Cim;
	//output reg 		[9:0] 	iter;
	output reg 					  done;
	output reg [7:0] rgb_data;
	
	wire signed 	[26:0] 	Zsq_temp, Zim_temp, Zre_temp, Zim_temp1;
	wire signed 	[26:0] 	Zim_sq_temp, Zre_sq_temp;
	reg signed 		[26:0] 	Zsq, Zre, Zim, Zre_sq, Zim_sq;
	reg [9:0] 	count;
	reg  [9:0]  iter;
	
	always @(posedge clk) begin
		if (reset || done) begin
			Zsq     <= 0;
			Zre 		<= 0;
			Zim 		<= 0;
			Zre_sq 	<= 0;
			Zim_sq 	<= 0;
			count 	<= 0;
			iter 		<= 0;
			done 		<= 0;
			rgb_data <= 8'h00;
		end
		else if (~done) begin
			Zsq     <= Zsq_temp;
			Zre 		<= Zre_temp;
			Zim 		<= Zim_temp;
			Zre_sq 	<= Zre_sq_temp;
			Zim_sq 	<= Zim_sq_temp;
			if (count == max_count || Zsq > 27'h200_0000) begin
				done 	<= 1;
				iter 	<= count;
				count 	<= count;
				if (count >= max_count) rgb_data <= 8'h01;
				else if (count >= (max_count>>2)) rgb_data <= 8'he0; 
				else if (count >= (max_count>>4)) rgb_data <= 8'hc0;
				else if (count >= (max_count>>5)) rgb_data <= 8'ha0;
				else if (count >= (max_count>>6)) rgb_data <= 8'h80;
				else if (count >= (max_count>>7)) rgb_data <= 8'h24;
				else rgb_data <= 8'h00;
//				if (count == max_count)
//					rgb_data <= 8'b11100000;
//				else
//					rgb_data <= 8'b00011100;
			end
			else begin
				done 	<= 0;
				iter  <= count;
				count <= count + 1;
				rgb_data <= 8'h00;
			end
		end
	end
	
  signed_mult im_mul(Zim_temp1, Zre, Zim);
  assign Zim_temp = (Zim_temp1 << 1) + Cim;
  assign Zre_temp = Zre_sq - Zim_sq + Cre;
  signed_mult re_sq(Zre_sq_temp, Zre_temp, Zre_temp);
  signed_mult im_sq(Zim_sq_temp, Zim_temp, Zim_temp);
  assign Zsq_temp = Zre_sq_temp + Zim_sq_temp;
endmodule

/*
module FIFO #(SIZE)
(
	input clk,
	input reset,
	input [26:0] data_in [SIZE-1:0];
	input [9:0] x_cood_in [SIZE-1:0];
	input [9:0] y_cood_in [SIZE-1:0];
	input done [SIZE-1:0];
	output is_empty;
	output [26:0] data_out;
	output [9:0] x_cood_out;
	output [9:0] y_cood_out;
	output [$clog2(SIZE)-1:0] index_out; 
);
	reg [26:0] data [SIZE-1:0];
	reg [$clog2(SIZE)-1:0] index [SIZE-1:0];
	reg [$clog2(SIZE)-1:0] index_cur; // index within "data" and "index", or "fifo index"
	
	genvar i;
	
	generate
		for (i = 0; i < SIZE; i = i+1) begin
			always @(posedge clk) begin 
				if (reset) begin
					data[i] <= 0;
					index[i] <= 0;
					index_cur <= 0;
				end
				else if (done[i]) begin
					data[index_cur] <= data_in[i];
					index_cur <= index_cur + 1;
				end
			end
		end
	endgenerate
	
	assign is_empty
	
endmodule
*/
//////////////////////////////////////////////////
//// signed mult of 4.23 format 2'comp////////////

module signed_mult (out, a, b);
	output 				  [26:0]	out;
	input signed		[26:0] 	a;
	input signed		[26:0] 	b;
	
	wire	signed		[26:0]	out;
	wire 	signed		[53:0]	mult_out;

	assign mult_out = a * b;
	assign out = {mult_out[53], mult_out[49:23]};
endmodule
//////////////////////////////////////////////////