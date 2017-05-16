`timescale 1ns/1ns

module testbench();
	
	reg 			clk_50, reset;
	reg 	[31:0] 	index;
  //wire signed [6:0]   testbench_out;
	
	//Initialize clocks and index
	initial begin
		clk_50 = 1'b0;

		index  = 32'd0;
	end
	
	//Toggle the clocks
	 always begin
	 	#10
	 	clk_50  = !clk_50;
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
 lab5 DUT  (
             .clk(clk_50), 
             .reset(reset),
			 .hps_input({1'b1, 11'b0, 10'd4, 10'd4})
           );
endmodule


module lab5(clk, reset, index, data, hps_input);
	parameter               SIZE = 50;
	parameter               SIZE2 = 2500;
	input 					clk, reset;
	input       [11:0]		index;
	input		[31:0]		hps_input;
	output      [6:0]       data;
	
	reg         [6:0]       p_in_array[0:SIZE2-1];
	wire        [6:0]       p_out_array[0:SIZE2-1]; 
	wire		[9:0]		hps_x, hps_y;
	wire					enable;
	
	integer x, y;
	always @(posedge clk) begin
		if (reset) begin
			for (x = 0; x < SIZE; x = x + 1) begin: INIT_NODE_X
				for (y = 0; y < SIZE; y = y + 1) begin: INIT_NODE_Y
					if(x == 0 || y == 0 || x == (SIZE-1) || y == (SIZE -1)) begin
						p_in_array[y*SIZE+x] <= 7'b0000111;
					end
//					else if( x == 1) begin
//						p_in_array[y*SIZE+x] <= 7'b0010001;
//					end
//					else if( x == SIZE-3) begin
//						p_in_array[y*SIZE+x] <= 7'b1000001;
//					end
					else begin 
						p_in_array[y*SIZE+x] <= 7'd0;
					end
				end
			end	
		end
		else begin
			for (x = 0; x < SIZE2; x = x + 1)
				if (hps_input[31] && x == (hps_input[19:10]*SIZE+hps_input[9:0]))
					p_in_array[x] <= 7'b0000111;
				else
					p_in_array[x] <= p_out_array[x];
		end 
	end
	
	assign data = p_in_array[index];	
		
	genvar i, j;
	generate 
	for (i = 0; i < SIZE; i = i + 1) begin: NODE_X
		for (j = 0; j < SIZE; j = j + 1) begin: NODE_Y
			if(i == 0 && j == 0) 					// check top left corner
				node_update x_dir(clk, reset, 1'b1, p_in_array[j*SIZE+i], 7'd0, 					p_in_array[j*SIZE+i+1], p_in_array[(j+1)*SIZE+i], 	7'd0, 						p_out_array[j*SIZE+i]  );
			else if (i == SIZE-1 && j == 0) 		// check top right corner
				node_update x_dir(clk, reset, 1'b1, p_in_array[j*SIZE+i], 7'd0, 					7'd0,		 			p_in_array[(j+1)*SIZE+i],	p_in_array[(j*SIZE+i-1)],	p_out_array[j*SIZE+i]  );
			else if (i == 0 && j == SIZE-1)			// check bottom left corner
				node_update x_dir(clk, reset, 1'b1, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i],	p_in_array[j*SIZE+i+1], 7'd0,					 	7'd0, 						p_out_array[j*SIZE+i]  );
			else if (i == SIZE-1 && j == SIZE-1) 	// check bottom right corner
				node_update x_dir(clk, reset, 1'b1, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i], 7'd0, 					7'd0,						p_in_array[(j*SIZE+i-1)], 	p_out_array[j*SIZE+i]  );
			else if (j == 0)						// check top row
				node_update x_dir(clk, reset, 1'b1, p_in_array[j*SIZE+i], 7'd0, 					p_in_array[j*SIZE+i+1], p_in_array[(j+1)*SIZE+i], 	p_in_array[(j*SIZE+i-1)], 	p_out_array[j*SIZE+i]  );
			else if (j == SIZE-1)					// check bottom row
				node_update x_dir(clk, reset, 1'b1, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i], p_in_array[j*SIZE+i+1], 7'd0, 						p_in_array[(j*SIZE+i-1)], 	p_out_array[j*SIZE+i]  );
			else if (i == 0)						// check left column
				node_update x_dir(clk, reset, 1'b1, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i], p_in_array[j*SIZE+i+1], p_in_array[(j+1)*SIZE+i], 	7'd0, 						p_out_array[j*SIZE+i]  );
			else if (i == SIZE-1)					// check right column
				node_update x_dir(clk, reset, 1'b1, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i], 7'd0, 					p_in_array[(j+1)*SIZE+i], 	p_in_array[(j*SIZE+i-1)], 	p_out_array[j*SIZE+i]  );
			else
				node_update x_dir(clk, reset, 1'b1, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i],  p_in_array[j*SIZE+i+1],  
													p_in_array[(j+1)*SIZE+i],  p_in_array[(j*SIZE+i-1)], p_out_array[j*SIZE+i]  );
		end
	end
	endgenerate 
	
//	always @(*) begin
//		for (x = 0; x < SIZE; x = x + 1) begin: INIT_NODE_X
//			for (y = 0; y < SIZE; y = y + 1) begin: INIT_NODE_Y
//
//				else if( x == 1) begin
//					p_in_array[y*SIZE+x] <= 7'b0010001;
//				end
//				else if( x == SIZE-3) begin
//					p_in_array[y*SIZE+x] <= 7'b1000001;
//				end
//				
//			end
//		end
//	end
	
	assign enable = hps_input[31];
	assign hps_x  = hps_input[9:0];
	assign hps_y  = hps_input[19:10];
	//assign p_out_array[hps_y*SIZE+hps_x] = enable ? 7'b0000111 : p_in_array[hps_y*SIZE+hps_x]; 
	
endmodule

//////////////////////////////////////////////////
//           node particle                      //
//   Each node stores the direction it's moving //
//////////////////////////////////////////////////
//       6     5     4     3 2         0    	//
//  +-----+-----+-----+-----+----------+		//
//  |  W  |  S  |  E  |  N  |Color(1-4)|		//
//  +-----+-----+-----+-----+----------+		//
//    											//
//////////////////////////////////////////////////                                        
module node_update (clk, reset, enable, node_in, n, e, s, w, node);
	output reg	[6:0]	node;
	input 		        clk, reset, enable;
	input wire	[6:0]	node_in;
	input wire  [6:0] 	n  , e  , s  , w;
	reg         [6:0] 	node_temp;
	reg         [2:0] 	density;
	
	always @(posedge clk) begin
		if(reset) begin
			node <= node_in;
		end
		else if(enable) begin
			node <= node_temp;
		end
	end

	always @(*) begin
		if(node_in[2:0] != 3'b111) begin
			density = 3'd0;		
			if (n[5] && s[3] && (!w[4] || !e[6])) begin
				node_temp[6] = 1'b1;
				node_temp[4] = 1'b1;
				node_temp[5] = 1'b0;
				node_temp[3] = 1'b0;
				density = density + 2;
			end
			else if (w[4] && e[6]  && (!n[5] || !s[3])) begin
				node_temp[5] = 1'b1;
				node_temp[3] = 1'b1;
				node_temp[6] = 1'b0;
				node_temp[4] = 1'b0;
				density = density + 2;
			end	
			else begin
				if (n[5]) begin
					density = density + 1;
					node_temp[5] = 1'b1;
				end
				else
					node_temp[5] = 1'b0;
				if (e[6]) begin
					density = density + 1;
					node_temp[6] = 1'b1;
				end
				else
					node_temp[6] = 1'b0;
				if (s[3]) begin
					density = density + 1;
					node_temp[3] = 1'b1;
				end
				else
					node_temp[3] = 1'b0;
				if (w[4]) begin
					density = density + 1;
					node_temp[4] = 1'b1;
				end
				else
					node_temp[4] = 1'b0;
			end
		end
		else begin
			if (n[5]) 
				node_temp[3] = 1'b1;
			else
				node_temp[3] = 1'b0;
			if (e[6])
				node_temp[4] = 1'b1;
			else
				node_temp[4] = 1'b0;
			if (s[3])
				node_temp[5] = 1'b1;
			else
				node_temp[5] = 1'b0;
			if (w[4])
				node_temp[6] = 1'b1;
			else
				node_temp[6] = 1'b0;
			density = 3'b111;
		end
		node_temp[2:0] = density;
	end
endmodule

module dual_clock_ram(
   output reg [6:0] q,
   input [6:0] d,
   input [18:0] write_address, read_address,
   input we, clk1, clk2
);
   reg [18:0] read_address_reg;
   (* ramstyle = "M10K" *) reg [6:0] mem [307199:0];
   always @ (posedge clk1)
   begin
      if (we)
         mem[write_address] <= d;
   end
   always @ (posedge clk2) begin
      q <= mem[read_address_reg];
      read_address_reg <= read_address;
   end
endmodule
