//DDA module
module DDA (clock, cubic, k1, k2, k3, y1, y2, y3, y4, reset, x1_out, x2_out, enable, go);
	input clock, reset, enable, go;
	//k1,k2,k3 are the spring constants
	//y1,y2,y3,y4 are the constants for x1,v1,x2,v2 respectively
	input wire signed [17:0] k1, k2, k3, y1, y2, y3, y4, cubic;
	output reg signed [17:0] x1_out, x2_out;
	reg signed	[17:0] v1, v2, x1, x2 ;
	wire signed	[17:0] v1new, v2new, x1new, x2new ;
	wire signed [17:0] damp1, damp2; 
	wire signed [17:0] cubic_1, cubic_2, cubic_3, spring_force1, spring_force2, spring_forcem;

	// clock divider to get system into the audio range
	reg [31:0] count;
	// analog update divided clock
	always @ (posedge clock) 
	begin
		if (reset) begin
			count <= 32'd0;
			x1  	<= y1;
			v1		<= y2;
			x2		<= y3;
			v2		<= y4;
		end
		else if (go) begin
			count <= count + 1; 
		end
		//AnalogClock = (count==0)
		if (!reset && (count == 65535) && go) begin
			x1 <= x1new;
			x2 <= x2new;
			v1 <= v1new;
			v2 <= v2new;
			count <= 32'd0;
		end
		if (!reset && enable) begin
			x1_out <= x1;
			x2_out <= x2;
		end
	end	

	//Initial Condition Sets
	//-0.5  18'h3_8000
	//0     18'h0_0000
	//0.5   18'h0_8000
	//0     18'h0_0000
	//1	  18'h1_0000
	
	////////////////////////////////////////////
	// Wire the spring forces
	// spring_force1 = k1 * (x_spring1 - left wall(-1) + cubic(x_spring1- left wall(-1))^3)
	// spring_force2 = k2 * (rightwall(1) - x_spring2))
	// spring_forcem = k3 * (x_spring2 - xspring1))
	signed_mult cu1(cubic_1, x1-18'h3_0000, x1-18'h3_0000);
	signed_mult cu2(cubic_2, cubic_1, x1-18'h3_0000);
	signed_mult cu3(cubic_3, cubic_2, cubic);
	signed_mult sp1(spring_force1, k1, ((x1-18'h3_0000) + cubic_3));
	signed_mult sp2(spring_force2, k2, (18'h1_0000-x2));
	signed_mult spm(spring_forcem, k3, (x2-x1));
	signed_mult d1 (damp1, 18'h0_1000, v1);
	signed_mult d2 (damp2, 18'h0_1000, v2);
	
	////////////////////////////////////////////
	// Wire the integrators
	// position1 = position1 + velocity1*dt
	// position2 = position2 + velocity2*dt
	// velocity1 = velocity1 + (-spring_force1 - damping + spring_forcem)*dt
	// velocity2 = velocity2 + ( spring_force2 - damping - spring_forcem)*dt
	integrator position1(x1new, x1, v1, 9);
	integrator position2(x2new, x2, v2, 9);
	integrator velocity1(v1new, v1, (-spring_force1-damp1+spring_forcem), 9);
	integrator velocity2(v2new, v2, (spring_force2-damp2-spring_forcem), 9);
	
	//assign x1_out = x1;
	//assign x2_out = x2;
	
endmodule
 
/////////////////////////////////////////////////
//// integrator /////////////////////////////////

module integrator(out, old, funct, dt);
	output signed 	[17:0] out; 		//the state variable V
	input signed 	[17:0] funct;     //the dV/dt function
	input signed 	[17:0] old;
	input 			[3:0]  dt ;		   // in units of SHIFT-right
	
	assign out = old + (funct>>>dt) ;
endmodule

//////////////////////////////////////////////////
//// signed mult of 2.16 format 2'comp////////////

module signed_mult (out, a, b);
	output 	signed	[17:0]	out;
	input 	signed	[17:0] 	a;
	input 	signed	[17:0] 	b;	
	wire 		signed	[35:0]	mult_out;

	assign mult_out = a * b;
	assign out = {mult_out[35], mult_out[32:16]};
endmodule

//////////////////////////////////////////////////
//// VGA bus-master///////////////////////////////
// assign other bus parameters elsewhere
module VGA_bus_master(clk, reset, x1, x2, bus_addr, bus_write_data, 
							 bus_read, bus_write, bus_byte_enable, bus_ack,
							 bus_read_data, VGA_HS, VGA_VS);
	
	input clk, reset;
	input signed [17:0] x1, x2;
	output reg [31:0] bus_write_data;
	output [31:0] bus_addr;
	output reg bus_read, bus_write;
	output reg [3:0] bus_byte_enable;
	input bus_ack;
	input [31:0] bus_read_data;
	input VGA_HS, VGA_VS;
	
	wire [31:0] vga_base_address = 32'h0800_0000 ;  // Avalon address
	reg [31:0] timer ;
	reg [3:0] state ;
	
	// store x1 and x2
	//reg [17:0] x1_reg, x2_reg;

	// pixel address
	reg [9:0] x_cood, y_cood, x_cood_cur, y_cood_1, y_cood_2;
	reg [7:0] current_pixel_color1, current_pixel_color2 ;
	
	// compute address
	assign bus_addr = vga_base_address + {22'b0, x_cood} + ({22'b0, y_cood}<<10) ;
	
	// simulate control signal from keyboard
	wire go;
	assign go = 1'b1;
	
	// keep track of the current waveform
	// 0 -> x1, 1 -> x2
	reg current;
	
	// set to 1 if we have cleared a column
	reg cleared;
	
	reg init;
	
	// set to 1 if we have written a pixel of a corresponding waveform
	reg written1, written2;
	
	always @(posedge clk) begin
		
		// reset state machine and read/write controls
		if (reset) begin
		
			state <= 2 ;
			
			// base address of upper-left corner of the screen
			x_cood <= 10'd0 ;
			y_cood <= 10'd0 ;
			x_cood_cur <= 10'd319;
			y_cood_1 <= 10'd0;
			y_cood_2 <= 10'd0;
			
			bus_write <= 0 ; // set to on if a write operation to bus
			bus_byte_enable <= 4'b0001; // check later to see if this matters
			bus_read <= 1'b0;
		   
			timer <= 0; // might be useful
			
			current <= 1'b0;
			cleared <= 1'b0;
			written1 <= 1'b0;
			written2 <= 1'b0;
			
			//init <= 1'b1;
			
		end
		else begin
			timer <= timer + 1;
		end
		
		// write to the bus-master
		// and put in a small delay to aviod bus hogging
		// timer delay can be set to 2**n-1, so 3, 7, 15, 31
		// bigger numbers mean slower frame update to VGA
		if (state==0 && go && (~VGA_HS | ~VGA_VS)) begin//(timer & 3)==0 )
			
			state <= 1;
			// clear a column
			if (~cleared) begin
				bus_write_data <= 8'h00;
				written1 <= 1'b0;
				written2 <= 1'b0;
				if (y_cood >= 479) begin
					cleared <= 1'b1;
				end
				else begin
					y_cood <= y_cood + 10'd1;
				end
			end
			
			// draw a waveform pixel on y-axis
			else begin
				// increment x_cood when cleared a column and 
				// written both waveform pixels
				if (written1 && written2) begin
					cleared <= 1'b0;
					if (x_cood >= 639) begin
						x_cood <= 10'd0;
					end
					else begin
						x_cood <= x_cood + 10'd1;
					end
				end
				else begin
					if (~current) begin // waveform of x1
						written1 <= 1'b1;
						current <= ~current;
						// calculate y coordinate
						if (x1 >= 0) begin
							y_cood <= 10'd127 - x1[16:10];
						end
						else begin
							y_cood <= 10'd255 - x1[16:10];
						end
					end
					else begin //waveform of x2
						bus_write_data <= 8'h1c;
						written2 <= 1'b1;
						current <= ~current;
						// calculate y coordinate
						if (x2 >= 0) begin
							y_cood <= 10'd352 - x2[16:10];
						end
						else begin
							y_cood <= 10'd479 - x2[16:10];
						end
					end
				end
			end
			
			bus_write <= 1'b1;
			
		end
		
		// and finish write
		if (state==1 && bus_ack==1) begin
			state <= 0 ;
			bus_write <= 1'b0;
		end
		
	end
	
endmodule
