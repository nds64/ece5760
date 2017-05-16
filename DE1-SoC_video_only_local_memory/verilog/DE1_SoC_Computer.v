

module DE1_SoC_Computer (
	////////////////////////////////////
	// FPGA Pins
	////////////////////////////////////

	// Clock pins
	CLOCK_50,
	CLOCK2_50,
	CLOCK3_50,
	CLOCK4_50,

	// ADC
	ADC_CS_N,
	ADC_DIN,
	ADC_DOUT,
	ADC_SCLK,

	// Audio
	AUD_ADCDAT,
	AUD_ADCLRCK,
	AUD_BCLK,
	AUD_DACDAT,
	AUD_DACLRCK,
	AUD_XCK,

	// SDRAM
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N,

	// I2C Bus for Configuration of the Audio and Video-In Chips
	FPGA_I2C_SCLK,
	FPGA_I2C_SDAT,

	// 40-Pin Headers
	GPIO_0,
	GPIO_1,
	
	// Seven Segment Displays
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,

	// IR
	IRDA_RXD,
	IRDA_TXD,

	// Pushbuttons
	KEY,

	// LEDs
	LEDR,

	// PS2 Ports
	PS2_CLK,
	PS2_DAT,
	
	PS2_CLK2,
	PS2_DAT2,

	// Slider Switches
	SW,

	// Video-In
	TD_CLK27,
	TD_DATA,
	TD_HS,
	TD_RESET_N,
	TD_VS,

	// VGA
	VGA_B,
	VGA_BLANK_N,
	VGA_CLK,
	VGA_G,
	VGA_HS,
	VGA_R,
	VGA_SYNC_N,
	VGA_VS,

	////////////////////////////////////
	// HPS Pins
	////////////////////////////////////
	
	// DDR3 SDRAM
	HPS_DDR3_ADDR,
	HPS_DDR3_BA,
	HPS_DDR3_CAS_N,
	HPS_DDR3_CKE,
	HPS_DDR3_CK_N,
	HPS_DDR3_CK_P,
	HPS_DDR3_CS_N,
	HPS_DDR3_DM,
	HPS_DDR3_DQ,
	HPS_DDR3_DQS_N,
	HPS_DDR3_DQS_P,
	HPS_DDR3_ODT,
	HPS_DDR3_RAS_N,
	HPS_DDR3_RESET_N,
	HPS_DDR3_RZQ,
	HPS_DDR3_WE_N,

	// Ethernet
	HPS_ENET_GTX_CLK,
	HPS_ENET_INT_N,
	HPS_ENET_MDC,
	HPS_ENET_MDIO,
	HPS_ENET_RX_CLK,
	HPS_ENET_RX_DATA,
	HPS_ENET_RX_DV,
	HPS_ENET_TX_DATA,
	HPS_ENET_TX_EN,

	// Flash
	HPS_FLASH_DATA,
	HPS_FLASH_DCLK,
	HPS_FLASH_NCSO,

	// Accelerometer
	HPS_GSENSOR_INT,
		
	// General Purpose I/O
	HPS_GPIO,
		
	// I2C
	HPS_I2C_CONTROL,
	HPS_I2C1_SCLK,
	HPS_I2C1_SDAT,
	HPS_I2C2_SCLK,
	HPS_I2C2_SDAT,

	// Pushbutton
	HPS_KEY,

	// LED
	HPS_LED,
		
	// SD Card
	HPS_SD_CLK,
	HPS_SD_CMD,
	HPS_SD_DATA,

	// SPI
	HPS_SPIM_CLK,
	HPS_SPIM_MISO,
	HPS_SPIM_MOSI,
	HPS_SPIM_SS,

	// UART
	HPS_UART_RX,
	HPS_UART_TX,

	// USB
	HPS_CONV_USB_N,
	HPS_USB_CLKOUT,
	HPS_USB_DATA,
	HPS_USB_DIR,
	HPS_USB_NXT,
	HPS_USB_STP
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Clock pins
input						CLOCK_50;
input						CLOCK2_50;
input						CLOCK3_50;
input						CLOCK4_50;

// ADC
inout						ADC_CS_N;
output					ADC_DIN;
input						ADC_DOUT;
output					ADC_SCLK;

// Audio
input						AUD_ADCDAT;
inout						AUD_ADCLRCK;
inout						AUD_BCLK;
output					AUD_DACDAT;
inout						AUD_DACLRCK;
output					AUD_XCK;

// SDRAM
output 		[12: 0]	DRAM_ADDR;
output		[ 1: 0]	DRAM_BA;
output					DRAM_CAS_N;
output					DRAM_CKE;
output					DRAM_CLK;
output					DRAM_CS_N;
inout			[15: 0]	DRAM_DQ;
output					DRAM_LDQM;
output					DRAM_RAS_N;
output					DRAM_UDQM;
output					DRAM_WE_N;

// I2C Bus for Configuration of the Audio and Video-In Chips
output					FPGA_I2C_SCLK;
inout						FPGA_I2C_SDAT;

// 40-pin headers
inout			[35: 0]	GPIO_0;
inout			[35: 0]	GPIO_1;

// Seven Segment Displays
output		[ 6: 0]	HEX0;
output		[ 6: 0]	HEX1;
output		[ 6: 0]	HEX2;
output		[ 6: 0]	HEX3;
output		[ 6: 0]	HEX4;
output		[ 6: 0]	HEX5;

// IR
input						IRDA_RXD;
output					IRDA_TXD;

// Pushbuttons
input			[ 3: 0]	KEY;

// LEDs
output		[ 9: 0]	LEDR;

// PS2 Ports
inout						PS2_CLK;
inout						PS2_DAT;

inout						PS2_CLK2;
inout						PS2_DAT2;

// Slider Switches
input			[ 9: 0]	SW;

// Video-In
input						TD_CLK27;
input			[ 7: 0]	TD_DATA;
input						TD_HS;
output					TD_RESET_N;
input						TD_VS;

// VGA
output		[ 7: 0]	VGA_B;
output					VGA_BLANK_N;
output					VGA_CLK;
output		[ 7: 0]	VGA_G;
output					VGA_HS;
output		[ 7: 0]	VGA_R;
output					VGA_SYNC_N;
output					VGA_VS;



////////////////////////////////////
// HPS Pins
////////////////////////////////////
	
// DDR3 SDRAM
output		[14: 0]	HPS_DDR3_ADDR;
output		[ 2: 0]  HPS_DDR3_BA;
output					HPS_DDR3_CAS_N;
output					HPS_DDR3_CKE;
output					HPS_DDR3_CK_N;
output					HPS_DDR3_CK_P;
output					HPS_DDR3_CS_N;
output		[ 3: 0]	HPS_DDR3_DM;
inout			[31: 0]	HPS_DDR3_DQ;
inout			[ 3: 0]	HPS_DDR3_DQS_N;
inout			[ 3: 0]	HPS_DDR3_DQS_P;
output					HPS_DDR3_ODT;
output					HPS_DDR3_RAS_N;
output					HPS_DDR3_RESET_N;
input						HPS_DDR3_RZQ;
output					HPS_DDR3_WE_N;

// Ethernet
output					HPS_ENET_GTX_CLK;
inout						HPS_ENET_INT_N;
output					HPS_ENET_MDC;
inout						HPS_ENET_MDIO;
input						HPS_ENET_RX_CLK;
input			[ 3: 0]	HPS_ENET_RX_DATA;
input						HPS_ENET_RX_DV;
output		[ 3: 0]	HPS_ENET_TX_DATA;
output					HPS_ENET_TX_EN;

// Flash
inout			[ 3: 0]	HPS_FLASH_DATA;
output					HPS_FLASH_DCLK;
output					HPS_FLASH_NCSO;

// Accelerometer
inout						HPS_GSENSOR_INT;

// General Purpose I/O
inout			[ 1: 0]	HPS_GPIO;

// I2C
inout						HPS_I2C_CONTROL;
inout						HPS_I2C1_SCLK;
inout						HPS_I2C1_SDAT;
inout						HPS_I2C2_SCLK;
inout						HPS_I2C2_SDAT;

// Pushbutton
inout						HPS_KEY;

// LED
inout						HPS_LED;

// SD Card
output					HPS_SD_CLK;
inout						HPS_SD_CMD;
inout			[ 3: 0]	HPS_SD_DATA;

// SPI
output					HPS_SPIM_CLK;
input						HPS_SPIM_MISO;
output					HPS_SPIM_MOSI;
inout						HPS_SPIM_SS;

// UART
input						HPS_UART_RX;
output					HPS_UART_TX;

// USB
inout						HPS_CONV_USB_N;
input						HPS_USB_CLKOUT;
inout			[ 7: 0]	HPS_USB_DATA;
input						HPS_USB_DIR;
input						HPS_USB_NXT;
output					HPS_USB_STP;

//=======================================================
//  REG/WIRE declarations
//=======================================================

HexDigit Digit0(HEX0, 3'b0);

wire reset = ~KEY[0];
wire stop_generate = ~KEY[1];
wire [31:0] bus_addr;
reg [31:0] bus_write_data, bus_read_data;
wire bus_ack;
reg bus_read, bus_write;
reg [4:0] bus_byte_enable;
//wire signed [17:0] x1, x2;
reg [31:0] timer ;
reg [3:0] state ;
reg [9:0] x_cood, y_cood;
wire [31:0] vga_base_address = 32'h0800_0000 ;

wire [31:0] pio_output;
wire hps_en;
wire [9:0] hps_x, hps_y;

assign hps_en = pio_output[31];
assign hps_x  = pio_output[9:0];
assign hps_y  = pio_output[19:10];

assign bus_addr = vga_base_address + {22'b0, x_cood} + ({22'b0, y_cood}<<10);

// set to 1 if the screen has been cleared initially to black
reg init;

wire [6:0] data;
wire [11:0] index;
assign index = count_x + count_y * SIZE;
/*lab5 test (
             .clk(clk_50), 
             .reset(init),
				 .index(index),
				 .data(data)
           );*/
			  
parameter                   SIZE = 45;
parameter                   SIZE2 = 2025;

parameter						 SIZE_X = 640;
parameter          	       SIZE_Y = 480;

reg           [6:0]         p_in_array[0:SIZE2-1];
wire          [6:0]         p_out_array[0:SIZE2-1]; 

wire clk;
assign clk = CLOCK2_50;

//reg we;
//reg [6:0] wr_data, rd_data;
//reg [18:0] wr_addr, rd_addr;

//dual_clock_ram m10k_block
//(
//	.q(rd_data),
//	.d(wr_data),
//	.write_address(wr_addr),
//	.read_address(rd_addr),
//	.we(we);
//	.clk1(CLOCK_50),
//	.clk2(CLOCK2_50)
//);

integer x, y;
always @(posedge CLOCK2_50) begin
	if (trigger) begin
		triggered <= 1;
		for (y = 0; y < SIZE; y = y + 1) begin
			for (x = 0; x < SIZE; x = x + 1) begin
				if(x == 0 || y == 0 || x == (SIZE-1) || y == (SIZE -1)) begin
					p_in_array[y*SIZE+x] <= 7'b0000111;
				end
				else if(SW[1]) begin
					if( x > (SIZE>>2) && x < ((SIZE>>2)+2) && y < SIZE-1 && y > (SIZE-10)) begin
						p_in_array[y*SIZE+x] <= 7'b0000111;
					end
					else if( x > (SIZE>>2) && x < ((SIZE>>2)+2) && y < SIZE-15 && y > 0) begin
						p_in_array[y*SIZE+x] <= 7'b0000111;
					end
					else if( x > (SIZE>>1) && x < ((SIZE>>1)+2) && y < SIZE-1 && y > 15) begin
						p_in_array[y*SIZE+x] <= 7'b0000111;
					end
					else if( x > (SIZE>>1) && x < ((SIZE>>1)+2) && y < 12 && y > 0) begin
						p_in_array[y*SIZE+x] <= 7'b0000111;
					end
				end
				else if(SW[3]) begin
					if( x > (SIZE>>1) && x < ((SIZE>>1)+4) && y < (SIZE>>1)+2 && y > (SIZE>>1)-2) begin
						p_in_array[y*SIZE+x] <= 7'b0010001;
					end
					else if( x < (SIZE>>1) && x > ((SIZE>>1)-4) && y < (SIZE>>1)+2 && y > (SIZE>>1)-2) begin
						p_in_array[y*SIZE+x] <= 7'b1000001;
					end
					else if( y < (SIZE>>1) && y > ((SIZE>>1)-4) && x < (SIZE>>1)+2 && x > (SIZE>>1)-2) begin
						p_in_array[y*SIZE+x] <= 7'b0001001;
					end
					else if(  y > (SIZE>>1) && y < ((SIZE>>1)+4) && x < (SIZE>>1)+2 && x > (SIZE>>1)-2) begin
						p_in_array[y*SIZE+x] <= 7'b0100001;
					end
				end
				else begin 
					p_in_array[y*SIZE+x] <= 7'd0;
				end
			end
		end	
	end
	else if (triggered) begin
		for (x = 0; x < SIZE2; x = x + 1) begin
			if (hps_en && x == (hps_y*SIZE+hps_x))
				p_in_array[x] <= 7'b0000111;
			else
				p_in_array[x] <= p_out_array[x];
		end
	end 
	else begin
		triggered <= 0;
	end
end

assign data = p_in_array[index];	

genvar i, j;
generate 
for (j = 0; j < SIZE; j = j + 1) begin: NODE_X
	for (i = 0; i < SIZE; i = i + 1) begin: NODE_Y
		if(i == 0 && j == 0) 					// check top left corner
			node_update x_dir(clk, reset, enable, p_in_array[j*SIZE+i], 7'd0, 					p_in_array[j*SIZE+i+1], p_in_array[(j+1)*SIZE+i], 	7'd0, 						p_out_array[j*SIZE+i]  );
		else if (i == SIZE-1 && j == 0) 		// check top right corner
			node_update x_dir(clk, reset, enable, p_in_array[j*SIZE+i], 7'd0, 					7'd0,		 			p_in_array[(j+1)*SIZE+i],	p_in_array[(j*SIZE+i-1)],	p_out_array[j*SIZE+i]  );
		else if (i == 0 && j == SIZE-1)			// check bottom left corner
			node_update x_dir(clk, reset, enable, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i],	p_in_array[j*SIZE+i+1], 7'd0,					 	7'd0, 						p_out_array[j*SIZE+i]  );
		else if (i == SIZE-1 && j == SIZE-1) 	// check bottom right corner
			node_update x_dir(clk, reset, enable, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i], 7'd0, 					7'd0,						p_in_array[(j*SIZE+i-1)], 	p_out_array[j*SIZE+i]  );
		else if (j == 0)						// check top row
			node_update x_dir(clk, reset, enable, p_in_array[j*SIZE+i], 7'd0, 					p_in_array[j*SIZE+i+1], p_in_array[(j+1)*SIZE+i], 	p_in_array[(j*SIZE+i-1)], 	p_out_array[j*SIZE+i]  );
		else if (j == SIZE-1)					// check bottom row
			node_update x_dir(clk, reset, enable, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i], p_in_array[j*SIZE+i+1], 7'd0, 						p_in_array[(j*SIZE+i-1)], 	p_out_array[j*SIZE+i]  );
		else if (i == 0)						// check left column
			node_update x_dir(clk, reset, enable, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i], p_in_array[j*SIZE+i+1], p_in_array[(j+1)*SIZE+i], 	7'd0, 						p_out_array[j*SIZE+i]  );
		else if (i == SIZE-1)					// check right column
			node_update x_dir(clk, reset, enable, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i], 7'd0, 					p_in_array[(j+1)*SIZE+i], 	p_in_array[(j*SIZE+i-1)], 	p_out_array[j*SIZE+i]  );
		else if( i == 1 && j < (SIZE>>4)) begin
			assign p_out_array[j*SIZE+i] = SW[0] ? 7'b0010001 : 7'd0;
		end
		else if( i < (SIZE>>4) && j == SIZE-2) begin
			assign p_out_array[j*SIZE+i] = SW[0] ? 7'b0001001 : 7'd0;
		end
		else
			node_update x_dir(clk, reset, enable, p_in_array[j*SIZE+i], p_in_array[(j-1)*SIZE+i],  p_in_array[j*SIZE+i+1],  
												p_in_array[(j+1)*SIZE+i],  p_in_array[(j*SIZE+i-1)], p_out_array[j*SIZE+i]  );
	end
end
endgenerate 

reg [7:0] rgb_data;
reg [9:0] x_cood_ref, y_cood_ref;

//reg [11:0] count_global;
reg [5:0] count_x, count_y;
reg [3:0] count_local_x, count_local_y;

reg start, enable, trigger, triggered;

always @(posedge CLOCK2_50) begin
	
	// reset state machine and read/write controls
	if (reset) begin
	
		state <= 2; // clear the screen at first
		init <= 0;
		start <= 0;
		enable <= 0;
		trigger <= 0;
		
		// base address of upper-left corner of the screen
		x_cood <= 10'd0 ;
		y_cood <= 10'd0 ;
		
		count_x <= 0;
		count_y <= 0;
		
		bus_write <= 0 ; // set to on if a write operation to bus
		bus_byte_enable <= 4'b0001; // check later to see if this matters
		bus_read <= 1'b0;
		
		timer <= 0; // might be useful
		
		count_local_x <= 0;
		count_local_y <= 0;
		x_cood_ref <= 139;
		y_cood_ref <= 59;
		
	end
	else begin
		timer <= timer + 1;
	end
	
	// write to the bus-master
	// and put in a small delay to aviod bus hogging
	// timer delay can be set to 2**n-1, so 3, 7, 15, 31
	// bigger numbers mean slower frame update to VGA
	if (state==0 && (~VGA_HS | ~VGA_VS) && (timer & 3)==0) begin //&& (timer & 1023)==0
		state <= 1;
		bus_write <= 1'b1;
//		if (count_global < 2499) begin
//			count_global <= count_global + 1;
//		end
//		else begin
//			count_global <= 0;
//		end
		count_local_x <= count_local_x + 1;
		if (count_local_x > 7) begin
			count_local_x <= 0;
			count_local_y <= count_local_y + 1;
			if (count_local_y > 7) begin
				count_local_y <= 0;
				x_cood_ref <= x_cood_ref + 8;
				count_x <= count_x + 1;
				if (count_x > SIZE-1) begin
					count_x <= 0 ;
					y_cood_ref <= y_cood_ref + 8;
					count_y <= count_y + 1 ;
					if (count_y > SIZE-1) begin
						count_y <= 0 ;
					end
				end
			end
		end
		x_cood <= x_cood_ref + count_local_x;
		y_cood <= y_cood_ref + count_local_y;
		if (x_cood_ref > 499) begin
			x_cood_ref <= 139 ;
			if (y_cood_ref > 419) begin
				y_cood_ref <= 59 ;
			end
		end
		bus_write_data <= (data[2:0] == 3'b111) ? {3'b0, data[2:0], 2'b0} : ((data[2:0] == 3'b0) ? 8'b0 : {3'b111, 5'b0});
		//bus_write_data <= {3'b111, 5'b0};
		
	end
	else if (state == 0) begin
		enable <= 0;
		trigger <= 0;
	end
	// and finish write
	if (state==1 && bus_ack==1) begin
		bus_write <= 1'b0;
		if (init) begin
			state <= 0;
			x_cood <= 139;
			y_cood <= 59;
			//count_global <= 0;
			init <= 0;
			start <= 1;
			trigger <= 1;
			enable <= 1;
			timer <= 0;
		end
		else if (start) begin
			state <= 0;
			enable <= 1;
		end
		else begin
			state <= 2;
		end
	end
	
	// initialize the screen to be blank in black
	if (state==2 && (~VGA_HS | ~VGA_VS)) begin //&& (~VGA_HS | ~VGA_VS)
		state <= 1;
		
		// VGA bus
		bus_write <= 1'b1;
		bus_write_data <= 8'h00; // black
		// write all the pixels
		x_cood <= x_cood + 10'd1 ;
		if (x_cood > 10'd639) begin
			x_cood <= 0 ;
			y_cood <= y_cood + 10'd1 ;
			if (y_cood > 10'd479) begin
				y_cood <= 0 ;
				init <= 1'b1;
			end
		end
		
		// M10K
//		we <= 1;
//		wr_addr <= y_cood * SIZE_Y + x_cood;
//		wr_data <= (x == 0 || y == 0 || x == (SIZE-1) || y == (SIZE -1)) 
//	? 7'b0000111 : (x == 3) ? 7'b0010001 : (x == SIZE_X-3) ? 7'b1000001 : 7'd0;
	end
	
end


//=======================================================
//  Structural coding
//=======================================================

Computer_System The_System (
	////////////////////////////////////
	// FPGA Side
	////////////////////////////////////

	// Global signals
	.system_pll_ref_clk_clk					(CLOCK_50), //?
	.system_pll_ref_reset_reset			(1'b0),
	
	// AV Config
	.av_config_SCLK							(FPGA_I2C_SCLK),
	.av_config_SDAT							(FPGA_I2C_SDAT),
	
	// PIO export
	.pio_0_external_connection_export   (pio_output),
	
	// bus master
	.bus_master_video_external_interface_address     (bus_addr),     // .address
	.bus_master_video_external_interface_byte_enable (bus_byte_enable), //  .byte_enable
	.bus_master_video_external_interface_read        (bus_read),        //   .read
	.bus_master_video_external_interface_write       (bus_write),       //   .write
	.bus_master_video_external_interface_write_data  (bus_write_data),  //   .write_data
	.bus_master_video_external_interface_acknowledge (bus_ack), //    .acknowledge
	.bus_master_video_external_interface_read_data   (bus_read_data),    //   .read_data

	// VGA Subsystem
	.vga_pll_ref_clk_clk 					(CLOCK2_50),
	.vga_pll_ref_reset_reset				(1'b0),
	.vga_CLK										(VGA_CLK),
	.vga_BLANK									(VGA_BLANK_N),
	.vga_SYNC									(VGA_SYNC_N),
	.vga_HS										(VGA_HS),
	.vga_VS										(VGA_VS),
	.vga_R										(VGA_R),
	.vga_G										(VGA_G),
	.vga_B										(VGA_B),
	
	// SDRAM
	.sdram_clk_clk								(DRAM_CLK),
   .sdram_addr									(DRAM_ADDR),
	.sdram_ba									(DRAM_BA),
	.sdram_cas_n								(DRAM_CAS_N),
	.sdram_cke									(DRAM_CKE),
	.sdram_cs_n									(DRAM_CS_N),
	.sdram_dq									(DRAM_DQ),
	.sdram_dqm									({DRAM_UDQM,DRAM_LDQM}),
	.sdram_ras_n								(DRAM_RAS_N),
	.sdram_we_n									(DRAM_WE_N),
	
	////////////////////////////////////
	// HPS Side
	////////////////////////////////////
	// DDR3 SDRAM
	.memory_mem_a			(HPS_DDR3_ADDR),
	.memory_mem_ba			(HPS_DDR3_BA),
	.memory_mem_ck			(HPS_DDR3_CK_P),
	.memory_mem_ck_n		(HPS_DDR3_CK_N),
	.memory_mem_cke		(HPS_DDR3_CKE),
	.memory_mem_cs_n		(HPS_DDR3_CS_N),
	.memory_mem_ras_n		(HPS_DDR3_RAS_N),
	.memory_mem_cas_n		(HPS_DDR3_CAS_N),
	.memory_mem_we_n		(HPS_DDR3_WE_N),
	.memory_mem_reset_n	(HPS_DDR3_RESET_N),
	.memory_mem_dq			(HPS_DDR3_DQ),
	.memory_mem_dqs		(HPS_DDR3_DQS_P),
	.memory_mem_dqs_n		(HPS_DDR3_DQS_N),
	.memory_mem_odt		(HPS_DDR3_ODT),
	.memory_mem_dm			(HPS_DDR3_DM),
	.memory_oct_rzqin		(HPS_DDR3_RZQ),
		  
	// Ethernet
	.hps_io_hps_io_gpio_inst_GPIO35	(HPS_ENET_INT_N),
	.hps_io_hps_io_emac1_inst_TX_CLK	(HPS_ENET_GTX_CLK),
	.hps_io_hps_io_emac1_inst_TXD0	(HPS_ENET_TX_DATA[0]),
	.hps_io_hps_io_emac1_inst_TXD1	(HPS_ENET_TX_DATA[1]),
	.hps_io_hps_io_emac1_inst_TXD2	(HPS_ENET_TX_DATA[2]),
	.hps_io_hps_io_emac1_inst_TXD3	(HPS_ENET_TX_DATA[3]),
	.hps_io_hps_io_emac1_inst_RXD0	(HPS_ENET_RX_DATA[0]),
	.hps_io_hps_io_emac1_inst_MDIO	(HPS_ENET_MDIO),
	.hps_io_hps_io_emac1_inst_MDC		(HPS_ENET_MDC),
	.hps_io_hps_io_emac1_inst_RX_CTL	(HPS_ENET_RX_DV),
	.hps_io_hps_io_emac1_inst_TX_CTL	(HPS_ENET_TX_EN),
	.hps_io_hps_io_emac1_inst_RX_CLK	(HPS_ENET_RX_CLK),
	.hps_io_hps_io_emac1_inst_RXD1	(HPS_ENET_RX_DATA[1]),
	.hps_io_hps_io_emac1_inst_RXD2	(HPS_ENET_RX_DATA[2]),
	.hps_io_hps_io_emac1_inst_RXD3	(HPS_ENET_RX_DATA[3]),

	// Flash
	.hps_io_hps_io_qspi_inst_IO0	(HPS_FLASH_DATA[0]),
	.hps_io_hps_io_qspi_inst_IO1	(HPS_FLASH_DATA[1]),
	.hps_io_hps_io_qspi_inst_IO2	(HPS_FLASH_DATA[2]),
	.hps_io_hps_io_qspi_inst_IO3	(HPS_FLASH_DATA[3]),
	.hps_io_hps_io_qspi_inst_SS0	(HPS_FLASH_NCSO),
	.hps_io_hps_io_qspi_inst_CLK	(HPS_FLASH_DCLK),

	// Accelerometer
	.hps_io_hps_io_gpio_inst_GPIO61	(HPS_GSENSOR_INT),

	//.adc_sclk                        (ADC_SCLK),
	//.adc_cs_n                        (ADC_CS_N),
	//.adc_dout                        (ADC_DOUT),
	//.adc_din                         (ADC_DIN),

	// General Purpose I/O
	.hps_io_hps_io_gpio_inst_GPIO40	(HPS_GPIO[0]),
	.hps_io_hps_io_gpio_inst_GPIO41	(HPS_GPIO[1]),

	// I2C
	.hps_io_hps_io_gpio_inst_GPIO48	(HPS_I2C_CONTROL),
	.hps_io_hps_io_i2c0_inst_SDA		(HPS_I2C1_SDAT),
	.hps_io_hps_io_i2c0_inst_SCL		(HPS_I2C1_SCLK),
	.hps_io_hps_io_i2c1_inst_SDA		(HPS_I2C2_SDAT),
	.hps_io_hps_io_i2c1_inst_SCL		(HPS_I2C2_SCLK),

	// Pushbutton
	.hps_io_hps_io_gpio_inst_GPIO54	(HPS_KEY),

	// LED
	.hps_io_hps_io_gpio_inst_GPIO53	(HPS_LED),

	// SD Card
	.hps_io_hps_io_sdio_inst_CMD	(HPS_SD_CMD),
	.hps_io_hps_io_sdio_inst_D0	(HPS_SD_DATA[0]),
	.hps_io_hps_io_sdio_inst_D1	(HPS_SD_DATA[1]),
	.hps_io_hps_io_sdio_inst_CLK	(HPS_SD_CLK),
	.hps_io_hps_io_sdio_inst_D2	(HPS_SD_DATA[2]),
	.hps_io_hps_io_sdio_inst_D3	(HPS_SD_DATA[3]),

	// SPI
	.hps_io_hps_io_spim1_inst_CLK		(HPS_SPIM_CLK),
	.hps_io_hps_io_spim1_inst_MOSI	(HPS_SPIM_MOSI),
	.hps_io_hps_io_spim1_inst_MISO	(HPS_SPIM_MISO),
	.hps_io_hps_io_spim1_inst_SS0		(HPS_SPIM_SS),

	// UART
	.hps_io_hps_io_uart0_inst_RX	(HPS_UART_RX),
	.hps_io_hps_io_uart0_inst_TX	(HPS_UART_TX),

	// USB
	.hps_io_hps_io_gpio_inst_GPIO09	(HPS_CONV_USB_N),
	.hps_io_hps_io_usb1_inst_D0		(HPS_USB_DATA[0]),
	.hps_io_hps_io_usb1_inst_D1		(HPS_USB_DATA[1]),
	.hps_io_hps_io_usb1_inst_D2		(HPS_USB_DATA[2]),
	.hps_io_hps_io_usb1_inst_D3		(HPS_USB_DATA[3]),
	.hps_io_hps_io_usb1_inst_D4		(HPS_USB_DATA[4]),
	.hps_io_hps_io_usb1_inst_D5		(HPS_USB_DATA[5]),
	.hps_io_hps_io_usb1_inst_D6		(HPS_USB_DATA[6]),
	.hps_io_hps_io_usb1_inst_D7		(HPS_USB_DATA[7]),
	.hps_io_hps_io_usb1_inst_CLK		(HPS_USB_CLKOUT),
	.hps_io_hps_io_usb1_inst_STP		(HPS_USB_STP),
	.hps_io_hps_io_usb1_inst_DIR		(HPS_USB_DIR),
	.hps_io_hps_io_usb1_inst_NXT		(HPS_USB_NXT)
);
endmodule
////////////////////////////////////////////////////