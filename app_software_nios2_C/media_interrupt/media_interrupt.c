#include "nios2_ctrl_reg_macros.h"
#include "address_map_nios2.h"

/* these globals are written by interrupt service routines; we have to declare 
 * these as volatile to avoid the compiler caching their values in registers */
extern volatile char byte1, byte2, byte3;			/* modified by PS/2 interrupt service routine */
extern volatile int record, play, buffer_index;	// used for audio
extern volatile int timeout;							// used to synchronize with the timer

/* function prototypes */
void VGA_text (int, int, char *);
void VGA_set_pixel_buffer_addr(unsigned int);
void VGA_box (int, int, int, int, short);
void HEX_PS2(char, char, char);

/********************************************************************************
 * This program demonstrates use of the media ports in the DE1-SoC Computer
 *
 * It performs the following: 
 *  1. records audio for about 10 seconds when an interrupt is generated by
 *  	   pressing KEY[1]. LEDR[0] is lit while recording. Audio recording is 
 *  	   controlled by using interrupts
 * 	2. plays the recorded audio when an interrupt is generated by pressing
 * 	   KEY[2]. LEDR[1] is lit while playing. Audio playback is controlled by 
 * 	   using interrupts
 * 	3. Draws a blue box on the VGA display, and places a text string inside
 * 	   the box. Also, moves the word ALTERA around the display, "bouncing" off
 * 	   the blue box and screen edges
 * 	4. Displays the last three bytes of data received from the PS/2 port 
 * 	   on the HEX displays on the DE1-SoC board. The PS/2 port is handled using 
 * 	   interrupts
 * 	5. The speed of refreshing the VGA screen are controlled by interrupts 
 * 	   from the interval timer
********************************************************************************/
int main(void)
{
	/* Declare volatile pointers to I/O registers (volatile means that IO load
	   and store instructions will be used to access these pointer locations, 
	   instead of regular memory loads and stores) */
	volatile int * interval_timer_ptr = (int *) TIMER_BASE;	// interal timer base address
	volatile int * KEY_ptr = (int *) KEY_BASE;					// pushbutton KEY address
	volatile int * audio_ptr = (int *) AUDIO_BASE;				// audio port address
	volatile int * PS2_ptr = (int *) PS2_BASE;					// PS/2 port address

	/* initialize some variables */
	byte1 = 0; byte2 = 0; byte3 = 0; 			// used to hold PS/2 data
	record = 0; play = 0; buffer_index = 0;	// used for audio record/playback
	timeout = 0;										// synchronize with the timer

	/* these variables are used for a blue box and a "bouncing" ALTERA on the VGA screen */
	int ALT_x1; int ALT_x2; int ALT_y; 
	int ALT_inc_x; int ALT_inc_y;
	int blue_x1; int blue_y1; int blue_x2; int blue_y2; 
	int screen_x; int screen_y; int char_buffer_x; int char_buffer_y;
	short color;

	/* set the interval timer period for scrolling the HEX displays */
	int counter = 0x960000;				// 1/(50 MHz) x (0x960000) ~= 200 msec
	*(interval_timer_ptr + 0x2) = (counter & 0xFFFF);
	*(interval_timer_ptr + 0x3) = (counter >> 16) & 0xFFFF;

	/* start interval timer, enable its interrupts */
	*(interval_timer_ptr + 1) = 0x7;	// STOP = 0, START = 1, CONT = 1, ITO = 1 
	
	*(KEY_ptr + 2) = 0xE; 			/* write to the pushbutton interrupt mask register, and
											 * set 3 mask bits to 1 (bit 0 is Nios II reset) */

	*(PS2_ptr) = 0xFF;				/* reset */
	*(PS2_ptr + 1) = 0x1; 			/* write to the PS/2 Control register to enable interrupts */

	NIOS2_WRITE_IENABLE( 0xC3 );	/* set interrupt mask bits for levels 0 (interval
											 * timer), 1 (pushbuttons), 6 (audio), and 7 (PS/2) */

	NIOS2_WRITE_STATUS( 1 );		// enable Nios II interrupts

	/* create a messages to be displayed on the VGA and LCD displays */
	char text_top_VGA[20] = "Altera DE1-SoC\0";
	char text_bottom_VGA[20] = "Computer\0";
	char text_ALTERA[10] = "ALTERA\0";
	char text_erase[10] = "      \0";
    
    /* Set the on-chip SRAM as the VGA pixel buffer */      
	VGA_set_pixel_buffer_addr(FPGA_ONCHIP_BASE);    

	/* the following variables give the size of the pixel buffer */
	screen_x = 319; screen_y = 239;
	color = 0;		// black
	VGA_box (0, 0, screen_x, screen_y, color);	// fill the screen with grey
    
	// draw a medium-blue box around the above text, based on the character buffer coordinates
	blue_x1 = 28; blue_x2 = 52; blue_y1 = 26; blue_y2 = 34;
	// character coords * 4 since characters are 4 x 4 pixel buffer coords (8 x 8 VGA coords)
	color = 0x187F;		// a medium blue color
	VGA_box (blue_x1 * 4, blue_y1 * 4, blue_x2 * 4, blue_y2 * 4, color);
	/* output text message in the middle of the VGA monitor */
	/* First clear the character buffer */
	int *p;
	for (p = FPGA_CHAR_BASE; p < FPGA_CHAR_END; ++p)
		*p = 0;
	VGA_text (blue_x1 + 5, blue_y1 + 3, text_top_VGA);
	VGA_text (blue_x1 + 5, blue_y1 + 4, text_bottom_VGA);

	char_buffer_x = 79; char_buffer_y = 59;
	ALT_x1 = 0; ALT_x2 = 5/* ALTERA = 6 chars */; ALT_y = 0; ALT_inc_x = 1; ALT_inc_y = 1;
	VGA_text (ALT_x1, ALT_y, text_ALTERA);
	while (1)
	{
		while (!timeout)
			;	// wait to synchronize with timer 

		/* move the ALTERA text around on the VGA screen */
		VGA_text (ALT_x1, ALT_y, text_erase);		// erase
		ALT_x1 += ALT_inc_x; 
		ALT_x2 += ALT_inc_x; 
		ALT_y += ALT_inc_y;

		if ( (ALT_y == char_buffer_y) || (ALT_y == 0) )
			ALT_inc_y = -(ALT_inc_y);
		if ( (ALT_x2 == char_buffer_x) || (ALT_x1 == 0) )
			ALT_inc_x = -(ALT_inc_x);

		if ( (ALT_y >= blue_y1 - 1) && (ALT_y <= blue_y2 + 1) )
		{
			if ( ((ALT_x1 >= blue_x1 - 1) && (ALT_x1 <= blue_x2 + 1)) ||
				((ALT_x2 >= blue_x1 - 1) && (ALT_x2 <= blue_x2 + 1)) )
			{
				if ( (ALT_y == (blue_y1 - 1)) || (ALT_y == (blue_y2 + 1)) )
					ALT_inc_y = -(ALT_inc_y);
				else
					ALT_inc_x = -(ALT_inc_x);
			}
		}
		VGA_text (ALT_x1, ALT_y, text_ALTERA);

		/* display PS/2 data (from interrupt service routine) on HEX displays */
		HEX_PS2 (byte1, byte2, byte3);
		timeout = 0;
	}
}

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor 
****************************************************************************************/
void VGA_text(int x, int y, char * text_ptr)
{
	int offset;
  	volatile char * character_buffer = (char *) FPGA_CHAR_BASE;	// VGA character buffer

	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		*(character_buffer + offset) = *(text_ptr);	// write to the character buffer
		++text_ptr;
		++offset;
	}
}

/****************************************************************************************
 * Subroutine to set the Onchip SRAM as the buffer for the VGA monitor 
****************************************************************************************/
void VGA_set_pixel_buffer_addr(unsigned int pixel_buffer_addr)
{
 	volatile int * pixel_buffer_regs = (int *) PIXEL_BUF_CTRL_BASE;	// pixel buffer control 
    
	*(pixel_buffer_regs + 1) = pixel_buffer_addr;   // change back buffer address    
    
	*(pixel_buffer_regs) = 1;                       // swap buffers
    
    while(*(pixel_buffer_regs + 3) & 0x1);          // check swap buffers status
    
	*(pixel_buffer_regs + 1) = pixel_buffer_addr;   // change back buffer address    
    
    return;
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
{
	int offset, row, col;
  	volatile short * pixel_buffer = (short *) FPGA_ONCHIP_BASE;	// VGA pixel buffer

	/* assume that the box coordinates are valid */
	for (row = y1; row <= y2; row++)
	{
		col = x1;
		while (col <= x2)
		{
			offset = (row << 9) + col;
			*(pixel_buffer + offset) = pixel_color;	// compute halfword address, set pixel
			++col;
		}
	}
}

/****************************************************************************************
 * Subroutine to show a string of HEX data on the HEX displays
****************************************************************************************/
void HEX_PS2(char b1, char b2, char b3)
{
	volatile int * HEX3_HEX0_ptr = (int *) HEX3_HEX0_BASE;
	volatile int * HEX5_HEX4_ptr = (int *) HEX5_HEX4_BASE;

	/* SEVEN_SEGMENT_DECODE_TABLE gives the on/off settings for all segments in 
	 * a single 7-seg display in the DE1-SoC Computer, for the hex digits 0 - F */
	unsigned char	seven_seg_decode_table[] = {	0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 
		  										0x7F, 0x67, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };
	unsigned char	hex_segs[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	unsigned int shift_buffer, nibble;
	unsigned char code;
	int i;

	shift_buffer = (b1 << 16) | (b2 << 8) | b3;
	for ( i = 0; i < 6; ++i )
	{
		nibble = shift_buffer & 0x0000000F;		// character is in rightmost nibble
		code = seven_seg_decode_table[nibble];
		hex_segs[i] = code;
		shift_buffer = shift_buffer >> 4;
	}
	/* drive the hex displays */
	*(HEX3_HEX0_ptr) = *(int *) (hex_segs);
	*(HEX5_HEX4_ptr) = *(int *) (hex_segs+4);
}
