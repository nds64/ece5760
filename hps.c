///////////////////////////////////
//ECE 5760 Final Project HPS Code//
//////////////////////////////////
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <sys/time.h> 
#include <math.h>

#include "address_map_arm_brl4.h"

/* function prototypes */
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);

// the bit position of the switch
#define bit_25 0x02000000

// base address of pio
#define PIO_BASE 0x00000000

/* function prototypes */


// the light weight buss base
void *h2p_lw_virtual_base;
volatile unsigned int *sw_base = NULL;

// pixel buffer
volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;


// /dev/mem file id
//int fd;

// shared memory 
key_t mem_key=0xf0;
int shared_mem_id; 
int *shared_ptr;
int shared_time;
int shared_note;
char shared_str[64];

// pixel macro
#define VGA_PIXEL(x,y,color) do{\
	char  *pixel_ptr ;\
	pixel_ptr = (char *)vga_pixel_ptr + ((y)<<10) + (x) ;\
	*(char *)pixel_ptr = (color);\
} while(0)
	

// measure time
struct timeval t1, t2;
double elapsedTime;


int main(void)
{

	// Declare volatile pointers to I/O registers (volatile 	// means that IO load and store instructions will be used 	// to access these pointer locations, 
	// instead of regular memory loads and stores) 

	// === shared memory =======================
	// with video process
	shared_mem_id = shmget(mem_key, 100, IPC_CREAT | 0666);
 	//shared_mem_id = shmget(mem_key, 100, 0666);
	shared_ptr = shmat(shared_mem_id, NULL, 0);

    // === FPGA ===
    volatile unsigned int *h2p_lw_led_addr=NULL;
    volatile unsigned int *h2p_lw_hex_addr=NULL;
    volatile unsigned int *h2p_lw_pio_addr=NULL;
    void *h2p_lw_virtual_base;
    int fd;
    // === HPS switch ===
    volatile unsigned int *led_addr=NULL;
    void *virtual_base_gpio;
    //int fd1 ;
    // =================
    volatile int i;

    // === get FPGA addresses ===
    // Open /dev/mem
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

    // get virtual addr that maps to physical
	h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
	if( h2p_lw_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return(1);
	}

    // Get the address that maps to the FPGA LED control
	h2p_lw_led_addr=(unsigned int *)(h2p_lw_virtual_base + (( LEDR_BASE ) ));

    // get address for hex display offset 0x10
    h2p_lw_hex_addr=(volatile unsigned int *)(h2p_lw_virtual_base + HEX3_HEX0_BASE);

    h2p_lw_pio_addr=(volatile unsigned int *)(h2p_lw_virtual_base + PIO_BASE);

    // === get HPS GPIO1 address ===
    // Open /dev/mem
        if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
                printf( "ERROR: could not open \"/dev/mem\"...\n" );
                return( 1 );
        }

    // get virtual addr that maps to physical
        virtual_base_gpio = mmap( NULL, 0x1000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HPS_GPIO1_BASE );
        if( virtual_base_gpio == MAP_FAILED ) {
                printf( "ERROR: mmap() failed...\n" );
                close( fd);
                return(1);
        }

    //Get the address that maps to the GPIO base
    led_addr=(volatile unsigned int *)(virtual_base_gpio);




	// === get VGA char addr =====================
	// get virtual addr that maps to physical
	vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE );	
	if( vga_char_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap2() failed...\n" );
		close( fd );
		return(1);
	}
    
  // Get the address that maps to the FPGA LED control 
	vga_char_ptr =(unsigned int *)(vga_char_virtual_base);

	// === get VGA pixel addr ====================
	// get virtual addr that maps to physical
	vga_pixel_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_ONCHIP_BASE);	
	if( vga_pixel_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
    
  // Get the address that maps to the FPGA pixel buffer
	vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);

	// ===========================================

	/* create a message to be displayed on the VGA 
          and LCD displays */
	char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
	char text_bottom_row[40] = "Cornell ece5760\0";
	char num_string[20], time_string[40] ;

	
	// clear the screen
	VGA_box (0, 0, 639, 479, 0x00);
	// clear the text
	VGA_text_clear();
	VGA_text (1, 58, text_top_row);
	VGA_text (1, 59, text_bottom_row);


	// zero global counter
	int total_count = 0 ;


    // =============================
    // On HPS side:
    //   read the GPIO input register at led_addr+0x14
    //   led_addr happens to be at offset zero from GPIO base address
    // On FPGA side:
    //   output to two FPGA PIO ports to control
    //   red LEDs and 4 hex digits
    char input[20];
    //float k1, k2, k3, x1, x2, v1, v2, test;
    int x_min, x_max, y_min, y_max;
    //int state = 0, cubic = 0, sel;
    int state = 0;
    //char input_c[100];
   // while(sel != 4){
	 // variables for mouse signals
  int fd1, bytes=-1;
  unsigned char data[3];
  int left_click, middle_click, right_click;
  signed char x, y;
  srand(t1.tv_sec * 1000);  

  const char *pDevice = "/dev/input/mice";

  // Open Mouse
  fd1 = open(pDevice, O_RDWR);
  if(fd1 == -1)
  {
    printf("ERROR Opening %s\n", pDevice);
    return -1;
  }
  
  int mp_x, mp_y;
  //int left,right,middle;
  mp_x = 320;
  mp_y = 240;
  
  unsigned int temp; //used to store the data that should be sent to FPGA
  while(1) 
  {     
        //31 bit = 1 - Sending mouse coordinate data
		//10 - 19 bits - x coordinate
		// 0 -  9 bits - y coordinate

    // Read Mouse     
    bytes = read(fd1, data, sizeof(data));
    if(bytes < 0){
    	continue;
    }

    if(bytes > 0)
    {
      left_click = data[0] & 0x1;
      right_click = data[0] & 0x2;
      middle_click = data[0] & 0x4;

      x = data[1];
      y = data[2];
    }
    
    //erase last mouse cursor
	VGA_PIXEL(mp_x,mp_y,0x00);
	VGA_PIXEL(mp_x,mp_y-1,0x00);
	VGA_PIXEL(mp_x,mp_y-2,0x00);
	VGA_PIXEL(mp_x,mp_y+1,0x00);
	VGA_PIXEL(mp_x,mp_y+2,0x00);
	VGA_PIXEL(mp_x-1,mp_y,0x00);
	VGA_PIXEL(mp_x-2,mp_y,0x00);
	VGA_PIXEL(mp_x+1,mp_y,0x00);
	VGA_PIXEL(mp_x+2,mp_y,0x00);



    
    mp_x = mp_x + x;  //calculate new mouse position
    mp_y = mp_y - y;

    //boundry condition
    if (mp_x < 2)
      mp_x = 2;
    if (mp_x > 630)
      mp_x = 625;
    if (mp_y < 2)
      mp_y = 2;
    if (mp_y > 475)
      mp_y = 470;
    
    if (left_click) {
        temp = 0;   //clear
	    temp = (mp_y << 10);//put y-coordinate to 10-19 bit
        temp = temp+mp_x;  //put x-coordinate to 0-9 bit
	    temp |= 1 << 31;  //set bit 31 to 1
        *h2p_lw_pio_addr = temp; //send to FPGA
		int i;
	    for (i=0;i<10000;i++); //debounce mouse click
      
    }
    printf("x=%d, y=%d, left=%d, dataSent=%x\n", mp_x, mp_y, left_click, temp);


    temp = 0;  
    *h2p_lw_pio_addr = temp; //reset, so FPGA will not get updates when it is not needed

	VGA_PIXEL(mp_x,mp_y,0xff);
	VGA_PIXEL(mp_x,mp_y-1,0xff);
	VGA_PIXEL(mp_x,mp_y-2,0xff);
	VGA_PIXEL(mp_x,mp_y+1,0xff);
	VGA_PIXEL(mp_x,mp_y+2,0xff);
	VGA_PIXEL(mp_x-1,mp_y,0xff);
	VGA_PIXEL(mp_x-2,mp_y,0xff);
	VGA_PIXEL(mp_x+1,mp_y,0xff);
	VGA_PIXEL(mp_x+2,mp_y,0xff);					
		
	} // end while(1)
   return 0;
}

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor 
****************************************************************************************/
void VGA_text(int x, int y, char * text_ptr)
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset;
	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		// write to the character buffer
		*(character_buffer + offset) = *(text_ptr);	
		++text_ptr;
		++offset;
	}
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor 
****************************************************************************************/
void VGA_text_clear()
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset, x, y;
	for (x=0; x<70; x++){
		for (y=0; y<40; y++){
	/* assume that the text string fits on one line */
			offset = (y << 7) + x;
			// write to the character buffer
			*(character_buffer + offset) = ' ';		
		}
	}
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	for (row = y1; row <= y2; row++)
		for (col = x1; col <= x2; ++col)
		{
			//640x480
			pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
			// set pixel color
			*(char *)pixel_ptr = pixel_color;		
		}
}


