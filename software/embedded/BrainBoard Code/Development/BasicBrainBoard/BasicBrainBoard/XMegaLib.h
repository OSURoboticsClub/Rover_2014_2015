/*
 * XMegaMacros.h
 *
 * Author: Nick
 */ 



//Error and Status LED outputs

#define STATUS1_SET(void) (PORTC.OUTSET = PIN5_bm)
#define STATUS1_CLR(void) (PORTC.OUTCLR = PIN5_bm)

//Check the DIP Switches on the BB
#define CHECK_DIP_SW_1(void) (!(PORTC.IN & PIN7_bm)) //Returns true if bit 1 of the DIP Switch is "ON"
#define CHECK_DIP_SW_2(void) (!(PORTC.IN & PIN6_bm)) //Returns true if bit 2 of the DIP Switch is "ON"

//RGB LED Control
//RGB LED Pin Descriptions (All on Port C)
//0 - Blue
//1 - Red
//4 - Green

#define RGB_BLUE_SET(void) (PORTC.OUTSET = PIN0_bm)
#define RGB_BLUE_CLR(void) (PORTC.OUTCLR = PIN0_bm)

#define RGB_RED_SET(void) (PORTC.OUTSET = PIN1_bm)
#define RGB_RED_CLR(void) (PORTC.OUTCLR = PIN1_bm)

#define RGB_GREEN_SET(void) (PORTC.OUTSET = PIN4_bm)
#define RGB_GREEN_CLR(void) (PORTC.OUTCLR = PIN4_bm)

enum RGBColors{
	RED,
	GREEN,
	BLUE,
	PURPLE,
	YELLOW,
	WHITE,
	ORANGE,
	OFF
	};

enum XMEGAID{
	DRIVE,
	ARM,
	RADIO,
	DEBUG_MODE
};  //Variable for internal ID use

//Define for the PWM cycle for an "ON" led. Used to adjust brightness
#define COLOR_ON 50

void RGBSetColor(RGBColors choice);
void initializeIO(void);  //Sets up all of the IO and associated settings
//void determineID(void);
void determineID(char * XmegaIDStr, XMEGAID & CurrentID);