/*
 * XMegaMacros.h
 *
 * Author: Nick
 */ 



//Error and Status LED outputs

#define STATUS1_SET(void) (PORTC.OUTSET = PIN5_bm)
#define STATUS1_CLR(void) (PORTC.OUTCLR = PIN5_bm)

#define STATUS2_SET(void) (PORTC.OUTSET = PIN6_bm)
#define STATUS2_CLR(void) (PORTC.OUTCLR = PIN6_bm)

#define ERROR_SET(void) (PORTC.OUTSET = PIN7_bm)
#define ERROR_CLR(void) (PORTC.OUTCLR = PIN7_bm)

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
	ORANGE
	};


//Define for the PWM cycle for an "ON" led. Used to adjust brightness
#define COLOR_ON 50

void RGBSetColor(RGBColors choice);