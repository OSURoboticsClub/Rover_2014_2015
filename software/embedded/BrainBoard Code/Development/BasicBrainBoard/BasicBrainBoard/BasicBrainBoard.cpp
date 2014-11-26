/*
 * BasicBrainBoard.cpp
 *
 * Created: 11/25/2014 6:03:12 PM
 *  Author: OSURC2
 */ 


#define F_CPU 2000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define STATUS1_SET(void) (PORTC.OUTSET = PIN5_bm)
#define STATUS1_CLR(void) (PORTC.OUTCLR = PIN5_bm)

#define STATUS2_SET(void) (PORTC.OUTSET = PIN6_bm)
#define STATUS2_CLR(void) (PORTC.OUTCLR = PIN6_bm)

#define ERROR_SET(void) (PORTC.OUTSET = PIN7_bm)
#define ERROR_CLR(void) (PORTC.OUTCLR = PIN7_bm)

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


void timer_init(void);
ISR(TCC1_OVF_vect){
	TCC1.INTFLAGS = TC1_OVFIF_bm;
}

ISR(TCC0_OVF_vect){
	TCC0.INTFLAGS = TC0_OVFIF_bm;
}

int main(void)
{
	//Initialization Code
	PORTC.DIRSET = (PIN5_bm | PIN6_bm | PIN7_bm); //Sets status/error led outputs
	PORTC.DIRSET = (PIN0_bm | PIN1_bm | PIN4_bm); //Set RGB Led outputs
	//PORTC.DIRCLR = PIN0_bm;
	timer_init();
	PMIC.CTRL |= PMIC_LOLVLEN_bm; //draws current for ?
    while(1)
    { 
		static long i = 0;
		i++;
		
		/***Setting color phase difference about a third***/
		TCC0.CCA = i % 75; 
		TCC0.CCB = (i + 33) % 75; 
		TCC1.CCA = (i + 66) % 75; 
		
		_delay_ms(25);
		/*
		for(int i = 0; i < 333; ++i){
			RGB_GREEN_SET();
			RGB_RED_CLR();
			_delay_ms(5);
			RGB_RED_SET();
			RGB_GREEN_CLR();
			_delay_ms(5);
		}
	*/
    }
}

void timer_init(void){
	TCC0.PER = 100;	//period for PWM
	TCC0.CTRLA = TC_CLKSEL_DIV256_gc; //sets the PWM base frequency by 2000000/256
	TCC0.CTRLB = TC_WGMODE_SINGLESLOPE_gc; //sets the wave generation mode to single slope
	TCC0.CTRLB |= (0b00110000); //
	TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc; //interrupt register
	TCC0.CCB = 10; 
	TCC0.CCA = 50;
	
	TCC1.PER = 100;
	TCC1.CTRLA = TC_CLKSEL_DIV256_gc;
	TCC1.CTRLB = TC_WGMODE_SINGLESLOPE_gc;
	TCC1.CTRLB |= (0b00010000);
	TCC1.INTCTRLA = TC_OVFINTLVL_LO_gc;
	TCC1.CCA = 50;
}
