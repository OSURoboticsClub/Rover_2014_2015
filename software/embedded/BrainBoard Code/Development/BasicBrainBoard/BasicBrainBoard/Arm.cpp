/*
 * Arm.cpp
 *
 * Created: 1/23/2015 7:20:37 PM
 *  Author: Brian Sia, Nick Ames
 */ 

#include "Arm.h"
//#include "XMegaLib.h"
//#include <avr/io.h>



/*
Description: This function holds all of the code for the arm firmware for the BB
Author: Brain Sia

Pseudocode (algorithm):
- Makes the Arm Arm
- Expand this portion

Usage Notes:
This function exists inside a while(1) so it will loop itself forever

*/
void armMain(){
	MD1_DIR_SET();
	MD1_STEP_CLR();
	
	while (1) {  //Main executing loop
		
		_delay_ms(10);
		
		MD1_STEP_SET();
		_delay_us(20);
		MD1_STEP_CLR();
		
	}
}

void armGPIOInit(){
	//Stepper Driver 1
	PORTE.DIRSET = (PIN4_bm); //Step Pin
	PORTE.DIRSET = (PIN7_bm); //Dir Pin
	PORTE.DIRSET = (PIN5_bm); //nEN Pin
}


void armInit(){
	armGPIOInit();
	MD1_nEN_CLR();
}