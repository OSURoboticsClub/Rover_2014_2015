/*
 * Radio.cpp
 *
 * Created: 3/4/2015 1:07:41 AM
 *  Author: Nick McComb
 */ 

#include "Radio.h"

/*
Description: This function holds all of the code for the radio firmware for the BB
Author: Nick McComb

Pseudocode (algorithm):
- Makes the Radio Function
- Expand this portion

Usage Notes:
This function exists inside a while(1) so it will loop itself forever

*/
void radioMain(){
	
	//Test limit switch and stepper motor code
	
	while(!CHECK_RADIO_LIMIT_SW()){
		RADIO_STEPPER_STEP_SET();
		_delay_us(500);
		RADIO_STEPPER_STEP_CLR();
		_delay_us(1000);
		
		_delay_ms(1);  //Slow everything down delay
	}
	
	while(1);
}


/********** 

Pin definitions:

Drive Board Connector
Pause - PF5
Drive_RX - PF2
Drive_TX - PF3

SmartFinder Connector
SmartFinder_RX - PE3
SmartFinder_TX - PE2

Stepper Driver
!Fault - PB3
Step - PE4
Direction - PE7
Enable - PE5

Limit Switch 
PF6

**********/ 

void radioInit(){
	//Setup pause input
	PORTF.DIRCLR = (PIN5_bm);
	
	//Setup Stepper Driver
	PORTB.DIRCLR = (PIN3_bm);  //nFault
	PORTE.DIRSET = (PIN4_bm);  //Step
	PORTE.DIRSET = (PIN7_bm);  //Direction
	PORTE.DIRSET = (PIN5_bm);  //Enable
	
	//Setup Limit Switch
	PORTF.DIRCLR = (PIN6_bm);
	
	//Potato!
}