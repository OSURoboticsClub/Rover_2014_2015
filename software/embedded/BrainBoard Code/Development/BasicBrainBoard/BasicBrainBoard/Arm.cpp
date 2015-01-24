/*
 * Arm.cpp
 *
 * Created: 1/23/2015 7:20:37 PM
 *  Author: nrpic_000
 */ 

#include "Arm.h"
#include <avr/io.h>

void armGPIOInit(){
	//Stepper Driver 1
	
	PORTE.DIRSET = (PIN4_bm); //Step Pin
	PORTE.DIRSET = (PIN7_bm); //Dir Pin
	PORTE.DIRSET = (PIN5_bm); //nEN Pin
	
	
}


void armInit(){
	MD1_nEN_CLR();
}