/*
 * XMegaLib.cpp
 *
 * Created: 12/3/2014 5:46:14 PM
 *  Author: OSURC2
 */ 

#include "XMegaLib.h"
#include <avr/io.h>

//This function handles making colors on the RGB LED
//Author: Nick M
void RGBSetColor(RGBColors choice){
	switch(choice){
		case RED:
			TCC0.CCA = 0;
			TCC0.CCB = COLOR_ON;
			TCC1.CCA = 0;
			break;
		case BLUE:
			TCC0.CCA = COLOR_ON;
			TCC0.CCB = 0;
			TCC1.CCA = 0;
			break;
		case GREEN:
			TCC0.CCA = 0;
			TCC0.CCB = 0;
			TCC1.CCA = COLOR_ON;
			break;
		case PURPLE:
			TCC0.CCA = COLOR_ON;
			TCC0.CCB = COLOR_ON;
			TCC1.CCA = 0;
			break;
		case YELLOW:
			TCC0.CCA = 0;
			TCC0.CCB = COLOR_ON;
			TCC1.CCA = COLOR_ON;
			break;
		case WHITE:
			TCC0.CCA = COLOR_ON;
			TCC0.CCB = COLOR_ON;
			TCC1.CCA = COLOR_ON;
			break;
		case ORANGE:
			TCC0.CCB = COLOR_ON;      //Red
			TCC1.CCA = COLOR_ON / 2;  //Green
			TCC0.CCA = 0;             //Blue
			break;
		case OFF:
		default:
			TCC0.CCB = 0;  //Red
			TCC1.CCA = 0;  //Green
			TCC0.CCA = 0;  //Blue
			break;
	}
}


//Initializes all I/O for the board
//Sets up DIR, and PULLUP/PULLDOWN Resistors, etc.
void initializeIO(){
	PORTC.DIRSET = (PIN5_bm); //Sets output LED (status/error)
	PORTC.DIRSET = (PIN0_bm | PIN1_bm | PIN4_bm); //Set RGB Led outputs
	
	PORTC.DIRCLR = (PIN6_bm | PIN7_bm); //Sets DIP Switch Input
	PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTC.PIN7CTRL = PORT_OPC_PULLUP_gc;
}


//This function handles determining the ID of the board, and putting
//the identification string in its variable
//Author: Nick M
void determineID(char * XmegaIDStr, XMEGAID & CurrentID){
	if      (!CHECK_DIP_SW_1() && !CHECK_DIP_SW_2()){
		CurrentID = DRIVE;
		strcpy(XmegaIDStr, "DRIVE");
	}
	else if (!CHECK_DIP_SW_1() && CHECK_DIP_SW_2()) {
		CurrentID = ARM;
		strcpy(XmegaIDStr, "ARM");
	}
	else if (CHECK_DIP_SW_1() && !CHECK_DIP_SW_2()){
		CurrentID = RADIO;
		strcpy(XmegaIDStr, "RADIO");
	}
	else if (CHECK_DIP_SW_1() && CHECK_DIP_SW_2()){
		CurrentID = DEBUG_MODE;
		strcpy(XmegaIDStr, "DEBUG_MODE");
	} 
	
}


