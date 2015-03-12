/*
 * XMegaLib.cpp
 *
 * Created: 12/3/2014 5:46:14 PM
 *  Author: OSURC2
 */ 

#ifndef XMEGALIB_H_
#define XMEGALIB_H_

#include "XMegaLib.h"
#include <avr/io.h>

//This function handles making colors on the RGB LED
//Author: Nick McComb
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

//Flushes the serial buffer that is passed into it
void FlushSerialBuffer(USART_data_t *UsartBuffer){
	while(USART_RXBufferData_Available(UsartBuffer)){
		USART_RXBuffer_GetByte(UsartBuffer);
	}
}



/** Begin Arm Interface Object **/

/* Public Functions */

//Constructor
armInfoObj::armInfoObj(void){
	resetData();   //Init the data
}

//Checks if the data is 'fresh'
//This function will return false if data has been read out of the 
	//object
bool armInfoObj::checkIfNewDataAvailable(){
	return newInformation;
}

//Returns the xAxisValue
uint8_t armInfoObj::getXAxisValue(void){
	readData();
	
	return xAxisValue;
}

uint8_t armInfoObj::getYAxisValue(void){
	readData();
	
	return yAxisValue;
}

//Should never return error, :D
ZAXISMODE armInfoObj::getZAxisMode(void) {
	readData();
	
	return zAxisMode;
	/*
	//Switches based on the number stored in the variable
	switch (ZAXISMODE) {
		case 0:
			return NEUTRAL;
			break;
		case 1:
			return MODE1;
			break;
		case 2:
			return MODE2;
			break;
		case 3:
			return MODE3;
			break;
		default:
			return ERROR;
	}
	*/
}

void armInfoObj::setGripSuccess(bool status){
	//TODO: Add flag to this
	if(status){
		gripSuccessStatus = SUCCESS;
	}
	else {
		gripSuccessStatus = FAIL;
	}
}

void armInfoObj::setActionsComplete(bool status){
	//TODO: Add flag to this 
	//TODO: Init this value as 0 after a packet was received
	if(status == true)
		actionsCompleteStatus = 1;
	else if (status == false)
		actionsCompleteStatus = 2;
		
	sendPacket(); //Sends the packet to the computer
}

/* End 'public' functions */

/* Begin serial interface functions */

volatile void armInfoObj::setXYAxes(uint8_t xAxisInput, uint8_t yAxisInput) {
	setData();
	xAxisValue = xAxisInput;
	yAxisValue = yAxisInput;
}


//Takes in an uint8_t, parses it into the xAxisMode enum
	// 0 - NEUTRAL - Neutral Position (Retracted)
	// 1 - MODE1   - Position 1  {To be defined}
	// 2 - MODE2   - Position 2  {To be defined}
	// 3 - MODE3   - Position 3  {To be defined}
	// 4 - ERROR   - Error state {To be defined}
volatile void armInfoObj::setZMode(uint8_t modeInput) {
	setData();
	
	switch (modeInput) {
		case 0:
			zAxisMode = NEUTRAL;
			break;
		case 1:
			zAxisMode = MODE1;
			break;
		case 2:
			zAxisMode = MODE2;
			break;
		case 3:
			zAxisMode = MODE3;
			break;
		default:
			zAxisMode = ERROR;
			break;
	}
	
}

// true  - init bit is set
// false - init bit is not set
volatile void armInfoObj::setInitMode(bool init){
	setData();
	if(init)
		initRobot = 1;
	else 
		initRobot = 0;
}

/* Private Functions */

//Flag that data has been read and there is no longer new data

inline void armInfoObj::readData(){
	newInformation = false;
}

inline void armInfoObj::setData(){
	newInformation = true;
}

//Resets information that should be in a neutral state for every new packet
void armInfoObj::resetData(){
	newInformation = false;
	gripSuccessStatus = NO_ATTEMPT; //Init the value as no attempt
	actionsCompleteStatus = 0;      //No data to send
}

//Function that handles actually sending the packet information
void armInfoObj::sendPacket(){
	//Handles sending the packet to the computer
}
/* End Private Functions */



#endif