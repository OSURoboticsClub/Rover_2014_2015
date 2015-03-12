/*
 * ComputerInterface.cpp
 *
 * Created: 3/5/2015 8:45:32 AM
 *  Author: Nick McComb
 */ 

#include "ComputerInterface.h"

//Interrupt when anything is recieved
ISR(USARTC0_RXC_vect){
	USART_RXComplete(&USART_PC_Data);
	
	if(processPackets){ //We should process packets, otherwise do nothing
		if(USART_RXBufferData_Available(&USART_PC_Data)){  //If data is available
			//Put byte into buffer, increment buffer index
			recieveBuffer[bufferIndex++] = USART_RXBuffer_GetByte(&USART_PC_Data);
		}
	}
	
	if(bufferIndex == currentPacketSize){
		FlushSerialBuffer(&USART_PC_Data);  //Clear the buffer incase something else has piled up
		
		//Process packet here
	}
}


void initializePacketProcessing(void){
	//Setup the packet size to test for
	switch(CurrentID){
		case DRIVE:
			currentPacketSize = DRIVE_PACKET_SIZE;
			break;
		case ARM:
			currentPacketSize = ARM_PACKET_SIZE;
			break;	
		case RADIO:    
			currentPacketSize = RADIO_PACKET_SIZE;
			break;
		default:  //To follow standard
			break;
	}
	
	
}


/*** Begin PC Interface Objects ***/


/** Begin Arm Interface Object **/

/* Public Functions */

//Constructor
armInfoObj::armInfoObj(void){
	resetData();   //Init the data
}

void armInfoObj::resetData(){
	newInformation = false;
	gripSuccessStatus = NO_ATTEMPT; //Init the value as no attempt
	actionsCompleteStatus = 0;      //No data to send
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

/* End Private Functions */