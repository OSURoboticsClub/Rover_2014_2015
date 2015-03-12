/*
 * ComputerInterface.cpp
 *
 * Created: 3/5/2015 8:45:32 AM
 *  Author: Nick McComb
 */ 

#include "ComputerInterface.h"

//Interrupt when anything is received
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


