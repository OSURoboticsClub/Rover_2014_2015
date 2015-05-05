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
	
	if(processPackets){
		
		recievedData[packetIndex] = USART_RXBuffer_GetByte(&USART_PC_Data);  //Read character off of buffer
		
		++packetIndex;
		
		if(packetIndex == targetPacketLength){
			packetIndex = 0;   //Reset the packet index
			freshData = 1;     //There is new data to process
			
			//Process packets
			RGBSetColor(BLUE); //Read packet color
			switch(GlobalCurrentID){
				case DRIVE: //Parse drive packet
					//LoL, ignore checksum
					if(recievedData[DRIVE_HEAD] == 255 && recievedData[DRIVE_FOOTER] == 255){ //basic verification, TODO: Add checksum verification
						driveData.leftSpeed = recievedData[LEFT_SPEED];
						driveData.rightSpeed = recievedData[RIGHT_SPEED];
						driveData.gimbalPitch = recievedData[GIMBAL_PITCH];
						driveData.gimbalRoll = recievedData[GIMBAL_ROLL];
						driveData.gimbalYaw = recievedData[GIMBAL_YAW];
					}
					break;
				case ARM:  //Parse arm packet
					
					break;
			}
		}
	}
	
	
}

//Workaround to make current ID Global
void initPCInterface(XMEGAID InputCurrentID){	
	GlobalCurrentID = InputCurrentID;
	
	if (InputCurrentID == DRIVE) {
		targetPacketLength = DRIVE_PACKET_LENGTH;
	}
	else if (InputCurrentID == ARM) {
		targetPacketLength = ARM_PACKET_LENGTH;
	}
	
	freshData = 0;
	packetIndex = 0;
	processPackets = false;
}