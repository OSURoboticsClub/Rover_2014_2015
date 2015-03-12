/*
 * ComputerInterface.h
 *
 * Created: 3/5/2015 8:45:11 AM
 *  Author: Nick McComb
 */ 


#ifndef COMPUTERINTERFACE_H_
#define COMPUTERINTERFACE_H_

#include "XMegaLib.h"

//Settings
#define DRIVE_PACKET_SIZE 10
#define ARM_PACKET_SIZE 10
#define RADIO_PACKET_SIZE 10

//Global Variables
volatile unsigned char bufferIndex;            //Used for indexing the receive buffer
volatile char recieveBuffer[MAX_PACKET_SIZE];  //Used for recieving from the computer
volatile char sendBuffer[100];                 //Used for sending to the computer

//Function prototypes


#endif /* COMPUTERINTERFACE_H_ */