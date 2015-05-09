/*
 * ComputerInterface.h
 *
 * Created: 3/5/2015 8:45:11 AM
 *  Author: Nick McComb
 */ 


#ifndef COMPUTERINTERFACE_H_
#define COMPUTERINTERFACE_H_

#include "XMegaLib.h"

void initPCInterface(XMEGAID InputCurrentID);
void sendDriveResponse(DRIVE_RESPONSE input);

//Packet lengths for the different packets
#define DRIVE_PACKET_LENGTH 8
#define DRIVE_RESPONSE_PACKET_LENGTH 10
#define ARM_PACKET_LENGTH 6
#define ARM_RESPONSE_PACKET_LENGTH 4

//Global variables used by the computer interface functions
char volatile packetIndex;    //Holds the current packet index that we are looking at
char recievedData[20];        //Holds the received data as its parsed
char volatile targetPacketLength;  //This is set based on GlobalCurrentID in initPCInterface()

//Debugging variables
int volatile invalidPacketCount;


#endif /* COMPUTERINTERFACE_H_ */