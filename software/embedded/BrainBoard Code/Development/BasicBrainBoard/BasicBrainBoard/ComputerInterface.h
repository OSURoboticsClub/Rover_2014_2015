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
#define ARM_PACKET_LENGTH 8
#define ARM_RESPONSE_PACKET_LENGTH 4

//Global variables used by the computer interface functions
char volatile packetIndex;    //Holds the current packet index that we are looking at
char recievedData[20];        //Holds the received data as its parsed
char volatile targetPacketLength;  //This is set based on GlobalCurrentID in initPCInterface()

//Debugging variables
int volatile invalidPacketCount;

//Enum declarations

//Drive packet from the computer
enum DRIVE_PACKET_FROM_COMP {
	DRIVE_HEAD,
	LEFT_SPEED,
	RIGHT_SPEED,
	GIMBAL_PITCH,
	GIMBAL_ROLL,
	GIMBAL_YAW,
	DRIVE_CHECKSUM,
	DRIVE_FOOTER
};

//Drive packet to the computer
enum DRIVE_PACKET_TO_COMP {
	DRIVE_RESPONSE_HEAD,
	IS_PAUSED_BYTE,
	LEFT_ABS_POSITION_B1,
	LEFT_ABS_POSITION_B2,
	LEFT_ABS_POSITION_B3,
	RIGHT_ABS_POSITION_B1,
	RIGHT_ABS_POSITION_B2,
	RIGHT_ABS_POSITION_B3,
	DRIVE_RESPONSE_CHECKSUM,
	DRIVE_RESPONSE_FOOTER
};

//Arm packet from the computer
enum ARM_PACKET_FROM_COMP {
	ARM_HEAD,
	ARM_COMMAND,
	X_AXIS_VALUE,
	Y_AXIS_VALUE,
	Z_AXIS_VALUE,
	GRIPPER_ROTATION,
	ARM_CHECKSUM,
	ARM_FOOTER
};

//Arm packet to the computer
enum ARM_PACKET_TO_COMP {
	ARM_TO_COMP_HEADER,
	ARM_TO_COMP_COMMAND,
	ARM_TO_COMP_CHECKSUM,
	ARM_TO_COMP_FOOTER
};



#endif /* COMPUTERINTERFACE_H_ */