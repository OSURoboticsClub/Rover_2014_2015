/*
 * XMegaMacros.h
 *
 * Author: Nick
 * 
 * This file contains functions that are general to the entire Xmega platform
 */ 

#ifndef XMEGALIB_H
#define XMEGALIB_H

#define F_CPU 32000000UL

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//These need to be included before anything that attempts to setup a USART
extern "C"{
	#include "usart_driver.h"
	#include "avr_compiler.h"
	#include <stdio.h>
};

#include "BrainBoard.h"
#include "SharedFunctions.h"

//Global Variables Catch (used to define global variables)
#ifdef XMEGALIB_GLOBALS
#define EXTERN
#else
#define EXTERN extern
#endif

//Global Variables
EXTERN USART_data_t USART_PC_Data;


//Error and Status LED outputs

#define STATUS1_SET(void) (PORTC.OUTSET = PIN5_bm)
#define STATUS1_CLR(void) (PORTC.OUTCLR = PIN5_bm)

//Check the DIP Switches on the BB
#define CHECK_DIP_SW_1(void) (!(PORTC.IN & PIN7_bm)) //Returns true if bit 1 of the DIP Switch is "ON"
#define CHECK_DIP_SW_2(void) (!(PORTC.IN & PIN6_bm)) //Returns true if bit 2 of the DIP Switch is "ON"

//RGB LED Control
//RGB LED Pin Descriptions (All on Port C)
//0 - Blue
//1 - Red
//4 - Green

#define RGB_BLUE_SET(void) (PORTC.OUTSET = PIN0_bm)
#define RGB_BLUE_CLR(void) (PORTC.OUTCLR = PIN0_bm)

#define RGB_RED_SET(void) (PORTC.OUTSET = PIN1_bm)
#define RGB_RED_CLR(void) (PORTC.OUTCLR = PIN1_bm)

#define RGB_GREEN_SET(void) (PORTC.OUTSET = PIN4_bm)
#define RGB_GREEN_CLR(void) (PORTC.OUTCLR = PIN4_bm)

enum RGBColors{
	RED,
	GREEN,
	BLUE,
	PURPLE,
	YELLOW,
	WHITE,
	ORANGE,
	OFF
	};

enum XMEGAID{
	DRIVE,
	ARM,
	RADIO,
	DEBUG_MODE
};

//Define for the PWM cycle for an "ON" led. Used to adjust brightness
#define COLOR_ON 50

void RGBSetColor(RGBColors choice);
void initializeIO(void);  //Sets up all of the IO and associated settings
//void determineID(void);

void determineID(char * XmegaIDStr, XMEGAID & CurrentID);

//Communication Variables
EXTERN char volatile freshData;           //Set high if new data was processed
EXTERN char volatile packetIndex;
EXTERN XMEGAID volatile GlobalCurrentID;
EXTERN char volatile targetPacketLength;  //This is set based on GlobalCurrentID in initPCInterface()
EXTERN bool volatile processPackets;     //Should we be waiting for computer packets?
EXTERN char recievedData[20];            //Holds the recieved data as its parsed

#define DRIVE_PACKET_LENGTH 8
#define DRIVE_RESPONSE_PACKET_LENGTH 10
#define ARM_PACKET_LENGTH 6
#define ARM_RESPONSE_PACKET_LENGTH 4

//DRIVE_PACKET_FROM_COMP
enum DRIVE_PACKET_FROM_COMP {
	DRIVE_HEAD = 0,
	LEFT_SPEED = 1,
	RIGHT_SPEED = 2,
	GIMBAL_PITCH = 3,
	GIMBAL_ROLL = 4, 
	GIMBAL_YAW = 5,
	DRIVE_CHECKSUM = 6,
	DRIVE_FOOTER = 7
	};

//Holds the infromation from the computer
struct DRIVE_DATA {
	char leftSpeed;
	char rightSpeed;
	char gimbalPitch;
	char gimbalRoll;
	char gimbalYaw;
	};

enum DRIVE_PACKET_TO_COMP {
	DRIVE_RESPONSE_HEAD, 
	IS_PAUSED_BYTE,
	LEFT_ABS_POSITION_B1,
	LEFT_ABS_POSITION_B2,
	LEFT_ABS_POSITION_B3,
	RIGHT_ABS_POSITION_B1,
	RIGHT_ABS_POSITION_B2,
	RIGHT_ABS_POSITION_B3,
	CHECKSUM,
	DRIVE_RESPONSE_FOOTER
	};
	
struct DRIVE_RESPONSE {
	char isPaused;
	long leftAbsPosition;
	long rightAbsPosition;
	};

//Need to place in a Union with Arm data
EXTERN volatile DRIVE_DATA driveData;

void FlushSerialBuffer(USART_data_t *UsartBuffer);
void initPCInterface(XMEGAID InputCurrentID);


#endif /* XMEGALIB_H */