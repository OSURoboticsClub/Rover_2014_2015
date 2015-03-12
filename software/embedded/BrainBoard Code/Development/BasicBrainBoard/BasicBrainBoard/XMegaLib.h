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
};

#include "BrainBoard.h"
#include "SharedFunctions.h"

/* Rover Settings */
#define MAX_PACKET_SIZE 15 //Guessing here?
#define SEND_BUFFER_SIZE 100

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

enum ZAXISMODE {
	NEUTRAL,
	MODE1,
	MODE2, 
	MODE3,
	ERROR
};

enum GRIPSTATUS {
	NO_ATTEMPT,
	SUCCESS,
	FAIL
};

/* Classes */
//PC Interface Objects

class armInfoObj {
	public:
		armInfoObj();  //Constructor
		//Arm Interface Functions (to be used in Arm.cpp)
			/* Reading Functions */
			bool checkIfNewDataAvailable();  //Returns true if there is 'fresh' data (false once anything has been read)
			uint8_t getXAxisValue(void);     //Returns the desired X-Axis value
			uint8_t getYAxisValue(void);     //Returns the desired Y-Axis value
			ZAXISMODE getZAxisMode(void);    //Returns an enum with the current mode configuration
			bool needToInit(void);           //Returns a true if the 
			/* Writing Functions */
			void setGripSuccess(bool status);      //Pass true if successful grip, false otherwise (only call if grip is requested)
			void setActionsComplete(bool status);  //Pass true if done, false if error {According action To Be Defined}

			
			
		//ComputerInterface Interface  (not to be used in Arm.cpp)
			volatile void setXYAxes(uint8_t xAxisInput, uint8_t yAxisInput);
			volatile void setZMode(uint8_t modeInput);
			volatile void setInitMode(bool init);		
		
	private:
		//General Functions
		inline void readData(void);		//Called whenever any information-reading members are called
		inline void setData(void); 		//Called whenever any information-setting members are called
		inline void resetData(void);    //Resets all of the information into a neutral state
		void sendPacket(void);  //Call when ready to send packet (TBImpleneted)
		
		//General Class Information Variables
		bool newInformation;		    //Contains whether the information is 'fresh'
										  //This is false whenever information has been
										  //read (eg, 'stale')
		//Data recieved from the computer
		volatile uint8_t xAxisValue;  //Value of the X-Axis [3mm increments]
		volatile uint8_t yAxisValue;  //Value of the Y-Axis [3mm increments]
		ZAXISMODE zAxisMode;  //Mode of the Z-Axis
										// NEUTRAL - Neutral Position (Retracted)
										// MODE1   - Position 1  {To be defined}
										// MODE2   - Position 2  {To be defined}
										// MODE3   - Position 3  {To be defined}
										// ERROR   - Error state {To be defined}
		volatile uint8_t initRobot;   //Run init / calibrate routine?
										// 0 - Don't init
										// 1 - Init now
		
		//Data sent to the computer
		GRIPSTATUS gripSuccessStatus;    //Holds value of whether the grip was successful / attempted 
											// NO_ATTEMPT - no grip was attempted
											// SUCCESS - grip was successful
											// FAIL - grip failed
		uint8_t actionsCompleteStatus;  //Holds value of whether requested actions were completed
											// 0 - No data to send
											// 1 - Actions Complete
											// 2 - Error
	
};

class driveData {
	uint8_t leftSpeed;  //Base value of left speed [decimeter / second]
	uint8_t rightSpeed; //Base value of right speed [decimeter / second]
	
	//More to be added (Gimbal, etc)
	
};


//Define for the PWM cycle for an "ON" led. Used to adjust brightness
#define COLOR_ON 50

//Function Prototypes
void RGBSetColor(RGBColors choice);
void initializeIO(void);  //Sets up all of the IO and associated settings
//void determineID(void);
void determineID(char * XmegaIDStr, XMEGAID & CurrentID);
void FlushSerialBuffer(USART_data_t *UsartBuffer);
//might not be used
void initializePacketProcessing(void);  //Sets up the packet processing


//Global Variables Catch (used to define global variables)
#ifdef XMEGALIB_GLOBALS
#define EXTERN
#else
#define EXTERN extern
#endif

//Global Variables (related to operation of the Rover, not the XMEGA)
//They are placed here because they are required by more than one of the files
EXTERN USART_data_t USART_PC_Data;
EXTERN volatile bool processPackets;
EXTERN XMEGAID CurrentID;
EXTERN char currentPacketSize;

#endif /* XMEGALIB_H */