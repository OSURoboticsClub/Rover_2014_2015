/*
 * IncFile1.h
 *
 * Created: 1/15/2015 7:26:31 PM
 *  Author: OSURC2
 */ 


#ifndef BRAINBOARD_H_
#define BRAINBOARD_H_



void SetXMEGA32MhzCalibrated();       //Sets the clock speed to 32Mhz
void uart_init(void);                 //Initializes UART
void timer_init(void);                //Initializes the timers (for UART)
void SendStringPC(char *stufftosend); //Sends a string over the FTDI chip

//Main and init functions that are shared between individual implementation
//and the main interface file



void debugMain();  //Contains misc. debug code. No specific purpose

void ARM_INIT(void); //This is for......arm
void SendStringArm(char *present); //Sends arm usart string

void GIM_BAL_INIT(void); //initialization for the gimbal
void SendStringGim(char *present); //Sends usart char string for gimbal pins


enum XMegaStates{
	WaitForPing,
	WaitForReady,
	MainProgram
}; // CurrentState = WaitForPing;


#endif /* BRAINBOARD_H_ */