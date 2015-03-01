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

void driveMain();  //Contains the drive code
void armMain();    //Contains the arm code
void radioMain();  //Contains the radio code
void debugMain();  //Contains misc. debug code. No specific purpose

void Saber_init(void);
void SendStringSABER(char *present);


enum XMegaStates{
	WaitForPing,
	WaitForReady,
	MainProgram
} CurrentState = WaitForPing;


#endif /* BRAINBOARD_H_ */