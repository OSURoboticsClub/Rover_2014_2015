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

void Saber_init_uno(void);		//Drive stuff start
void Saber_init_dos(void);
void Saber_init_tres(void);

void SendStringSABER_UNO(char *present);
void SendStringSABER_DOS(char *present);
void SendStringSaber_TRES(char *present);
//Drive stuff end

void ARM_INIT(void); //This is for......arm
void SendStringArm(char *present); //Sends arm usart string

void GIM_BAL_INIT(void); //initialization for the gimbal
void SendStringGim(char *present); //Sends usart char string for gimbal pins


enum XMegaStates{
	WaitForPing,
	WaitForReady,
	MainProgram
} CurrentState = WaitForPing;


#endif /* BRAINBOARD_H_ */