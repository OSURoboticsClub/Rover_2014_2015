/*
 * BasicBrainBoard.cpp
 *
 *  Microcontroller Control code for OSU Robotics Club's Mars Rover Team's Rover.
 *  This code and related documentation (including hardware design/files/doc) can 
 *  be found at https://github.com/OSURoboticsClub/Rover2015
 *  
 *  This board is designed to preform 3 main functions: Drive, Arm, and Radio.
 *
 *  Authors: Nick McComb, Brain Sia, Cameron Stuart
 */ 

#define F_CPU 32000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "BrainBoard.h"
#include "XMegaLib.h"
#include "Arm.h"


extern "C"{
	#include "usart_driver.h"
	#include "avr_compiler.h"
};

USART_data_t USART_PC_Data;

//Misc. ISRs
ISR(TCC1_OVF_vect){
	TCC1.INTFLAGS = TC1_OVFIF_bm;
}
ISR(TCC0_OVF_vect){
	TCC0.INTFLAGS = TC0_OVFIF_bm;
}
ISR(USARTC0_RXC_vect){
	USART_RXComplete(&USART_PC_Data);
}
ISR(USARTC0_DRE_vect){
	USART_DataRegEmpty(&USART_PC_Data);
}

char recieveChar;


/*
Description: Main function that initializes all of the general hardware. There are more
specific inits in each of the boards' main functions (to maintain interchangeability).

Author: Nick McComb

Pseudocode:
- Init
- State Machine:
	- WaitForPing (default):
		- Wait until I recieve a 'p' from the PC Comms
			- return "ID" string over PC Comms
			- Change State -> WaitForReady
	- WaitForReady:
		- Wait until recieve a 'r' from the PC Comms
			- Change State -> MainProgram
	- MainProgram:
		- Determine which board we are
			- Launch the 'main' function associated with the board
				-INFINITE LOOP

*/
int main(void)
{
	SetXMEGA32MhzCalibrated();
	
	//Main's Variable Declarations
	char XmegaIDStr[11];
	XMEGAID CurrentID;
		
	//Initialization Code
	uart_init();
	initializeIO();
	determineID(XmegaIDStr, CurrentID);
	timer_init();  //Initialize Timers
	sei(); //Enable interrupts
	
	PMIC.CTRL |= PMIC_LOLVLEN_bm; //draws current for ?

	
    while(1)
    { 
		
		switch (CurrentState){
			case WaitForPing:
				RGBSetColor(RED);
				if(USART_RXBufferData_Available(&USART_PC_Data)){
					recieveChar = USART_RXBuffer_GetByte(&USART_PC_Data);  //Read character off of buffer
					if (recieveChar == 'p'){
						SendStringPC(XmegaIDStr); //Identify itself
						CurrentState = WaitForReady;
					}
					//else, do nothing and wait for more chars
				}
				break;
			case WaitForReady:
				RGBSetColor(YELLOW);
				char recieveChar;
				if(USART_RXBufferData_Available(&USART_PC_Data)){
					recieveChar = USART_RXBuffer_GetByte(&USART_PC_Data);  //Read character off of buffer
					if (recieveChar == 'r'){
						CurrentState = MainProgram;
					}
					//else, do nothing and wait for more chars
				}
				break;
			case MainProgram:
				//RGBSetColor(GREEN);
				switch (CurrentID) {
					case DRIVE:
						while (1) {
							driveMain();
						}
						break;
					case ARM:
						while (1) {
							armMain();
						}
						break;
					case RADIO:
						while (1) {
							radioMain();
						}
						break;
					case DEBUG_MODE:
						while(1) {
							debugMain();
						}
					break;
				}
				break;
		}
		
		
    }
}

/*
Description: This function holds all of the code for the drive firmware for the BB
Author: Cameron Stuart

Pseudocode (algorithm):
- Makes the Rover Drive
- Expand this portion

Usage Notes:
This function exists inside a while(1) so it will loop itself forever

*/
void driveMain(){
	static char miscStr[10];
	RGBSetColor(BLUE);
	strcpy(miscStr,"Blue!\n\r");
	SendStringPC(miscStr);
	_delay_ms(500);
	RGBSetColor(RED);
	strcpy(miscStr,"Red!\n\r");
	SendStringPC(miscStr);
	_delay_ms(500);
}

/*
Description: This function holds all of the code for the arm firmware for the BB
Author: Brain Sia

Pseudocode (algorithm):
- Makes the Arm Arm
- Expand this portion

Usage Notes:
This function exists inside a while(1) so it will loop itself forever

*/
void armMain(){
	armGPIOInit();
	armInit();
	
	MD1_DIR_SET();
	MD1_STEP_CLR();
	
	while (1) {  //Main executing loop
		
		_delay_ms(10);
		
		MD1_STEP_SET();
		_delay_us(20);
		MD1_STEP_CLR();
		
	}
}

/*
Description: This function holds all of the code for the radio firmware for the BB
Author: TBD

Pseudocode (algorithm):
- Makes the Radio Function
- Expand this portion

Usage Notes:
This function exists inside a while(1) so it will loop itself forever

*/
void radioMain(){
	
}

/*
Description: General-Purpose debug function. No designated function, available 
for all who program the board.
*/
void debugMain(){
	
}


//Inits the UART for the board
void uart_init(void){
	PORTC.DIRSET = PIN3_bm;																			//Sets TX Pin as output
	PORTC.DIRCLR = PIN2_bm;																			//Sets RX pin as input
	
	USART_InterruptDriver_Initialize(&USART_PC_Data, &USARTC0, USART_DREINTLVL_LO_gc);				//Initialize USARTC0 as interrupt driven serial and clear it's buffers
	USART_Format_Set(USART_PC_Data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);	//Set the data format of 8 bits, no parity, 1 stop bit
	USART_RxdInterruptLevel_Set(USART_PC_Data.usart, USART_RXCINTLVL_LO_gc);						//Enable the receive interrupt
	USART_Baudrate_Set(&USARTC0, 207 , 0);															//Set baudrate to 9600 with 32Mhz system clock
	USART_Rx_Enable(USART_PC_Data.usart);															//Enable receiving over serial
	USART_Tx_Enable(USART_PC_Data.usart);															//Enable transmitting over serial
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
}

//Initializes timers
void timer_init(void){
	TCC0.PER = 100;	//period for PWM
	TCC0.CTRLA = TC_CLKSEL_DIV256_gc; //sets the PWM base frequency by 2000000/256
	TCC0.CTRLB = TC_WGMODE_SINGLESLOPE_gc; //sets the wave generation mode to single slope
	TCC0.CTRLB |= (0b00110000); //output pins on red and blue LED
	TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc; //interrupt register
	TCC0.CCB = 10; 
	TCC0.CCA = 50; 
	
	TCC1.PER = 100;
	TCC1.CTRLA = TC_CLKSEL_DIV256_gc;
	TCC1.CTRLB = TC_WGMODE_SINGLESLOPE_gc;
	TCC1.CTRLB |= (0b00010000); //output pins on green LED
	TCC1.INTCTRLA = TC_OVFINTLVL_LO_gc;
	TCC1.CCA = 50;
}

//Sends a string to the computer
void SendStringPC(char *stufftosend){
	for(int i = 0 ; stufftosend[i] != '\0' ; i++){
		while(!USART_IsTXDataRegisterEmpty(&USARTC0));
		USART_PutChar(&USARTC0, stufftosend[i]);
	}
}

//Configures the XMEGA to run on it's 32Mhz internal? oscillator
void SetXMEGA32MhzCalibrated(){
	CCP = CCP_IOREG_gc;						//Disable register security for oscillator update
	OSC.CTRL = OSC_RC32MEN_bm;				//Enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)); //Wait for oscillator to be ready
	CCP = CCP_IOREG_gc;						//Disable register security for clock update
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;		//Switch to 32MHz clock


	CCP = CCP_IOREG_gc;						//Disable register security for oscillator update
	OSC.CTRL |= OSC_RC32KEN_bm;				//Enable 32Khz oscillator
	while(!(OSC.STATUS & OSC_RC32KRDY_bm)); //Wait for oscillator to be ready
	/*
	OSC.DFLLCTRL &= ~OSC_RC32MCREF_bm;		//Set up calibration source to be 32Khz crystal
	DFLLRC32M.CTRL |= DFLL_ENABLE_bm;		//Enable calibration of 32Mhz oscillator 
	*/
}




/*

"Recycle Bin":  (old code that I want to keep for reference purposes)


//if(USART_RXBufferData_Available(&USART_PC_Data)){
//	recieveChar = USART_RXBuffer_GetByte(&USART_PC_Data);

#define PACKETSIZE 10
volatile char recieveBuffer[PACKETSIZE];
volatile char SendBuffer[100];


//enum { HEADER, COMMAND, BASEROTVAL1, BASEROTVAL2, ACT1VAL1, ACT1VAL2, ACT2VAL1, ACT2VAL2, CHECKSUM, TAIL};


*/