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
//Drive start
USART_data_t SABER_UNO;
USART_data_t SABER_DOS;
USART_data_t SABER_TRES;
//Drive end
//Arm start
USART_data_t ARM_USART;
//Arm end
//Gimbal start
USART_data_t GIMBAL_USART;
//Gimbal end

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

//Drive start
ISR(USARTE0_RXC_vect){
	USART_RXComplete(&SABER_UNO);
}
ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&SABER_UNO);
}

ISR(USARTE1_RXC_vect){
	USART_RXComplete(&SABER_DOS);
}
ISR(USARTE1_DRE_vect){
	USART_DataRegEmpty(&SABER_DOS);
}

ISR(USARTF0_RXC_vect){
	USART_RXComplete(&SABER_TRES);
}
ISR(USARTF0_DRE_vect){
	USART_DataRegEmpty(&SABER_TRES);
}
//Saber end

//Arm start
ISR(USARTD1_RXC_vect){
	USART_RXComplete(&ARM_USART);
}
ISR(USARTD1_DRE_vect){
	USART_DataRegEmpty(&ARM_USART);
}
//Arm end

//Gimbal start
ISR(USARTD0_RXC_vect){
	USART_RXComplete(&GIMBAL_USART);
}
ISR(USARTD0_DRE_vect){
	USART_DataRegEmpty(&GIMBAL_USART);
}
//Gimbal end

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
				RGBSetColor(GREEN);
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
//DO NOT Connect to motor at this point without figuring out units and encoder, see comment below
void driveMain(){
	int check = 0;
	Saber_init_uno();
	char cmmd[5] = {'D', ',' , 's'};
	SendStringSABER_UNO("D,start\n");
	char speed[5] = {'0', '0', '0', '\n', '\0'};
	int i = 0;
	int rem = check;
	while(1){
	//SendStringSABER_UNO("D,units 180 degrees = ") //Incomplete need to know more info on encoder to set units
	}
	/*while(1){
		i = 0;
		check = 0;
		rem = check;
		for(check = 0; check < 200; check++){
			rem = check;
			while(check){//converting int into the array
				speed[i++] = check % 10; //last digit
				check /= 10; //move number to the right
				if(i == 3)
					break;
			}
			check = rem;
			i = 0;
			//SendStringSABER_UNO(cmmd);
			SendStringSABER_UNO(speed);		
		//	_delay_ms(500);
		}
		_delay_ms(500);
		for(check; check < 0; check--){
			rem = check;
			while(check){//converting int into the array
				speed[i++] = check % 10; //last digit
				check /= 10; //move number to the right
				if(i == 3)
				break;
			}
			check = rem;
			i = 0;
			//SendStringSABER_UNO(cmmd);
			SendStringSABER_UNO(speed);
			//_delay_ms(500);
		}
	}*/
	
while(1){
	SendStringSABER_UNO("D,s010\n");
	SendStringPC("Sent to saber\n");
}

}

//Drive saber send functions start
void SendStringSABER_UNO(char *present){
	for(int i = 0 ; present[i] != '\0' ; i++){
		while(!USART_IsTXDataRegisterEmpty(&USARTE0));
		USART_PutChar(&USARTE0, present[i]);
	}
}

void SendStringSABER_DOS(char *present){
	for(int i = 0 ; present[i] != '\0' ; i++){
		while(!USART_IsTXDataRegisterEmpty(&USARTE1));
		USART_PutChar(&USARTE1, present[i]);
	}
}
void SendStringSABER_TRES(char *present){
	for(int i = 0 ; present[i] != '\0' ; i++){
		while(!USART_IsTXDataRegisterEmpty(&USARTF0));
		USART_PutChar(&USARTF0, present[i]);
	}
}//Drive saber send functions end

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

//Start arm usart sending functions
void SendStringArm(char *present){
	for(int i = 0 ; present[i] != '\0' ; i++){
		while(!USART_IsTXDataRegisterEmpty(&USARTD1));
		USART_PutChar(&USARTD1, present[i]);
	}
}//end arm string send, pins set for USARTD1

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
//Gimbal send string function, its all by itself with its init so far
void SendStringGim(char *present){
	for(int i = 0 ; present[i] != '\0' ; i++){
		while(!USART_IsTXDataRegisterEmpty(&USARTD0));
		USART_PutChar(&USARTD0, present[i]);
	}
} //End gimbal send string functions, USARTD0

//DRIVE INIT START
//May want to check init dos y tres to make sure that they were correctly altered from uno for their respective pins
void Saber_init_uno(){	//USARTE0
	PORTE.DIRSET = PIN3_bm;																			//Sets TX Pin as output
	PORTE.DIRCLR = PIN2_bm;																			//Sets RX pin as input
	
	USART_InterruptDriver_Initialize(&SABER_UNO, &USARTE0, USART_DREINTLVL_LO_gc);				//Initialize USARTE0 as interrupt driven serial and clear it's buffers
	USART_Format_Set(SABER_UNO.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);	//Set the data format of 8 bits, no parity, 1 stop bit
	USART_RxdInterruptLevel_Set(SABER_UNO.usart, USART_RXCINTLVL_LO_gc);						//Enable the receive interrupt
	USART_Baudrate_Set(&USARTE0, 207 , 0);															//Set baudrate to 9600 with 32Mhz system clock
	USART_Rx_Enable(SABER_UNO.usart);															//Enable receiving over serial
	USART_Tx_Enable(SABER_UNO.usart);															//Enable transmitting over serial
	
	
	
}
void Saber_init_dos(){ //USARTE1
	PORTE.DIRSET = PIN7_bm;																			//Sets TX Pin as output
	PORTE.DIRCLR = PIN6_bm;																			//Sets RX pin as input
	
	USART_InterruptDriver_Initialize(&SABER_DOS, &USARTE1, USART_DREINTLVL_LO_gc);				//Initialize USARTE1 as interrupt driven serial and clear it's buffers
	USART_Format_Set(SABER_DOS.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);	//Set the data format of 8 bits, no parity, 1 stop bit
	USART_RxdInterruptLevel_Set(SABER_DOS.usart, USART_RXCINTLVL_LO_gc);						//Enable the receive interrupt
	USART_Baudrate_Set(&USARTE1, 207 , 0);															//Set baudrate to 9600 with 32Mhz system clock
	USART_Rx_Enable(SABER_DOS.usart);															//Enable receiving over serial
	USART_Tx_Enable(SABER_DOS.usart);
}

void Saber_init_tres(){ //USARTF0
	PORTE.DIRSET = PIN3_bm;																			//Sets TX Pin as output
	PORTE.DIRCLR = PIN2_bm;																			//Sets RX pin as input
	
	USART_InterruptDriver_Initialize(&SABER_TRES, &USARTF0, USART_DREINTLVL_LO_gc);				//Initialize USARTF0 as interrupt driven serial and clear it's buffers
	USART_Format_Set(SABER_TRES.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);	//Set the data format of 8 bits, no parity, 1 stop bit
	USART_RxdInterruptLevel_Set(SABER_TRES.usart, USART_RXCINTLVL_LO_gc);						//Enable the receive interrupt
	USART_Baudrate_Set(&USARTF0, 207 , 0);															//Set baudrate to 9600 with 32Mhz system clock
	USART_Rx_Enable(SABER_TRES.usart);															//Enable receiving over serial
	USART_Tx_Enable(SABER_TRES.usart);
} //End drive inits

void ARM_INIT(){ //USARTD1
	PORTD.DIRSET = PIN7_bm;																			//Sets TX Pin as output
	PORTD.DIRCLR = PIN6_bm;																			//Sets RX pin as input
	
	USART_InterruptDriver_Initialize(&ARM_USART, &USARTD1, USART_DREINTLVL_LO_gc);				//Initialize USARTD1 as interrupt driven serial and clear it's buffers
	USART_Format_Set(ARM_USART.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);	//Set the data format of 8 bits, no parity, 1 stop bit
	USART_RxdInterruptLevel_Set(ARM_USART.usart, USART_RXCINTLVL_LO_gc);						//Enable the receive interrupt
	USART_Baudrate_Set(&USARTD1, 207 , 0);															//Set baudrate to 9600 with 32Mhz system clock
	USART_Rx_Enable(ARM_USART.usart);															//Enable receiving over serial
	USART_Tx_Enable(ARM_USART.usart);
} //End of arm init, may want to double check everything for correct pins and what not

void GIM_BAL_INIT(){//USARTD0
	PORTD.DIRSET = PIN3_bm;																			//Sets TX Pin as output
	PORTD.DIRCLR = PIN2_bm;																			//Sets RX pin as input
	
	USART_InterruptDriver_Initialize(&GIMBAL_USART, &USARTD0, USART_DREINTLVL_LO_gc);				//Initialize USARTD0 as interrupt driven serial and clear it's buffers
	USART_Format_Set(GIMBAL_USART.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);	//Set the data format of 8 bits, no parity, 1 stop bit
	USART_RxdInterruptLevel_Set(GIMBAL_USART.usart, USART_RXCINTLVL_LO_gc);						//Enable the receive interrupt
	USART_Baudrate_Set(&USARTD0, 207 , 0);															//Set baudrate to 9600 with 32Mhz system clock
	USART_Rx_Enable(GIMBAL_USART.usart);															//Enable receiving over serial
	USART_Tx_Enable(GIMBAL_USART.usart);
}//end of gimbal usart init, may want to double check as well


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