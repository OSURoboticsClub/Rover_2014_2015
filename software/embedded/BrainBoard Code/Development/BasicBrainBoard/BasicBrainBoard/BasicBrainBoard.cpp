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

//#include "BrainBoard.h"
#define XMEGALIB_GLOBALS //Used to declare program-wide globals
#include "XMegaLib.h"  //Needs to be included pre-USART

//#include "Arm.h"
//#include "Drive.h"

//Misc. ISRs
ISR(TCC1_OVF_vect){
	TCC1.INTFLAGS = TC1_OVFIF_bm;
}
ISR(TCC0_OVF_vect){
	TCC0.INTFLAGS = TC0_OVFIF_bm;
}

ISR(USARTC0_DRE_vect){
	USART_DataRegEmpty(&USART_PC_Data);
}

char recieveChar;
XMegaStates CurrentState = WaitForPing;

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
			- Call the 'init' function associated with the board once
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
	initPCInterface(CurrentID);
	timer_init();  //Initialize Timers
	sei(); //Enable interrupts
	
	/*  //Legacy from first packetized attempt
	initializePacketProcessing(); //Needs to be ran _after_ determineID()
	processPackets = false;       //Not ready to receive packets just yet
	*/	

	//TODO: Is the following line needed?
	//PMIC.CTRL |= PMIC_LOLVLEN_bm;   //draws current for ?

	//Uncomment the line below if you want to bypass poking
	//CurrentState = MainProgram;


	//We need to init drive early so that the roving light will blink
	if(CurrentID == DRIVE){
		driveInit();
	}

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
					else if (recieveChar == 'p'){  //Hack to make device always respond to poke
						SendStringPC(XmegaIDStr); //Identify itself
						//CurrentState = WaitForReady;
					}
					
					//else, do nothing and wait for more chars
				}
				break;
			case MainProgram:
				RGBSetColor(GREEN);
				processPackets = true;
				switch (CurrentID) {
					case DRIVE:
						//driveInit();  //Moved to before the state machine
						while (1) {
							driveMain();
						}
						break;
					case ARM:
						armInit();
						while (1) {
							armMain();
						}
						break;
					case RADIO:
						radioInit();
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




//???????????????

//Start arm usart sending functions
void SendStringArm(char *present){
	for(int i = 0 ; present[i] != '\0' ; i++){
		while(!USART_IsTXDataRegisterEmpty(&USARTD1));
		USART_PutChar(&USARTD1, present[i]);
	}
}//end arm string send, pins set for USARTD1



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
	/*
	CCP = CCP_IOREG_gc;						//Disable register security for oscillator update
	OSC.CTRL = OSC_RC32MEN_bm;				//Enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)); //Wait for oscillator to be ready
	CCP = CCP_IOREG_gc;						//Disable register security for clock update
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;		//Switch to 32MHz clock


	CCP = CCP_IOREG_gc;						//Disable register security for oscillator update
	OSC.CTRL |= OSC_RC32KEN_bm;				//Enable 32Khz oscillator
	while(!(OSC.STATUS & OSC_RC32KRDY_bm)); //Wait for oscillator to be ready
	*/
	/*
	OSC.DFLLCTRL &= ~OSC_RC32MCREF_bm;		//Set up calibration source to be 32Khz crystal
	DFLLRC32M.CTRL |= DFLL_ENABLE_bm;		//Enable calibration of 32Mhz oscillator 
	*/
	
	int temp = 0;																			//Temporary variable for helping avoid 4 clock cycle limitation when updating secure registers
	
	//Enable external 8MHz oscillator
	OSC.XOSCCTRL = (OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc);
	//OSC.XOSCCTRL = (OSC_FRQRANGE_2TO9_gc | OSC_XOSCSEL_XTAL_16KCLK_gc);						//Set external oscillator to be between 2 and 9 MHz and select it
	OSC.CTRL |= OSC_XOSCEN_bm;																//Enable the external oscillator
	while(!(OSC.STATUS & OSC_XOSCRDY_bm));									//While the external oscillator is not ready, set the error led																		//Clear the error led if the external oscillator has stabilized
	
	//Enable phase locked loop to multiply external oscillator by 4 to get 32MHz
	temp = ((OSC_PLLSRC_XOSC_gc & OSC_PLLSRC_gm) | (OSC_PLLFAC_gm & 2));				//Set the external oscillator as the clock source for the pll and set to multiply by 4
	CCP = CCP_IOREG_gc;																		//Disable register security so we can update the pll control settings
	OSC.PLLCTRL = temp;																		//Write pll control settings to register
	OSC.CTRL |= OSC_PLLEN_bm;																//Enable the pll
	while(!(OSC.STATUS & OSC_PLLRDY_bm));									//While the pll is not ready, set the error led																			//Disable the error led if successfully stabilized
	
	//Set system pll clock divisions and set up as source for all system clocks
	temp = ((CLK_PSADIV_gm & CLK_PSADIV_1_gc) | (CLK_PSBCDIV_gm & CLK_PSBCDIV_1_1_gc));		//Set system to use pll divided by 1 (no division)
	CCP = CCP_IOREG_gc;																		//Disable register security so we can update the clock source division setting
	CLK.CTRL = temp;																		//Write division settings to register
	
	temp = CLK_SCLKSEL_PLL_gc;																//Set pll as system clock source
	CCP = CCP_IOREG_gc;																		//Disable register security so we can update the system clock
	CLK.CTRL = temp;
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