/*
 * BasicBrainBoard.cpp
 *
 * Created: 11/25/2014 6:03:12 PM
 *  Author: OSURC2
 */ 


#define F_CPU 32000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "XMegaLib.h"

extern "C"{
	#include "usart_driver.h"
	#include "avr_compiler.h"
};

//Temp Prototypes
void SetXMEGA32MhzCalibrated();
void uart_init(void);
void timer_init(void);
void SendStringPC(char *stufftosend);

USART_data_t USART_PC_Data;

#define GRIP_BM_SERIAL (1 << 1)

#define PACKETSIZE 10
volatile char recieveBuffer[PACKETSIZE];
volatile char SendBuffer[100];

//enum { HEADER, COMMAND, BASEROTVAL1, BASEROTVAL2, ACT1VAL1, ACT1VAL2, ACT2VAL1, ACT2VAL2, CHECKSUM, TAIL};

enum XMegaStates{
	WaitForHost,
	MainProgram
} CurrentState = WaitForHost;


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

int main(void)
{
	SetXMEGA32MhzCalibrated();
	
	//Initialization Code
	uart_init();
	PORTC.DIRSET = (PIN5_bm | PIN6_bm | PIN7_bm); //Sets status/error led outputs
	PORTC.DIRSET = (PIN0_bm | PIN1_bm | PIN4_bm); //Set RGB Led outputs
	//PORTC.DIRCLR = PIN0_bm;
	timer_init();  //Initialize Timers
	PMIC.CTRL |= PMIC_LOLVLEN_bm; //draws current for ?

	sei(); //Enable interrupts
	
    while(1)
    { 
		if(USART_RXBufferData_Available(&USART_PC_Data)){
			recieveChar = USART_RXBuffer_GetByte(&USART_PC_Data);
			switch(recieveChar){
				case 'r':
					SendStringPC("Red LED.\r\n");
					RGBSetColor(RED);
					break;
				case 'b':
					SendStringPC("Blue LED.\r\n");
					RGBSetColor(BLUE);	
					break;
				case 'g':
					SendStringPC("Green LED.\r\n");
					RGBSetColor(GREEN);				
					break;
				case 'y':
					SendStringPC("Yellow LED.\r\n");
					RGBSetColor(YELLOW);
					break;
				default:
					SendStringPC("Can't do that.\r\n");
					break;
					
			}
			_delay_ms(10);
		}
		
		
    }
}

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


void SendStringPC(char *stufftosend){
	for(int i = 0 ; stufftosend[i] != '\0' ; i++){
		while(!USART_IsTXDataRegisterEmpty(&USARTC0));
		USART_PutChar(&USARTC0, stufftosend[i]);
	}
}


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