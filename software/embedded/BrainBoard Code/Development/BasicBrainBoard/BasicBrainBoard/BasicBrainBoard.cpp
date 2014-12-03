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

extern "C"{
	#include "usart_driver.h"
	#include "avr_compiler.h"
};

//Temp Prototypes
void SetXMEGA32MhzCalibrated();

#define STATUS1_SET(void) (PORTC.OUTSET = PIN5_bm)
#define STATUS1_CLR(void) (PORTC.OUTCLR = PIN5_bm)

#define STATUS2_SET(void) (PORTC.OUTSET = PIN6_bm)
#define STATUS2_CLR(void) (PORTC.OUTCLR = PIN6_bm)

#define ERROR_SET(void) (PORTC.OUTSET = PIN7_bm)
#define ERROR_CLR(void) (PORTC.OUTCLR = PIN7_bm)

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

int swap = 0;

USART_data_t USART_PC_Data;

#define GRIP_BM_SERIAL (1 << 1)

#define PACKETSIZE 10
volatile char recieveBuffer[PACKETSIZE];
volatile char SendBuffer[100];

enum { HEADER, COMMAND, BASEROTVAL1, BASEROTVAL2, ACT1VAL1, ACT1VAL2, ACT2VAL1, ACT2VAL2, CHECKSUM, TAIL};

enum XMegaStates{
	WaitForHost,
	ARMControl
} CurrentState = WaitForHost;

void uart_init(void);
void timer_init(void);
void SendStringPC(char *stufftosend);

ISR(TCC1_OVF_vect){
	TCC1.INTFLAGS = TC1_OVFIF_bm;
}

ISR(TCC0_OVF_vect){
	TCC0.INTFLAGS = TC0_OVFIF_bm;
}

int main(void)
{
	SetXMEGA32MhzCalibrated();
	
	//Initialization Code
	uart_init();
	PORTC.DIRSET = (PIN5_bm | PIN6_bm | PIN7_bm); //Sets status/error led outputs
	PORTC.DIRSET = (PIN0_bm | PIN1_bm | PIN4_bm); //Set RGB Led outputs
	//PORTC.DIRCLR = PIN0_bm;
	timer_init();
	PMIC.CTRL |= PMIC_LOLVLEN_bm; //draws current for ?
	
	

	sei();
    while(1)
    { 
		static long i = 0;
		i++;
		
		SendStringPC("ID: ArmControl\r\n");
		
		SendStringPC("Hi: ");
		
		/***Setting color phase difference about a third***/
		TCC0.CCA = i % 75; 
		TCC0.CCB = (i + 33) % 75; 
		TCC1.CCA = (i + 66) % 75; 
		
		_delay_ms(500);
		/*
		for(int i = 0; i < 333; ++i){
			RGB_GREEN_SET();
			RGB_RED_CLR();
			_delay_ms(5);
			RGB_RED_SET();
			RGB_GREEN_CLR();
			_delay_ms(5);
		}
	*/
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


ISR(USARTC0_RXC_vect){
	USART_RXComplete(&USART_PC_Data);
}


ISR(USARTC0_DRE_vect){
	USART_DataRegEmpty(&USART_PC_Data);
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