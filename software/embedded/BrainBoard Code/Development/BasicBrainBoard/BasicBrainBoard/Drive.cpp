/*
 * Drive.cpp
 *
 * Created: 3/3/2015 11:36:19 PM
 *  Author: nrpic_000
 */ 

//#include "BrainBoard.h"
//#include "XMegaLib.h"
#include "Drive.h"

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
	/*
	int check = 0;
	Saber_init_uno();
	char cmmd[5] = {'1', ',' , 's'};
	SendStringSABER_UNO("1,start\n");
	char speed[5] = {'0', '0', '0', '\n', '\0'};
	int i = 0;
	int rem = check;
	while(1){
	SendStringSABER_UNO("1,units 1 rotation = 2000 lines\n "); 
	} 
	while(1){
		i = 0;
		check = 0;
		rem = check;
		for(check = 0; check < 15; check++){
			rem = check;
			while(check){//converting int into the array
				speed[i++] = check % 10; //last digit
				check /= 10; //move number to the right
				if(i == 3)
					break;
			}
			check = rem;
			i = 0;
			SendStringSABER_UNO(cmmd);
			SendStringSABER_UNO(speed);		
			_delay_ms(2000);
		}
		_delay_ms(2000);
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
			SendStringSABER_UNO(cmmd);
			SendStringSABER_UNO(speed);
			_delay_ms(2000);
		}
	}*/
	
	//while(1){
		//SendStringSABER_UNO("1,s10\n");
	//	SendStringPC("Sent to saber\n");
	//}
	Saber_init_uno();
	//super useless comment
	_delay_ms(1000);
	SendStringSABER_UNO("1,start \n");
	_delay_ms(1000);
	SendStringSABER_UNO("1,units ");
	SendStringSABER_UNO(("1 rotation = 2000 "));
	SendStringSABER_UNO("lines \n");
	_delay_ms(1000);
	SendStringSABER_UNO("1,s10 \n");
	
	while(1);
}


void driveInit() {
	//This code is ran once before driveMain is run forever
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


//Gimbal send string function, its all by itself with its init so far
void SendStringGim(char *present){
	for(int i = 0 ; present[i] != '\0' ; i++){
		while(!USART_IsTXDataRegisterEmpty(&USARTD0));
		USART_PutChar(&USARTD0, present[i]);
	}
} //End gimbal send string functions, USARTD0



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