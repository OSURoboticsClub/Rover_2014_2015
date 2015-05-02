
//#include "BrainBoard.h"
#include "XMegaLib.h"
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
	//Imaginary function from comp for speed is char compspeed();
	int check = 0;
	//Saber_init_uno();
	char cmmd[4] = "1,s";
	char cap = '\0';
	char *speed; //call function i_to_st(int value) to turn the speed int into a c-string 
	char *all; //use add_st(char*, char*) a couple times to put all the strings together for a solid one to send to the kangaroo; that functions puts st2 at the end of st1
	
	
	int i = 0;
	int rem = check;
	
	//Sabertooth_UNO is the middle one

	Saber_init_uno();
	
	char recieveChar;
	
	/*
	while(1){
		if(USART_RXBufferData_Available(&USART_PC_Data)){
			recieveChar = USART_RXBuffer_GetByte(&USART_PC_Data);
			
			while(!USART_IsTXDataRegisterEmpty(&USARTE0));
				USART_PutChar(&USARTE0, recieveChar);
		}
	}
	*/
		
	SendStringSABER_UNO("1,start\n");
	SendStringSABER_UNO("2,start\n");
	SendStringPC("1,start\r\n");
	_delay_ms(1000);

	//Set the units for the encoder. The values entered are rough and might
	//need to be tuned. Calculated from the website. 
	SendStringSABER_UNO("1, UNITS 917 cm = 270000 lines\n"); //check out exact values
	_delay_ms(10);
	SendStringSABER_UNO("2, UNITS 917 cm = 270000 lines\n"); //check out exact values
	
	SendStringPC("1, UNITS 917 cm = 270000 lines\r\n");
	
	//SendStringSABER_UNO("UNITS 917 cm = 270000 lines\n");
	//SendStringSABER_UNO("lines \n");z

	_delay_ms(1000);
	
	while (1)
	{
		SendStringSABER_UNO("1, s50\n");  //Set the speed to 50 cm/s
		SendStringPC("1, s50\r\n");
		_delay_ms(10);
	}
	
	

	while(1){
		if(USART_RXBufferData_Available(&SABER_UNO)){
			recieveChar = USART_RXBuffer_GetByte(&USART_PC_Data);
			char txByte[2] = { recieveChar , '\0' };
			SendStringPC(&recieveChar);
		}
	}
	

	//ALGORITHM after exact functions are available a while loop will iterate through and at each start
	//will call for a speed from RC or comp (or both?) and put that value in the speed string
	//After that all the strings (cmmd,speed,cap) are put into all (ex all = cmmd+speed+cap) then
	//SendString is called on all.c_str (returns a c string version that the SendString function can use\

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
		_delay_us(50);  //DEGBUGGING
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


void RC_init(){	//Sets correct RC pins as inputs and sets up a timer
	PORTB.DIRCLR = PIN3_bm;																			//Sets RX pin as input CH1
	PORTB.DIRCLR = PIN2_bm;																			//Sets RX pin as input CH2
	PORTB.DIRCLR = PIN1_bm;																			//Sets RX pin as input CH3
	
	PORTA.DIRCLR = PIN7_bm;																			//Sets RX pin as input CH4
	PORTA.DIRCLR = PIN6_bm;																			//Sets RX pin as input CH7
	PORTA.DIRCLR = PIN5_bm;																			//Sets RX pin as input Ch8
	
	/*
	DDRB &= ~(1<<PB3); //PB3 as input, CH1
	DDRB &= ~(1<<PB2); //PB2 as input, CH2
	DDRB &= ~(1<<PB1); //PB1 as input, CH 3
	
	DDRA &= ~(1<<PA7); //PA7 as input, CH4
	DDRA &= ~(1<<PA6); //PA6 as input, CH 7
	DDRA &= ~(1<<PA5); //PA5 as input, CH 8
	*/
	/*PORTB |= (1<<PB3);//These lines enable pull up resistors on the pins, NOT NEEDED!
	PORTB |= (1<<PB2);
	PORTB |= (1<<PB1);
	
	PORTA |= (1<<PA7);
	PORTA |= (1<<PA6);
	PORTA |= (1<<PA5); */
}

unsigned long cyclesto_ms(unsigned long cycles){
	unsigned long cycle_period= (unsigned long)pow(3.125, -5);
	return(cycles*cycle_period);
}

unsigned long read(int ch){
	unsigned long width = 0;
	unsigned long numloops = 0;
	unsigned long maxloops = 100000; //May need to come up with a better number for this
	
	if(ch = 2){
		while((!(PORTB.IN & PIN3_bm))){
			if(numloops++ == maxloops)
			return 0;
			width++;
		}
		return (cyclesto_ms(width *21 + 16)); //the 21 and 16 have to do with 20 clock cycles and 16 clocks between edge and start of loop, unsure of reasoning
	}
	
	else if(ch = 3){
		while( !(PORTB.IN & PIN1_bm )){
			if(numloops++ == maxloops)
			return 0;
			width++;
		}
		
		return (cyclesto_ms(width *21 + 16)); //the 21 and 16 have to do with 20 clock cycles and 16 clocks between edge and start of loop, unsure of reasoning
	}
	
	else if(ch = 7){
		while( !(PORTA.IN & PIN6_bm)){
			if(numloops++ == maxloops)
			return 0;
			width++;
		}
		
		return (cyclesto_ms(width *21 + 16)); //the 21 and 16 have to do with 20 clock cycles and 16 clocks between edge and start of loop, unsure of reasoning
		
	}
	
	
}

int RCSpeed(int ch){ //Does work on RC signal to determine speed value to send to the kangaroo
	char command = 0;
	float ratio;
	if(ch = 2){
		short channel = read(2);
		channel -= CH2_STOP;
		ratio = ((float)channel / (float)CH2_MAGNATUDE);
		if(ratio < -1)
		ratio = -1;
		else if(ratio > 1)
		ratio = 1;
		ratio = abs(ratio) *100;
		
		return((int) ratio);
		
	}
	else if(ch = 3){
		short channel = read(3);
		channel -= CH3_STOP;
		ratio = ((float)channel / (float)CH3_MAGNATUDE);
		if(ratio < -1)
		ratio = -1;
		else if(ratio > 1)
		ratio = 1;
		ratio = abs(ratio) *100;
		
		return((int) ratio);
		
	}
	else if(ch = 6){
		short channel = read(6);
		return((int)channel);
	}
	
	else{
		return 0;
	}
	
	
	
}


char * i_to_st(int value){
int digits = 0;
int val, nw;
char *num = NULL;
val = 0;
nw = value;

while(nw != 0){
	nw = nw/10;
	digits++;
}
nw = value;
num = (char*) malloc(digits+1);
for(int i = 0; i < digits; i++){
	num[i] = nw%10;
	nw = nw/10; 
}		
}

char * add_st(char *st1, char *st2){
	int len1, len2,big;
	char *nw;
	len1 = strlen(st1);
	len2 = strlen(st2);
	big = len1+len2;
	nw = (char *) malloc(big+1);
	for(int i = 0; i < len1; i++){
		nw[i] = st1[i];
	}
	for(int i = len1; i < big; i++){
		nw[i] = st2[i];
	}
	
	
	
}
