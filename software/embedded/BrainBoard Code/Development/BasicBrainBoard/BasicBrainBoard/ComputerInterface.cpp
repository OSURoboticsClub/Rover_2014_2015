/*
 * ComputerInterface.cpp
 *
 * Created: 3/5/2015 8:45:32 AM
 *  Author: Nick McComb
 */ 

#include "ComputerInterface.h"

//Interrupt when anything is recieved
ISR(USARTC0_RXC_vect){
	USART_RXComplete(&USART_PC_Data);
}