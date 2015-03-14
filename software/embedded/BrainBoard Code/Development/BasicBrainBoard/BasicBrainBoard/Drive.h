/*
 * Drive.h
 *
 * Created: 3/3/2015 11:26:19 PM
 *  Author: nrpic_000
 */ 


#ifndef DRIVE_H_
#define DRIVE_H_

#include "XMegaLib.h"

/**** USART types ****/

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

/**** End USART types ****/

/**** ISR's ****/

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

/**** END ISR's ****/

/**** Function Prototypes ****/

void Saber_init_uno(void);
void Saber_init_dos(void);
void Saber_init_tres(void);

void SendStringSABER_UNO(char *present);
void SendStringSABER_DOS(char *present);
void SendStringSaber_TRES(char *present);

void convert(int num, char* box);

/**** END Function Prototypes ****/

#endif /* DRIVE_H_ */