/*
 * Drive.h
 *
 * Created: 3/3/2015 11:26:19 PM
 *  Author: nrpic_000
 */ 


#ifndef DRIVE_H_
#define DRIVE_H_


#define CHANNEL2 2  //Vertical Left Stick
#define CHANNEL3 3  //Vertical Right Stick
#define CHANNEL5 5  //Speed control
#define CHANNEL6 6  //Enable

#define CH2_MIN 927
#define CH2_MAX 1654
#define CH2_STOP ((CH2_MAX-CH2_MIN)/2 + CH2_MIN)
#define CH2_MAGNATUDE ((CH2_MAX - CH2_MIN) / 2)

#define CH3_MIN 919
#define CH3_MAX 1644
#define CH3_STOP ((CH3_MAX-CH3_MIN)/2 + CH3_MIN)
#define CH3_MAGNATUDE ((CH3_MAX - CH3_MIN) / 2)

#define CH5_MIN 918
#define CH5_MAX 1648
#define CH5_RANGE (CH5_MAX - CH5_MIN)

#define CH6_MIN 950   //Higher than actual, use < statement
#define CH6_MAX 1630  //Lower than actual, use > statement

#define STOP_CONSTANT 50  //Originally 20

#include "XMegaLib.h"
#include "math.h"
#include "string.h"




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

//Below are functions for RC control (Need hardware testing)
void RC_init();
unsigned long cyclesto_ms(unsigned long cycles);
unsigned long read(int ch);
char RCSpeed(int ch); //Does work on RC signal values to determine speed value to send to the kangaroo




/**** END Function Prototypes ****/

#endif /* DRIVE_H_ */