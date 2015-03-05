/*
 * Arm.h
 *
 * Created: 1/23/2015 7:20:11 PM
 *  Author: Brian Sia, Nick Ames
 */ 


#ifndef ARM_H_
#define ARM_H_

#include "XMegaLib.h"

/*** Pin Definitions ***/
/* Stepper Drivers */
#define NFAULT PB3 /* Active-low fault indicator for all stepper drivers. */
#define S1_STEP PE4
#define S1_DIR PE7
#define S1_NEN PE5 /* Active-low enable signal. */
#define Step Stepper Driver 2 PE3
#define Dir Stepper Driver 2 PE2
#define nEN Stepper Driver 2 PE0
#define Step Stepper Driver 3 PD6
#define Dir Stepper Driver 3 PE1
#define nEN Stepper Driver 3 PD7
#define Step Stepper Driver 4 PD5
#define Dir Stepper Driver 4 PD4
#define nEN Stepper Driver 4 PD2
#define Step Stepper Driver 5 PD1
#define Dir Stepper Driver 5 PD0
#define nEN Stepper Driver 5 PD3
#define Force Ch. 1 Arm PA3/ADCA3
#define Force Ch. 2 Arm PA2/ADCA2
#define Force Ch. 3 Arm PA4/ADCA4
#define Force Ch. 4 Arm PA5/ADCA5
#define Limit Switch 1 Arm PF6
#define Limit Switch 2 Arm PF7
#define Limit Switch 3 Arm PF4
#define Limit Switch 4 Arm PF0
#define Limit Switch 5 Arm PF1
#define nPAUSE input Drive Board Com. PF5
#define TX Drive Board Com. PF3/TXF0
#define RX Drive Board Com. PF2/RXF0


//Macros
#define MD1_STEP_SET(void) (PORTE.OUTSET = PIN4_bm);
#define MD1_STEP_CLR(void) (PORTE.OUTCLR = PIN4_bm);

#define MD1_DIR_SET(void) (PORTE.OUTSET = PIN7_bm);
#define MD1_DIR_CLR(void) (PORTE.OUTCLR = PIN7_bm);

#define MD1_nEN_SET(void) (PORTE.OUTSET = PIN5_bm);
#define MD1_nEN_CLR(void) (PORTE.OUTCLR = PIN5_bm);

//Arm Prototypes
void armGPIOInit(void);


#endif /* ARM_H_ */