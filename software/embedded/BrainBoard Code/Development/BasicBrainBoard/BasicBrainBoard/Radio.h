/*
 * Radio.h
 *
 * Created: 3/4/2015 1:07:19 AM
 *  Author: Nick McComb
 */ 


#ifndef RADIO_H_
#define RADIO_H_

#include "XMegaLib.h"

/**********

Pin definitions:

Drive Board Connector
Pause - PF5
Drive_RX - PF2
Drive_TX - PF3

SmartFinder Connector
SmartFinder_RX - PE3
SmartFinder_TX - PE2

Stepper Driver
!Fault - PB3
Step - PE4
Direction - PE7
Enable - PE5

Limit Switch
PF6

**********/


/* RADIO STEPPER CONTROL */

#define RADIO_STEPPER_STEP_SET(void) (PORTE.OUTSET = PIN4_bm)
#define RADIO_STEPPER_STEP_CLR(void) (PORTE.OUTCLR = PIN4_bm)

#define RADIO_STEPPER_DIR_SET(void) (PORTE.OUTSET = PIN7_bm)
#define RADIO_STEPPER_DIR_CLR(void) (PORTE.OUTCLR = PIN7_bm)

#define RADIO_STEPPER_ENABLE(void) (PORTE.OUTSET = PIN5_bm)
#define RADIO_STEPPER_DISABLE(void) (PORTE.OUTCLR = PIN5_bm)

#define CHECK_RADIO_LIMIT_SW(void) (PORTF.IN & PIN6_bm)

/* END RADIO STEPPER CONTROL */

#endif /* RADIO_H_ */