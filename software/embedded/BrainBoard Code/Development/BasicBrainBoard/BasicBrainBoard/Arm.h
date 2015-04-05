/*
 * Arm.h
 *
 * Created: 1/23/2015 7:20:11 PM
 *  Author: Nick Ames
 */
#ifndef ARM_H_
#define ARM_H_

/** External arm API **/

/* Initialize pins and peripherals needed by the arm board. */
void armInit();

/* Operate the arm board. */
void armMain();

/** Internal arm functions. **/

/* Setup the ADC to digitize the flex sensors. */
void init_flex();

/* Setup the stepper driver control pins, limit 
 * switch pins, and step generation interrupts. */
void init_steppers();



#endif /* ARM_H_ */