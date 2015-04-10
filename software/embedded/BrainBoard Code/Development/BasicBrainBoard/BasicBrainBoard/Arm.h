/*
 * Arm.h
 *
 * Created: 1/23/2015 7:20:11 PM
 *  Author: Nick Ames
 */
#ifndef ARM_H_
#define ARM_H_
#include <stdint.h>

/*** External arm API ***/

/* Initialize pins and peripherals needed by the arm board. */
void armInit();

/* Operate the arm board. */
void armMain();

/*** Configuration ***/
/* Data used to control step generation.
 * The resulting variable is a five element array, with each element
 * corresponding to the axis having the same index in arm_axis_t. */
volatile struct {
	int32_t current; /* Current position, in steps. */
	int32_t target; /* Target position, in steps. */
	/* Axis speed.
	 * NOTE: This number is the period of each step,
	 * in units of 32us (1/32kHz). Lower number indicate
	 * higher speeds.
	 *   period [32us units] = 1/(32e-6 * speed[steps/s]) */
	uint16_t step_period;
	/* Positive (away from limit) DIR pin value. */
	uint8_t pos_dir;
} 


#define ARM_X_SP 32
#define ARM_Y_SP 32
#define ARM_Z_SP 32
#define ARM_ROTATE_SP 32
#define ARM_GRIP_SP 32

/* Positive (away from limit) DIR value. */
#define 




/*** Internal arm functions. ***/

/* Setup the ADC to digitize the flex sensors. */
void init_flex();

/* Setup the stepper driver control pins, limit 
 * switch pins, and step generation interrupts. */
void init_steppers();

/* Return the voltage (in millivolts, from 0-2048)
 * of the given flex channel (1-4). */
int flex_voltage(int8_t channel);

/* Arm axises. */
typedef enum {ARM_X, ARM_Y, ARM_Z, ARM_ROTATE, ARM_GRIP} arm_axis_t;

/* Set the desired position of the axis, in steps.
 * The step positions start at 0 at the limit
 * switch and increase from there. */
void set_target(arm_axis_t axis, uint32_t position);

/* Return the current position of the axis, in steps.
 * The step positions start at 0 at the limit
 * switch and increase from there. */
uint32_t get_position(arm_axis_t axis);

/* Set the current position of the axis to be 0.
 * This function should only be called when an
 * axis is pressing its limit switch. */
void set_zero(arm_axis_t axis);

/* Return true if the given limit switch is pressed,
 * false if it is not. Due to electrical noise issues,
 * this function should only be relied on for
 * unpressed->pressed transitions. */
bool limit_pressed(arm_axis_t axis);

/* If true, the nFAULT line is being pulled low by one of the
 * stepper drivers, indicating a fault condition. */
bool stepper_fault();

/* Enable all stepper drivers. */
void stepper_enable();

/* Disable all stepper drivers.
 * NOTE: This only disables the drivers, it doesn't stop
 * step generation. Don't call during normal operation. */
void stepper_disable();

#endif /* ARM_H_ */