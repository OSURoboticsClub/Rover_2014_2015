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

/* Configuration
 * NOTE: Due to the way the code is designed, configuration
 * occurs in Arm.c, not here. */

/*** Internal arm functions. ***/

/* Setup the ADC to digitize the flex sensors. */
void init_flex();

/* Setup the stepper driver control pins, limit 
 * switch pins, and step generation interrupts. */
void init_steppers();

/* Return the voltage (in millivolts, from 0-2048)
 * of the given flex channel (1-4). */
int16_t flex_voltage(uint8_t channel);

/* Arm axises. */
typedef enum {ARM_X, ARM_Y, ARM_Z, ARM_ROTATE, ARM_GRIP} arm_axis_t;

/* Set the desired position of the axis, in steps.
 * The step positions start at 0 at the limit
 * switch and increase from there. */
void set_target(arm_axis_t axis, int32_t position);

/* Return the current position of the axis, in steps.
 * The step positions start at 0 at the limit
 * switch and increase from there. */
int32_t get_position(arm_axis_t axis);

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
void enable_steppers();

/* Disable all stepper drivers and set target positions
 * to current positions, to stop step generation. */
void disable_steppers();

/* Returns true if any axises are in motion. */
bool arm_moving();

#endif /* ARM_H_ */