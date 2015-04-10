/*
 * Arm.cpp
 *
 * Created: 1/23/2015 7:20:37 PM
 *  Author: Nick Ames
 */ 
#include "XMegaLib.h"
#include "Arm.h"

/* This source file controls the Arm Daughterboard,
 * which moves the sample pickup arm. 
 * 
 * The sample pickup arm has five axises 
 * (X, Y, Z cartesian movement, gripper rotation, and gripper open/close).
 * Each axis is moved by a stepper motor and has a single limit switch 
 * at one end to provide a position reference. The limit switches are
 * normally-closed to ground, and are pulled high when pressed. When a
 * limit switch input is high, it experiences frequent very short low
 * pulses due to electrical noise from the stepper driver. These pulses
 * could be filtered out by polling with an interrupt routine, but
 * that won't be necessary as long as the limit switched are used only
 * for unpressed->pressed transitions.
 * 
 * The Arm Daughterboard is equipped with four analog inputs designed
 * to measure resistive flex sensors mounted in the gripper. These
 * sensors are used to tell when a sample has been gripped successfully.
 * 
 * Finally, the Arm Daughterboard is also equipped with a connection to
 * the Drive Daughterboard. This connection carries a bi-directional RS-232
 * link and an active-low PAUSE signal. When the pause signal is asserted,
 * all movement will cease.
 */

/* Data used to control step generation.
 * The resulting variable is a five element array, with each element
 * corresponding to the axis having the same index in arm_axis_t. */
static volatile struct {
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
} AxisData[5];

/* Setup the ADC to digitize the flex sensors. */
void init_flex(){
	
}

/* Setup the stepper driver control pins, limit 
 * switch pins, and step generation interrupts. */
void init_steppers(){
	/* Setup limit switch pins. */
	/* Axises:         X       Y         Z         A         B */
	PORTF.DIRCLR = PIN6_bm | PIN7_bm | PIN4_bm | PIN0_bm | PIN1_bm;
	PORTCFG.MPCMASK  = PIN6_bm | PIN7_bm | PIN4_bm | PIN0_bm | PIN1_bm;
	PORTF.PIN6CTRL = PORT_OPC_PULLUP_gc;
	
	/* Setup stepper driver control pins. */
	PORTB.DIRCLR = PIN3_bm; /* nFAULT */
	PORTB.PIN3CTRL = PORT_OPC_PULLUP_gc; /* nFAULT pull-up */
	
	PORTE.DIRSET = PIN4_bm | PIN7_bm | PIN5_bm; /* X: Step, Dir, nEN. */
	PORTE.DIRSET = PIN3_bm | PIN2_bm | PIN0_bm; /* Y: Step, Dir, nEN. */
	PORTD.DIRSET = PIN6_bm | PIN7_bm; /* Z: Step, nEN. */
	PORTE.DIRSET = PIN1_bm; /* Z: Dir. */
	PORTD.DIRSET = PIN5_bm | PIN4_bm | PIN2_bm; /* A: Step, Dir, nEN. */
	PORTD.DIRSET = PIN1_bm | PIN0_bm | PIN3_bm; /* B: Step, Dir, nEN. */
}

/* Initialize pins and peripherals needed by the arm board. */
void armInit(){
	init_flex();
	init_steppers();
}

/* Operate the arm board. */
void armMain(){
	
}