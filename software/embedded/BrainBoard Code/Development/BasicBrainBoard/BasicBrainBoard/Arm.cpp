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

/* Step generation is performed by five "threads" triggered by 
 * the capture compare interrupts of timers TCE0 and TCE1.
 * When triggered, the generate_step() is run on the corresponding
 * axis, and the interrupt is reset if necessary to generate more
 * steps.
 * 
 * TCE0 and TCE1 are configured to count upward at 31.250 kHz,
 * overflowing to 0 when then reach the 16 bit MAX of 65535.
 * Compare channels are used to run interrupts when the timer
 * reaches a certain value. generate_step() sets the
 * CC to the current timer value plus the step period, thus
 * timing the steps.
 * 
 * Because the stepper drivers step on a rising edge, the
 * step line must be reset to low after each step. In order
 * to accomplish this, generate_step() runs twice for each
 * step generated, toggling the step line each time. As a
 * result, the values of current and target are twice that
 * true values commanded by the application, but this
 * difference is taken into account by set_target(),
 * get_position(), etc.
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
	const uint16_t step_period;
	const bool pos_dir; /* Positive (away from limit) DIR pin value. */
	
	PORT_t * const step_port; /* Port containing step pin. */
	const uint8_t step_pin; /* Step pin bit mask. */
	PORT_t * const dir_port; /* Port containing dir pin. */
	const uint8_t dir_pin; /* Dir pin bit mask. */
	PORT_t * const nen_port; /* Port containing nEN. */
	const uint8_t nen_pin; /* nEN pin bit mask. */
	PORT_t * const limit_port; /* Port containing limit switch. */
	const uint8_t limit_pin; /* Limit pin bit mask. */
	//TODO: timer interrupt bits
} ArmAxis[5] { /*       Step      |      Dir       |      nEN       |     Limit      */
	{0,0,32,true, &PORTE, PIN4_bm, &PORTE, PIN7_bm, &PORTE, PIN5_bm, &PORTF, PIN6_bm}, /* X axis */
	{0,0,32,true, &PORTE, PIN3_bm, &PORTE, PIN2_bm, &PORTE, PIN0_bm, &PORTF, PIN7_bm}, /* Y axis */
	{0,0,32,true, &PORTD, PIN6_bm, &PORTE, PIN1_bm, &PORTD, PIN7_bm, &PORTF, PIN4_bm}, /* Z axis */
	{0,0,32,true, &PORTD, PIN5_bm, &PORTD, PIN4_bm, &PORTD, PIN2_bm, &PORTF, PIN0_bm}, /* Rotation */
	{0,0,32,true, &PORTD, PIN1_bm, &PORTD, PIN0_bm, &PORTD, PIN3_bm, &PORTF, PIN1_bm} /* Grip */
};


//ISRs


/* Generate a step on the given axis and schedule to
 * run again if necessary. */
void generate_step(arm_axis_t axis){
	int32_t diff = ArmAxis[axis].target - ArmAxis[axis].current;
	if(0 == diff){
		//TODO: clear interrupt
		return;
	} else if(diff > 0){
		if(ArmAxis[axis].pos_dir){
			ArmAxis[axis].dir_port->OUTSET = ArmAxis[axis].dir_pin;
		} else {
			ArmAxis[axis].dir_port->OUTCLR = ArmAxis[axis].dir_pin;
		}
		ArmAxis[axis].step_port->OUTTGL = ArmAxis[axis].step_pin;
		ArmAxis[axis].current++;
	} else {
		if(ArmAxis[axis].pos_dir){
			ArmAxis[axis].dir_port->OUTCLR = ArmAxis[axis].dir_pin;
		} else {
			ArmAxis[axis].dir_port->OUTSET = ArmAxis[axis].dir_pin;
		}
		ArmAxis[axis].step_port->OUTTGL = ArmAxis[axis].step_pin;
		ArmAxis[axis].current--;
	}
	
	//Do interrupt stuff
	//TODO: Don't forget to sent an UPDATE command after setting CC register.
}
	

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
	for(int i=0; i < 5; i++){
		ArmAxis[i].step_port->DIRSET = ArmAxis[i].step_pin;
		ArmAxis[i].dir_port->DIRSET = ArmAxis[i].dir_pin;
		ArmAxis[i].nen_port->DIRSET = ArmAxis[i].nen_pin;
	}
	enable_steppers();
	
	/* Setup timers stuff. */
}

/* Initialize pins and peripherals needed by the arm board. */
void armInit(){
	init_flex();
	init_steppers();
}

/* Operate the arm board. */
void armMain(){
	
}

/* Set the desired position of the axis, in steps.
 * The step positions start at 0 at the limit
 * switch and increase from there. */
void set_target(arm_axis_t axis, int32_t position){
	cli();
	ArmAxis[axis].target = position * 2;
	sei();
}

/* Return the current position of the axis, in steps.
 * The step positions start at 0 at the limit
 * switch and increase from there. */
int32_t get_position(arm_axis_t axis){
	int32_t value;
	cli();
	value = ArmAxis[axis].current / 2;
	sei();
	return value;
}

/* Set the current position of the axis to be 0.
 * This function should only be called when an
 * axis is pressing its limit switch. */
void set_zero(arm_axis_t axis){
	cli();
	ArmAxis[axis].current = 0;
	sei();
};

/* Return true if the given limit switch is pressed,
 * false if it is not. Due to electrical noise issues,
 * this function should only be relied on for
 * unpressed->pressed transitions. */
bool limit_pressed(arm_axis_t axis){
	return ArmAxis[axis].limit_port->IN & ArmAxis[axis].limit_pin;
}

/* If true, the nFAULT line is being pulled low by one of the
 * stepper drivers, indicating a fault condition. */
bool stepper_fault(){
	return !(PORTB.IN & PIN3_bm);
}

/* Enable all stepper drivers. */
void enable_steppers(){
	for(int i=0; i < 5; i++){
		ArmAxis[i].nen_port->OUTCLR = ArmAxis[i].nen_pin;
	}
}

/* Disable all stepper drivers.
 * NOTE: This only disables the drivers, it doesn't stop
 * step generation. Don't call during normal operation. */
void disable_steppers(){
	for(int i=0; i < 5; i++){
		ArmAxis[i].nen_port->OUTSET = ArmAxis[i].nen_pin;
	}
}