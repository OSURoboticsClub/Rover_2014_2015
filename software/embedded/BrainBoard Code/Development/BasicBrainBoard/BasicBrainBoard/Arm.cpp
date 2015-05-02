/*
 * Arm.cpp
 *
 * Created: 1/23/2015 7:20:37 PM
 *  Author: Nick Ames
 */ 
#include "XMegaLib.h"
#include "Arm.h"
#include <avr/pgmspace.h>
#include <stddef.h>

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
 * overflowing to 0 when they reach the 16 bit MAX of 65535.
 * Compare channels are used to run interrupts when the timer
 * reaches a certain value. generate_step() sets the
 * CC to the current timer value plus the step period, thus
 * timing the steps.
 * 
 * Because the stepper drivers step on a rising edge, the
 * step line must be reset to low after each step.
 * To accomplish this, generate_step() runs twice for each
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
	
	register8_t * const cnt_l; /* CNTL for this axis. */
	register8_t * const cc_buf_l; /* CCxBUFL for this axis. */
	register8_t * const int_ctrl; /* CC interrupt register for this axis. */
	const uint8_t int_bm; /* Interrupt bit mask for this axis. */
	//TODO: timer interrupt bits
} ArmAxis[5] { /*              Step      |      Dir       |      nEN       |     Limit      |    CNTL     |   CCxBUFL      |     INTCTRLB    | Interrupt bit mask */
	{0,0, 1000, true, &PORTE, PIN4_bm, &PORTE, PIN7_bm, &PORTE, PIN5_bm, &PORTF, PIN6_bm, &(TCE0.CNTL), &(TCE0.CCABUFL), &(TCE0.INTCTRLB), TC0_CCAINTLVL1_bm}, /* X axis */
	{0,0, 1000, true, &PORTE, PIN3_bm, &PORTE, PIN2_bm, &PORTE, PIN0_bm, &PORTF, PIN7_bm, &(TCE0.CNTL), &(TCE0.CCBBUFL), &(TCE0.INTCTRLB), TC0_CCBINTLVL1_bm}, /* Y axis */
	{0,0, 1000, true, &PORTD, PIN6_bm, &PORTE, PIN1_bm, &PORTD, PIN7_bm, &PORTF, PIN4_bm, &(TCE0.CNTL), &(TCE0.CCCBUFL), &(TCE0.INTCTRLB), TC0_CCCINTLVL1_bm}, /* Z axis */
	{0,0, 1000, true, &PORTD, PIN5_bm, &PORTD, PIN4_bm, &PORTD, PIN2_bm, &PORTF, PIN0_bm, &(TCE0.CNTL), &(TCE0.CCDBUFL), &(TCE0.INTCTRLB), TC0_CCDINTLVL1_bm}, /* Rotation */
	{0,0, 1000, true, &PORTD, PIN1_bm, &PORTD, PIN0_bm, &PORTD, PIN3_bm, &PORTF, PIN1_bm, &(TCE1.CNTL), &(TCE1.CCABUFL), &(TCE1.INTCTRLB), TC1_CCAINTLVL1_bm} /* Grip */
};

/* Generate a step on the given axis and schedule to
 * run again if necessary. */
void generate_step(arm_axis_t axis){
	int32_t diff = ArmAxis[axis].target - ArmAxis[axis].current;
	if(0 == diff){
		/* Clear interrupt. */
		*(ArmAxis[axis].int_ctrl) &= ~ArmAxis[axis].int_bm;
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
	
	/* Set counter compare buffer register */
	uint16_t new_cc;
	new_cc = *(ArmAxis[axis].cnt_l);
	new_cc |= *(ArmAxis[axis].cnt_l + 1) << 8;
	new_cc += ArmAxis[axis].step_period;
	*(ArmAxis[axis].cc_buf_l) = 0xFF & new_cc;
	*(ArmAxis[axis].cc_buf_l + 1) =  new_cc >> 8;
	
	/* Force CC register update. */
	TCE0.CTRLFSET = TC_CMD_UPDATE_gc;
	TCE1.CTRLFSET = TC_CMD_UPDATE_gc;
	
	/* Enable interrupt. */
	*(ArmAxis[axis].int_ctrl) |= ArmAxis[axis].int_bm;
}

ISR(TCE0_CCA_vect){
	generate_step(ARM_X);
}

ISR(TCE0_CCB_vect){
	generate_step(ARM_Y);
}

ISR(TCE0_CCC_vect){
	generate_step(ARM_Z);
}

ISR(TCE0_CCD_vect){
	generate_step(ARM_ROTATE);
}

ISR(TCE1_CCA_vect){
	generate_step(ARM_GRIP);
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
	
	/* Setup timers. */
	TCE0.CTRLB = TC_WGMODE_NORMAL_gc | TC0_CCDEN_bm | TC0_CCCEN_bm | TC0_CCBEN_bm | TC0_CCAEN_bm;
	TCE1.CTRLB = TC_WGMODE_NORMAL_gc | TC1_CCAEN_bm;
	TCE0.PERL = 0xFF;
	TCE0.PERH = 0xFF;
	TCE0.CTRLFSET = TC_CMD_UPDATE_gc;
	TCE1.CTRLFSET = TC_CMD_UPDATE_gc;
	TCE0.CTRLA = TC_CLKSEL_DIV1024_gc;
	TCE1.CTRLA = TC_CLKSEL_DIV1024_gc;
	
	/* Enable medium level interrupts. */
	PMIC.CTRL |= MEDLVLEN_bm;
}

/* Read NVM signature. From http://www.avrfreaks.net/forum/xmega-production-signature-row */
static uint8_t ReadCalibrationByte( uint8_t index ){
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return( result );
} 

/* Setup the ADC to digitize the flex sensors. */
void init_flex(){
	//1 = ADCA3
	//2 = ADCA2
	//3 = ADCA4
	//4 = ADCA5
	//use signed mode
	//use either AREFA or AREFB
	//use CLK/256
	ADCA.CTRLB = ADC_FREERUN_bm;
	ADCA.REFCTRL = ADC_REFSEL1_bm;
	ADCA.PRESCALER = ADC_PRESCALER2_bm | ADC_PRESCALER1_bm;
	ADCA.CALL = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
	ADCA.CALH = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
	ADCA.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;
	ADCA.CH2.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;
	ADCA.CH3.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc;
	ADCA.CTRLA = ADC_ENABLE_bm | ADC_CH3START_bm | ADC_CH2START_bm | ADC_CH1START_bm | ADC_CH0START_bm;
}



/* Return the voltage (in millivolts, from 0-2048)
 * of the given flex channel (1-4). */
int16_t flex_voltage(uint8_t channel){
	int16_t result;
	switch(channel){
		case 1:
			result = ADCA.CH0.RESL;
			result |= ADCA.CH0.RESH << 8;
			break;
		case 2:
			result = ADCA.CH1.RESL;
			result |= ADCA.CH1.RESH << 8;
			break;
		case 3:
			result = ADCA.CH2.RESL;
			result |= ADCA.CH2.RESH << 8;
			break;
		case 4:
			result = ADCA.CH3.RESL;
			result |= ADCA.CH3.RESH << 8;
			break;
		
		default:
			result = 0;
	}
	if(result < 0) result = 0;
	return result;
}


/* Initialize pins and peripherals needed by the arm board. */
void armInit(){
	init_flex();
	init_steppers();
}

/* Operate the arm board. */
void armMain(){
	RGBSetColor(BLUE);
	_delay_ms(1000);
	set_target(ARM_X, 50);
	while(1){
		generate_step(ARM_X);
		_delay_ms(50);
	}
	while(1);
}

/* Set the desired position of the axis, in steps.
 * The step positions start at 0 at the limit
 * switch and increase from there. */
void set_target(arm_axis_t axis, int32_t position){
	cli();
	ArmAxis[axis].target = position * 2;
	sei();
	generate_step(axis);
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