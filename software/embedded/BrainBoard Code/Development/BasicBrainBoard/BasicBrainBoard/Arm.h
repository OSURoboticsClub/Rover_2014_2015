/*
 * Arm.h
 *
 * Created: 1/23/2015 7:20:11 PM
 *  Author: nrpic_000
 */ 


#ifndef ARM_H_
#define ARM_H_

//Macros
#define MD1_STEP_SET(void) (PORTE.OUTSET = PIN4_bm);
#define MD1_STEP_CLR(void) (PORTE.OUTCLR = PIN4_bm);

#define MD1_DIR_SET(void) (PORTE.OUTSET = PIN7_bm);
#define MD1_DIR_CLR(void) (PORTE.OUTCLR = PIN7_bm);

#define MD1_nEN_SET(void) (PORTE.OUTSET = PIN5_bm);
#define MD1_nEN_CLR(void) (PORTE.OUTCLR = PIN5_bm);

//Arm Prototypes
void armGPIOInit(void);
void armInit();


#endif /* ARM_H_ */