/*
 * XMegaLib.cpp
 *
 * Created: 12/3/2014 5:46:14 PM
 *  Author: OSURC2
 */ 

#include "XMegaLib.h"
#include <avr/io.h>

//This function handles making colors on the RGB LED
void RGBSetColor(RGBColors choice){
	switch(choice){
		case RED:
			TCC0.CCA = 0;
			TCC0.CCB = COLOR_ON;
			TCC1.CCA = 0;
			break;
		case BLUE:
			TCC0.CCA = COLOR_ON;
			TCC0.CCB = 0;
			TCC1.CCA = 0;
			break;
		case GREEN:
			TCC0.CCA = 0;
			TCC0.CCB = 0;
			TCC1.CCA = COLOR_ON;
			break;
		case PURPLE:
			TCC0.CCA = COLOR_ON;
			TCC0.CCB = COLOR_ON;
			TCC1.CCA = 0;
			break;
		case YELLOW:
			TCC0.CCA = 0;
			TCC0.CCB = COLOR_ON;
			TCC1.CCA = COLOR_ON;
			break;
		case WHITE:
			TCC0.CCA = COLOR_ON;
			TCC0.CCB = COLOR_ON;
			TCC1.CCA = COLOR_ON;
			break;
		case ORANGE:
			TCC0.CCB = COLOR_ON;      //Red
			TCC1.CCA = COLOR_ON / 2;  //Green
			TCC0.CCA = 0;             //Blue
			break;
	}
	
}