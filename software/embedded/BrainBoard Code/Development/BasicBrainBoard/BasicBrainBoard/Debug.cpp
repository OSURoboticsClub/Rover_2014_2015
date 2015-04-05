/*
 * Debug.cpp
 *
 * Created: 3/5/2015 8:52:33 AM
 *  Author: N/A
 */ 

#include "Debug.h"

/* Debug message functions. When called, the resulting string
 * is sent up the wire to the control computer, where it will
 * (hopefully) be logged in a text file.
 * Arguments and functionality are the same as printf(). */
void debug_msg(const char *format, ...){
	//TODO
}

/* Debug function for use with format strings in FLASH memory.
 * See http://www.nongnu.org/avr-libc/user-manual/pgmspace.html
 * to learn why this is a good idea! */
void debug_msg_p(const char *progmem_format, ...){
	//TODO
}