/*
 * Debug.cpp
 *
 * Created: 3/5/2015 8:52:33 AM
 *  Author: N/A
 */ 

#include "Debug.h"
#include <stdio.h>
#include <stdarg.h>

//TODO: Should the buffers be allocated
//      on the stack instead?
/* Debugging message output buffer. */
static char OutBuf[DEBUG_MAX_LEN];

/* debug_msg_p() format string buffer. */
static char FmtBuf[DEBUG_MAX_LEN];

/* Debug message functions. When called, the resulting string
 * is sent up the wire to the control computer, where it will
 * (hopefully) be logged in a text file.
 * Arguments and functionality are the same as printf(). */
void debug_msg(const char *format, ...){
	va_list args;
	va_start(args, format);
	vsnprintf(OutBuf, DEBUG_MAX_LEN, format, args);
	//TODO: send out string in proper format
	va_end(args);
}

/* Debug function for use with format strings in FLASH memory.
 * See http://www.nongnu.org/avr-libc/user-manual/pgmspace.html
 * to learn why this is a good idea! */
void debug_msg_p(const char *progmem_format, ...){
	for(int i=0; i<DEBUG_MAX_LEN; i++){
		FmtBuf[i] = pgm_read_byte(progmem_format + i);
		if('\0' == FmtBuf[i])break;
	}
	FmtBuf[DEBUG_MAX_LEN - 1] = '\0';
	va_list args;
	va_start(args, progmem_format);
	vsnprintf(OutBuf, DEBUG_MAX_LEN, FmtBuf, args);
	//TODO: send out string in proper format
	va_end(args);
}