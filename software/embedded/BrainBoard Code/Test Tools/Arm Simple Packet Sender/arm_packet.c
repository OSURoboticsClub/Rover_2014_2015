/* Arm board simple packet sender.
 * Sends packets telling the arm board to do things to stdout.
 * Pipe into a serial port. */
#include <stdio.h>
#include <unistd.h>

int main(){
	while(1){
		putc('P', stderr);
		usleep(1000000);
		putchar(0x255);
		putchar(0x04); /* Command */
		putchar(0x00);
		putchar(0x00);
		putchar(0x00);
		putchar(0x00);
		putchar(0x00);
		putchar(0x255);
		fflush(stdout);
		fflush(stderr);
	}
}
		
