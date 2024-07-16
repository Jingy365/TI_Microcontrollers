#include <msp430.h> 
#include <stdint.h>
#include "add2.h"  // "" for inside project

/**
 * main.c
 */


volatile int16_t res1;

void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	while(1){
	    res1 = add2(10, 5);

	}

}
