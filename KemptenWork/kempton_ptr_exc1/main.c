#include <msp430.h> 
#include <stdint.h>

/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    int8_t *ptr = (int8_t*)0x204;
    *ptr = 0xFF;
    ptr = (int8_t*)0x202;
    *ptr = 0x01;

    int16_t *ptr1 = (int16_t*)0x1C01;
    *ptr1 = 0xABCD;

	
	return 0;
}
