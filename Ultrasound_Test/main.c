#include <msp430.h> 
#include <stdint.h>


/**
 * main.c
 */
void ultrasound_init(void){
    P3SEL0 |= BIT4;    //Capture compare signal (TB0CCI3A)
    P3SEL1 &= ~BIT4;
    P4DIR |= BIT4;     // Set P4.4 as output for trigger
    P4OUT &= ~BIT4;    // Set low output to P4.4
}

void TB0_init(void){
    TB0CTL = TBSSEL__SMCLK |
            ID__1 |

}


int main(void)
{

	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	PM5CTL0 &= ~LOCKLPM5;
	
	return 0;
}
