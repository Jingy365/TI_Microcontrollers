#include <msp430.h> 


/**
 * main.c
 */
void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	PM5CTL0 &= ~LOCKLPM5;               //Enable digital IO
	P1DIR |= BIT0;
	P1SEL1 &= ~BIT0;
	P1SEL0 |= BIT0;

	TA0CTL = TASSEL__SMCLK |
	        ID__1 |
	        MC__UP|
	        TACLR |
	        TAIE_0;

	TA0EX0 = TAIDEX__1;
	TA0CCR0 = 1000;
	TA0CCTL1 = CAP__COMPARE |
	        OUTMOD_7 |
	        CCIE_0;
	TA0CCR1 = 250;


	while(1){

	}


	

}
