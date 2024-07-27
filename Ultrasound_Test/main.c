#include <msp430.h> 
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/**
 * main.c
 */

volatile int32_t temp[2];
volatile int64_t time_diff;
volatile float distance;
volatile int i = 0;
char dst_char[5];

void ultrasound_init(void){
    P1DIR |= BIT2 | BIT1; //TRIGGER P1.2
    P1SEL1 &= ~(BIT2 | BIT4);
    P1SEL0 |= BIT2 | BIT4;

    P1DIR &= ~BIT4; //ECHO P1.4
}


void SMCLK_init(void){
    CSCTL0 = CSKEY;
    CSCTL1 = DCOFSEL_0;
    CSCTL2 = SELS__DCOCLK; //1MHz
    CSCTL4 = SMCLKOFF_0;

}

void trigger_timer_init(void){
    TA1CTL = TASSEL__SMCLK | //SMCLK runs at 1MHz
            ID__1 |
            MC__CONTINUOUS |
            //MC__UP |
            TACLR |
            TAIE_0 |
            TAIFG_0 ;

    //TA1CCR0 = 64000;

    TA1EX0 = TAIDEX__1;

    TA1CCTL1 = OUTMOD_7 |
            CAP__COMPARE |
            CCIE_0;
    TA1CCR1 = 10; //reset to 0 when it reaches this value


}

void echo_timer_init(){
    TB0CTL = TBSSEL__SMCLK | //SMCLK runs at 1MHz
                ID__1 |
                MC__CONTINUOUS |
                //MC__UP |
                TBCLR |
                TBIE_0 |
                TBIFG_0 ;

        TB0EX0 = TBIDEX__1;

        //TB0CCR0 = 64000;


        TB0CCTL1 = CM__BOTH |
                SCS__SYNC |
                CAP__CAPTURE |
                CCIE_1 |
                CCIS__CCIA;
}

void main(void)
{

	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	PM5CTL0 &= ~LOCKLPM5;
	P1OUT |= BIT1;
	ultrasound_init();
	SMCLK_init();
	trigger_timer_init();
	echo_timer_init();
	__enable_interrupt();

	 while(1){
	     //time_diff = temp[1] - temp[0];

	     distance = time_diff/58;
	        if(distance <= 100){
	                    P1OUT &= ~BIT1;
	                }
	                else{
	                    P1OUT |= BIT1;
	                }
            //__delay_cycles(50);

	 }
}

#pragma vector = TIMER0_B1_VECTOR;
__interrupt void trigger_timer(void){
    temp[i] = TB0CCR1;
    TB0CCTL1 &= ~CCIFG_1;
    i += 1;

    if(i==2){



        if(temp[1] < temp[0]){
            time_diff = temp[0] - temp[1];
        }
        else{
            time_diff = temp[1] - temp[0];
        }
        i=0;


        distance = time_diff/58; //should be 270cm ish


        if(distance <= 100){
            P1OUT &= ~BIT1;


        }
        else{
            P1OUT |= BIT1;

        }
       // __delay_cycles(5000); //5000

    }


}







