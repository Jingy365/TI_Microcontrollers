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
volatile int distance;
volatile int count = 0;
char dist_char[5];

void ultrasound_init(void){
    P1DIR |= BIT2 | BIT1; //TRIGGER P1.2
    P1SEL1 &= ~(BIT2 | BIT3);
    P1SEL0 |= BIT2 | BIT3;

    P1DIR &= ~BIT3; //ECHO P1.3
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

        TA1CCTL2 = CM__BOTH |
                SCS__SYNC |
                CAP__CAPTURE |
                CCIE_1 |
                CCIS__CCIA;
        TA1IV = TAIV__TACCR2;
}


void UART_setup(void){
    UCA1CTLW0 = UCSWRST |
            UCSSEL__SMCLK;
    UCA1BRW = 8;
    UCA1MCTLW = 0xD600;

    P2SEL0 &= ~BIT5;
    P2SEL1 |= BIT5;

    UCA1CTLW0 &= ~UCSWRST;

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
	UART_setup();
	__enable_interrupt();

	 while(1){
	     //time_diff = temp[1] - temp[0];

	     distance = time_diff/58;


	     //distance = (temp[1]*170/1000000);
	     //ltoa(distance, dist_char, 10);
	     //UCA1TXBUF = dist_char;
	        if(distance <= 100){
	                    P1OUT &= ~BIT1;
	                }
	                else{
	                    P1OUT |= BIT1;
	                }

            //__delay_cycles(50);

	 }
}

#pragma vector = TIMER1_A1_VECTOR ;
__interrupt void trigger_timer(void){
    temp[count] = TA1R;
    TA1CCTL2 &= ~CCIFG_1;
    count += 1;

    if(count==2){



        if(temp[1] < temp[0]){
            time_diff = temp[0] - temp[1];
        }
        else{
            time_diff = temp[1] - temp[0];
        }
        count=0;


        distance = time_diff/58; //should be 270cm ish
        //distance = (temp[1]*170/1000000);
        //ltoa(distance, dist_char, 10);
        //UCA1TXBUF = dist_char;

        if(distance <= 100){
            P1OUT &= ~BIT1;


        }
        else{
            P1OUT |= BIT1;

        }
       // __delay_cycles(5000); //5000

    }


}







