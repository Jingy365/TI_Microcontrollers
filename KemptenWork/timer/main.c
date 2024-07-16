//=====================================================
//  Autor:                Name
//  Erstellungsdatum:     dd.mm.yyyy
//  Beschreibung:         4. Labor zur Vorlesung Embedded Systems
//  Datei:                main.c - Digitale I/O und Interrupts
//=====================================================

#include <msp430.h>





void DoSomething(void){
  __delay_cycles(250000UL);
}

void sysInit(void) {
    CSCTL0 = CSKEY;                     //unlock CS-registers through the password
    CSCTL1 = DCORSEL + DCOFSEL_3;       //run DCO at 8 MHz
    CSCTL3 = DIVM__32 + DIVS_0;         //run CPU-Clock (MCLK) at 8 MHz / 32 = 250 kHz and SMCLK at 8 MHz
    CSCTL0_H = 0;                       //re-lock CS-registers
}






int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;           //CODE 1

    //sysInit();

                                        //CODE 2
    P5DIR = ~(BIT5 + BIT6);
    P5REN = BIT5 + BIT6;
    P5OUT = BIT5 + BIT6;

                                        //CODE 3
    TA0CTL = TASSEL__SMCLK |
             ID__2 |
             MC__UP |
             TACLR |
             TAIE_1 |
             TAIFG_0;
                                        //CODE 4
    TA0EX0 = TAIDEX__8;
    TA0CCR0 = 62500;
    P1DIR = 0xFF;
    P1OUT = 0;


    PM5CTL0 &= ~LOCKLPM5;               //Enable digital IO

    P1OUT |= BIT1;                                    //CODE 5
    P5IES = BIT5 | BIT6;
 //   P5IE = BIT6 | BIT5;                                  //CODE 6
    P5IFG = 0;
    __enable_interrupt();
     while(1){

                                        //CODE 7
//        DoSomething();
//        P1OUT ^= BIT1;
        P5IES = BIT5 | BIT6;

                                        //CODE 8

/*
         if (P5IFG & BIT6){
           P5IFG &= ~BIT6;
           P1OUT |= BIT0;
         }
         if (P5IFG & BIT5){
           P1OUT &= ~BIT0;
           P5IFG &= ~BIT5;
         }*/



                                        //CODE 9
    };



}


#pragma vector = TIMER0_A1_VECTOR
__interrupt void timerA0(void){
    TA0CTL &= ~TAIFG_1;
    P1OUT ^= BIT1;
    P1OUT ^= BIT0;
}
