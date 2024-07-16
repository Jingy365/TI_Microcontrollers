/********************************************************************//**
 * @file main.c
 *
 * RC-BUGGY HANDHELD MODULE
 * -------------------------
 *
 * 2x Potentiometer input --> XBee UART output
 * - Potentiometer @ P1.3 (0 .. 3.3V) for left .. right
 * - Potentiometer @ P1.4 (0 .. 3.3V) for forward .. backward
 * - Green LED P1.1 outputs PWM L-R
 * - Reed LED P1.0 outputs PWM F-B
 * - 6 Byte UART output @ P6.0 every x ms: 2x 12 bit + crc checksum
 *
 * Board: MSP-EXP430FR5994
 * - Author: Stix
 * - Created: 14.11.2022 08:00
 * - 20.01.2023: fixed LED PWM routine flicker
 ***********************************************************************/

/*** included files ****************************************************/
#include <msp430.h>
#include <RC-buggy_sender.h>


/*** function declarations *********************************************/
void setup_ADC(void);



/********************************************************************//**
 * @brief main function
 * - inits hardware
 * - outputs ADC to UART
 ***********************************************************************/
int main(void)
{
    setup_hardware();
    setup_ADC();

    __enable_interrupt();

    while(1)
    {
        sendMessage(ADC12MEM0, ADC12MEM1);
       // sendMessage(0, 4000);
    }
}



/********************************************************************//**
 * @brief setup_ADC
 ***********************************************************************/
void setup_ADC(void){
    //write the code for the initialization of the ADC below
    P1SEL0 |= BIT3 | BIT4; //Setting P1.3 and P1.4 as analogue outputs
    P1SEL1 |= BIT3 | BIT4;
    //PM5CTL0 &= ~LOCKLPM5;

    ADC12MCTL0 = ADC12WINC_0 | //
            ADC12DIF_0 |
            ADC12VRSEL_0 |
            ADC12EOS_0 |
            ADC12INCH_3;

    ADC12MCTL1 = ADC12WINC_0 |
                ADC12DIF_0 |
                ADC12VRSEL_0 |
                ADC12EOS_1 |
                ADC12INCH_4;

    ADC12CTL1 = ADC12PDIV__1 |
            ADC12SHS_0 |
            ADC12DIV_0 |
            ADC12SSEL_0 |
            ADC12CONSEQ_3 |
            ADC12SHP_1;

    ADC12CTL2 = ADC12RES__12BIT |
            ADC12DF_0 |
            ADC12PWRMD_0;

    ADC12CTL3 = ADC12CSTARTADD__ADC12MEM0;

    ADC12CTL0 = ADC12SHT0_10 |
                ADC12MSC_1 |
                ADC12ON_1 |
                ADC12ENC_1 |
                ADC12SC_1;  //?



}

/*
#pragma vector = TIMER1_A1_VECTOR
__interrupt void ISR_update_controlPWM(void){
}
*/



