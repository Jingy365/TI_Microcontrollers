/*
 * LP_FR5994_io.c
 *
 *  Created on: 27.03.2023
 *      Author: stix
 */


#include "LP_FR5994_io.h"


void setup_io(int qty, const char* argN, ...) {
    va_list args_ptr;
    int usePWM = false;

    /**** All pins input, pull-down ****/
    // By default all pins are inputs, prepared for internal pull-down
    // but resistors are not enabled
    P1REN = 0xFF;    // enable all internal pull-down resistors
    P2REN = 0xFF;
    P3REN = 0xFF;
    P4REN = 0xFF;
    P5REN = 0xFF;
    P6REN = 0xFF;
    P7REN = 0xFF;
    P8REN = 0xFF;

    va_start(args_ptr, argN);
    while (qty--) {
        if (strcmp(argN, "LED_RED") == 0) {
            /**** LEDs: output, OFF ****/
            P1OUT &= ~LED_RED;
            P1DIR |= LED_RED;
        }
        else if (strcmp(argN, "LED_GREEN") == 0) {
            /**** LEDs: output, OFF ****/
            P1OUT &= ~LED_GREEN;
            P1DIR |= LED_GREEN;
        }
        else if (strcmp(argN, "KEY_S1") == 0) {
            /**** Pushbutton switches: input, pull-up ****/
            P5OUT |= KEY_S1;
        }
        else if (strcmp(argN, "KEY_S2") == 0) {
            /**** Pushbutton switches: input, pull-up ****/
            P5OUT |= KEY_S2;
        }
        else if (strcmp(argN, "PWM_RED") == 0) {
            /**** LED: output, OFF ****/
            P1OUT &= ~LED_RED;
            P1DIR |= LED_RED;
            // bind output pin
            P1SEL0 |= LED_RED;
            P1SEL1 &= ~LED_RED;
            usePWM = true;
        }
        else if (strcmp(argN, "PWM_GREEN") == 0) {
            /**** LEDs: output, OFF ****/
            P1OUT &= ~LED_GREEN;
            P1DIR |= LED_GREEN;
            // bind output pin
            P1SEL0 |= LED_GREEN;
            P1SEL1 &= ~LED_GREEN;
            usePWM = true;
        }
        argN = va_arg(args_ptr, char*);
    }
    va_end(args);

    // configure Timer A0
    if(usePWM){
        TA0EX0 = TAIDEX__1;           // Prescaler: 1:1

        TA0CTL =
                TASSEL__SMCLK |       // Count SMCLK
                ID__1 |               // Prescaler 1:1
                MC__UP;               // Start Counter


        TA0CCTL1 =
                CAP__COMPARE |        // Compare Mode
                OUTMOD__RESET_SET |   // reset @ compare, set @ CCR0
                CCIE_1 |              // interrupt enabled
                OUT__HIGH;            // prepare manual output
        TA0CCTL2 =
                CAP__COMPARE |        // Compare Mode
                OUTMOD__RESET_SET |   // reset @ compare, set @ CCR0
                CCIE_1 |              // interrupt enabled
                OUT__HIGH;            // prepare manual output

        // Total prescaler = 1:1
        TA0CCR0 = PERIOD_LED;         // set PERIOD
    }

    #ifdef DEBUG_MODE
    P8DIR |= DEBUG; // DEBUG_PIN
    P8DIR |= DEBUG2; // DEBUG_PIN2
    #endif

    PM5CTL0 &= ~LOCKLPM5;   // unlock IO configuration
}


void setSYSCLK(uint8_t mCLK, uint8_t smCLK) {

    CSCTL0 = CSKEY;                     //unlock CS-registers through the password

    switch (mCLK){
    case 1:
        CSCTL1 = DCORSEL | DCOFSEL_0;       //run DCO at 1 MHz
        CSCTL3 = DIVM__1;                   //run CPU-Clock (MCLK) at 1 MHz
        switch(smCLK){
        case 1:
            CSCTL3 |= DIVS__1;              //run SMCLK at 1:1 = 1 MHz
            break;
        default:
            while(1);   // SMCLK undefined
        }
        break;

    case 8:
        CSCTL1 = DCORSEL | DCOFSEL_3;       //run DCO at 8 MHz
        CSCTL3 = DIVM__1;                   //run CPU-Clock (MCLK) at 8 MHz
        switch(smCLK){
        case 2:
            CSCTL3 |= DIVS__4;              //run SMCLK at 8:4 = 2 MHz
            break;
        case 8:
            CSCTL3 |= DIVS__1;              //run SMCLK at 8:1 = 8 MHz
            break;
        default:
            while(1);   // SMCLK undefined
        }
        break;

    case 16:
        FRCTL0 = FWPW | NWAITS_1;           //unlock and set FRAM Wait states
        CSCTL1 = DCORSEL | DCOFSEL_4;       //run DCO at 16 MHz
        CSCTL3 = DIVM__1;                   //run CPU-Clock (MCLK) at 16 MHz
        switch(smCLK){
        case 2:
            CSCTL3 |= DIVS__8;              //run SMCLK at 16:8 = 2 MHz
            break;
        case 8:
            CSCTL3 |= DIVS__2;              //run SMCLK at 16:2 = 8 MHz
            break;
        default:
            while(1);   // SMCLK undefined
        }
        break;

    default:
        while(1);   //MCLK undefined
    }

     CSCTL0_H = 0;                       //re-lock CS-registers
}



/*** Interrupt Handlers ************************************************/

/********************************************************************//**
 *   @brief Timer 0 ISR (CCR1 and CCR2)
 *   update PWM output
 *
 *   inputs: pwmRed, pwmGreen
 *
 *   PWM_FAST:
 *   - no boundary limit checks
 *   - inputs > PERIOD will stop PWM
 *   - inputs < 0 will stop PWM (write to CCRx restarts)
 *   - input == PERIOD is still PWM output
 *   - runs 14..18 ticks faster than PWM_SAFE
 *
 *   PWM_SAFE:
 *   - limits inputs < 0 to zero
 *   - accepts inputs > PERIOD and outputs DC HIGH level
 *   - accepts inputs < 0 and  outputs DC LOW level
 ***********************************************************************/
#ifdef PWM_SAFE
#pragma vector = TIMER0_A1_VECTOR
__interrupt void ISR_update_ledPWM_overflow(void) {

    switch(__even_in_range(TA0IV, TAIV__TACCR2))
    {
        case TAIV__NONE:
            break;

        case TAIV__TACCR1:
            if (pwmRed > PERIOD_LED){
                // manually set out to HIGH
                TA0CCTL1 &= ~OUTMOD;
                TA0CCTL1 |= OUT__HIGH;
            }
            else{
                TA0CCTL1 |= OUTMOD__RESET_SET;
                TA0CCR1 = pwmRed < 0 ? 0 : pwmRed;
            }
            break;

        case TAIV__TACCR2:
            if (pwmGreen > PERIOD_LED){
                // manually set out to HIGH
                TA0CCTL2 &= ~OUTMOD;
                TA0CCTL2 |= OUT__HIGH;
            }
            else{
                TA0CCTL2 |= OUTMOD__RESET_SET;
                TA0CCR2 = pwmGreen < 0 ? 0 : pwmGreen;
            }
            break;

        default: __never_executed();
    }
}
#endif

#ifdef PWM_FAST
#pragma vector = TIMER0_A1_VECTOR
__interrupt void ISR_update_ledPWM_overflow(void) {

    switch(__even_in_range(TA0IV, TAIV__TACCR2))
    {
        case TAIV__NONE:
            break;

        case TAIV__TACCR1:
            TA0CCR1 = pwmRed;
            break;

        case TAIV__TACCR2:
            TA0CCR2 = pwmGreen;
            break;

        default: __never_executed();
    }
}
#endif
