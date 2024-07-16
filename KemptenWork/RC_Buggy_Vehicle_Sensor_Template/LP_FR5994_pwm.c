/*
 * pwm_output.c
 *
 *  Created on: 19.01.2023
 *      Author: stix
 */

#include <LP_FR5994_pwm.h>


void setup_ledPWM(void){
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

    // bind output pins
    P1SEL0 |= LED_GREEN | LED_RED;
    P1SEL1 &= ~( LED_GREEN | LED_RED);

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

