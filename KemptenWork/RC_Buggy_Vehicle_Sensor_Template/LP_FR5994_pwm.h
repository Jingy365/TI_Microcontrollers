/********************************************************************//**
 * @file LP_FR5994_pwm.h
 * PWM support for LaunchPad MSP-EXP430FR5994 LEDs
 * P1.0 (red) and P1.1 (green) using Timer A0
 *
 * Board: MSP-EXP430FR5994
 * - Author: Stix
 * - Created: 20.01.2023 08:00
 ***********************************************************************/

#ifndef PWM_OUTPUT_H_
#define PWM_OUTPUT_H_


/*** included files ****************************************************/
#include <LP_FR5994.h>
#include <msp430.h>
#include <stdint.h>


/*** constants *********************************************************/
#define PERIOD_LED      4095    // PWM count
#define PWM_SAFE        // select PWM_SAFE or PWM_FAST

#define OUTMOD__OUTPUT      OUTMOD_0
#define OUTMOD__RESET       OUTMOD_5
#define OUTMOD__RESET_SET   OUTMOD_7

/*** globals ***********************************************************/
int16_t pwmRed;
int16_t pwmGreen;


/********************************************************************//**
 * @brief setup_ledPWM
 * - initializes PWM on Timer A0 CCR1, Pin 1.0 (red LED)
 * - initializes PWM on Timer A0 CCR2, Pin 1.1 (green LED)
 ***********************************************************************/
void setup_ledPWM(void);




#endif /* PWM_OUTPUT_H_ */
