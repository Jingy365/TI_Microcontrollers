/********************************************************************//**
 * @file LP_FR5994_io.h
 *
 * Pin definitions and PWM support for LaunchPad MSP-EXP430FR5994
 * P1.0 (red) and P1.1 (green) using Timer A0
 *
 * Board: MSP-EXP430FR5994
 * - Author: Stix
 * - Created: 31.03.2023 11:15
 *
 * Usage:
 * - include LP_FR5994_io.h
 * - call setup_io() for initialization
 *   example: setup_io(3, "KEY_S1", "PWM_RED", "LED_GREEN");
 * - option: PWM_SAFE or PWM_FAST
 ***********************************************************************/

#ifndef LP_FR5994_IO_H_
#define LP_FR5994_IO_H_


/*** included files ****************************************************/
#include <msp430.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


/*** constants *********************************************************/
#define DEBUG_MODE

#define PERIOD_LED      4095    // PWM count
#define PWM_SAFE        // select PWM_SAFE or PWM_FAST

#define OUTMOD__OUTPUT      OUTMOD_0
#define OUTMOD__RESET       OUTMOD_5
#define OUTMOD__RESET_SET   OUTMOD_7

#ifndef MCLK
#define MCLK            16       // CPU clock in MHz
#endif

#ifndef SMCLK
#define SMCLK           2       // Submodule Clock in MHz
#endif

/*** pin functions *****************************************************/
#define KEY_S1              BIT6
#define S1_PRESSED          (!(P5IN & KEY_S1))
#define S1_RELEASED         (P5IN & KEY_S1)
#define KEY_S2              BIT5
#define S2_PRESSED          (!(P5IN & KEY_S2))
#define S2_RELEASED         (P5IN & KEY_S2)

#define LED_RED             BIT0
#define LED_RED_ON          (P1OUT |= LED_RED)
#define LED_RED_OFF         (P1OUT &= ~LED_RED)
#define LED_RED_TOGGLE      (P1OUT ^= LED_RED)

#define LED_GREEN           BIT1
#define LED_GREEN_ON        (P1OUT |= LED_GREEN)
#define LED_GREEN_OFF       (P1OUT &= ~LED_GREEN)
#define LED_GREEN_TOGGLE    (P1OUT ^= LED_GREEN)

// Debug pin (P8.1)
#define DEBUG               BIT1
#define DEBUG_ON            (P8OUT |= DEBUG)
#define DEBUG_OFF           (P8OUT &= ~DEBUG)
#define DEBUG_TOGGLE        (P8OUT ^= DEBUG)

// Debug pin (P8.2)
#define DEBUG2              BIT2
#define DEBUG2_ON           (P8OUT |= DEBUG2)
#define DEBUG2_OFF          (P8OUT &= ~DEBUG2)
#define DEBUG2_TOGGLE       (P8OUT ^= DEBUG2)


/*** globals ***********************************************************/
int16_t pwmRed;
int16_t pwmGreen;


/*** function declarations *********************************************/

/********************************************************************//**
 * @brief setSYSCLK
 * - sets CPU speed
 *
 * possible parameters:
 * mCLK in MHz: 1 MHz, 8 MHz, 16 MHz
 * smCLK in MHZ: 1 MHz, 2 MHz, 8 MHz
 ***********************************************************************/
void setSYSCLK(uint8_t mCLK, uint8_t smCLK);


/********************************************************************//**
 * @brief setup_IO
 * - initializes standard pushbuttons and LED outputs for
 *   the FR5994 LaunchPad
 *
 * Parameter:
 * qty: number of parameter strings, 1 .. 4
 * "KEY_S1"    --> S1, Port 5.6
 * "KEY_S2"    --> S2, Port 5.5
 * "LED_RED"   --> LED1, Port 1.0, (macro LED_RED)
 * "LED_GREEN" --> LED2, Port 1.1 (macro LED_GREEN)
 * "PWM_RED"   --> PWM on Timer A0 CCR1, Pin 1.0 (global pwmRed)
 * "PWM_GREEN" --> PWM on Timer A0 CCR2, Pin 1.1  (global pwmGreen)
 * other pins will be configured as inputs with internal pull-down resistor
 * PWM:
 * - 12 bit globals pwmRed & pwmGreen:
 *         0 -->   0 % duty cycle
 *      4095 --> 100 %
 * - PWM frequency: SMCLK / 4096 = 488 Hz @ 2 MHz SMCLK
 ***********************************************************************/
void setup_io(int qty, const char* argN, ...);


#endif /* LP_FR5994_IO_H_ */
