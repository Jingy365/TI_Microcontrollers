/********************************************************************//**
 * @file LP_FR5994.h
 * Pin definitions for LaunchPad MSP-EXP430FR5994
 *
 * Board: MSP-EXP430FR5994
 * - Author: Stix
 * - Created: 04.04.2020 08:00
 * 29.11.2022: added KEY_S1, KEY_S2
 * 17.01.2023: added functions()
 ***********************************************************************/

#ifndef LP_FR5994_H_
#define LP_FR5994_H_

/*** included files ****************************************************/
#include <msp430.h>
#include <stdint.h>


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


/*** constants *********************************************************/
#define MCLK            16       // CPU clock in MHz
#define SMCLK           2       // Submodule Clock in MHz


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
 * Switches:
 * S1, Port 5.6
 * S2, Port 5.5
 * LED1, Port 1.0, LED_RED
 * LED2, Port 1.1, LED_GREEN
 * Port 5.2, adjacent 0 V output
 * Port 6.2, adjacent 3.3 V output
 * other pins configured as inputs with internal pull-down resistor
 ***********************************************************************/
void setup_IO(void);




#endif /* LP_FR5994_H_ */


