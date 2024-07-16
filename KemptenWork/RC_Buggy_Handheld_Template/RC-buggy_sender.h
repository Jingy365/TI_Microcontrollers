/*
 * RC-buggy_sender.h
 *
 *  Created on: Nov 23, 2022
 *      Author: stix
 */

#ifndef RC_BUGGY_SENDER_H_
#define RC_BUGGY_SENDER_H_


/*** included files ****************************************************/
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "LP_FR5994_io.h"   // Pin definitions for LaunchPad MSP-EXP430FR5994
#include "steeringTable.h"  // steering adjustment lookup table


/*** constants *********************************************************/
#define MCLK            8       // CPU clock in MHz
#define SMCLK           2       // Submodule Clock in MHz

/*** choose XBee module ************************************************/
#define _XBEE_SX868_ // (XBee S2C works fine when SX868 is defined)
#if defined(_XBEE_S2C_)
    #define CYCLIC_SEND_MILLIS 15
#elif defined(_XBEE_SX868_)
    #define CYCLIC_SEND_MILLIS 30
#else
    #error **** xbee module not defined ****
#endif


/*** function declarations *********************************************/


/********************************************************************//**
 * @brief define_Modes()
 * - reads input keys
 * - selects slow mode, test mode or normal mode
 ***********************************************************************/
void define_Modes(uint8_t* isSlowMode_p);


/*********************************************************************//**
* @brief pack2x7bit()
* - pack a 14 bit value in two 7-bit bytes,
* - H-bits are always low
*
* in: hex14bit
* out: 2 bytes starting at address packed
************************************************************************/
void pack2x7bit(uint8_t* packed, uint16_t hex14bit);


/********************************************************************//**
 * @brief sendMessage()
 * - reads joystick inputs
 * - corrects inputs for joystick center position
 * - in Slow Mode: reduces possible drive speed
 *
 * without steering gear: joystick and servo in same direction
 * RC-Buggy: joystick and servo direction inverted
 ***********************************************************************/
void sendMessage(uint16_t steeringCenter, uint16_t driveCenter);


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
 * @brief setup_hardware()
 * - calls subfunctions to setup sender hardware
 *   except setup_ADC() which is done in main() as an exercise
 *
 * initializes hardware
 * - input keys S1 and S2
 * - PWM output to LEDs
 * - UART output mode to TX module
 * - cyclic timer interrupt for UART
 ***********************************************************************/
void setup_hardware(void);


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


/********************************************************************//**
 * @brief setup_PWM
 * - initializes PWM on Timer A0 CCR1, Pin 1.0 (red LED)
 * - initializes PWM on Timer A0 CCR2, Pin 1.1 (green LED)
 *
 * prepare both: compare interrupt and overflow interrupt
 * overflow interrupt is used at low brightness of LED
 ***********************************************************************/
void setup_ledPWM(void);


/********************************************************************//**
 * @brief setup_triggerTimer
 * prepares TimerA1 to
 * - trigger TX ISR on overflow
 ***********************************************************************/
void setup_triggerTimer(void);


/********************************************************************//**
 * @brief setup_UART
 * - prepares UART (115200 baud, 8N1)
 * - initializes pins: P6.0 as UART TX, P6.1 as UART RX
 ***********************************************************************/
void setup_UART(void);


/********************************************************************//**
 * @brief trim_Poti
 * - corrects ADC value according to calibrated center position
 ***********************************************************************/
uint16_t trim_Poti(uint16_t adc, uint16_t adcZero);


/********************************************************************//**
 * @brief testMode
 * - two different Test modes for range and reliability tests
 * - sinus mode outputs sin signal on steering servo
 * - fixed1000 mode outputs fixed angle on steering servo
 * - drive stays always centered (stopped)
 *
 * S1 during sinus test mode enables fixed 1000 test mode
 * S2 during fixed 1000 test mode enables sinus test mode
 ***********************************************************************/
void testMode(void);








#endif /* RC_BUGGY_SENDER_H_ */
