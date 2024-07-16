/*
 * RC-buggy_rpmSens.h
 *
 *  Created on: 01.03.2023
 *      Author: Stix
 */

#ifndef RC_BUGGY_RPMSENS_H_
#define RC_BUGGY_RPMSENS_H_

/*** included files ****************************************************/
#include <LP_FR5994.h> //Pin definitions for LaunchPad MSP-EXP430FR5994
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>


/*** data types ********************************************************/
typedef enum {FRONT_L, FRONT_R, REAR_L, REAR_R} wheel_t;


/*** globals ***********************************************************/
extern volatile int32_t ticksCCR3;
extern volatile int32_t ticksCCR4;
extern volatile int32_t ticksCCR5;
extern volatile int32_t ticksCCR6;


/********************************************************************//**
 * @brief setup_capture
 * - prepares capture of speed signals
 *
 ***********************************************************************/
void setup_capture(void);


/********************************************************************//**
 * @brief calcRPM
 * - calculates RPM from ticks
 * - reads direction bit
 * - valid outputs (14 bit): -8192 .. 0 .. 8191
 ***********************************************************************/
int16_t calcRPM(wheel_t wheel);


/********************************************************************//**
 * @brief lowpassFilter
 * - very simple integer low pass filter
 ***********************************************************************/
uint32_t lowpassFilter(uint32_t input, uint32_t* lastOutput_p);



#endif /* RC_BUGGY_RPMSENS_H_ */
