/*
 *  I2C_AS5048B_angleSens.h
 *
 *  I2C routines for reading the angle sensor AS5048B
 *
 *  Created on: 07.03.2023
 *      Author: stix
 */


#ifndef I2C_AS5048B_ANGLESENS_H_
#define I2C_AS5048B_ANGLESENS_H_

/*** included files ****************************************************/
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "LP_FR5994_io.h"  // Pin definitions for LaunchPad MSP-EXP430FR5994
#include "LP_FR5994_i2c.h" // I2C routines for LaunchPad MSP-EXP430FR5994


/*** constants *********************************************************/
#define MCLK            16       // CPU clock in MHz
#define SMCLK           2       // Submodule Clock in MHz

#define ANGLE_SENSOR_ADDRESS 0x40

#define NO_ERROR        0


/*** function declarations *********************************************/

/********************************************************************//**
 * @brief readAngle
 * - reads angle from AS5048B device
 *
 * i2c_address:     I2C address of AS5048B sensor
 * returns 14 bit angle
 ***********************************************************************/
int16_t readAngle(uint8_t i2c_address);


/********************************************************************//**
 * @brief readMagnitude
 * - reads Magnitude from AS5048B device
 *
 * i2c_address:     I2C address of AS5048B sensor
 * returns 14 bit Magnitude
 ***********************************************************************/
uint16_t readMagnitude(uint8_t i2c_address);



/********************************************************************//**
 * @brief setup_angle_sensor_AS5048B
 * - setup procedure for AS5048B angle sensor
 * @return error detection
 ***********************************************************************/
int setup_angle_sensor_AS5048B(void);


#endif /* I2C_AS5048B_ANGLESENS_H_ */
