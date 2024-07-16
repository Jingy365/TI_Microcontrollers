/*
 *  I2C_ADXL_accSens.h
 *
 *  I2C routines for reading the accelerometer ADXL346
 *
 *  Created on: 17.01.2023
 *      Author: stix
 */


#ifndef I2C_ADXL346_ACCSENS_H_
#define I2C_ADXL346_ACCSENS_H_

/*** included files ****************************************************/
#include <LP_FR5994.h>     // Pin definitions for LaunchPad MSP-EXP430FR5994
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "LP_FR5994_i2c.h" // I2C routines for LaunchPad MSP-EXP430FR5994


/*** constants *********************************************************/
#define MCLK            16       // CPU clock in MHz
#define SMCLK           2       // Submodule Clock in MHz

#define ACC_SENSOR_ADDRESS 0x53
#define DEVID           0x00
#define THRESH_TAP      0x1D
#define OFSX            0x1E
#define OFSY            0x1F
#define OFSZ            0x20
#define DUR             0x21
#define LATENT          0x22
#define WINDOW          0x23
#define THRESH_ACT      0x24
#define THRESH_INACT    0x25
#define TIME_INACT      0x26
#define ACT_INACT_CTL   0x27
#define THRESH_FF       0x28
#define TIME_FF         0x29
#define TAP_AXES        0x2A
#define ACT_TAP_STATUS  0x2B
#define BW_RATE         0x2C
#define POWER_CTL       0x2D
#define     LINK            0x20
#define     AUTO_SLEEP      0x10
#define     MEASURE         0x08
#define     SLEEP           0x04
#define     WAKEUP          0x03
#define INT_ENABLE      0x2E
#define     DATA_READY      0x80
#define     SINGLE_TAP      0x40
#define     DOUBLE_TAP      0x20
#define     ACTIVITY        0x10
#define     INACTIVITY      0x08
#define     FREE_FALL       0x04
#define     WATERMARK       0x02
#define     OVR_ORIENT      0x01
#define INT_MAP         0x2F
#define INT_SOURCE      0x30
#define DATA_FORMAT     0x31
#define DATAX0          0x32
#define DATAX1          0x33
#define DATAY0          0x34
#define DATAY1          0x35
#define DATAZ0          0x36
#define DATAZ1          0x37
#define FIFO_CTL        0x38
#define FIFO_STATUS     0x39
#define TAP_SIGN        0x3A
#define ORIENT_CONF     0x3B
#define ORIENT          0x3C

#define NO_ERROR        0
#define ADXL346_ERROR   1
#define ERROR_ADXL346   1


/*** data types ********************************************************/
typedef  struct accData_t{
        int16_t x;
        int16_t y;
        int16_t z;
        int16_t angX;
        int16_t angY;
        int16_t angZ;
}accData_t;


/*** function declarations *********************************************/

/********************************************************************//**
 * @brief readAcceleration
 * - reads 3-axis acceleration data from ADXL346 I2C device
 *
 * i2c_address:     I2C address of ADXL346 sensor
 * returns 14 bit x, y, z acceleration data
 ***********************************************************************/
accData_t readAcceleration(uint8_t i2c_address);


/********************************************************************//**
 * @brief setup_acceleration_sensor_ADXL346
 * - setup procedure for ADXL346 acceleration sensor
 * @return error detection
 ***********************************************************************/
int setup_acceleration_sensor_ADXL346(void);


#endif /* I2C_ADXL346_ACCSENS_H_ */
