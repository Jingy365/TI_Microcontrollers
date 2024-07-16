/*
 *  LP_FR5994_i2c.h
 *
 *  I2C routines for LaunchPad MSP-EXP430FR5994
 *
 *  Created on: 07.03.2023
 *      Author: stix
 */


#ifndef LP_FR5994_I2C_H_
#define LP_FR5994_I2C_H_

/*** included files ****************************************************/
#include <LP_FR5994.h>     // Pin definitions for LaunchPad MSP-EXP430FR5994
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>


/*** constants *********************************************************/
#define MCLK            16       // CPU clock in MHz
#define SMCLK           2       // Submodule Clock in MHz

#define I2C_DATA        BIT0
#define I2C_CLK         BIT1
#define I2C_SPEED       400     // I2C speed in kHz

#define I2C_IS_IDLE     0
#define I2C_IS_BUSY     1
#define I2C_ERR_NACK    0xFF    // not acknowledge
#define I2C_ERR_TIMEOUT 0xFE    // SCL low time out
#define I2C_ERR_RX_TOUT 0xFD    // Time out in RX routine
#define I2C_ERR_TX_TOUT 0xFC    // Time out in TX routine
#define I2C_MAX_WAIT    5000    // Time out counter Start value,
                                // empirically determined

#define MAX_SIZE_RX 8
#define MAX_SIZE_TX 8



/*** globals ***********************************************************/
uint8_t rxData_i2c[MAX_SIZE_RX];
uint8_t rx_index_i2c;
uint8_t txData_i2c[MAX_SIZE_TX];
uint8_t tx_index_i2c;
uint8_t i2c_byteCount;

// global Flags:
extern uint8_t isRxComplete;


/*** function declarations *********************************************/

/********************************************************************//**
 * @brief readI2C
 * - reads up to 265 bytes from I2C device
 *
 * device_address:  I2C address of device
 * reg_name:        device register to start reading
 * size:            number of bytes to read
 * @return error detection
 ***********************************************************************/
int readI2C(uint8_t device_address, uint8_t reg_name, uint8_t readSize);


/********************************************************************//**
 * @brief setup_I2C
 * - prepares UCB2 for I2C
 ***********************************************************************/
void setup_I2C(void);


/********************************************************************//**
 * @brief write32BitsI2C
 * - writes 4 bytes to I2C device address
 *
 * device_address:  I2C address of device
 * reg_name:        device register to write to
 * data:            value to be written
 * @return error detection
 ***********************************************************************/
int write32BitsI2C(uint8_t device_address, uint8_t reg_name, uint32_t data);


/********************************************************************//**
 * @brief writeByteI2C
 * - writes a single byte to I2C device address
 *
 * device_address:  I2C address of device
 * reg_name:        device register to write to
 * data:            value to be written
 * @return error detection
 ***********************************************************************/
int writeByteI2C(uint8_t device_address, uint8_t reg_name, uint8_t data);


#endif /* LP_FR5994_I2C_H_ */
