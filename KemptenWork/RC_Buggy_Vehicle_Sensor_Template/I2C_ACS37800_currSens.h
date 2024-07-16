/*
 *  I2C_ADXL_accSens.h
 *
 *  I2C routines for reading the current sensor ACS37800
 *
 *  Created on: 07.03.2023
 *      Author: stix
 */


#ifndef I2C_ACS37800_CURRSENS_H_
#define I2C_ACS37800_CURRSENS_H_

/*** included files ****************************************************/
#include <LP_FR5994.h>     // Pin definitions for LaunchPad MSP-EXP430FR5994
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "LP_FR5994_i2c.h" // I2C routines for LaunchPad MSP-EXP430FR5994


/*** constants *********************************************************/
#define MCLK            16       // CPU clock in MHz
#define SMCLK           2       // Submodule Clock in MHz

#define CURR_SENSOR_ADDRESS 0x60
//#define _MODIFY_ACS37800_ADDRESS_  // Caution: do not define this!
// except for intended modification of device address
// or other contents of EEPROM

#define ACCESS_CODE_REG 0x2F

/*
// Register 0x0F/0x1F
#define I2C_BYPASS_N_EN    24
#define I2C_N              14
#define I2C_DIS_SLV_ADDR    9
#define I2C_SLV_ADDR        2
*/

#define NO_ERROR        0


/* Reg 0x0B/0x1B Control Bits */
#define QVO_FINE                    (0x000001FF) /* Offset adjustment for the current channel */
#define QVO_FINE_SL                           0  /* Left shift for number entries*/
#define SNS_FINE                    (0x0007FE00) /* Gain adjustment for the current channel */
#define SNS_FINE_SL                           9  /* Left shift for number entries*/
#define CRS_SNS                     (0x00380000) /* Coarse Gain adjustment for the current channel */
#define CRS_SNS_SL                           19  /* Left shift for number entries*/
#define IAVGSELEN                   (0x00400000) /* Current Averaging selection enable */
#define IAVGSEL__VRMS               (0x00000000) /* select vrms for averaging */
#define IAVGSEL__IRMS               (0x00400000) /* select irms for averaging */
#define PAVGSELEN                   (0x00800000) /* Power Averaging selection enable */
#define PAVGSEL__VRMS               (0x00000000) /* select vrms for averaging */
#define PAVGSEL__PACT               (0x00800000) /* select pactive for averaging */

/* Reg 0x0C/0x1C Control Bits */
#define RMS_AVG_1                   (0x0000007F) /* Average of the rms voltage or current – stage 1 */
#define RMS_AVG_1_SL                          0  /* Left shift for number entries*/
#define RMS_AVG_2                   (0x0001FF80) /* Average of the rms voltage or current – stage 2 */
#define RMS_AVG_2_SL                          7  /* Left shift for number entries*/
#define VCHAN_OFFSET_CODE           (0x01FE0000) /* Controls the room offset for the voltage channel */
#define VCHAN_OFFSET_CODE_SL                 17  /* Left shift for number entries*/

/* Reg 0x0D/0x1D Control Bits */ //to be done
/* Reg 0x0E/0x1E Control Bits */

/* Reg 0x0F/0x1F Control Bits */
#define I2C_SLV_ADDR                (0x000001FC) /* I2C slave address EEPROM value */
#define I2C_SLV_ADDR_SL                       2  /* Left shift for number entries*/
#define I2C_DIS_SLV_ADDR            (0x00000200) /* disable analog I2C slave address feature */
#define I2C_SLV_ADDR__ANALOG        (0x00000000) /* use address as set by DIO_1 and DIO_0 */
#define I2C_SLV_ADDR__EEPROM        (0x00000200) /* use address as set in EEPROM */
#define DIO_0_SEL                   (0x00000C00) /* Select output mode for DIO_0 */
#define DIO_0__ZC                   (0x00000000) /* Zero crossing */
#define DIO_0__OVRMS                (0x00000400) /* VRMS overvoltage flag */
#define DIO_0__UVRMS                (0x00000800) /* VRMS undervoltage flag */
#define DIO_0__OVORUV               (0x00000C00) /* VRMS overvoltage OR undervoltage flag */
#define DIO_1_SEL                   (0x00003000) /* Select output mode for DIO_1 */
#define DIO_1__OCF                  (0x00000000) /* Overcurrent fault */
#define DIO_1__UVRMS                (0x00001000) /* VRMS undervoltage flag */
#define DIO_1__OVRMS                (0x00002000) /* VRMS overvoltage flag */
#define DIO_1__OVORUVORFL           (0x00003000) /* VRMS overvoltage OR undervoltage OR latched overcurrent fault flag */
#define RMS_N                       (0x00FFC000) /* Number of samples for RMS calculations */
#define RMS_N_SL                             14  /* Left shift for number entries*/
#define BYPASS_N_EN                 (0x01000000) /* select how to define number of samples for RMS calculations */
#define BYPASS_N__AUTO              (0x00000000) /* define number of samples by voltage zero crossing */
#define BYPASS_N__MANUAL            (0x01000000) /* use manually defined number of samples */



/*** data types ********************************************************/
typedef  struct power_t{
        int16_t current;
        int16_t voltage;
}power_t;


/*** function declarations *********************************************/

/********************************************************************//**
 * @brief readRMS
 * - reads voltage and current data from ACS37800 device
 *
 * i2c_address:     I2C address of ACS37800 sensor
 * returns 14 bit voltage and current
 ***********************************************************************/
power_t readPower(void);


/********************************************************************//**
 * @brief setup_current_sensor_ACS37800
 * - setup procedure for ACS37800 electrical sensor
 * @return error detection
 ***********************************************************************/
int setup_current_sensor_ACS37800(void);


/********************************************************************//**
 * @brief read_current_sensor_ACS37800
 * - reads one register from ACS37800
 * @return 32 bit register data
 ***********************************************************************/
uint32_t read_current_sensor_ACS37800 (uint8_t reg_name);


/********************************************************************//**
 * @brief write_current_sensor_ACS37800
 * - writes one register to ACS37800
 ***********************************************************************/
void write_current_sensor_ACS37800 (uint8_t reg_name, uint32_t data);



#if defined(_MODIFY_ACS37800_ADDRESS_)
/********************************************************************//**
 * @brief setACS378address
 * - assign a device address to ACS37800
 * DO NOT KEEP THIS CODE IN PRODUCTION FIRMWARE
 * as the EEPROM access_code is included.
 * Limit bus access to a single ACS37800
 ***********************************************************************/
void setACS378address(uint8_t intended_address);
#endif

#endif /* I2C_ACS37800_CURRSENS_H_ */
