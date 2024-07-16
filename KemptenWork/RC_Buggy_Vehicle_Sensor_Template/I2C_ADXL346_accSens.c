/*
 *  I2C_ADXL_accSens.c
 *
 *  Created on: 17.01.2023
 *      Author: stix
 */

/*** included files ****************************************************/
#include <I2C_ADXL346_accSens.h>


/*** functions *********************************************************/

accData_t readAcceleration(uint8_t i2c_address){

    accData_t acceleration = {0};

    if (readI2C(i2c_address, INT_SOURCE, 1)){
        return acceleration; //todo: acceleration.error handling
    }


    if (rxData_i2c[0] & DATA_READY){
        readI2C(i2c_address, DATAX0, 6);
        acceleration.x = (((int16_t)rxData_i2c[1] << 8) | (rxData_i2c[0] >> 0));
        acceleration.y = (((int16_t)rxData_i2c[3] << 8) | (rxData_i2c[2] >> 0));
        acceleration.z = (((int16_t)rxData_i2c[5] << 8) | (rxData_i2c[4] >> 0));
    }
    return acceleration;
}


int setup_acceleration_sensor_ADXL346(void){

    int errors = NO_ERROR;

    __delay_cycles(160000);
    // test device ID
    if(errors = readI2C(ACC_SENSOR_ADDRESS, DEVID, 1))
        return errors;
    if (rxData_i2c[0] != 0xE6)
        return ERROR_ADXL346;
    // measure mode
    errors = writeByteI2C(ACC_SENSOR_ADDRESS, POWER_CTL, (MEASURE));
    return errors;
}


