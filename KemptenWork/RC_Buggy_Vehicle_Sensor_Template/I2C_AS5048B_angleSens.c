/*
 *  I2C_AS5048B_angleSens.c
 *
 *  Created on: 17.04.2023
 *      Author: stix
 */

/*** included files ****************************************************/
#include <I2C_AS5048B_angleSens.h>


/*** functions *********************************************************/

int16_t readAngle(uint8_t i2c_address){

    int16_t angle = 0;

    readI2C(i2c_address, 0xFE, 2);
    // combine to 14 bit data
    angle = (rxData_i2c[1] << 6) | (rxData_i2c[0] & 0x3F);
    // inversion
    angle -= 8192;
    angle = -angle;
    //angle += 8192;

    return angle;
}


uint16_t readMagnitude(uint8_t i2c_address){

    uint16_t magnitude = 0;

    readI2C(i2c_address, 0xFC, 2);
    // combine to 14 bit data
    magnitude = (rxData_i2c[1] << 6) | (rxData_i2c[0] & 0x3F);

    return magnitude;
}


int setup_angle_sensor_AS5048B(void){

    int errors = NO_ERROR;

    // No initialization necessary
    return errors;
}




