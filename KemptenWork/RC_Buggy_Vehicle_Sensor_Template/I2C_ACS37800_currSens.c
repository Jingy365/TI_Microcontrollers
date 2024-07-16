/*
 *  I2C_ACS37800_currSens.c
 *
 *  Created on: 07.03.2023
 *      Author: stix
 */

/*** included files ****************************************************/
#include <I2C_ACS37800_currSens.h>


/*** functions *********************************************************/

power_t readPower(void){

    volatile int16_t pactive;
    volatile power_t power = {0};

    /*readI2C(i2c_address, 0x20, 4);
    // discard LS-bits and combine to 14 bit data
    power.voltage = (rxData_i2c[1] << 6) | (rxData_i2c[0] >> 2);
    power.current = (rxData_i2c[3] << 6) | (rxData_i2c[2] >> 2);*/

    // read vrmsavgonesec
    power.voltage = (uint16_t)read_current_sensor_ACS37800(0x26);

    // read pactavgonesec
    pactive = (uint16_t)read_current_sensor_ACS37800(0x28);

    //power.current = ((int32_t)pactive << 17)/power.voltage;
    power.current = ((int32_t)pactive << 15)/power.voltage;

    // discard LS-bits to 14 bit data
    power.voltage >>= 2;
    //power.current >>= 2;

    return power;
}


uint32_t read_current_sensor_ACS37800 (uint8_t reg_name){
    readI2C(CURR_SENSOR_ADDRESS, reg_name, 4);
    return ( (uint32_t)rxData_i2c[0] |
            ((uint32_t)rxData_i2c[1] << 8) |
            ((uint32_t)rxData_i2c[2] << 16) |
            ((uint32_t)rxData_i2c[3] << 24) );
}


void write_current_sensor_ACS37800 (uint8_t reg_name, uint32_t data){
    write32BitsI2C(CURR_SENSOR_ADDRESS, reg_name, data);
}


int setup_current_sensor_ACS37800(void){

    int errors = NO_ERROR;
    volatile uint32_t memoryDump[16] = {0};
    uint8_t reg_addr;
    int i;

    /* EEPROM Register: 0x0F, Shadow Register 0x1F
     * once EEPROM is set, writing to Shadow isn't necessary any more
     */
    uint32_t data = 0;

    __delay_cycles(160000);

    // enable  write access for EEPROM and Shadow Memory
    write_current_sensor_ACS37800(ACCESS_CODE_REG, 0x4F70656E);

    // select averaging mode
    data = read_current_sensor_ACS37800(0x1B);
    data = (data & ~IAVGSELEN) | IAVGSEL__VRMS;
    data = (data & ~PAVGSELEN) | PAVGSEL__PACT;
    write_current_sensor_ACS37800(0x1B, data);

    // set number of averages in stage 1
    data = read_current_sensor_ACS37800(0x1C);
    data = (data & ~RMS_AVG_1) | ((uint32_t)8 << RMS_AVG_1_SL);
    write_current_sensor_ACS37800(0x1C, data);

    // set number samples for RMS calculation
    data = read_current_sensor_ACS37800(0x1F);
    data = (data & ~BYPASS_N_EN) | BYPASS_N__MANUAL;
    data = (data & ~RMS_N) | ((uint32_t)4 << RMS_N_SL);
    write_current_sensor_ACS37800(0x1F, data);

    // lock EEPROM and Shadow Memory
    write_current_sensor_ACS37800(ACCESS_CODE_REG, 0x00000000);

    // Read all of EEPROM and Shadow Memory for manual check in debugger:
    i = 3;
    for (reg_addr = 0x0B; reg_addr <= 0x0F; reg_addr++){
        memoryDump[i++] = read_current_sensor_ACS37800(reg_addr);
    }
    i = 0x0b;
    for (reg_addr = 0x1B; reg_addr <= 0x1F; reg_addr++){
        memoryDump[i++] = read_current_sensor_ACS37800(reg_addr);
    }

    // stop here to inspect Shadow Memory and EEPROM
    // while(1);

    return errors;
}


#if defined(_MODIFY_ACS37800_ADDRESS_)
void setACS378address(uint8_t intended_address){
    // search current device address
    uint8_t addr = 0;
    uint8_t status;
    uint32_t data = 0;

    do {
       status = readI2C(addr++, 0x1F, 4);
    } while (status == I2C_ERR_NACK);
    addr--;

    readI2C(addr, 0x0F, 4);

    // enable EEPROM write access
    write32BitsI2C(addr, ACCESS_CODE_REG, 0x4F70656E);

    // Manual check: if write access is granted, bit0 will be set
    readI2C(addr, 0x30, 4);

    // rms calculation over fixed 1000 samples
    data |= (uint32_t)1 << I2C_BYPASS_N_EN;
    data |= (uint32_t)(1000 & 0x3FF) << I2C_N;

    // set intended address
    data |= (1 << I2C_DIS_SLV_ADDR);
    data |= ((intended_address & 0x7F) << I2C_SLV_ADDR);
    write32BitsI2C(addr, 0x0F, data);


    // Stop here. Reboot via power cycle necessary.
    // Undefine _MODIFY_ACS37800_ADDRESS_ and reprogram MSP430  now!
    while(1);
}
#endif



