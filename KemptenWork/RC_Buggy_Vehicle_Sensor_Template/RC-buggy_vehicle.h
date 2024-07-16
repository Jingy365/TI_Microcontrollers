/*
 * RC-buggy_receiver.h
 *
 *  Created on: Nov 23, 2022
 *      Author: user
 */

#ifndef RC_BUGGY_VEHICLE_H_
#define RC_BUGGY_VEHICLE_H_


/*** included files ****************************************************/
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "LP_FR5994_io.h" //Pin definitions for LaunchPad MSP-EXP430FR5994
#include "LP_FR5994_spi.h" //Used to send to Nucleo board
#include "DMA_SA_DA.h"
#include <I2C_ADXL346_accSens.h>
#include <I2C_ACS37800_currSens.h>
#include "I2C_AS5048B_angleSens.h"
#include <RC-buggy_rpmSens.h>
#include <sinusTable.h>


/*** constants *********************************************************/
#define CONTROL_NEUTRAL 0       // Neutral position for control servos
#define SERVO_NEUTRAL   2048    // Neutral position for control servos
#define MCLK            16      // CPU clock in MHz
#define SMCLK           2       // Submodule Clock in MHz

#define TELEMETRY_SIZE  16      // Number of ints to be sent in packet
#define NAME_TRANSFERS  2       // Number of name transfers prior to data TX

#define MATLAB_SPI_SIZE 17      // Number of words in MatLab SPI packet
#define DMA_SPI_RX_SIZE ((MATLAB_SPI_SIZE * 2) + 1)  // DMA RX Byte counter
#define DMA_SPI_TX_SIZE (MATLAB_SPI_SIZE * 2)        // DMA TX Byte counter

/*** choose XBee module ************************************************/
#define _XBEE_S2C_
#if defined(_XBEE_S2C_)
    // cyclic RX 4 bytes every 15 ms
    #define UART_SYNC_TIME_MS   0.5
    #define TELEMETRY_CYCLE_MS  50
    #define FILTER_CYCLE_MS     10
    #define WATCHDOG_TIMEOUT_MS 150
#elif defined(_XBEE_SX868_)
    // cyclic RX 4 bytes every 30 ms
    #define UART_SYNC_TIME_MS   1
    #define TELEMETRY_CYCLE_MS  50
    #define FILTER_CYCLE_MS     10
    #define WATCHDOG_TIMEOUT_MS 200
#else
    #error **** xbee module not defined ****
#endif

// Timeout definitions in 4 µs ticks, 250 * 4 µs = 1 ms
#define UART_SYNC_TIME ((uint16_t)(UART_SYNC_TIME_MS * 250))
#define FILTER_CYCLE ((uint16_t)(FILTER_CYCLE_MS * 250))
#define TELEMETRY_CYCLE ((uint16_t)(TELEMETRY_CYCLE_MS * 250))
#define WATCHDOG_TIMEOUT ((uint16_t)((uint16_t)WATCHDOG_TIMEOUT_MS * 250))

/*** macro for servo control pwm ***************************************/
// map x = -2048..2047 to CALC_PWM_PULSE_WIDTH(x) = 2000..3999
#define CALC_PWM_PULSE_WIDTH(x) (((uint32_t)((((x) + 2048) >> 1) + SERVO_NEUTRAL) * 125 ) >> 7)


/*** globals ***********************************************************/
extern int16_t steering_g;
extern int16_t drive_g;

extern uint16_t startupCounter;


uint8_t rxData_uart[6];

// SPI buffers
struct spiData{
    struct rxData{
        uint16_t addressDummy;
        int16_t debug[MATLAB_SPI_SIZE];
    }RX;
    struct txData{
        uint16_t addressDummy;
        int16_t data[MATLAB_SPI_SIZE];
    }TX;
}spi;

int16_t spiBuff_RX[MATLAB_SPI_SIZE + 1];
int16_t spiBuff_TX[MATLAB_SPI_SIZE + 1];

union txData_u {
    struct data{
        uint8_t startOfFrame;
        uint8_t data0[2];
        uint8_t data1[2];
        uint8_t data2[2];
        uint8_t data3[2];
        uint8_t data4[2];
        uint8_t data5[2];
        uint8_t data6[2];
        uint8_t data7[2];
        uint8_t data8[2];
        uint8_t data9[2];
        uint8_t data10[2];
        uint8_t data11[2];
        uint8_t data12[2];
        uint8_t data13[2];
        uint8_t data14[2];
        uint8_t data15[2];
        uint8_t crc16[2];
        uint8_t endOfFrame;
    }tx;
    struct dataName{
        uint8_t startOfFrame;
        uint8_t nameByte;
        uint8_t index;
        uint8_t nameString[30];
        uint8_t crc16[2];
        uint8_t endOfFrame;
    }txName;
    /* size 0 is sufficient, as size of union = size of biggest element */
    uint8_t txData[0];
}u;


// global Flags:
uint8_t isUARTDataReceived;
uint8_t isTXcomplete;
uint8_t isTelemetryRequest;
/*
 * Watchdog mechanism: as long as no signal is received isOnline switches to false,
 * now drive and steering will output fail safe values
 */
uint8_t isOnline;


/*** function declarations *********************************************/

/*************************************************************************
 * Use 16 bit CRC module to calculate CRC checksum
 ************************************************************************/
uint16_t calcCRC16(uint8_t* data, int size);


/*********************************************************************//**
* @brief pack2x7bit()
* - pack a 14 bit value in two 7-bit bytes,
* - H-bits are always low
*
* in: hex14bit
* out: 2 bytes starting at address packed
************************************************************************/
void pack2x7bit(uint8_t* packed, uint16_t hex14bit);


/*********************************************************************//**
* @brief prepareSPImessage()
* - packs data to be sent to the Nucleo board via SPI
* - End of Chip Select interrupt will start the DMA for tramsmission of
*   this data
************************************************************************/
void prepareSPImessage(void);


/*************************************************************************
 * read RTC and convert to millis
 * overflow @ 10.000, output 0...9999
 ************************************************************************/
uint16_t readRTCmillis(void);


/********************************************************************//**
 * @brief readUART()
 * - extracts steering and drive from data[]
 * - checks CRC checksum
 * - counts CRC errors
 * - if errors in a row: sets steering and drive to neutral
  ***********************************************************************/
void readUART(int16_t* steering_p, int16_t* drive_p);


/********************************************************************//**
 * @brief sendNames
 * - transfers internal variable names to the Processing panel
 * - enables UART TX DMA
  ***********************************************************************/
void sendNames(uint16_t index, char dataNames[TELEMETRY_SIZE][30]);


/********************************************************************//**
 * @brief setup_ADC
 * - initializes AD Channel 3, Pin 1.3 for 12 bit conversion
 *   (steering angle feedback)
 * - starts ADC sequence
 ***********************************************************************/
void setup_ADC(void);


/********************************************************************//**
 * @brief setup_DMA_SPI_RX
 * - DMA0 handles SPI from SIMULINK NUCLEO Board
 *   (debug data, SIMULINK output)
 ***********************************************************************/
void setup_DMA_SPI_RX(void);


/********************************************************************//**
 * @brief setup_DMA_SPI_TX
 * - DMA1 handles SPI to SIMULINK NUCLEO Board
 *  (SIMULINK input)
 ***********************************************************************/
void setup_DMA_SPI_TX(void);


/********************************************************************//**
 * @brief setup_DMA_RX
 * - DMA5 handles UART from XBee Module
 *  (RC input)
 ***********************************************************************/
void setup_DMA_RX(void);


/********************************************************************//**
 * @brief setup_DMA_RX
 * - DMA4 handles UART to XBee Module
 *  (telemetry output)
 ***********************************************************************/
void setup_DMA_TX(void);


/********************************************************************//**
 * @brief setup_ReferenceTimer
 * - start 32 kHz crystal
 * - starts RTC in Counter Mode
 ***********************************************************************/
void setup_referenceTimer(void);


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
 * @brief setup_triggerTimer
 * prepares TimerA1 to
 * - trigger UART read on start of frame
 * - release watchdog interrupt on overflow
 ***********************************************************************/
void setup_triggerTimer(void);


/********************************************************************//**
 * @brief setup_UART
 * - prepares UART (9600 baud, 8N1)
 * - initializes pins: P6.0 as UART TX, P6.1 as UART RX
 ***********************************************************************/
void setup_UART(void);


/********************************************************************//**
 * @brief setup_watchdogTimer
 * prepares TimerA4 to
 * - release watchdog interrupt on overflow
 ***********************************************************************/
void setup_syncWatchdogTimer(void);


/*********************************************************************//**
* @brief unPack2x7bit()
* - unpack 14 bit value out of two 7-bit bytes,
* - H-bits are always low
*
* in: 2 bytes starting at address packed
* out: hex14bit
************************************************************************/
uint16_t unPack2x7bit(uint8_t* packed);


/********************************************************************//**
 * @brief writeUART()
 * - enables UART TX interrupt unless S1 is pressed
  ***********************************************************************/
void writeUART(void);




#endif /* RC_BUGGY_VEHICLE_H_ */
