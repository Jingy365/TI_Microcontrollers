/*
 *  RC-buggy_sender.c
 *
 *  Created on: Nov 23, 2022
 *      Author: stix
 */

#include <RC-buggy_sender.h>
#include <LP_FR5994_io.h>

/*** globals ***********************************************************/
uint16_t steering_g = 2048;
int16_t  drive_g = 2048;



/*** Functions *********************************************************/

void define_Modes(uint8_t* isSlowMode_p){

    // S1 during recover from RESET enables slow mode
    if(S1_PRESSED){
        *isSlowMode_p = false;
    }

    // S2 during recover from RESET enables test mode
    if(S2_PRESSED){
        testMode();
        // this line will never be reached
    }
}



void pack2x7bit(uint8_t* packed, uint16_t hex14bit){
    packed[0] = (hex14bit >> 7) & 0x7F;
    packed[1] = hex14bit & 0x7F;
}



void sendMessage(uint16_t steering, uint16_t drive){
    static uint16_t steeringCenter = 0;
    static uint16_t driveCenter = 0;
    static uint8_t isSlowMode = true;

    uint16_t steeringRaw;
    int16_t driveRaw;

    // at first call of sendMessage only:
    if (steeringCenter == 0 && driveCenter == 0){

        define_Modes(&isSlowMode);

        // wait for first ADC result
        __delay_cycles (1000000);

        // memorize center positions
        steeringCenter = (ADC12MEM0);
        driveCenter = (ADC12MEM1);

        // limit driveCenter to +/- 10 %
        driveCenter = driveCenter > 2048 * 1.1 ? 2048 * 1.1 : driveCenter;
        driveCenter = driveCenter < 2048 * 0.9 ? 2048 * 0.9 : driveCenter;
    }

    // case: joystick and servo in same direction
    // steeringRaw = trim_Poti((4095 - ADC12MEM0), steeringCenter);

    // case: joystick and servo direction inverted
    steeringRaw = trim_Poti(steering, steeringCenter);
    driveRaw = trim_Poti(drive, driveCenter);

    // Speed Limit Mode
    if(isSlowMode){
        driveRaw -= driveCenter;
        if (driveRaw > 0){
            driveRaw *= 0.6; // limit forward speed
        }
        else {
            driveRaw *= 0.8; // limit reverse speed
        }
        driveRaw += driveCenter;
    }

    // present prepared values to ISR
    drive_g = driveRaw;
    steering_g = steeringTable[steeringRaw];

    // make sure to reach 0 %
    pwmGreen = drive_g - (int16_t)(PERIOD_LED * 0.01);
    pwmRed = steering_g - (int16_t)(PERIOD_LED * 0.01);


}



void setup_hardware(void){
    setSYSCLK(MCLK, SMCLK);
    setup_io(2, "PWM_RED", "PWM_GREEN");
    setup_UART();
    setup_triggerTimer();
}



void setup_triggerTimer(void){
    TA1EX0 = TAIDEX__1;          // Prescaler

    //TA1CCR0 = 0xFFFF;            // upper count limit (131 msec @ TAIDEX__1)
    //TA1CCR0 = 50000;             // upper count limit (100 msec @ TAIDEX__1)
    TA1CCR0 = CYCLIC_SEND_MILLIS * 500UL;
#if ((CYCLIC_SEND_MILLIS * 500UL) > 0xFFFF)
    #error **** TA1 upper count limit exceeded ****
#endif

    TA1CTL =
            TASSEL__SMCLK |      // Count SMCLK
            ID__4 |              // Prescaler: 2 usec/tick @ TAIDEX__1
            MC__UP;              // Start Counter


    TA1CCTL0 =
            CAP__COMPARE |       // Compare Mode
            CCIE_1;              // overflow interrupt enabled
}



void setup_UART(void){
    // 8N1
    UCA3CTLW0 =
            UCPEN_0 |           // no parity
            UC7BIT__8BIT |      // 8
            UCSPB_0 |           // one stop bit
            UCSYNC__ASYNC |     //
            UCMODE_0 |          // UART mode
            UCSSEL__SMCLK |     // source clock
            UCSWRST__ENABLE;    // hold in RESET to enable baud rate setup

    // baud rate
    //UCA3BRW = 13;           // for SMCLK = 2 MHz, 9600 baud
    UCA3BRW = 1;            // for SMCLK = 2 MHz, 115200 baud
    UCA3MCTLW_H = 0x4A;     // for SMCLK = 2 MHz, 115200 baud
    UCA3MCTLW |=
            UCBRF_1 |
            UCOS16_1;

    // connect TX pin, RX pin to UART
    P6SEL0 |= (BIT0 | BIT1);
    P6SEL1 &= ~(BIT0 | BIT1);

    // release RESET
    UCA3CTLW0 &= ~UCSWRST;
}



uint16_t trim_Poti(uint16_t adc, uint16_t adcZero){

    uint16_t trimmedADC;

    if(adc >= adcZero){
        trimmedADC = (double)(adc - adcZero) * 2047 / (4095 - adcZero) + 2048;
    }
    else{
        trimmedADC = (double)adc * 2048 / adcZero;
    }

    return trimmedADC;
}



void testMode(void){
    uint8_t isTest1000Mode = false;

    while(1){
        static double time = 0.0;
        drive_g = 2048;

        if(isTest1000Mode){
            steering_g = 1000;
        }
        else {
            // sin() takes around 60.000 cycles (7.5 ms)
            steering_g = 2048 * sin(time) + 2047;
            time += 7.5 / 1000.0;
        }

        // TEST: ALWAYS SEND STEERING 1000
        if(S1_PRESSED){
            isTest1000Mode = true;
        }

        // TEST: SEND SINUS TO SERVO
        if(S2_PRESSED){
            isTest1000Mode = false;
        }

        pwmGreen = drive_g;
        pwmRed = steering_g;
    }
}



/*** Interrupt Handlers ************************************************/

/********************************************************************//**
 *   @brief TA1 overflow ISR
 * - combines and sends ADC12 steering and drive
 * - sends 6 Bytes: 12 Bit steering, 12 Bit drive, 14 Bit CRC Checksum
 * 6 bytes @ 9600 8N1 takes 6 ms
 ***********************************************************************/
#pragma vector = TIMER1_A0_VECTOR
__interrupt void ISR_sendMessage(void) {

    uint8_t data[6];
    int i;

   // calculate CRC checksum
   CRCINIRES = 0x4BEE;
   CRCDI = steering_g;
   CRCDI = drive_g;

   // prepare output buffer
   pack2x7bit(&data[0], steering_g);
   pack2x7bit(&data[2], drive_g);
   pack2x7bit(&data[4], CRCINIRES);

   // send message
   for (i = 0; i < 6; i++){
       UCA3TXBUF = data[i];
       while(!(UCA3IFG & UCTXIFG));
   }
}

