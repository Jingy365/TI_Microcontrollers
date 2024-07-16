#define DEBUG_MODE
/*
 * LP_FR5994.c
 *
 *  Created on: 17.01.2023
 *      Author: stix
 */

#include "LP_FR5994.h"

void setup_IO(void){

    /**** All pins input, pull-down ****/
    // By default all pins are inputs, prepared for internal pull-down
    // but resistor is not enabled
    P1REN = 0xFF;    // enable all internal pull-down resistors
    P2REN = 0xFF;
    P3REN = 0xFF;
    P4REN = 0xFF;
    P5REN = 0xFF;
    P6REN = 0xFF;
    P7REN = 0xFF;
    P8REN = 0xFF;

    /**** Pushbutton switches: input, pull-up ****/
    P5OUT |= BIT5 | BIT6;

    /**** LEDs: output, OFF ****/
    P1DIR |= LED_RED | LED_GREEN;

    /**** 2x PWM: output, OFF ****/
    P1DIR |= BIT2 | BIT3;

#ifdef DEBUG_MODE
    P8DIR |= BIT1; // DEBUG_PIN
    P8DIR |= BIT2; // DEBUG_PIN2
#endif


    PM5CTL0 &= ~LOCKLPM5;   // unlock IO configuration
}


void setSYSCLK(uint8_t mCLK, uint8_t smCLK) {

    CSCTL0 = CSKEY;                     //unlock CS-registers through the password

    switch (mCLK){
    case 1:
        CSCTL1 = DCORSEL | DCOFSEL_0;       //run DCO at 1 MHz
        CSCTL3 = DIVM__1;                   //run CPU-Clock (MCLK) at 1 MHz
        switch(smCLK){
        case 1:
            CSCTL3 |= DIVS__1;              //run SMCLK at 1:1 = 1 MHz
            break;
        default:
            while(1);   // SMCLK undefined
        }
        break;

    case 8:
        CSCTL1 = DCORSEL | DCOFSEL_3;       //run DCO at 8 MHz
        CSCTL3 = DIVM__1;                   //run CPU-Clock (MCLK) at 8 MHz
        switch(smCLK){
        case 2:
            CSCTL3 |= DIVS__4;              //run SMCLK at 8:4 = 2 MHz
            break;
        case 8:
            CSCTL3 |= DIVS__1;              //run SMCLK at 8:1 = 8 MHz
            break;
        default:
            while(1);   // SMCLK undefined
        }
        break;

    case 16:
        FRCTL0 = FWPW | NWAITS_1;           //unlock and set FRAM Wait states
        CSCTL1 = DCORSEL | DCOFSEL_4;       //run DCO at 16 MHz
        CSCTL3 = DIVM__1;                   //run CPU-Clock (MCLK) at 16 MHz
        switch(smCLK){
        case 2:
            CSCTL3 |= DIVS__8;              //run SMCLK at 16:8 = 2 MHz
            break;
        case 8:
            CSCTL3 |= DIVS__2;              //run SMCLK at 16:2 = 8 MHz
            break;
        default:
            while(1);   // SMCLK undefined
        }
        break;

    default:
        while(1);   //MCLK undefined
    }

     CSCTL0_H = 0;                       //re-lock CS-registers
}

