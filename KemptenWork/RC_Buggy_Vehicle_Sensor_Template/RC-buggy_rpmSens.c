/*
 * RC-buggy_rpmSens.c
 *
 *  Created on: 01.03.2023
 *      Author: Stix
 */

/*** included files ****************************************************/
#include <RC-buggy_rpmSens.h>
#include <limits.h>
#include <stdbool.h>

/*** globals ***********************************************************/
volatile int32_t ticksCCR3 = 0;
volatile int32_t ticksCCR4 = 0;
volatile int32_t ticksCCR5 = 0;
volatile int32_t ticksCCR6 = 0;

volatile int32_t lastReadingCCR3 = 0;
volatile int32_t lastReadingCCR4 = 0;
volatile int32_t lastReadingCCR5 = 0;
volatile int32_t lastReadingCCR6 = 0;

volatile uint16_t overflowCountCCR3 = 0;
volatile uint16_t overflowCountCCR4 = 0;
volatile uint16_t overflowCountCCR5 = 0;
volatile uint16_t overflowCountCCR6 = 0;



int16_t calcRPM(wheel_t wheel){
    uint32_t rpm;
    uint32_t ticks;

    static struct {
        uint32_t rpm;
        int wasForward;
    } memory[4];

    int isForward;

    __disable_interrupt();

    switch(wheel){

    case FRONT_L:
        // read ticks and direction
        ticks = ticksCCR6;
        isForward = !(P3IN & BIT3);
        break;

    case FRONT_R:
        // read ticks and direction
        ticks = ticksCCR5;
        isForward = !(P3IN & BIT2);
        break;

    case REAR_L:
        // read ticks and direction
        ticks = ticksCCR4;
        isForward = !(P3IN & BIT1);
        break;

    case REAR_R:
        // read ticks and direction
        ticks = ticksCCR3;
        isForward = !(P3IN & BIT0);
        break;
    }

    __enable_interrupt();

    rpm = (uint32_t)15000000 / ticks; // rising or falling edge only
    //rpm = (uint32_t)7500000 / ticks; // both edges considered
    rpm = lowpassFilter(rpm, &memory[wheel].rpm);

    // set filtered rpm to zero on direction change
    if (isForward && !memory[wheel].wasForward){
        memory[wheel].wasForward = true;
        rpm = 0;
        memory[wheel].rpm = 0;
    }
    if (!isForward && memory[wheel].wasForward){
        memory[wheel].wasForward = false;
        rpm = 0;
        memory[wheel].rpm = 0;
    }

    // we transmit 14 bits, thus -8192...8191
    if (isForward)
        return (int16_t) rpm;
    else
        return (int16_t) -rpm;
}


uint32_t lowpassFilter(uint32_t input, uint32_t* lastOutput_p)
{
    int32_t diff = input - *lastOutput_p;

    // the higher k, the more direct. k = 64 means no filter
    int k = 20;

    int output = *lastOutput_p + ((k * diff) >> 6) ;
    *lastOutput_p = output;
    return input; // omit filter
    //return output;
}


void setup_capture(void){

    TB0EX0 = TBIDEX__1;          // Prescaler

    TB0CCR0 = 0xFFFF;            // upper count limit (32.768 ms @ 1:1)
    TB0CTL =
            TBCLGRP_0 |          // each latch loads independently
            CNTL__16 |           // 16 bit counter
            TBSSEL__SMCLK |      // Count SMCLK
            ID__1 |              // Prescaler: 0.5 usec/tick @ TAIDEX__1
            MC__CONTINUOUS|      // Start Counter
            TBIE_0;              // generic overflow interrupt

    TB0CCTL0 =  // overflow detection
            CM__NONE |
            CLLD_1 |
            CAP__COMPARE |       // Compare Mode
            CCIFG_0 |
            OUT__LOW |
            CCIE_1;              // overflow interrupt

    TB0CCTL3 =  // RPM time measurement
            CM__RISING |
            CCIS__CCIA |
            SCS__ASYNC |
            CAP__CAPTURE |       // Capture Mode
            COV_0 |
            CCIFG_0 |
            CCIE_1;              // capture interrupt

    // copy CCTL3 to all remaining channels
    TB0CCTL4 = TB0CCTL3;
    TB0CCTL5 = TB0CCTL3;
    TB0CCTL6 = TB0CCTL3;

    /*
    // set capture input pins to input with pullup
    P3DIR &= ~(BIT7 | BIT6 | BIT5 | BIT4);
    P3OUT |= BIT7 | BIT6 | BIT5 | BIT4;
*/

    // connect capture input pins
    P3SEL0 |= BIT7 | BIT6 | BIT5 | BIT4;
    P3SEL1 &= ~(BIT7 | BIT6 | BIT5 | BIT4);

    // set direction input pins to input with pullup
    P3DIR &= ~(BIT3 | BIT2 | BIT1 | BIT0);
    P3OUT |= BIT3 | BIT2 | BIT1 | BIT0;
}

/********************************************************************//**
 *   @brief Timer B0 Overflow ISR
 *   RPM measurement: keep track of overflows
 ***********************************************************************/
#pragma vector = TIMER0_B0_VECTOR
__interrupt void ISR_overflow_rpm(void) {
    static int32_t delta3 = 0;
    static int32_t delta4 = 0;
    static int32_t delta5 = 0;
    static int32_t delta6 = 0;


    if (overflowCountCCR3 == 0){
        delta3 = (int32_t)0x10000 - lastReadingCCR3;
    }
    if (delta3 > ticksCCR3){
        ticksCCR3 = delta3;
    }
    overflowCountCCR3++;
    delta3 += 0x10000;
    if (delta3 < 0){
        delta3 -= 0x10000; // prevent overflow
        overflowCountCCR3--;
    }


    if (overflowCountCCR4 == 0){
        delta4 = (int32_t)0x10000 - lastReadingCCR4;
    }
    if (delta4 > ticksCCR4){
        ticksCCR4 = delta4;
    }
    overflowCountCCR4++;
    delta4 += 0x10000;
    if (delta4 < 0){
        delta4 -= 0x10000; // prevent overflow
        overflowCountCCR4--;
    }


    if (overflowCountCCR5 == 0){
        delta5 = (int32_t)0x10000 - lastReadingCCR5;
    }
    if (delta5 > ticksCCR5){
        ticksCCR5 = delta5;
    }
    overflowCountCCR5++;
    delta5 += 0x10000;
    if (delta5 < 0){
        delta5 -= 0x10000; // prevent overflow
        overflowCountCCR5--;
    }


    if (overflowCountCCR6 == 0){
        delta6 = (int32_t)0x10000 - lastReadingCCR6;
    }
    if (delta6 > ticksCCR6){
        ticksCCR6 = delta6;
    }
    overflowCountCCR6++;
    delta6 += 0x10000;
    if (delta6 < 0){
        delta6 -= 0x10000; // prevent overflow
        overflowCountCCR6--;
    }

}

/********************************************************************//**
 *   @brief Timer B0 Capture ISR
 *   calculate RPM from wheel sensor inputs
 ***********************************************************************/
#pragma vector = TIMER0_B1_VECTOR
__interrupt void ISR_measure_rpm(void) {

    switch ( __even_in_range(TB0IV, TBIV__TBCCR6) )
    {
        case TBIV__TBCCR3:
            ticksCCR3 = TB0CCR3 - lastReadingCCR3;
            ticksCCR3 += ((uint32_t)overflowCountCCR3 << 16);
            lastReadingCCR3 = TB0CCR3; //todo: HW-Register nur einmal lesen!
            overflowCountCCR3 = 0;
            if (TB0CCTL3 & COV){
                TB0CCTL3 &= ~COV;
                ticksCCR3 = 0;
            }
            break;

        case TBIV__TBCCR4:
            ticksCCR4 = TB0CCR4 - lastReadingCCR4;
            ticksCCR4 += ((uint32_t)overflowCountCCR4 << 16);
            lastReadingCCR4 = TB0CCR4; //todo: HW-Register nur einmal lesen!
            overflowCountCCR4 = 0;
            if (TB0CCTL4 & COV){
                TB0CCTL4 &= ~COV;
                ticksCCR4 = 0;
            }
            break;

        case TBIV__TBCCR5:
            ticksCCR5 = TB0CCR5 - lastReadingCCR5;
            ticksCCR5 += ((uint32_t)overflowCountCCR5 << 16);
            lastReadingCCR5 = TB0CCR5; //todo: HW-Register nur einmal lesen!
            overflowCountCCR5 = 0;
            if (TB0CCTL5 & COV){
                TB0CCTL5 &= ~COV;
                ticksCCR5 = 0;
            }
            break;

        case TBIV__TBCCR6:
            ticksCCR6 = TB0CCR6 - lastReadingCCR6;
            ticksCCR6 += ((uint32_t)overflowCountCCR6 << 16);
            lastReadingCCR6 = TB0CCR6; //todo: HW-Register nur einmal lesen!
            overflowCountCCR6 = 0;
            if (TB0CCTL6 & COV){
                TB0CCTL6 &= ~COV;
                ticksCCR6 = 0;
            }
            break;
        default: __never_executed();
    }
}
