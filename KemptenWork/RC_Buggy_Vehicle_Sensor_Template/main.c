/********************************************************************//**
 * @file main.c
 *
 * RC-BUGGY VEHICLE MODULE
 * ------------------------
 *
 * XBee Receiver UART --> PWM output
 * - 6 Byte UART input @ P6.1 every x ms: 2x 12 bit + crc checksum
 * - control PWM outputs on P1.2 (steering) and P1.3 (drive)
 * - Fail safe to neutral if cyclic input is missing
 * - Green LED P1.1 outputs drive PWM
 * - Red LED P1.0 outputs steering PWM
 *
 *
 * Board: MSP-EXP430FR5994
 * - Author: Stix
 * - Created: 14.11.2022 08:00
 *
 ***********************************************************************/

/*** included files ****************************************************/
#include <msp430.h>
#include <RC-buggy_vehicle.h>


/*** constants *********************************************************/


/*** globals ***********************************************************/
int16_t steering_g = CONTROL_NEUTRAL;
int16_t drive_g = CONTROL_NEUTRAL;
// Flags:
extern uint8_t isUARTDataReceived;
uint16_t startupCounter = 2 / 0.02; // waitTime/controlPWM_Period



/*** function declarations *********************************************/
void setup_controlPWM(void);


/********************************************************************//**
 * @brief main function
 * - initializes hardware
 * - loop:
 * - synchronized read of drive/steering (UART)
 ***********************************************************************/
int main(void)
{
    setup_hardware();
    setup_controlPWM();

    __enable_interrupt();

    while (1)
    {
        if (isUARTDataReceived){
            isUARTDataReceived = false;
            readUART(&steering_g, &drive_g);
            prepareSPImessage();
        }

        if (isTelemetryRequest){
            isTelemetryRequest = false;
            writeUART();
        }
    }
}


void setup_controlPWM(void){
    TA1CTL = TASSEL__SMCLK |
                ID__1 |
                MC__UP |
                TACLR |
                TAIE_1 |
                TAIFG_0;

        TA1EX0 = TAIDEX__1;
        TA1CCR0 = 40000;
        TA1CCTL1 = CAP__COMPARE |
                OUTMOD_7 |
                CCIE_0;
        TA1CCR1 = CONTROL_NEUTRAL;

        TA1CCTL2 = CAP__COMPARE |
                    OUTMOD_7 |
                    CCIE_0;
        TA1CCR2 = CONTROL_NEUTRAL;

        P1DIR |= BIT2+BIT3;
        P1SEL0 |= BIT2+BIT3;
        P1SEL1 &= ~(BIT2+BIT3);

}


/*** Interrupt Handler *************************************************/
#pragma vector = TIMER1_A1_VECTOR
__interrupt void ISR_update_controlPWM(void) {
    uint16_t drive, steering;
    TA1CTL &= ~TAIFG_1;
    if (!isOnline) {
        // fail safe: set outputs neutral
        // steering_g = CONTROL_NEUTRAL;
        drive_g = CONTROL_NEUTRAL;
    }

    if (startupCounter) {
        // after power-up: 2 sec neutral to calibrate motor controller
        startupCounter--;
        drive_g = CONTROL_NEUTRAL;
    }

    steering = CALC_PWM_PULSE_WIDTH(steering_g);
    drive = CALC_PWM_PULSE_WIDTH(drive_g);

    //steering = 10000;   //CODE 1
    //drive = 2000;       //CODE 2
    TA1CCR1 = steering;
    TA1CCR2 = drive;

    //TA1CCR1 = 2000;
    //TA1CCR2 = 3999;


}


