/*
 *  LP_FR5994_spi.c
 *
 *  Created on: 06.11.2023
 *      Author: stix
 */


/*** included files ****************************************************/
#include <LP_FR5994_spi.h>


/*******************************************************************************
    Setup SPI
    UCB0 RX/TX Data as Slave
    Transmit to ST Nucleo Board H7A3ZI
*******************************************************************************/
void setup_UCB0_SPI_Slave(void)
{
    /* PIN Configuration
     * P1.6 MOSI       UCB0SIMO
     * P1.7 MISO       UCB0SOMI
     * P2.2 CLK        UCB0CLK
     * P5.3 CS         GP_IN
     */

    // SPI mode is selected when the UCSYNC bit is set
    UCB0CTLW0 |= UCSYNC;

    // The recommended eUSCI initialization or reconfiguration process is:
    // 1. Set UCSWRST.     BIS.B #UCSWRST,&UCxCTL1
    UCB0CTLW0 |= UCSWRST;

    // 2. Initialize all eUSCI registers with UCSWRST = 1 (including UCxCTL1).
        /*The SPI in the microcontroller must be configured in mode 0,0 or
        1,1 in 8-bit operating mode.*/

        // SPI mode features include:
        // 7-bit or 8-bit data length
        // LSB-first or MSB-first data transmit and receive
        // 3-pin and 4-pin SPI operation
        // Master or slave modes
        // Independent transmit and receive shift registers
        // Separate transmit and receive buffer registers
        // Continuous transmit and receive operation
        // Selectable clock polarity and phase control
        // Programmable clock frequency in master mode
        // Independent interrupt capability for receive and transmit
        // Slave operation in LPM4

    #define UCSTEM (0x0002) /* STE mode select in master mode */

    // UCBxCTLW0 eUSCI_Bx Control Word 0
    UCB0CTLW0 =
            UCCKPH_0 |           // data valid on first (rising) CLK edge
            /*UCCKPH_1 |           // data valid on second (falling) CLK edge*/
            UCCKPL__LOW   |      // CLK polarity, inactive is 0:LOW / 1: HIGH
            /*UCCKPL__HIGH  |      // CLK polarity, inactive is 1:HIGH / 0: LOW*/
            UCMSB_1 |            // MSB first, 0:LSB / 1:MSB
            UC7BIT__8BIT |       // 8-bit data, 0:8-bits / 1:7-bits
            UCMST__SLAVE |       // SPI mode, 0:Slave / 1: Master
            /*UCMODE_2 |           // 4 Pin SPI, CS active LOW*/
            UCMODE_0 |           // 3 Pin SPI, don't use STE (=CS)
            UCSYNC__SYNC |       // synchronous mode, 0:UART-Mode / 1:SPI-Mode
            UCSSEL__UCLK |       // BRCLK source is UCxCLK (we are in slave mode)
            /*UCSTEM |             // STE pin generates /CS, ignored in slave mode*/
            UCSWRST;             // hold eUSCI in RESET state


    // UCBxBRW eUSCI_Bx Bit Rate Control Word
    UCB0BRW = 0;  // slave Mode: don't care?

    // UCBxSTATW eUSCI_Bx Status Read/write Word
    UCB0STAT = 0b00000000;
    /*            0                     loopback mode disabled
     *             00                   reset Error Flags
     *              (0000)              reserved
     *                   0              UCBUSY, read only
     */

    // UCBxRXBUF eUSCI_Bx Receive Buffer Read/write Word
    // UCBxTXBUF eUSCI_Bx Transmit Buffer Read/write Word
    // UCBxIFG eUSCI_Bx Interrupt Flag Read/write Word
    // UCBxIV eUSCI_Bx Interrupt Vector Read Word

    // 3. Configure ports.
    // DIR not necessary, directions controlled by UCBx module
    P1SEL0 &= ~(BIT6 | BIT7);   // MOSI, MISO
    P1SEL1 |= (BIT6 | BIT7);
    P2SEL0 &= ~BIT2;            // CLK
    P2SEL1 |= BIT2;


    // P5.3 receives ChipSelect
    // interrupt on CS (both edges, start with rising edge)
    P5DIR &= ~BIT3;
    P5OUT |= BIT3;        // Pull-up CS
    P5REN |= BIT3;        // Pull-up CS
    P5IES &= ~BIT3;       // INT on rising edge
    /*P5IES |= BIT3;        // INT on falling edge*/
    P5IFG &= ~BIT3;       // clear pending INT
    P5IE |= BIT3;         // enable CS INT


    // 4. Ensure that any input signals into the SPI module such as UCxSOMI (in master mode)
    // or UCxSIMO and UCxCLK (in slave mode) have settled to their final voltage levels
    // before clearing UCSWRST and avoid any unwanted transitions during operation.
    __delay_cycles(1000UL);

    // 5. Clear UCSWRST.    BIC.B #UCSWRST,&UCxCTL1
    // while(P5IN & BIT2);  // wait until CLK is low: workaround, see Error USCI47
                            // this line is removed to achieve proper RC-Buggy function
                            // even without Nucleo Board
                            // todo: Timeout?
    UCB0CTLW0 &= ~UCSWRST;

    // UCBxIE eUSCI_Bx Interrupt Enable Read/write Word
    //UCB0IE = UCRXIE_1;

    return;
}

