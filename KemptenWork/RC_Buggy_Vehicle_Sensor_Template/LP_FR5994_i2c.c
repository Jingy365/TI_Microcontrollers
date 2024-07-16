/*
 *  LP_FR5994_i2c.c
 *
 *  Created on: 07.03.2023
 *      Author: stix
 */

/*** included files ****************************************************/
#include <LP_FR5994_i2c.h>

/*** globals ***********************************************************/
int i2cStatus = I2C_IS_IDLE;

/* read I2C into global rxData[]*/
int readI2C(uint8_t device_address, uint8_t reg_name, uint8_t readSize){
    uint16_t timeOutCounter = I2C_MAX_WAIT;

    // set counter for automatic stop generation
    UCB2CTLW0 |= UCSWRST;       // prepare setup, resets UCB2IE (!)
    UCB2TBCNT = 1;              // set counter for TX of command
    UCB2I2CSA = device_address; // slave address
    UCB2CTLW0 &= ~UCSWRST;      // issue setup

    // switch to TX and generate start condition
    UCB2CTLW0 |= UCTR_1 | UCTXSTT_1;

    txData_i2c[0] = reg_name;
    tx_index_i2c = 0;
    UCB2IE |= UCTXIE;           // enable TX-interrupt

    // limit readSize to size of array
    readSize = readSize > MAX_SIZE_RX ? MAX_SIZE_RX : readSize;

    i2c_byteCount = readSize;       // send size to ISR (for RX part)
    rx_index_i2c = 0;
    i2cStatus = I2C_IS_BUSY;

    UCB2IE |= UCSTPIE;         // enable STOP-interrupt (TX complete)
    UCB2IE |= UCNACKIE;        // enable Not-Acknowledge-interrupt (Error)
    UCB2IE |= UCCLTOIE;        // enable Clock-Low-Timeout-interrupt (Error)

    while (i2cStatus == I2C_IS_BUSY){
        if(timeOutCounter-- == 0){
            i2cStatus = I2C_ERR_RX_TOUT;
            break;
        }
    }

    return i2cStatus;
}


void setup_I2C(void){

    // force stop condition to interrupt possible slave TX
    // STOP = rising edge on SDA while SCL is high
    P7DIR |= I2C_CLK;   // CLK: out
    P7DIR &= ~I2C_DATA; // Data: in
    P7OUT |= I2C_DATA;  // Data: pull-up, (not down)

    // toggle CLK until Data is released
    while((P7IN & I2C_DATA) == 0){
        P7OUT &= ~I2C_CLK;
        __delay_cycles(100);
        P7OUT |= I2C_CLK;
        __delay_cycles(100);
    }

    // configure I2C pins (device specific)
    P7SEL0 |= BIT0 | BIT1;
    P7SEL1 &= ~(BIT0 | BIT1);
    P7REN  &= ~(BIT0 | BIT1);

    //prepare setup
    UCB2CTLW0 |= UCSWRST;

    //eUSCI_B2 Control Word 0
    UCB2CTLW0 =
            UCA10_0 |       // 0: 7 bit, 1: 10 bit own address mode
            UCSLA10__7BIT | // slave address mode
            UCMM__SINGLE |  // single/multi master
            UCMST__MASTER | // own mode
            UCMODE_3 |      // 3: I2C mode
            UCSSEL__SMCLK | // clock source
            UCTXACK_0 |     // acknowledge of slave address
            UCTR__TX |      // select: transmitter or receiver
            UCTXNACK_0 |    // generate NAK (Not Acknowledge)?
            UCTXSTP_0 |     // send STOP to cancel any open communication
            UCTXSTT_0 |     // send START?
            UCSWRST;        // reset module

    //eUSCI_B2 Control Word 1
    UCB2CTLW1 =
            UCETXINT_0 |    // don't care in master mode
            UCCLTO_0 |      // clock low timeout, 0: disable
            UCSTPNACK_0 |   // ack of last byte in master rx mode
            UCSWACK_0 |     // ack by module or software
            UCASTP_2 |      // automatic stop generation?
            UCGLIT_0;       // deglitch time

    //eUSCI_Bx Bit Rate Control Word
    UCB2BRW = SMCLK * 1000 / I2C_SPEED; //SMCLK: in MHz, I2C_SPEED in kHz

    // TX bytes count
    UCB2TBCNT = 1;

    // eUSCI_B in operational state
    UCB2CTL1 &= ~UCSWRST;
}


/* write 32 bits to I2C register*/
int write32BitsI2C(uint8_t device_address, uint8_t reg_name, uint32_t data){
    uint16_t timeOutCounter = I2C_MAX_WAIT;

    // set counter for automatic stop generation
    UCB2CTLW0 |= UCSWRST;       // prepare setup, resets UCB2IE (!)
    UCB2TBCNT = 5;              // set counter for TX of command
    UCB2I2CSA = device_address; // slave address
    UCB2CTLW0 &= ~UCSWRST;      // issue setup

    // switch to TX and generate start condition
    UCB2CTLW0 |= UCTR_1 | UCTXSTT_1;

    txData_i2c[0] = reg_name;
    txData_i2c[1] = data;
    txData_i2c[2] = data >> 8;
    txData_i2c[3] = data >> 16;
    txData_i2c[4] = data >> 24;
    tx_index_i2c = 0;
    UCB2IE |= UCTXIE;           // enable TX-interrupt

    i2c_byteCount = 0;          // set readSize to 0 (to prevent follow-up read)
    i2cStatus = I2C_IS_BUSY;
    UCB2IE |= UCTXCPTIE;        // enable STOP-interrupt (TX complete)
    UCB2IE |= UCNACKIE;        // enable Not-Acknowledge-interrupt (Error)
    UCB2IE |= UCCLTOIE;        // enable Clock-Low-Timeout-interrupt (Error)

    while (i2cStatus == I2C_IS_BUSY){
        if(timeOutCounter-- == 0){
            i2cStatus = I2C_ERR_TX_TOUT;
            break;
        }
    }

    return i2cStatus;
}


/* write single byte to I2C register*/
int writeByteI2C(uint8_t device_address, uint8_t reg_name, uint8_t data){
    uint16_t timeOutCounter = I2C_MAX_WAIT;

    // set counter for automatic stop generation
    UCB2CTLW0 |= UCSWRST;       // prepare setup, resets UCB2IE (!)
    UCB2TBCNT = 2;              // set counter for TX of command
    UCB2I2CSA = device_address; // slave address
    UCB2CTLW0 &= ~UCSWRST;      // issue setup

    // switch to TX and generate start condition
    UCB2CTLW0 |= UCTR_1 | UCTXSTT_1;

    txData_i2c[0] = reg_name;
    txData_i2c[1] = data;
    tx_index_i2c = 0;
    UCB2IE |= UCTXIE;           // enable TX-interrupt

    i2c_byteCount = 0;          // set readSize to 0 (to prevent follow-up read)
    i2cStatus = I2C_IS_BUSY;
    UCB2IE |= UCTXCPTIE;        // enable STOP-interrupt (TX complete)
    UCB2IE |= UCNACKIE;        // enable Not-Acknowledge-interrupt (Error)
    UCB2IE |= UCCLTOIE;        // enable Clock-Low-Timeout-interrupt (Error)

    while (i2cStatus == I2C_IS_BUSY){
        if(timeOutCounter-- == 0){
            i2cStatus = I2C_ERR_TX_TOUT;
            break;
        }
    }

    return i2cStatus;
}


/*** Interrupt Handler *************************************************/

/********************************************************************//**
 *   @brief I2C ISR
 *   handle I2C events
 ***********************************************************************/
#pragma vector=USCI_B2_VECTOR
__interrupt void ISR_i2c(void)
{
    switch(__even_in_range(UCB2IV, UCIV__UCBIT9IFG))
    {
        case UCIV__NONE:
            break;

        case UCIV__UCALIFG:      // Arbitration lost
            break;

        case UCIV__UCNACKIFG:    // NAK received (master only)
            // report error and finish
            i2cStatus = I2C_ERR_NACK;
            break;

        case UCIV__UCSTTIFG:     // START condition
            break;

        case UCIV__UCSTPIFG:    // TX complete, STOP
            // finish TX part, after Command TX: switch to RX
            if (UCB2IE & UCTXIE0_1){
                if (i2c_byteCount){
                    // set counter for automatic stop generation
                    UCB2CTLW0 |= UCSWRST;       // prepare setup
                    UCB2TBCNT = i2c_byteCount;  // set counter for RX of data
                    UCB2CTLW0 &= ~UCSWRST;      // issue setup

                    // switch to RX and generate start condition
                    UCB2CTLW0 = (UCB2CTLW0 & ~UCTR_1) | UCTXSTT_1;

                    UCB2IE |= UCRXIE0;          // enable RX-interrupt
                    UCB2IE |= UCSTPIE;          // re-enable STOP-interrupt (TX complete)
                    UCB2IE |= UCNACKIE;         // enable Not-Acknowledge-interrupt (Error)
                    UCB2IE |= UCCLTOIE;         // enable Clock-Low-Timeout-interrupt (Error)
                } else {
                    // TX is finished, no RX bytes were requested
                    i2cStatus = I2C_IS_IDLE;
                }
            }
            // finish RX part, after RX complete: issue rxReady Flag
            else if (UCB2IE & UCRXIE0_1){
                i2cStatus = I2C_IS_IDLE;
                UCB2IE &= ~UCSTPIE;       // disable STOP-interrupt
            }
            break;

        case UCIV__UCRXIFG0:     // RXIFG0
            // receive single byte
            rxData_i2c[rx_index_i2c++] = UCB2RXBUF;
            break;

        case UCIV__UCTXIFG0:     // TXIFG0
            // send single byte
            UCB2TXBUF = txData_i2c[tx_index_i2c++];
            break;

        case UCIV__UCBCNTIFG:    // Byte counter zero
            break;

        case UCIV__UCCLTOIFG:    // Clock low time-out
            // report error and finish
            i2cStatus = I2C_ERR_TIMEOUT;
            break;


        default: __never_executed();
    }
}
