/********************************************************************//**
 * @file LP_FR5994_spi.h
 *
 * SPI routines for Launchpad MSP430FR5994
 *
 * Board: MSP-EXP430FR5994
 * - Author: Stix
 * - Created: 06.11.2023 13:30
 *
 * Usage:
 * - include LP_FR5994_spi.h
 * - call setup_UCB0_SPI_Slave() for initialization
 ***********************************************************************/


#ifndef _LPFR5994_SPI_H
#define _LPFR5994_SPI_H

/*** included files ****************************************************/
#include <LP_FR5994.h>     // Pin definitions for LaunchPad MSP-EXP430FR5994
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

// Chip Select
#define CS_ACTIVE          (P5IN & BIT0)
#define CS_PASSIVE         (!(P5IN & BIT0))


/*******************************************************************************
    Setup SPI
    UCB1 TX Data as Slave
    Transmit to ST Nucleo Board
*******************************************************************************/
void setup_UCB0_SPI_Slave(void);


#endif /* LP_FR5994_SPI_H_ */
