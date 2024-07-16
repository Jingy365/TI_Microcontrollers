/*
=====================================================
  Autor:                Name
  Erstellungsdatum:     dd.mm.yyyy
  Beschreibung:         2. Labor zur Vorlesung Embedded Systems
  Datei:                main.c - Bitmanipulationen
=====================================================
*/

#include <msp430.h> 
#include <stdint.h>

#include <gpio.h>
#include "led_driver.h"

//*********     CODE1    ***********
#define PORTPINa0 0
#define PORTPINa1 1
#define PORTPINa2 2
#define PORTPINa3 3
#define PORTPINa4 4
#define PORTPINa5 5
#define PORTPINa6 6
#define PORTPINa7 7
//*********     CODE2    ***********
#define BITFIELD12 0b00000110
#define BITFIELD2 0b00000100
// ********  CODE3   ***************
#define PORTPINb0_H 0b1
#define PORTPINb1_H 0b10
#define PORTPINb2_H 0b100
#define PORTPINb3_H 0b1000
#define PORTPINb4_H 0b10000
#define PORTPINb5_H 0b100000
#define PORTPINb6_H 0b1000000
#define PORTPINb7_H 0b10000000

#define PORTPINb0_L 0
#define PORTPINb1_L 0
#define PORTPINb2_L 0
#define PORTPINb3_L 0
#define PORTPINb4_L 0
#define PORTPINb5_L 0
#define PORTPINb6_L 0
#define PORTPINb7_L 0
//********   CODE 4  ********************


//********   CODE 5  ********************
#define RGB_LED_FIELD      0b11110000
#define RGB_LED_ENABLED    0b00000000
#define RGB_LED_DISABLED   0b01000000
#define RGB_LED_RED        0b10000000
#define RGB_LED_GREEN      0b00010000
#define RGB_LED_BLUE       0b00100000
//********   CODE 6  ********************



void initPort1(void){
    P1DIR = 0b11111111;                 //pins 7 to 0 -> outputs
    P1OUT = 0b00000000;                 //put the output pins 7 to 0 -> low
    P1REN = 0b00000000;                 //no internal pull up / pull down on pins 7 to 0
}


void initPort5a(void){
    P5DIR = ( (0b1 << PORTPINa7) +        //Port 5 Pin 7 -> output
              (0b0 << PORTPINa6) +        //Port 5 Pin 6 -> input
              (0b0 << PORTPINa5) +        //Port 5 Pin 5 -> input
              (0b1 << PORTPINa4) +        //Port 5 Pin 4 -> output
              (0b1 << PORTPINa3) +        //Port 5 Pin 3 -> output
              (0b1 << PORTPINa2) +        //Port 5 Pin 2 -> output
              (0b1 << PORTPINa1) +        //Port 5 Pin 1 -> output
              (0b1 << PORTPINa0) );       //Port 5 Pin 0 -> output

     P5REN = ( (0b0 << PORTPINa7) |       //Port 5 Pin 7 -> disable internal pull up / pull down
               (0b1 << PORTPINa6) |       //Port 5 Pin 6 -> enable internal pull up / pull down
               (0b1 << PORTPINa5) |       //Port 5 Pin 5 -> enable internal pull up / pull down
               (0b0 << PORTPINa4) |       //Port 5 Pin 4 -> disable internal pull up / pull down
               (0b0 << PORTPINa3) |       //Port 5 Pin 3 -> disable internal pull up / pull down
               (0b0 << PORTPINa2) |       //Port 5 Pin 2 -> disable internal pull up / pull down
               (0b0 << PORTPINa1) |       //Port 5 Pin 1 -> disable internal pull up / pull down
               (0b0 << PORTPINa0) );      //Port 5 Pin 0 -> disable internal pull up / pull down

     P5OUT = ( (0b0 << PORTPINa7) |       //Port 5 Pin 7 -> output low
               (0b1 << PORTPINa6) |       //Port 5 Pin 6 -> set internal resistor to pull up mode
               (0b1 << PORTPINa5) |       //Port 5 Pin 5 -> set internal resistor to pull up mode
               (0b0 << PORTPINa4) |       //Port 5 Pin 4 -> output low
               (0b0 << PORTPINa3) |       //Port 5 Pin 3 -> output low
               (0b0 << PORTPINa2) |       //Port 5 Pin 2 -> output low
               (0b0 << PORTPINa1) |       //Port 5 Pin 1 -> output low
               (0b0 << PORTPINa0) );      //Port 5 Pin 0 -> output low
}


void initPort5b(void){

    P5DIR = ( PORTPINb7_H |     //Port 5 Pin 7 -> output
              PORTPINb6_L |     //Port 5 Pin 6 -> input
              PORTPINb5_L |     //Port 5 Pin 5 -> input
              PORTPINb4_H |     //Port 5 Pin 4 -> output
              PORTPINb3_H |     //Port 5 Pin 3 -> output
              PORTPINb2_H |     //Port 5 Pin 2 -> output
              PORTPINb1_H |     //Port 5 Pin 1 -> output
              PORTPINb0_H );    //Port 5 Pin 0 -> output

     P5REN = ( PORTPINb7_L +    //Port 5 Pin 7 -> disable internal pull up / pull down
               PORTPINb6_H +    //Port 5 Pin 6 -> enable internal pull up / pull down
               PORTPINb5_H +    //Port 5 Pin 5 -> enable internal pull up / pull down
               PORTPINb4_L +    //Port 5 Pin 4 -> disable internal pull up / pull down
               PORTPINb3_L +    //Port 5 Pin 3 -> disable internal pull up / pull down
               PORTPINb2_L +    //Port 5 Pin 2 -> disable internal pull up / pull down
               PORTPINb1_L +    //Port 5 Pin 1 -> disable internal pull up / pull down
               PORTPINb0_L );   //Port 5 Pin 0 -> disable internal pull up / pull down

     P5OUT = ( PORTPINb6_H +       //Port 5 output pins 7,4,3,2,1,0 -> low. Input pin 6 -> set internal resistor to pull up mode.
               PORTPINb5_H );      //Port 5 output pins 7,4,3,2,1,0 -> low. Input pin 5 -> set internal resistor to pull up mode.
}



int main(void)
{
    uint8_t out;
    WDTCTL = WDTPW | WDTHOLD;                         //stop watchdog timer
    initPort1();                                      //initialize port 1
    initPort5a();                                     //initialize port 5
    //initPort5b();                                   //alternative function for port 5 initialization
    PM5CTL0 &= ~LOCKLPM5;

    volatile unsigned char aufgabe1 = 1;              //CODE7

    while(aufgabe1){                                     //loop for "Aufgabe 1"
        if(!(P5IN & (1 << PORTPINa6))){                        //CODE8
        P1OUT |= (1 << PORTPINa0);

        }
        else{
            P1OUT &= ~(1 << PORTPINa0);
        }

        if(!(P5IN & BIT5)){                        //CODE8
                P1OUT |= BIT1;

                }
                else{
                    P1OUT &= ~BIT1;
                }

    }

    init_GPIO();                                      //initialize digital I/O

    char on_off = 0;
    while(1){                                         //loop for ever
      out = getLEDpattern();                          //generate different Bit pattern


      if(on_off==1){
          out = (out & ~BITFIELD12) | BITFIELD12;
      }
      else{
 //         out = (out & ~BITFIELD12); //CODE10
      }

      if(!(P5IN&BIT6)){
          on_off = 1;
      }//CODE9
      if(!(P5IN&BIT5)){
          on_off = 0;
      }

#define RGB_LED_FIELD      0b11110000
#define RGB_LED_ENABLED    0b00000000
#define RGB_LED_DISABLED   0b01000000
#define RGB_LED_RED        0b10000000
#define RGB_LED_GREEN      0b00010000
#define RGB_LED_BLUE       0b00100000
      if((out & BIT1) && (out & BIT2)){
      out = (out & ~RGB_LED_FIELD) | RGB_LED_GREEN | RGB_LED_ENABLED;
      }
      else{
            out = (out & ~RGB_LED_FIELD) | RGB_LED_RED | RGB_LED_ENABLED;
            }


      writeLEDpattern(out);                           //write output signals
    }
}
