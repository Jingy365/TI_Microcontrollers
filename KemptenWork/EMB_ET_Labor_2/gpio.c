/*
=====================================================
  Autor:                Name
  Erstellungsdatum:     dd.mm.yyyy
  Beschreibung:         2. Labor zur Vorlesung Embedded Systems
  Datei:                GPIO.h - Initialisierung der digitalen I/O
=====================================================
*/

#include <msp430.h>

void init_GPIO(void){
    P3DIR = 0xFF;
    P3OUT = 0;
    PM5CTL0 &= ~LOCKLPM5;
    P5DIR = 0x00;
    P5REN = 0xFF;
    P5OUT = 0xFF;
}


