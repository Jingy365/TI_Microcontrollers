/*
=====================================================
  Autor:                Name
  Erstellungsdatum:     dd.mm.yyyy
  Beschreibung:         2. Labor zur Vorlesung Embedded Systems
  Datei:                led_driver.c - Ansteuerung der LEDs an Port P3
=====================================================
*/

#include <msp430.h>
#include <stdint.h>

#include "led_driver.h"

#define RGB_LED_WRITE(param) P3OUT = ( ( ( (param) & ~RGB_LED_FIELD) ) | ( ((~(param)) & RGB_LED_FIELD) ) )

uint8_t getLEDpattern(void){
    static uint8_t cnt = 1;
    static uint8_t cnt2 = 0;
    uint8_t LED_COLOR;
    uint8_t OUT_IM;
    __delay_cycles(100*1000UL);
    LED_COLOR = ( (cnt >> 2) << 7 ) | ((uint8_t)(cnt << 6) >> 2);
    OUT_IM = LED_COLOR;
    OUT_IM = OUT_IM | cnt2;
    cnt++;
    cnt2++;
    if (cnt == 8) cnt = 1;
    if (cnt2 == 16) cnt2 = 0;
    return(OUT_IM);
}

void writeLEDpattern(uint8_t out){
    RGB_LED_WRITE(out);
}

