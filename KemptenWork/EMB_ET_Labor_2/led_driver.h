/*
=====================================================
  Autor:                Name
  Erstellungsdatum:     dd.mm.yyyy
  Beschreibung:         2. Labor zur Vorlesung Embedded Systems
  Datei:                led_driver.h - Ansteuerung der LEDs an Port P3
=====================================================
*/

#ifndef LED_DRIVER_H_
#define LED_DRIVER_H_


#define RGB_LED_FIELD      0b11110000
#define RGB_LED_ENABLED    0b00000000
#define RGB_LED_DISABLED   0b01000000
#define RGB_LED_RED        0b10000000
#define RGB_LED_GREEN      0b00010000
#define RGB_LED_BLUE       0b00100000

uint8_t getLEDpattern(void);
void writeLEDpattern(uint8_t out);


#endif /* LED_DRIVER_H_ */
