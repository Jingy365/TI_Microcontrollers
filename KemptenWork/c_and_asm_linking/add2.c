
#include "add2.h"

int16_t add2(int16_t a, int16_t b){
    volatile int16_t temp = 0;
    return a+b+temp;
}



/*

;
; .global add2
; .text
;add2:
; add R13, R12
; reta
*/
