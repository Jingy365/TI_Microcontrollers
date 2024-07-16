;-------------------------------------------------------------------------------
; MSP430 Assembler Code Template for use with TI Code Composer Studio
;
;
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430.h"       ; Include device header file
            
;-------------------------------------------------------------------------------
            .def    RESET                   ; Export program entry-point to
                                            ; make it known to linker.
;-------------------------------------------------------------------------------
            .text                           ; Assemble into program memory.
            .retain                         ; Override ELF conditional linking
                                            ; and retain current section.
            .retainrefs                     ; And retain any sections that have
                                            ; references to current section.

;-------------------------------------------------------------------------------
RESET       mov.w   #__STACK_END,SP         ; Initialize stackpointer
StopWDT     mov.w   #WDTPW|WDTHOLD,&WDTCTL  ; Stop watchdog timer
 .global res1
 .global res2

 .bss res1, 2
 .bss res2, 2
 .text
 movx.a #0x3C00, SP
main:
 movx.a #0xABCDE, R12
 movx.a #0xEDCBA, R13
 pushx.a R12
 pushx.a R13
 mov #10, R12
 mov #5, R13
 calla #add2
 mov R12, &res1
 popx.a R13
 popx.a R12
 jmp main

add2:
 add R13, R12
 reta







;-------------------------------------------------------------------------------
; Main loop here
;-------------------------------------------------------------------------------

                                            

;-------------------------------------------------------------------------------
; Stack Pointer definition
;-------------------------------------------------------------------------------
            .global __STACK_END
            .sect   .stack
            
;-------------------------------------------------------------------------------
; Interrupt Vectors
;-------------------------------------------------------------------------------
            .sect   ".reset"                ; MSP430 RESET Vector
            .short  RESET
            
