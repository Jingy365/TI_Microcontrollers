;=====================================================
;  Autor:                Name
;  Erstellungsdatum:     dd.mm.yyyy
;  Beschreibung:         3. Labor zur Vorlesung Embedded Systems
;  Datei:                main.asm - Geräteregister, Adressierungsarten, Makros, Unterprogramme und Stack
;=====================================================

;
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430.h"       ; Include device header file
            
;-------------------------------------------------------------------------------
            .def    RESET                   ; Export program entry-point to
                                            ; make it known to linker.

;--------------------------C O N S T A N T S---------------------------------------

TIME		.set 60000

BIT0_1		.set 00000001B
BIT1_1     	.set 00000010B
BIT2_1     	.set 00000100B
BIT3_1		.set 00001000B
BIT4_1     	.set 00010000B
BIT5_1    	.set 00100000B
BIT6_1		.set 01000000B
BIT7_1    	.set 10000000B

BIT0_0	 	.set 00000000B
BIT1_0      .set 00000000B
BIT2_0    	.set 00000000B
BIT3_0		.set 00000000B
BIT4_0    	.set 00000000B
BIT5_0    	.set 00000000B
BIT6_0		.set 00000000B
BIT7_0    	.set 00000000B


;---------------------------V A R I A B L E S--------------------------------------

			.global RAMFUNC
			.bss RAMFUNC, 50			; reserve space for RAMFUNC

;--------------------------   M A C R O    -----------------------------------------
DELAY		.macro time, Reg
				mov time, Reg
start?:			sub #1, Reg
				jnz start?
				nop
			.endm
;-----------------------------------------------------------------------------------

;-------------------------------------------------------------------------------
            .text                           ; Assemble into program memory.
            .retain                         ; Override ELF conditional linking
                                            ; and retain current section.
            .retainrefs                     ; And retain any sections that have
                                            ; references to current section.

;-------------------------------------------------------------------------------
RESET       mov.w   #WDTPW|WDTHOLD,&WDTCTL  ; Stop watchdog timer

											;CODE 1

			mov.b #(BIT7_1+BIT6_1+BIT5_1+BIT4_1+BIT3_1+BIT2_1+BIT1_1+BIT0_1), &P2DIR	;CODE 2



			mov    #0, PM5CTL0
			movx.a #StartLed1Flash, R4		;CODE 3



;-------------------------------------------------------------------------------
; Main starts loop here
;-------------------------------------------------------------------------------
 mov.b #0x9F, &0x244
 mov.b #0x60, &0x246
 mov.b #0x60, &0x242
 mov.b #0xFF, &0x204
 movx.a #0x3C00, sp

main:			calla #hallo


						;CODE 4
				bit.b #BIT6, &P5IN
				jz switch_on_LED1
				bic.b #BIT0, &P1OUT
				jmp check_switch_2			;CODE 5



switch_on_LED1:
 bis.b #BIT0, &P1OUT
 jmp check_switch_2

check_switch_2:
 				bit.b #BIT5, &P5IN
				jz switch_on_LED2
				bic.b #BIT1, &P1OUT
				jmp main

switch_on_LED2:
 bis.b #BIT1, &P1OUT
 jmp main

hallo:
 bis.b #BIT1, &P1OUT
 reta
											;CODE 6
StartLed1Flash:		DELAY #TIME, R9			;CODE 7
					xor.b #BIT1_1, P1OUT	;CODE 8
					jmp StartLed1Flash
StopLed1Flash:		mov R6, R6				;CODE 9



											;CODE 10





            
;-------------------------------------------------------------------------------
; Interrupt Vectors
;-------------------------------------------------------------------------------
            .sect   ".reset"                ; MSP430 RESET Vector
            .short  RESET
            
