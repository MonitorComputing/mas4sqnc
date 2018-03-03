;**********************************************************************
;                                                                     *
; Description: Controller for multiple aspect colour light signal and *
;              occupation block with positional train detector at     *
;              block exit.                                            *
;                                                                     *
;              This is a specialisation for United Kingdom Railways 4 *
;              aspect MAS colour light signals.                       *
;                                                                     *
;              Speed signalling input causes signals in rear of red   *
;              stop aspect to display steady single yellow, flashing  *
;              single yellow, and flashing double yellow. Speed       *
;              signalling automatically requires signal be approach   *
;              cleared.                                               *
;                                                                     *
; Author: Chris White (whitecf69@gmail.com)                           *
;                                                                     *
; Copyright (C) 2018 by Monitor Computing Services Limited, licensed  *
; under CC BY-NC-SA 4.0. To view a copy of this license, visit        *
; https://creativecommons.org/licenses/by-nc-sa/4.0/                  *
;                                                                     *
; This program is distributed in the hope that it will be useful, but *
; WITHOUT ANY WARRANTY; without even the implied warranty of          *
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                *
;                                                                     *
;**********************************************************************
;                                                                     *
;                            +---+ +---+                              *
;                 Red  <- RA2|1  |_| 18|RA1 -> Green                  *
;              Yellow  <- RA3|2      17|RA0 -> Second Yellow          *
;          !Detecting <-> RA4|3      16|RA7 -> !Emitter               *
;               Sensor -> RA5|4      15|RA6 -> !Block occupied        *
;                            |5      14|                              *
;     !Latch Signal On -> RB0|6      13|RB7 <- !ToTi (block occupied) *
;       !Line reversed -> RB1|7      12|RB6 <- !Approach clear        *
;   Line bidirectional -> RB2|8      11|RB5 <-> Next / <- !Inhibit    *
;    !Speed signalling -> RB3|9      10|RB4 <-> Previous              *
;                            +---------+                              *
;                                                                     *
;**********************************************************************


;**********************************************************************
; Configuration directives and constant definitions
;**********************************************************************
#include "blcksqnc/blcksqnc_def.inc"

; Aspect values
ASPDYFLG    EQU     ASPW2FLG
ASPGREEN    EQU     ASPCLR

; Aspect output constants
REDMSK      EQU     B'00000100' ; Mask for red aspect output
YELLOWMSK   EQU     B'00001000' ; Mask for yellow aspect output
DBLYLWMSK   EQU     B'00001001' ; Mask for double yellow aspect output
GREENMSK    EQU     B'00000010' ; Mask for green aspect output
BLANKMSK    EQU     B'00000000' ; Mask for blank aspect output


;**********************************************************************
; Variable registers
;**********************************************************************
#include "blcksqnc/blcksqnc_ram.inc"
afterRAM0
            endc
endRAM0     EQU afterRAM0 - 1
#if RAM0_End < endRAM0
    error "This program ran out of Bank 0 RAM!"
#endif


;**********************************************************************
; EEPROM initialisation
;**********************************************************************
#include "blcksqnc/blcksqnc_rom.inc"


;**********************************************************************
; Code
;**********************************************************************
UserInputs  macro

    btfss   inputs,SPDBIT   ; Skip if not speed signalling (active low) ...
    bcf     inputs,APRBIT   ; ... else speed signalling implies approach clear

    endm


UserPrevTx  macro

    movwf   FSR             ; Save the status to be sent

    ; Suppress speed signalling input unless aspect is stop
    movf    aspVal,W        ; Get aspect display value
    btfss   STATUS,Z        ; Skip if zero (red) ...
    bsf     FSR,SPDFLG      ; ... else send normal speed to previous controller

    ; Next speed signalling is not propogated if aspect is stop
    movf    aspVal,W        ; Get aspect display value
    btfsc   STATUS,Z        ; Skip if not zero (not red) ...
    goto    PrevSpeedSet    ; ... else don't propogate speed signalling

    ; Next speed signalling is not propogated if aspect is clear
    addlw   ASPINCR         ; Increment aspect value
    btfsc   STATUS,C        ; Skip if no overflow ...
    goto    PrevSpeedSet    ; ... else don't propogate speed signalling

    btfss   nxtCntlr,SPDFLG ; Skip if next signal at normal speed ...
    bcf     FSR,SPDFLG      ; ... else propogate speed signalling to previous

PrevSpeedSet
    movf    FSR,W           ; Get back the status to be sent

    endm

; Include serial link interface macros
;  - Serial link bit timing is performed by link service routines
#define CLKD_SERIAL
#include "blcksqnc/utility_pic/asyn_srl.inc"
#include "blcksqnc/utility_pic/link_hd.inc"
#include "blcksqnc/blcksqnc_cod.inc"


;**********************************************************************
; Subroutine to return aspect output value in accumulator
;**********************************************************************
GetAspectOutput
    movf    aspVal,W        ; Get aspect display value
    btfsc   STATUS,Z        ; Skip if not zero (red) ...
    retlw   REDMSK          ; ... else display red aspect

    movwf   FSR             ; Save aspect display value
    movlw   ASPINCR
    btfss   nxtCntlr,SPDFLG ; Skip if next signal at normal speed ...
    subwf   FSR,F           ; ... else reduce aspect to display
    btfsc   STATUS,Z        ; Skip if not reduced to zero (red) ...
    retlw   YELLOWMSK       ; ... else display steady single yellow aspect
    movf    FSR,W           ; Get, possibly reduced, aspect display value

    xorlw   ASPGREEN        ; Test for green aspect required
    btfsc   STATUS,Z        ; Skip if not zero (not green) ...
    retlw   GREENMSK        ; ... else display green aspect

    movlw   HALFSEC
    andwf   secCount,W      ; Test for flashing aspect blanking period

    movlw   YELLOWMSK       ; Display yellow aspect
    btfsc   FSR,ASPDYFLG    ; Skip if double yellow not required ...
    movlw   DBLYLWMSK       ; ... else display double yellow aspect

    btfsc   nxtCntlr,SPDFLG ; Skip if speed signalling ...
    return                  ; ... else display (double) yellow aspect
    btfss   STATUS,Z        ; Skip if aspect blanking period ...
    return                  ; ... else display (double) yellow aspect
    retlw   BLANKMSK        ; Blank aspect display

;**********************************************************************
; End of source code
;**********************************************************************

#if CodeEnd < $
    error "This program is just too big!"
#endif
    end     ; directive 'end of program'
