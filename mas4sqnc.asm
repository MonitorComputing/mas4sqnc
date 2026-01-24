;**********************************************************************
;                                                                     *
;    Description:   Controller for multiple aspect colour light       *
;                   signal and occupation block with positional train *
;                   detector at block exit.                           *
;                                                                     *
;                   This is a specialisation for United Kingdom       *
;                   Railways 4 aspect MAS signals.                    *
;                                                                     *
;    Author:        Chris White                                       *
;    Company:       Monitor Computing Services Ltd.                   *
;                                                                     * 
;                                                                     *
;**********************************************************************
;                                                                     *
;    Copyright (C) 2018  Monitor Computing Services Ltd.              *
;                                                                     *
;    This program is free software; you can redistribute it and/or    *
;    modify it under the terms of the GNU General Public License      *
;    as published by the Free Software Foundation; either version 2   *
;    of the License, or any later version.                            *
;                                                                     *
;    This program is distributed in the hope that it will be useful,  *
;    but WITHOUT ANY WARRANTY; without even the implied warranty of   *
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the    *
;    GNU General Public License for more details.                     *
;                                                                     *
;    You should have received a copy of the GNU General Public        *
;    License (http://www.gnu.org/copyleft/gpl.html) along with this   *
;    program; if not, write to:                                       *
;       The Free Software Foundation Inc.,                            *
;       59 Temple Place - Suite 330,                                  *
;       Boston, MA  02111-1307,                                       *
;       USA.                                                          *
;                                                                     *
;**********************************************************************
;                                                                     *
;                            +---+ +---+                              *
;                 Red  <- RA2|1  |_| 18|RA1 -> Green                  *
;              Yellow  <- RA3|2      17|RA0 -> Second Yellow          *
;          !Detecting <-> RA4|3      16|                              *
;                            |4      15|                              *
;                            |5      14|                              *
;     !Latch Signal On -> RB0|6      13|RB7 <-> Next / <- !Inhibit    *
;       !Line reversed -> RB1|7      12|RB6 <-> Previous              *
;   Line bidirectional -> RB2|8      11|RB5 ->  !Emitter              *
;         Normal speed -> RB3|9      10|RB4 <-  Sensor                *
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
afterRAM
            endc
endRAM      EQU afterRAM - 1
#if RAM_End < endRAM
    error "This program ran out of RAM!"
#endif


;**********************************************************************
; EEPROM initialisation
;**********************************************************************
#include "blcksqnc/blcksqnc_rom.inc"


;**********************************************************************
; Code
;**********************************************************************
UserNextRx  macro

    btfss   inputs,SPDBIT   ; Skip if normal speed input set ...
    andlw   ~SPDMSK         ; ... else display flashing warning aspects

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
    btfsc   STATUS,Z        ; Skip if not zero (not red) ...
    retlw   REDMSK          ; ... else display red aspect

    xorlw   ASPGREEN        ; Test for green aspect required
    btfsc   STATUS,Z        ; Skip if not zero (not green) ...
    retlw   GREENMSK        ; ... else display green aspect

    movlw   HALFSEC
    andwf   secCount,W      ; Test for flashing aspect blanking period

    movlw   YELLOWMSK       ; Display yellow aspect
    btfsc   aspVal,ASPDYFLG ; Skip if double yellow not required ...
    movlw   DBLYLWMSK       ; ... else display double yellow aspect

    btfsc   nxtCntlr,SPDFLG ; Skip if next signal not at normal speed ...
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
