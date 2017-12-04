;**********************************************************************
;                                                                     *
;    Description:   Controller for occupation block with positional   *
;                   train detector at exit.                           *
;                                                                     *
;                   Receives train detection state from previous (in  *
;                   rear) controller which it uses as entry detector  *
;                   for occupation block.                             *
;                   Sends value of signal aspect (increment of local  *
;                   value of signal aspect) along with special speed  *
;                   indication and block reversed to previous         *
;                   controller.                                       *
;                                                                     *
;                   Receives value of signal aspect to be displayed   *
;                   along with special speed indication and block     *
;                   reversed from next (in advance) controller.       *
;                   Sends train detection state to next controller.   *
;                                                                     *
;                   If no data is received from next controller link  *
;                   input is treated as a level input indicating      *
;                   to display a stop aspect or to cycle aspect from  *
;                   stop to clear at fixed intervals after the        *
;                   passing of a train.                               *
;                                                                     *
;                   Outputs aspect display for United Kingdom         *
;                   Railways 4 aspect MAS signals.                    *
;                                                                     *
;    Author:        Chris White                                       *
;    Company:       Monitor Computing Services Ltd.                   *
;                                                                     * 
;                                                                     *
;**********************************************************************
;                                                                     *
;    Copyright (C) 2017  Monitor Computing Services Ltd.              *
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
;             Emitter  <- RA2|1  |_| 18|RA1                           *
;              Sensor  -> RA3|2      17|RA0                           *
;          !Detecting  <- RA4|3      16|                              *
;                            |4      15|                              *
;                            |5      14|      Aspects:                *
;                         RB0|6      13|RB7 -> Red                    *
; Next <-> / !Inhibit  -> RB1|7      12|RB6 -> Yellow                 *
;            Previous <-> RB2|8      11|RB5 -> Double yellow          *
;       Special speed  -> RB3|9      10|RB4 -> Green                  *
;                            +---------+                              *
;                                                                     *
;**********************************************************************


;**********************************************************************
; Configuration directives and constant definitions
;**********************************************************************
#include "blcksqnc/blcksqnc_def.inc"
; Aspect values
ASPDYFLG    EQU     ASPCLFLG
ASPGREEN    EQU     ASPCLR

; Aspect output constants
REDOUT      EQU     7           ; Red aspect output bit
REDMSK      EQU     B'10000000' ; Mask for red aspect output bit
YELLOWOUT   EQU     6           ; Yellow aspect output bit
YELLOWMSK   EQU     B'01000000' ; Mask for yellow aspect output bit
DOUBLEOUT   EQU     4           ; Double yellow aspect output bit
DBLYLWMSK   EQU     B'00010000' ; Mask for double yellow aspect output bit
GREENOUT    EQU     5           ; Green aspect output bit
GREENMSK    EQU     B'00100000' ; Mask for green aspect output bit
  
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
#include "blcksqnc/blcksqnc_cod.inc"

;**********************************************************************
; Subroutine to return aspect output mask in accumulator
;**********************************************************************
GetAspectMask
    movf    aspOut,W        ; Get aspect display value
    btfsc   STATUS,Z        ; Skip if not zero (not red) ...
    retlw   REDMSK          ; ... else display red aspect

    xorlw   ASPGREEN        ; Test for green aspect required
    btfsc   STATUS,Z        ; Skip if not zero (not green) ...
    retlw   GREENMSK        ; ... else display green aspect

    movlw   HALFSEC
    andwf   secCount,w      ; Test for flashing aspect blanking period

    movlw   YELLOWMSK       ; Display yellow aspect
    btfsc   aspOut,ASPDYFLG ; Skip if double yellow not required ...
    movlw   DBLYLWMSK       ; ... else display double yellow aspect

    btfss   nxtCntlr,SPDFLG ; Skip if next signal at special speed ...
    return                  ; ... else display (double) yellow aspect
    btfss   STATUS,Z        ; Skip if aspect blanking period ...
    return                  ; ... else display (double) yellow aspect
    retlw   0               ; Blank aspect display

;**********************************************************************
; End of source code
;**********************************************************************

#if CodeEnd < $
    error "This program is just too big!"
#endif
    end     ; directive 'end of program'
