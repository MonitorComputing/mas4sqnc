;**********************************************************************
;                                                                     *
;    Filename:	    blcksqnc.asm                                      *
;    Date:          26 April 2002                                     *
;    File Version:  1                                                 *
;                                                                     *
;    Author:        Chris White                                       *
;    Company:       Monitor Computing Services Ltd.                   *
;                                                                     * 
;                                                                     *
;**********************************************************************
;                                                                     *
;    Copyright (C) 2002  Monitor Computing Services Ltd.              *
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


	list      p=16C84

;**********************************************************************
; Include and configuration directives                                *
;**********************************************************************

#include <p16C84.inc>

	__CONFIG   _CP_OFF & _WDT_OFF & _PWRTE_ON & _XT_OSC

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
; Selection is: Code Protection - off, Watchdog Timer - off, Power-up Timer - on,
;               Oscillator - Crystal/Resonator


; Include serial interface macro definitions
#include <\dev\projects\picsrl\asyn_srl.inc>


;**********************************************************************
; Constant definitions                                                *
;**********************************************************************

INDPORT		EQU     PORTA
INTIND		EQU     1
USRIND		EQU     2

PORTASTATUS	EQU     B'11110001'
PORTBSTATUS	EQU     B'00001011'

; Interrupt & timing constants
RTCCINT		EQU     158       ; 1Mhz / 100 (adjusted for RTCC write inhibit) = 10Khz

INT5KBIT	EQU     2         ; Interrupts per serial bit @ 5K baud
INT5KINI	EQU     3         ; Interrupts per initial Rx serial bit @ 5K (start bit)
INT2K5BIT	EQU     4         ; Interrupts per serial bit @ 2K5 baud
INT2K5INI	EQU     6         ; Interrupts per initial Rx serial bit @ 2K5 (start bit)

; Next signal interface constants
RXNFLAG		EQU     0         ; Receive byte buffer 'loaded' status bit
RXNERR		EQU     1         ; Receive error status bit
RXNBREAK	EQU     2         ; Received 'break' status bit
RXNTRIS		EQU     TRISB     ; Rx port direction register
RXNPORT		EQU     PORTB     ; Rx port data register
RXNBIT		EQU     1         ; Rx input bit

; Previous signal interface constants
TXPFLAG		EQU     3         ; Transmit byte buffer 'clear' status bit
TXPTRIS		EQU     TRISB     ; Tx port direction register
TXPPORT		EQU     PORTB     ; Tx port data register
TXPBIT		EQU     2         ; Tx output bit

; Monitor interface constants
RXMFLAG		EQU     4         ; Receive byte buffer 'loaded' status bit
RXMERR		EQU     5         ; Receive error status bit
RXMBREAK	EQU     6         ; Received 'break' status bit
TXMFLAG		EQU     7         ; Transmit byte buffer 'clear' status bit
TXMTRIS		EQU     TRISA     ; Tx port direction register
TXMPORT		EQU     PORTA     ; Tx port data register
TXMBIT		EQU     3         ; Tx output bit
RXMTRIS		EQU     TRISA     ; Rx port direction register
RXMPORT		EQU     PORTA     ; Rx port data register
RXMBIT		EQU     4         ; Rx input bit


;**********************************************************************
; Variable definitions                                                *
;**********************************************************************

		CBLOCK  0x0C

; Status and accumulator storage registers
w_isr         ; 'w' register, accumulator, store during ISR
pclath_isr    ; PCLATH register store during ISR
status_isr    ; status register store during ISR

; Serial interface registers
srlIfStat     ; Serial I/F status flags

; Next signal interface registers
serNRxTmr     ; Interrupt counter for serial bit timing
serNRxReg     ; Data shift register
serNRxByt     ; Data byte buffer
serNRxBitCnt  ; Bit down counter

; Previous signal interface registers
serPTxTmr     ; Interrupt counter for serial bit timing
serPTxReg     ; Data shift register
serPTxByt     ; Data byte buffer
serPTxBitCnt  ; Bit down counter

; Monitor interface registers
serMRxTmr     ; Interrupt counter for serial bit timing
serMRxReg     ; Data shift register
serMRxByt     ; Data byte buffer
serMRxBitCnt  ; Bit down counter
serMTxTmr     ; Interrupt counter for serial bit timing
serMTxReg     ; Data shift register
serMTxByt     ; Data byte buffer
serMTxBitCnt  ; Bit down counter

		ENDC


;**********************************************************************
; EEPROM initialisation                                               *
;**********************************************************************

		ORG     0x2100            ; EEPROM data area

EEaspectTime	DE      5 + 1             ; Seconds to delay between aspect changes


;**********************************************************************
; Reset vector                                                        *
;**********************************************************************

		ORG     0x000             ; Processor reset vector
BootVector	clrf    INTCON            ; Disable interrupts
		clrf    INTCON            ; Ensure interrupts are disabled
  		goto    Boot              ; Jump to beginning of program


;**********************************************************************
; Interrupt vector                                                    *
;**********************************************************************

		ORG     0x004             ; Interrupt vector location
IntVector	movwf   w_isr             ; Save off current W register contents
		swapf	STATUS,W          ; Swap status register into W register
		BANKSEL TMR0              ; Ensure register page 0 is selected
		movwf	status_isr        ; save off contents of STATUS register
		movf	PCLATH,W          ; Move PCLATH register into W register
		movwf	pclath_isr        ; save off contents of PCLATH register
		movlw   high BeginISR     ; Load ISR address high byte ...
		movwf   PCLATH            ; ... into PCLATH to set code block
		goto    BeginISR          ; Jump to interrupt service routine


;**********************************************************************
; Monitor interface routine macro invocations                         *
;**********************************************************************

EnableMRx	EnableRx  RXMTRIS, RXMPORT, RXMBIT
		return


InitMRx		InitRx  serMRxTmr, srlIfStat, RXMFLAG, RXMERR, RXMBREAK
		return


SrvcMRx		ServiceRx serMRxTmr, RXMPORT, RXMBIT, serMRxBitCnt, INT5KINI, srlIfStat, RXMERR, RXMBREAK, serMRxReg, serMRxByt, RXMFLAG, INT5KBIT


EnableMTx	EnableTx  TXMTRIS, TXMPORT, TXMBIT
		return


InitMTx		InitTx  serMTxTmr, srlIfStat, TXMFLAG
		return


SrvcMTx		ServiceTx serMTxTmr, srlIfStat, serMTxByt, serMTxReg, TXMFLAG, serMTxBitCnt, INT5KBIT, TXMPORT, TXMBIT, 0, 0


LinkMRx
SerMRx		SerialRx srlIfStat, RXMFLAG, serMRxByt


LinkMTx
SerMTx		SerialTx srlIfStat, TXMFLAG, serMTxByt


;**********************************************************************
; Monitor code                                                        *
;**********************************************************************

; Select to display 'user' banner
#define GOTUSERBANNER
; Select to run user code when booted
#define MONUSERON
#include <\dev\projects\monitor\pic\pic_mntr.inc>


;**********************************************************************
; Signal interface routine macro invocations                          *
;**********************************************************************

EnableNRx	EnableRx  RXNTRIS, RXNPORT, RXNBIT
		return


InitNRx		InitRx  serNRxTmr, srlIfStat, RXNFLAG, RXNERR, RXNBREAK
		return


SrvcNRx		ServiceRx serNRxTmr, RXNPORT, RXNBIT, serNRxBitCnt, INT2K5INI, srlIfStat, RXNERR, RXNBREAK, serNRxReg, serNRxByt, RXNFLAG, INT2K5BIT


LinkNRx
SerNRx		SerialRx srlIfStat, RXNFLAG, serNRxByt


EnablePTx	EnableTx  TXPTRIS, TXPPORT, TXPBIT
		return


InitPTx		InitTx  serPTxTmr, srlIfStat, TXPFLAG
		return


SrvcPTx		ServiceTx serPTxTmr, srlIfStat, serPTxByt, serPTxReg, TXPFLAG, serPTxBitCnt, INT2K5BIT, TXPPORT, TXPBIT, 0, 0


LinkPTx
SerPTx		SerialTx srlIfStat, TXPFLAG, serPTxByt


;**********************************************************************
; Main program initialisation code                                    *
;**********************************************************************

Boot
		; Clear I/O ports
		clrf    PORTA
		clrf    PORTB

		BANKSEL OPTION_REG

		; Program I/O port bit directions
		movlw   PORTASTATUS
		movwf   TRISA
		movlw   PORTBSTATUS
		movwf   TRISB

 		; Set option register:
		;   Port B pull-up       - off
 		;   Prescaler assignment - watchdog timer
		clrf    OPTION_REG
		bsf     OPTION_REG,NOT_RBPU
		bsf     OPTION_REG,PSA

		BANKSEL TMR0

		movlw   PORTASTATUS       ; For Port A need to write one to each bit ...
		movwf   PORTA             ; ... being used for input

		; Initialise serial links
		SerInit    srlIfStat, serPTxTmr, serPTxReg, serPTxByt, serPTxBitCnt, serNRxTmr, serNRxReg, serNRxByt, serNRxBitCnt
		SerInit    srlIfStat, serMTxTmr, serMTxReg, serMTxByt, serMTxBitCnt, serMRxTmr, serMRxReg, serMRxByt, serMRxBitCnt
		call    EnableNRx
		call    InitNRx
		call    EnablePTx
		call    InitPTx
		call    EnableMRx
		call    InitMRx
		call    EnableMTx
		call    InitMTx

		call    UserInit          ; Run user initialisation code

		; Initialise interrupts
		movlw   RTCCINT
		movwf   TMR0              ; Initialise RTCC for timer interrupts
		clrf    INTCON            ; Disable all interrupt sources
		bsf	INTCON,T0IE       ; Enable RTCC interrupts
		bsf	INTCON,GIE        ; Enable interrupts


		goto    MonitorMain       ; Run monitor program


;**********************************************************************
; Interrupt service routine (ISR) code                                *
;**********************************************************************

BeginISR	btfss   INTCON,T0IF       ; Test for RTCC Interrupt
		retfie                    ; If not, skip service routine

		bsf     INDPORT,INTIND

		bcf     INTCON,T0IF       ; Reset the RTCC Interrupt bit
		movlw   RTCCINT
		addwf   TMR0,F            ; Reload RTCC

		call    SrvcMRx           ; Perform monitor interface Rx service
		call    SrvcMTx           ; Perform monitor interface Tx service
		call    SrvcNRx           ; Perform 'next' signal interface Rx service
		call    SrvcPTx           ; Perform 'previous' signal interface Tx service

		MonitorISR

EndISR		movf    pclath_isr,W      ; Retrieve copy of PCLATH register
		movwf	PCLATH            ; Restore pre-isr PCLATH register contents
		swapf   status_isr,W      ; Swap copy of STATUS register into W register
		movwf	STATUS            ; Restore pre-isr STATUS register contents
		swapf   w_isr,F           ; Swap pre-isr W register value nibbles
		swapf   w_isr,W           ; Swap pre-isr W register into W register

		bcf     INDPORT,INTIND

		retfie                    ; return from Interrupt


;**********************************************************************
; User constants                                                      *
;**********************************************************************

; Timing constants

INTMILLI	EQU     10 + 1      ; Interrupts per millisecond

SECMILLILOW	EQU     0xE8        ; Milliseconds per second low byte
SECMILLIHIGH	EQU     0x03        ; Milliseconds per second high byte

NEXTTIMEOUT	EQU     100 + 1     ; 'Next' signal link failed timeout in milliseconds

; Inhibit (force display of red aspect) input constants
INHPORT		EQU     PORTB       ; Inhibit input port
INHIN		EQU     1           ; Inhibit input bit

; Detector input constants
DETPORT		EQU     PORTB       ; Detector input port
DETIN		EQU     0           ; Detector input bit

INPHIGHWTR	EQU     200         ; Input debounce accumulator "On" threshold value
INPLOWWTR	EQU     55          ; Input debounce accumulator "Off" threshold value

; Signalling status constants
BLKSTATE	EQU     B'00000011' ; Mask to isolate signal block state bits

; State values, 'this' block
BLOCKCLEAR	EQU     0           ; Block clear state value
TRAINENTERING	EQU     1           ; Train entering block state value
BLOCKOCCUPIED	EQU     2           ; Block occupied state value
TRAINLEAVING	EQU     3           ; Train leaving block state value

; State values, 'next' block
NEXTSQNCING	EQU     0           ; Sequencing signal aspects state value
NEXTAPPRCHING	EQU     1           ; Train approaching block state value
NEXTENTERING	EQU     2           ; Train entering block state value
NEXTOCCUPIED	EQU     3           ; Block occupied state value

ASPSTATE	EQU     B'11000000' ; Mask to isolate aspect value bits
ASPSTSWP	EQU     B'00001100' ; Mask to isolate swapped nibbles aspect value bits
ASPINCR		EQU     B'01000000' ; Value to increment aspect value
ASPGREEN	EQU     B'11000000' ; Green aspect value
ASPDOUBLE	EQU     B'10000000' ; Mask to test for double yellow aspect value

INHBIT		EQU     4           ; Inhibit bit in status byte
INHSTATE	EQU     B'00010000' ; Mask to isolate inhibit state bit

DETBIT		EQU     5           ; Detector bit in status byte
DETSTATE	EQU     B'00100000' ; Mask to isolate detector state bit

; Aspect output constants
ASPPORT		EQU     PORTB       ; Aspect output port
DOUBLEOUT	EQU     7           ; Second yellow aspect output bit
GREENOUT	EQU     6           ; Green aspect output bit
YELLOWOUT	EQU     5           ; Yellow aspect output bit
REDOUT		EQU     4           ; Red aspect output bit


;**********************************************************************
; User variables                                                      *
;**********************************************************************

		CBLOCK

milliCount    ; Interrupt counter for millisecond timing

secCountLow   ; Millisecond counter (low byte) for second timing
secCountHigh  ; Millisecond counter (high byte) for second timing

detAcc        ; Detector input debounce accumulator
inhAcc        ; Inhibit input debounce accumulator

sigState      ; Signalling status
              ;   bits 0,1 - Signal block state
              ;     3 - Train leaving block
              ;     2 - Block occupied
              ;     1 - Train entering Block
              ;     0 - Block Clear
              ;   bit 4 - Inhibit state
              ;   bit 5 - Detector state
              ;   bits 6,7 - Aspect value
              ;     3 - Green
              ;     2 - Double Yellow
              ;     1 - Yellow
              ;     0 - Red

nxtState      ; Signalling status received from 'next' signal
              ;   bit 5 - Detector state
              ;   bits 6,7 - Aspect value
              ;     3 - Green
              ;     2 - Double Yellow
              ;     1 - Yellow
              ;     0 - Red

aspectTime    ; Aspect interval for simulating 'next' signal when non connected
nxtTimer      ; Second counter for simulating 'next' signal when non connected
nxtLnkTmr     ; Millisecond counter for timing out 'next' signal link
telemData     ; Data received from 'next' or sent to 'previous' signal

		ENDC


;**********************************************************************
; User banner display code                                            *
;**********************************************************************

UserBanner	movlw   crCode
		call    TxLoop
		movlw   lfCode
		call    TxLoop
		movlw   'M'
		call    TxLoop
		movlw   'A'
		call    TxLoop
		movlw   'S'
		movlw   ' '
		call    TxLoop
		movlw   's'
		call    TxLoop
		movlw   'e'
		call    TxLoop
		movlw   'q'
		call    TxLoop
		movlw   'u'
		call    TxLoop
		movlw   'e'
		call    TxLoop
		movlw   'n'
		call    TxLoop
		movlw   'c'
		call    TxLoop
		movlw   'e'
		call    TxLoop
		movlw   'r'
		call    TxLoop
		movlw   ' ' 
		call    TxLoop
		movlw   '1'
		call    TxLoop
		movlw   'a'
		call    TxLoop
		movlw   crCode
		call    TxLoop
		movlw   lfCode

TxLoop		movwf   FSR               ; Copy W to FSR, for sending
LoopTx		call    LinkMTx           ; Try to send data
		btfss   STATUS,Z          ; Skip if data sent ...
		goto    LoopTx            ; ... otherwise keep trying to send

		return


;**********************************************************************
; User initialisation code                                            *
;**********************************************************************

UserInit	; Initialise millisecond Interrupts counter
		movlw   INTMILLI
		movwf   milliCount

		; Initialise one second milliseconds counter
		movlw   SECMILLILOW
		movwf   secCountLow
		movlw   SECMILLIHIGH
		movwf   secCountHigh

		; Clear input debounce accumulators
		clrf    detAcc
		clrf    inhAcc

		; Initialise aspect values to show green
		movlw   ASPGREEN
		movwf   sigState
		movwf   nxtState

		; Initialise timers
		movlw   low EEaspectTime
		call    GetEEPROM
		movwf   aspectTime        ; Initialise aspect interval for 'next' signal
		movwf   nxtTimer          ; Initialise timer used to simulate 'next' signal
		movlw   NEXTTIMEOUT       ; Initialise 'next' signal ...
		movwf   nxtLnkTmr         ; ... link timeout

		clrf    telemData 

		return


;**********************************************************************
; User interrupt service routine (ISR) code                           *
;**********************************************************************

UserInt
		; Run interrupt counter for millisecond timing

		decfsz  milliCount,W      ; Decrement millisecond Interrupt counter into W
		movwf   milliCount        ; Put decremented count back into counter

		; Debounce detector input

		btfss   DETPORT,DETIN     ; Skip if detector input is set ...
		goto    DecDetAcc         ; ... otherwise jump if not set

		incfsz  detAcc,W          ; Increment detector input accumulator into W
		movwf   detAcc            ; Put incremented count back into accumulator
		goto    DetDbncEnd   

DecDetAcc	decfsz  detAcc,W          ; Decrement detector input accumulator into W
		movwf   detAcc            ; Put decremented count back into accumulator

DetDbncEnd

		; Debounce inhibit input

		btfss   INHPORT,INHIN     ; Skip if inhibit input is set ...
		goto    DecInhAcc         ; ... otherwise jump if not set

		incfsz  inhAcc,W          ; Increment inhibit input accumulator into W
		movwf   inhAcc            ; Put incremented count back into accumulator
		goto    InhDbncEnd   

DecInhAcc	decfsz  inhAcc,W          ; Decrement inhibit input accumulator into W
		movwf   inhAcc            ; Put decremented count back into accumulator

InhDbncEnd
		return


;**********************************************************************
; User main loop code                                                 *
;**********************************************************************

UserMain                                  ; Top of main processing loop

		bsf     INDPORT,USRIND

		; Perform timing operations

Timing		decfsz  milliCount,W      ; Test millisecond Interrupts counter
		goto    TimingEnd         ; Skip timing if a millisecond hasn't elapsed

		movlw   INTMILLI          ; Reload millisecond Interrupts counter
		movwf   milliCount

		decfsz  nxtLnkTmr,W       ; Decrement 'next' signal link timeout timer into W
		movwf   nxtLnkTmr         ; Put decremented count back into timer

		decfsz  secCountLow,F     ; Decrement second milliseconds counter low byte ...
		goto    TimingEnd         ; ... skipping this jump if reached zero
		decfsz  secCountHigh,F    ; Decrement second milliseconds counter high byte
		goto    TimingEnd         ; Jump timing processing if a second has not elapsed

		movlw   SECMILLILOW       ; Reload one second milliseconds ...
		movwf   secCountLow       ; ... counter low byte
		movlw   SECMILLIHIGH      ; Reload one second milliseconds ...
		movwf   secCountHigh      ; ... counter high byte

		decfsz  nxtTimer,W        ; Decrement 'next' signal timer into W
		movwf   nxtTimer          ; Put decremented count back into timer

TimingEnd

		; Check status of detector input

Detect		btfss   sigState,DETBIT   ; Skip if detector state is "On" ...
		goto    DetectOff         ; ... otherwise jump if state is "Off"

		movf    detAcc,W          ; Test if detector debounce accumulator ...
		sublw   INPLOWWTR         ; ... is above "Off" threshold
		btfss   STATUS,C          ; Skip if at or below threshold ...
		goto    DetectEnd         ; ... otherwise jump if above threshold

		; Detector has turned "Off"
		bcf     sigState,DETBIT   ; Set detector state to "Off"
		goto    DetectEnd

DetectOff	movf    detAcc,W          ; Test if detector debounce accumulator ...
		sublw   INPHIGHWTR        ; ... is above "On" threshold
		btfsc   STATUS,C          ; Skip if above threshold ...
		goto    DetectEnd         ; ... otherwise jump if at or below threshold

		; Detector has turned "On"
		bsf     sigState,DETBIT   ; Set detector state to "On"

DetectEnd

		; Look for status from 'next' signal

		call    LinkNRx           ; Check for data from 'next' signal
		btfss   STATUS,Z          ; Skip if data received ...
		goto    TimeoutNext       ; ... otherwise check for link timedout

		; Decode received data
		movwf   telemData         ; Store the received data
		swapf   telemData,W       ; Copy received data but with nibbles swapped
		comf    telemData,F       ; One's complement the received data
		xorwf   telemData,W       ; Exclusive or complemented and swapped data
		btfss   STATUS,Z          ; Skip if result is zero, data ok ...
		goto    NxtBlkEnd         ; ... otherwise ignore received data

		comf    telemData,W       ; Store (original) received data ...
                andlw   ~BLKSTATE         ; ... (with simulation state ...
		iorlw   NEXTSQNCING       ; ...  set to 'sequence aspects') ...
		movwf   nxtState          ; ... as 'next' signal status

		movlw   NEXTTIMEOUT       ; Reset 'next' signal ...
		movwf   nxtLnkTmr         ; ... link timeout

		; If 'next' signal link is not timed out then ignore inhibit input
		bcf     sigState,INHBIT   ; Set inhibit state to "Off"

		goto    NxtBlkEnd

		; Test if 'next' signal link has timedout, i.e. not connected

TimeoutNext	decfsz  nxtLnkTmr,W       ; Skip if link timeout elapsed ...
		goto    NxtBlkEnd         ; ... otherwise keep waiting for data


		; Link to 'next' signal timedout

		; Check status of inhibit input

Inhibit		btfss   sigState,INHBIT   ; Skip if inhibit state is "On"
		goto    InhibitOff        ; Jump if state is "Off"

		movf    inhAcc,W          ; Test if inhibit debounce accumulator ...
		sublw   INPLOWWTR         ; ... is above "Off" threshold
		btfss   STATUS,C          ; Skip if at or below threshold ...
		goto    InhibitEnd        ; ... otherwise jump if above threshold

		; Inhibit input has turned "Off"
		bcf     sigState,INHBIT   ; Set inhibit state to "Off"
		goto    InhibitEnd

InhibitOff	movf    inhAcc,W          ; Test if inhibit debounce accumulator ...
		sublw   INPHIGHWTR        ; ... is above "On" threshold
		btfsc   STATUS,C          ; Skip if above threshold ...
		goto    InhibitEnd        ; ... otherwise jump if at or below threshold

		; Inhibit input has turned "On"
		bsf     sigState,INHBIT   ; Set inhibit state to "On"

InhibitEnd
		; Simulate 'next' signal

		movlw   high NxtBlkTable  ; Load jump table address high byte ...
		movwf   PCLATH            ; ... into PCLATH to make jump in same code block
		movf    nxtState,W        ; Use current state value ...
		andlw   BLKSTATE
		addwf   PCL,F             ; ... as offset into state jump table

NxtBlkTable	goto    NxtBlkSeqncing    ; State 0 - Sequencing signal aspects
		goto    NxtBlkApproach    ; State 1 - Train approaching block
		goto    NxtBlkEntering    ; State 2 - Train entering block
		goto    NxtBlkOccupied    ; State 3 - Block occupied

#if (high NxtBlkTable) != (high $)
    error "Next signal block state jump table split across page boundary"
#endif


NxtBlkSeqncing	; State - Sequencing signal aspects

		decfsz  nxtTimer,W        ; Skip if signalling timer elapsed ...
		goto    NxtBlkDetect      ; ... otherwise skip signal aspect sequencing

		; Advance 'next' signal aspect
		movlw   ASPINCR
		addwf   nxtState,W        ; Increment to next aspect value
		btfss   STATUS,C          ; Skip if overflow, already showing 'green'
		movwf   nxtState          ; Store new aspect value

		; Set signalling timer for duration of aspect
		movf    aspectTime,W
		movwf   nxtTimer

NxtBlkDetect	btfss   sigState,DETBIT   ; Skip if detector "On" ...
		goto    NxtBlkEnd         ; ... otherwise remain in current state

		; Train detected,
		;   next state - Train approaching
		movlw   ~BLKSTATE
                andwf   nxtState,W
		iorlw   NEXTAPPRCHING
		movwf   nxtState


NxtBlkApproach	; State - Train approaching

		btfsc   sigState,DETBIT   ; Skip if detector "Off" ...
		goto    NxtBlkEnd         ; ... otherwise remain in current state

		; Train no longer detected,
		;   next state - Train entering block
		; (set 'next' signal aspect value to 'red' at same time)
		movlw   ~(BLKSTATE | ASPSTATE)
                andwf   nxtState,W
		iorlw   NEXTENTERING
		movwf   nxtState

		bsf     nxtState,DETBIT   ; Set 'next' signal detector to 'On'

		; Set signalling timer for time taken to enter block by train
		movf    aspectTime,W
		movwf   nxtTimer


NxtBlkEntering	; State - Train entering block

		decfsz  nxtTimer,W        ; Skip if signalling timer elapsed ...
		goto    NxtBlkEnd         ; ... otherwise remain in current state

		; Train has entered 'next' signalling block,
		;   next state - Block occupied
                movlw   ~BLKSTATE
                andwf   nxtState,W
		iorlw   NEXTOCCUPIED
		movwf   nxtState

		bcf     nxtState,DETBIT   ; Set 'next' signal detector to 'Off'

		; Start timer to simulate train traversing signal block
		movf    aspectTime,W
		movwf   nxtTimer


NxtBlkOccupied	; State - Block occupied

		decfsz  nxtTimer,W        ; Skip if signalling timer elapsed ...
		goto    NxtBlkEnd         ; ... otherwise remain in current state

		; Train has traversed signalling block,
		;   next state - Sequencing signal aspects
                movlw   ~BLKSTATE
                andwf   nxtState,W
		iorlw   NEXTSQNCING
		movwf   nxtState

		; Start timer to simulate time taken by train to leave signal block
		movf    aspectTime,W
		movwf   nxtTimer

NxtBlkEnd

		; Run signal block state machine

		movlw   high BlockTable   ; Load jump table address high byte ...
		movwf   PCLATH            ; ... into PCLATH to make jump in same code block
		movf    sigState,W        ; Use current state value ...
		andlw   BLKSTATE
		addwf   PCL,F             ; ... as offset into state jump table

BlockTable	goto    BlockClear        ; State  0 - Block clear
		goto    TrainEntering     ; State  1 - Train entering Block
		goto    BlockOccupied     ; State  2 - Block occupied
		goto    TrainLeaving      ; State  3 - Train leaving block

#if (high BlockTable) != (high $)
    error "Signal block state jump table split across page boundary"
#endif


BlockClear	; State - Block clear

		; Set signal aspect
		movlw   ~ASPSTATE
		andwf   sigState,F        ; Clear current aspect value bits
		movlw   ASPINCR
		addwf   nxtState,W        ; Increment 'next' aspect value into W
		btfsc   STATUS,C          ; Skip if no overflow ...
		iorlw   ASPGREEN          ; ... otherwise set for green aspect
		andlw   ASPSTATE          ; Isolate new aspect value bits   
		iorwf   sigState,F        ; Set new aspect value

BlockDetect	btfss   sigState,DETBIT   ; Skip if detector "On" ...
		goto    BlockEnd          ; ... otherwise remain in current state

		; Train detected at block entrance,
		;   next state - Train entering block
		; (set signal aspect value to 'red' at same time)
		movlw   ~(BLKSTATE | ASPSTATE)
                andwf   sigState,W
		iorlw   TRAINENTERING
		movwf   sigState


TrainEntering	; State - Train entering block

		btfsc   sigState,DETBIT   ; Skip if detector "Off" ...
		goto    BlockEnd          ; ... otherwise remain in current state

		; Train no longer detected at block entrance,
		;   next state - Block occupied
                movlw   ~BLKSTATE
                andwf   sigState,W
		iorlw   BLOCKOCCUPIED
		movwf   sigState


BlockOccupied	; State - Block occupied

		btfss   nxtState,DETBIT   ; Skip if 'next' detector "On" ...
		goto    BlockEnd          ; ... otherwise remain in current state

		; Train detected at block exit,
		;   next state - Train leaving block
                movlw   ~BLKSTATE
                andwf   sigState,W
		iorlw   TRAINLEAVING
		movwf   sigState


TrainLeaving	; State - Train leaving block

		btfsc   nxtState,DETBIT   ; Skip if 'next' detector "Off" ...
		goto    BlockEnd          ; ... otherwise remain in current state

		; Train no longer detected at block exit,
		;   next state - block clear

                movlw   ~BLKSTATE
                andwf   sigState,W
		iorlw   BLOCKCLEAR
		movwf   sigState

BlockEnd
		; Set aspect display output

		btfsc   sigState,INHBIT   ; Skip if not forced red aspect display ...
		goto    RedAspect         ; ... otherwise display red aspect

		movlw   ASPSTATE          ; Test for red aspect required
		andwf   sigState,W
		btfsc   STATUS,Z          ; Skip if not zero (not red) ...
		goto    RedAspect         ; ... otherwise display red aspect

		xorlw   ASPGREEN          ; Test for green aspect required
		btfsc   STATUS,Z          ; Skip if not zero (not green) ...
		goto    GreenAspect       ; ... otherwise display green aspect

		andlw   ASPDOUBLE         ; Test for double yellow aspect required
		btfsc   STATUS,Z          ; Skip if not zero (not double yellow) ...
		goto    DblYllAspect      ; ... otherwise display double yellow

		; Display (single) yellow aspect
		bcf     ASPPORT,DOUBLEOUT
		bcf     ASPPORT,GREENOUT
		bsf     ASPPORT,YELLOWOUT
		bcf     ASPPORT,REDOUT
		goto    AspectEnd

		; Display double yellow aspect
DblYllAspect	bsf     ASPPORT,DOUBLEOUT
		bcf     ASPPORT,GREENOUT
		bsf     ASPPORT,YELLOWOUT
		bcf     ASPPORT,REDOUT
		goto    AspectEnd

		; Display green aspect
GreenAspect	bcf     ASPPORT,DOUBLEOUT
		bsf     ASPPORT,GREENOUT
		bcf     ASPPORT,YELLOWOUT
		bcf     ASPPORT,REDOUT
		goto    AspectEnd

		; Display red aspect
RedAspect	bcf     ASPPORT,DOUBLEOUT
		bcf     ASPPORT,GREENOUT
		bcf     ASPPORT,YELLOWOUT
		bsf     ASPPORT,REDOUT

AspectEnd

		; Send status to 'previous' signal

		; Encode status
		swapf   sigState,W        ; Copy status but with nibbles swapped

		btfsc   sigState,INHBIT   ; Skip if not forced red aspect display ...
		andlw   ~ASPSTSWP         ; ... otherwise report aspect as red

		movwf   telemData
		comf    telemData,W       ; One's complement aspect and detector state
		andlw   0x0F              ; Isolate aspect and detector state (swapped)
		movwf   telemData

		movf    sigState,W
		andlw   0xF0              ; Isolate aspect and detector state (unswapped)

		btfsc   sigState,INHBIT   ; Skip if not forced red aspect display ...
		andlw   ~ASPSTATE         ; ... otherwise report aspect as red

		iorwf   telemData,W       ; Combine complemented and uncomplemented data

		movwf   FSR
		call    LinkPTx           ; Send data to 'previous' signal

		bcf     INDPORT,USRIND

		return                    ; End of main processing loop


;**********************************************************************
; End of source code
;**********************************************************************

		end                       ; directive 'end of program'

