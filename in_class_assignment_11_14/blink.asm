; Addresses for I/O
.NAME	HEX = 	0xFFFFF000
.NAME	LEDR =	0xFFFFF020
.NAME 	TIMER = 0xFFFFF100
.NAME 	TIMERLIM = 0xFFFFF104
.NAME   TIMERCTRL = 0xFFFFF108
.NAME 	SWITCH = 0xFFFFF090
.NAME   SWITCHCTRL = 0xFFFFF094

; IDN Values
.NAME	TI	= 0
.NAME	KY	= 1
.NAME	SW	= 10
.NAME	constant = 0xFFFFFFFF

.ORG 	0x20
InterruptHandler:
	; Interrupts are disabled by HW before this
	RSR		T0, IDN 					; T0 is interrupt device number
	; SW		T0, HEX(Zero)
	; BR		Loop
	LW 		T1, KY(Zero)
	SW		T1, HEX(Zero)
	BEQ 	T0, T1, TimerHandler 		; Is a timer interrupt
	; Execution should not occur here!!
	;LW		T0, constant(Zero)
	;ADDI	A3, A3, 1
    ;SW		A3, HEX(Zero)
	RETI

TimerHandler: 
	;LW		A2, constant(Zero)
    ;SW		A2, HEX(Zero)	
    ADDI	A3, A3, 1
    SW		A3, HEX(Zero)
    RETI

; Processor Initialization
.ORG	0x100
	XOR		Zero, Zero, Zero			; Zero the Zero register
    AND     A3, Zero, Zero              ; A3 will be displayed on HEX
    AND 	A2, Zero, Zero
	;SW		A3, HEX(Zero)
	ADDI 	Zero, S1, 1000				; S1 = 100 ms
	;SW 		S1, TIMERLIM(Zero)			; Sets TLIM = 1000 ms
	ADDI 	Zero, T0, 16				; For turning on IE bit for devices => 16 = b10000
	SW 		T0, SWITCHCTRL(Zero)
	RSR 	T0, PCS
	ORI		T0, T0, 1					; For turning on IE bit for PCS
	WSR 	PCS, T0
	; AND		T1, T1, Zero
	; ADDI	Zero, T1, 3

InfiniteLoop:
	; LW 		T1, TIMER(Zero)
	; SW		T1, HEX(Zero)
	; LW		T1, constant(Zero)
	;ADDI    A3, A3, 1                 ; increment A3
    ;SW		A3, HEX(Zero)
	; BNE		T1, A2, InfiniteLoop 				; Main Loop. Interrupts should occur here.
	ADDI 	A2, A2, 1
	SW 		A2, LEDR(Zero)
	BR		InfiniteLoop

; Loop:
; 	ADDI	Zero, A3, 3
; 	SW		A3, HEX(Zero)
; 	BR		Loop