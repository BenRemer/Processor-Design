.ORG 	0x20
InterruptHandler:
	; Interrupts are disabled by HW before this
	RSR		T0, IDN 					; T0 is interrupt device number
	LW 		T1, TI(Zero)
	BEQ 	T0, T1, TimerHandler 		; Is a timer interrupt
	; Execution should not occur here!!
	RETI

TimerHandler: 
    ADDI    Zero, A3, 1                 ; increment A3
    SW		A3, HEX(Zero)				
    BR 		EndInterruptHandler

EndInterruptHandler:
	RETI

; Processor Initialization
.ORG	0x100
	XOR		Zero, Zero, Zero			; Zero the Zero register
	ADDI 	Zero, S1, 100				; S1 = 100 ms
    AND     A3, Zero, Zero              ; A3 will be displayed on HEX
	SW 		S1, TIMERLIM(Zero)			; Sets TLIM = 100 ms
	ADDI 	Zero, T0, 16				; For turning on IE bit for devices
	SW 		T0, TIMERCTRL(Zero)
	RSR 	T0, PCS
	ORI		T0, T0, 1					; For turning on IE bit for PCS
	WRS 	PCS, T0

InfiniteLoop:
	BR		InfiniteLoop 				; Main Loop. Interrupts should occur here.

; Addresses for I/O
.NAME	HEX = 	0xFFFFF000
.NAME 	TIMER = 0xFFFFF100
.NAME 	TIMERLIM = 0xFFFFF104
.NAME   TIMERCTRL = 0xFFFFF108

; IDN Values
.NAME	TI	= 00