; Addresses for I/O
.NAME	HEX =  	0xFFFFF000
.NAME	LEDR = 	0xFFFFF020
.NAME	KEY =  	0xFFFFF080
.NAME   KEYCTL= 0xFFFFF084
.NAME	SW =  	0xFFFFF090
.NAME   SWCTL = 0xFFFFF094
.NAME   TIMER = 0xFFFFF100
.NAME   TLIM = 	0xFFFFF104
.NAME   TCTL = 	0xFFFFF108

; The stack is at the top of memory
.NAME	StkTop = 65536

.NAME   Upper = 0x3E0		; 11111 00000
.NAME   Lower = 0x01F		; 00000 11111
.NAME   StartSpeed = 2
.NAME   DefaultSpeed = 500
.NAME   IncrementValue = 250
.NAME   MinSpeed = 250
.NAME   MaxSpeed = 2000


; Processor Initialization
.ORG 0x100
	XOR 	Zero, Zero, Zero 		; Zero out zero register
	SW 		Zero, LEDR(Zero) 		; Turn off LEDR

    ADDI    Zero, S0, StartSpeed	; Set speed to 2
    SW      Zero, HEX(Zero)			; Display 0's on hex

    ADDI 	Zero, S1, 0 			; Initialize counter

    ADDI	Zero, T0, 0x1000 		; Write interrupt handler address
	WSR		IHA, T0

    ADDI    Zero, T0, 0x10 			; Enable device interrupts
    SW      T0, KEYCTL(Zero)
    SW      T0, SWCTL(Zero)
    SW      T0, TCTL(Zero)

    ADDI    Zero, S2, DefaultSpeed	; set default speed
    SW      S2, TLIM(Zero)			

    ADDI    Zero, T0, 0x01 			; Enable processor interrupts
    WSR     PCS, T0

InfiniteLoop:
	BR 		InfiniteLoop              
   

.ORG 0x1000
IntHandler:
   
    RSR     A0, IDN 			; Get device
    ADDI    Zero, A1, 1
    BEQ     A0, A1, Timer 		; timer = 1
    ADDI    A1, A1, 1
    BEQ     A0, A1, Key 		; key = 2
    ADDI    A1, A1, 1             
    BEQ     A0, A1, Switch 		; switch = 3
    ADDI    A1, A1, 1  

    ; Should never happen
    RETI            

    Timer:
        LW		T0, TCTL(Zero) 		; unset the ready bit
	    ANDI	T0, T0, 0xFFFFFFFE
	    SW		T0, TCTL(Zero)

        ADDI    Zero, A2, 17		; if counter is above 18, reset
        BGT     S1, A2, Reset
        BR      States

        Reset:
            ADDI    Zero, S1, 0

        States:
            ADDI    Zero, A2, 0
            BEQ     S1, A2, State0
            ADDI    Zero, A2, 2
            BEQ     S1, A2, State2
            ADDI    Zero, A2, 4
            BEQ     S1, A2, State4
            ADDI    Zero, A2, 6
            BEQ     S1, A2, State6
            ADDI    Zero, A2, 8
            BEQ     S1, A2, State8
            ADDI    Zero, A2, 10
            BEQ     S1, A2, State10
            ADDI    Zero, A2, 12
            BEQ     S1, A2, State12
            ADDI    Zero, A2, 13
            BEQ     S1, A2, State13
            ADDI    Zero, A2, 14
            BEQ     S1, A2, State14
            ADDI    Zero, A2, 15
            BEQ     S1, A2, State15
            ADDI    Zero, A2, 16
            BEQ     S1, A2, State16
            ADDI    Zero, A2, 17
            BEQ     S1, A2, State17

            BR      LightsOff

            State0:
            State2:
            State4:
            State12:
            State14:
            State16:
                ADDI    Zero, A1, Upper
                SW      A1, LEDR(Zero)
                BR      RetTimer

            State6:
            State8:
            State10:
            State13:
            State15:
            State17:
                ADDI    Zero, A1, Lower
                SW      A1, LEDR(Zero)
                BR      RetTimer

            LightsOff:
                SW      Zero, LEDR(Zero)
                BR      RetTimer

        RetTimer:       
            ADDI    S1, S1, 1 	; Increment counter
            RETI

    Key:
        ADDI    Zero, A2, 1
        LW      A0, KEY(Zero)
        ANDI    A0, A1, 1 			; Check Key[0]
        BEQ     A1, A2, IncrLen 	; If Key[0] == 1 increase
        ADDI    Zero, A2, 2
        ANDI    A0, A1, 2
        BEQ     A1, A2, DecrLen		; If Key[1] == 1 decrese 
        BR      Switch

        IncrLen:					; slowing down
            ADDI    Zero, A1, IncrementValue
            LW      A0, TLIM(Zero)
            ADD     A2, A0, A1
            ADDI    Zero, A1, MaxSpeed
            BGT     A2, A1, Switch 
            SW      A2, TLIM(Zero)
            ADDI    S0, S0, 1
            SW      S0, HEX(Zero)
            BR      Switch
        
        DecrLen:					; speeding up
            ADDI    Zero, A1, IncrementValue
            LW      A0, TLIM(Zero)
            SUB     A2, A0, A1
            ADDI    Zero, A1, MinSpeed
            BLT     A2, A1, Switch
            SW      A2, TLIM(Zero)
            SUBI    S0, S0, 1
            SW      S0, HEX(Zero)
            BR      Switch

    Switch:  
        LW      A0, SW(Zero)
        ANDI    A0, A0, 0x3F 		; Don't check SW[6-9]
        ADDI    Zero, A2, 0

        SW0:
	        ADDI    Zero, A3, 1
	        ANDI    A0, A1, 1
	        BNE     A1, A3, SW1
	        ADDI    Zero, T0, 0
	        LSHF    A3, S0, T0   
	        OR      A2, A2, A3

        SW1:
            ADDI    Zero, A3, 2   
            ANDI    A0, A1, 2
            BNE     A1, A3, SW2
            ADDI    Zero, T0, 4
            LSHF    A3, S0, T0
            OR      A2, A2, A3

        SW2:
            ADDI    Zero, A3, 4   
            ANDI    A0, A1, 4
            BNE     A1, A3, SW3
            ADDI    Zero, T0, 8
            LSHF    A3, S0, T0
            OR      A2, A2, A3

        SW3:
            ADDI    Zero, A3, 8   
            ANDI    A0, A1, 8
            BNE     A1, A3, SW4
            ADDI    Zero, T0, 12
            LSHF    A3, S0, T0
            OR      A2, A2, A3

        SW4:
            ADDI    Zero, A3, 16
            ANDI    A0, A1, 16
            BNE     A1, A3, SW5
            ADDI    Zero, T0, 16
            LSHF    A3, S0, T0
            OR      A2, A2, A3

        SW5:
            ADDI    Zero, A3, 32
            ANDI    A0, A1, 32
            BNE     A1, A3, RetSW
            ADDI    Zero, T0, 20
            LSHF    A3, S0, T0
            OR      A2, A2, A3

        RetSW:
            SW      A2, HEX(Zero)
            RETI