; Addresses for I/O
.NAME   HEX= 0xFFFFF000
.NAME   LEDR=0xFFFFF020
.NAME   KEY= 0xFFFFF080
.NAME   SW=  0xFFFFF090
    ; Now the actual code
    .ORG 0x100
    addi    Zero,s0,1
    addi    s0,s0,1
    addi    s0, s0,1
    addi    s0,s0,1
    Loop:
    sw      s0,LEDR(Zero)
    sw      s0,HEX(Zero)
    br      Loop