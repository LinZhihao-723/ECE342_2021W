	addi s0, zero, pointer1
	addi s1, zero, pointer2
// Testing load
	lw   s0, s0, 0 				// x8 = array1
	lw   s1, s1, 0   			// x9 = array2
	lw   a0, s0, 0				// x10 = 0x00010203
	lh   a1, s0, 10	            // x11 = 0x00000A0B
	lh   a2, s0, 6              // x12 = 0xFFFFF0F1
	lhu  a3, s0, 10	        	// x13 = 0x00000A0B
	lb   a5, s0, 11             // x15 = 0x0000000A
	lbu  a7, s0, 11 			// x17 = 0x0000000A
	lb   a6, s0, 5				// x16 = 0xFFFFFFF2
	lbu  s2, s0, 5				// x18 = 0x000000F2
	lhu  a4, s0, 6             	// x14 = 0x0000F0F1
// Testing store
	lui  t1, 0xECE00            // x6 = 0xECE003FA
	addi t1, t1, 0x3FA          // x6 = 0x000003FA
	sw   s1, t1, 8
	sh   s1, t1, 4
	sh   s1, t1, 6
	sb   s1, t1, 0
	sb   s1, t1, 1 
	sb   s1, t1, 2      			  
	sb   s1, t1, 3
	lw   s3, s1, 0				// x19 = 0xFAFAFAFA
	lw   s4, s1, 4				// x20 = 0x03FA03FA
	lw   s5, s1, 8				// x21 = 0xECE003FA

// A simple for loop
	addi s6, zero, 8            // x22 = 8, loop upper bound
	addi s7, zero, 0            // x23 = 0, loop counter
	addi s1, s1, 16
loop1_start:
	bge  s7, s6, loop1_end
	lb   s8, s0, 0
	sb   s1, s8, 0
	addi s0, s0, 1
	addi s1, s1, 1
	addi s7, s7, 1
	jal  zero, loop1_start
loop1_end:
	addi s6, zero, -4            // x22 = 4, loop upper bound
	addi s7, zero, -8            // x23 = 0, loop counter
loop2_start:
	beq  s7, s6, loop2_end
	lh   s8, s0, 0
	sh   s1, s8, 0
	addi s0, s0, 2
	addi s1, s1, 2
	addi s7, s7, 1
	jal  zero, loop2_start
loop2_end:
	addi t2, zero, array2
	addi t2, t2, 16
	lw   t3, t2, 0				// x28 = 0x00010203
	lw   t4, t2, 4				// x29 = 0xF0F1F2F3
	lw   t5, s1, -8				// x30 = 0x0A0B0C0D
	lw   t6, s1, -4             // x31 = 0xFAFBFCFD
    sh   s0, t6, -14
	lw   sp, s0, -16            // x2 = 0xFCFD0203
measure_end:
	addi zero, zero, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

pointer1:
.dw array1 

array1:
.dw 0x00010203
.dw 0xF0F1F2F3
.dw 0x0A0B0C0D
.dw 0xFAFBFCFD

pointer2:
.dw array2

array2:
.dw 0x00000000
.dw 0x00000000
.dw 0x00000000
.dw 0x00000000
.dw 0x00000000
.dw 0x00000000
.dw 0x00000000
.dw 0x00000000

// Basic details
.org	0x800
.dw 0x0           // Start of PC
.dw measure_end	  // End of PC
.dw 125			  // Num Instruction
.dw 25			  // Minimum IPC * 128

// Which register to check
.db 0b00000101
.db 0b11111100
.db 0b00111111
.db 0b11110000

// Expected register values
.org	0x820
.dw 0x00000000, 0x00000000, 0xFCFD0203, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
.dw 0x00000000, 0x00000000, 0x00010203, 0x00000A0B, 0xFFFFF0F1, 0x00000A0B, 0x0000F0F1, 0x0000000A
.dw 0xFFFFFFF2, 0x0000000A, 0x000000F2, 0xFAFAFAFA, 0x03FA03FA, 0xECE003FA, 0x00000000, 0x00000000
.dw 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00010203, 0xF0F1F2F3, 0x0A0B0C0D, 0xFAFBFCFD
