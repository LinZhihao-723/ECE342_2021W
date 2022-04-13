//  This File will test the basic arithematic instructions

	addi s2, zero, 4    		// x18 = 4 
	addi s3, zero, -3   		// x19 = -3
	sub  s4, s2, s3 			// x20 = 7
	sub  s5, s3, s2				// x21 = -7
	add  s6, s4, s5				// x22 = 0
	lui  t1, 0xFFFF0   			// x6 = 0xFFFF0000
	addi t2, zero, 0xFF0      	// x7 = 0xFFFFFFF0
	or   s7, t1, t2 			// x23 = 0xFFFFFFF0
	and  s8, t1, t2				// x24 = 0xFFFF0000
	xor  s9, t1, t2				// x25 = 0x0000FFF0
	addi a0, zero, 0x00a        // x10 = 0x0000000A 
	andi a1, a0, 0x006          // x11 = 0x00000002
	ori	 a2, a0, 0x006			// x12 = 0x0000000E
	xori a3, a0, 0x006			// x13 = 0x0000000C
	slli a4, t1, 3              // x14 = 0xFFF80000
	srli a5, t1, 8              // x15 = 0x00FFFF00
	srai a6, t1, 8              // x16 = 0xFFFFFF00
	addi a7, zero, 31  			// x17 = 0x0000001F
	addi s10, zero, 1  			// x26 = 0x00000001
	sll  t3, s10, a7 			// x28 = 0x80000000
	srl  t4, t3, a7				// x29 = 0x00000001
	sra  t5, t3, a7             // x30 = 0xFFFFFFFF
	addi s10, zero, 16			// x26 = 0x00000010
	lui  s11, 0xFFFF0           // x27 = 0xFFFF0000
	slt  ra, s10, s11           // x1 = 0x00000000
	slt  sp, s11, s10           // x2 = 0x00000001
	sltu gp, s10, s11           // x3 = 0x00000001
	sltu tp, s11, s10           // x4 = 0x00000000
	slti s0, s10, 0xFFF         // x8 = 0x00000000
	sltiu s1, s10, 0xFFF        // x9 = 0x00000001
	auipc t0, 0xFFCB0           // x5 = 0xFFCB0078
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

// Basic details
.org	0x800
.dw 0x0           // Start of PC
.dw measure_end	  // End of PC
.dw 32			  // Num Instruction
.dw 25			  // Minimum IPC * 128

// Which register to check
.db 0b11111111
.db 0b11111111
.db 0b11111111
.db 0b11111111

// Expected register values
.org	0x820
.dw 0x00000000, 0x00000000, 0x00000001, 0x00000001, 0x00000000, 0xFFCB0078, 0xFFFF0000, 0xFFFFFFF0
.dw 0x00000000, 0x00000001, 0x0000000A, 0x00000002, 0x0000000E, 0x0000000C, 0xFFF80000, 0x00FFFF00
.dw 0xFFFFFF00, 0x0000001F, 0x00000004, 0xfffffffd, 0x00000007, 0xfffffff9, 0x00000000, 0xFFFFFFF0
.dw 0xFFFF0000, 0x0000FFF0, 0x00000010, 0xFFFF0000, 0x80000000, 0x00000001, 0xFFFFFFFF, 0x00000000
