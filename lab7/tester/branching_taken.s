// Error flags
	addi s5, s6, 0
	addi s6, s6, 0

// Test jal instruction
	jal ra, testfunc
	addi s5, s5, 1	
	addi s6, s6, 1
	jal zero, part2	
	addi s5, s5, 1
	addi s6, s6, 1

// when we return, skip two instructions
testfunc:
	jalr zero, ra, 8
	addi s5, s5, 1
	addi s6, s6, 1

// Test jalr similarly
part2:
	addi s7, zero, testfunc
	jalr ra, s7, 0
	addi s5, s5, 1
	addi s6, s6, 1
	jal zero, part3
	addi s5, s5, 1
	addi s6, s6, 1

// Test branches
part3:
	addi a2, zero, -4
	addi a3, zero, -1
	addi a4, zero, 0
	addi a5, zero, 2
	addi a6, zero, 3

// beq
beq1:
	beq a2, a2, beq2
 	addi s5, s5, 1
 	addi s6, s6, 1
beq2: 
	beq a2, a2, beq3
	addi s5, s5, 1
	addi s6, s6, 1
beq3:
	beq a2, a2, blt1
	addi s5, s5, 1
	addi s6, s6, 1	

// blt
blt1:
	blt a2, a3, blt2
	addi s5, s5, 1
	addi s6, s6, 1
blt2:
	blt a2, a4, blt3
	addi s5, s5, 1
	addi s6, s6, 1
blt3:
	blt a2, a6, blt4
	addi s5, s5, 1
	addi s6, s6, 1
blt4:
	blt a4, a6, blt5
	addi s5, s5, 1
	addi s6, s6, 1
blt5:
	blt a5, a6, bne1
	addi s5, s5, 1
	addi s6, s6, 1

// bne (new 2)
bne1:
	bne a2, a3, bne2
	addi s5, s5, 1
	addi s6, s6, 1
bne2:
	bne a3, a1, bge1
	addi s5, s5, 1
	addi s6, s6, 1

// bge (new 4)
bge1:
	bge a3, a2, bge2
	addi s5, s5, 1
	addi s6, s6, 1
bge2:
	bge a4, a3, bge3
	addi s5, s5, 1
	addi s6, s6, 1
bge3:
	bge a5, a4, bge4
	addi s5, s5, 1
	addi s6, s6, 1
bge4:
	bge a6, a6, bltu1
	addi s5, s5, 1
	addi s6, s6, 1

// bltu (new 4)
bltu1:
	bltu a2, a3, bltu2
	addi s5, s5, 1
	addi s6, s6, 1
bltu2:
	bltu a5, a6, bltu3
	addi s5, s5, 1
	addi s6, s6, 1
bltu3:
	bltu a5, a2, bltu4
	addi s5, s5, 1
	addi s6, s6, 1
bltu4:
	bltu a4, a3, bgeu1
	addi s5, s5, 1
	addi s6, s6, 1

// bgeu (new 4)
bgeu1:
	bgeu a6, a5, bgeu2
	addi s5, s5, 1
	addi s6, s6, 1
bgeu2:
	bgeu a5, a5, bgeu3
	addi s5, s5, 1
	addi s6, s6, 1
bgeu3:
	bgeu a3, a2, bgeu4
	addi s5, s5, 1
	addi s6, s6, 1
bgeu4:
	bgeu a2, a5, measure_end
	addi s5, s5, 1
	addi s6, s6, 1

measure_end:
	jal ra, measure_end
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
.dw 38			  // Num Instruction
.dw 25			  // Minimum IPC * 128

// Which register to check
.db 0b00000010
.db 0b00000000
.db 0b01100000
.db 0b00000000

// Expected register values
.org	0x820
.dw 0x00000000, 0x00000168, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
.dw 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
.dw 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
.dw 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000