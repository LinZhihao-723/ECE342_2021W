
	// Temp values
	addi s2, zero, -4
	addi s3, zero, -1
	addi s4, zero, 0
	addi s5, zero, 2
	addi s6, zero, 3
	addi s7, zero, -4
	addi s8, zero, -1
	addi s9, zero, 0	

	// Error flag
	addi t3, zero, 0

	// Test beq
	beq s2, s3, error
	beq s2, s4, error
	beq s2, s6, error
	beq s4, s6, error
	beq s5, s6, error

	// Test bne
	bne s2, s7, error
	bne s3, s8, error
	bne s4, s9, error
	bne s2, s2, error

	// Test blt
	blt s3, s2, error
	blt s4, s2, error
	blt s6, s2, error
	blt s6, s4, error
	blt s6, s5, error
	blt s2, s2, error
	blt s4, s4, error
	blt s6, s6, error

	// Test bge
	bge s2, s3, error
	bge s2, s4, error
	bge s2, s6, error
	bge s4, s6, error
	bge s5, s6, error

	// Test bltu
	bltu s3, s2, error
	bltu s2, s5, error
	bltu s3, s4, error
	bltu s5, s4, error
	bltu s6, s5, error

	// Test bgeu
	bgeu s2, s3, error
	bgeu s5, s2, error
	bgeu s4, s3, error
	bgeu s4, s5, error
	bgeu s5, s6, error

measure_end:
	jal zero, measure_end
error:
	addi t3, zero, 1
	jal t4, measure_end

// Basic details
.org	0x800
.dw 0x0           // Start of PC
.dw measure_end	  // End of PC
.dw 42			  // Num Instruction
.dw 25			  // Minimum IPC * 128

// Which register to check
.db 0b00000000
.db 0b00000000
.db 0b00000000
.db 0b00110000

// Expected register values
.org	0x820
.dw 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
.dw 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
.dw 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
.dw 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
