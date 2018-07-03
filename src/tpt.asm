MULI 0b0000   // zero out accumulator
MUL ACC REG0  // zero out reg0
ADDI 0b0010   // add 2 to accumulator
ADDI 0b0010   // add 2 to accumulator
ADD REG0 REG0 // acc -> reg0
ADDI 0b0010   // accumlator is now 6
JNEZ ACC ACC  // Spin (jump to address 6, loop forever)
