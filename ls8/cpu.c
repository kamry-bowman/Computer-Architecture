#include "cpu.h"

#define DATA_LEN 6


/**
 * Load the binary bytes from a .ls8 source file into a RAM array
 */
void cpu_load(struct cpu *cpu)
{
  char data[DATA_LEN] = {
    // From print8.ls8
    0b10000010, // LDI R0,8
    0b00000000,
    0b00001000,
    0b01000111, // PRN R0
    0b00000000,
    0b00000001  // HLT
  };

  int address = 0;

  for (int i = 0; i < DATA_LEN; i++) {
    cpu->ram[address++] = data[i];
  }

  // TODO: Replace this with something less hard-coded
}

unsigned char cpu_ram_read(struct cpu *cpu, int address) {
  return cpu->ram[address];
}

void cpu_ram_write(struct cpu *cpu, int address, int value) {
  return cpu->ram[address] = value;
}

/**
 * ALU
 */
void alu(struct cpu *cpu, unsigned char op, unsigned char regA, unsigned char regB)
{
  switch (op) {
    case ADD:
      cpu->registers
      cpu->registers[operandA], cpu->registers[operandB]

    case AND:
    case DEC:
    case DIV:
    case INC:
    case MOD:
    case MUL:
    case NOT:
    case OR:
    case SHL:
    case SHR:
    case SUB:
    case XOR:
  }
}

/**
 * Run the CPU
 */
void cpu_run(struct cpu *cpu)
{
  int running = 1; // True until we get a HLT instruction

  while (running) {
    // TODO
    // 1. Get the value of the current instruction (in address PC).
    int IR = cpu_ram_read(cpu, cpu->PC);
    // 2. Figure out how many operands this next instruction requires
    // 3. Get the appropriate value(s) of the operands following this instruction
    int mask = 0b11000000;
    int operandA;
    int operandB;
    switch(IR & mask) {
      case 0b01000000:
        operandA = cpu_ram_read(cpu, cpu->PC + 1);

      case 0b10000000:
        operandB = cpu_ram_read(cpu, cpu->PC + 2);
    }

    // 4. switch() over it to decide on a course of action.
    switch(IR) {
      // ALU Operations
      case ADD:
      case AND:
      case DEC:
      case DIV:
      case INC:
      case MOD:
      case MUL:
      case NOT: 
      case OR:
      case SHL:  
      case SHR:
      case SUB:
      case XOR:
        alu(cpu, IR, cpu->registers[operandA], cpu->registers[operandB]);

      case CMP:
        // how to update flags?
        

        
      case INT:
        
        
    }


    // 5. Do whatever the instruction should do according to the spec.
    // 6. Move the PC to the next instruction.
  }
}

/**
 * Initialize a CPU struct
 */
void cpu_init(struct cpu *cpu)
{
  cpu->PC = 0;
  cpu->registers = calloc(8, sizeof(unsigned char));
  cpu->ram = calloc(256, sizeof(unsigned char));
}
