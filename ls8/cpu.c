#include "cpu.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h> // for isprint()

struct termios oldattr;
int oldfcntl;


void set_terminal(void)
{
	struct termios newattr;

	// Non-canonical, no-echo mode
	tcgetattr( STDIN_FILENO, &oldattr );

	newattr = oldattr;
	newattr.c_lflag &= ~( ICANON | ECHO);

	tcsetattr( STDIN_FILENO, TCSANOW, &newattr );

	// Non-blocking mode
	oldfcntl = fcntl(STDIN_FILENO, F_GETFL, 0);
	int newfcntl = oldfcntl | O_NONBLOCK;

	fcntl(STDIN_FILENO, F_SETFL, newfcntl);
}

/**
 * Reset the terminal to what it was before
 */
void reset_terminal(void)
{
	fcntl(STDIN_FILENO, F_SETFL, oldfcntl);
	tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
}


#define DATA_LEN 6

/**
 * Load the binary bytes from a .ls8 source file into a RAM array
 */
void cpu_load(struct cpu *cpu, char *path)
{

  // char data[DATA_LEN] = {
  //     // From print8.ls8
  //     0b10000010, // LDI R0,8
  //     0b00000000,
  //     0b00001000,
  //     0b01000111, // PRN R0
  //     0b00000000,
  //     0b00000001 // HLT
  // };

// "examples/conditionals.ls8"
// "examples/conditionals.ls8"
  FILE *file = fopen(path, "r");

  if (file == NULL) {
    perror("File not found.");
    exit(1);
  }

  int address = 0;
  char line[256];
  while (fgets(line, 256, file)) {
    char * endpoint;
    unsigned char val = strtoul(line, &endpoint, 2);
    if (line != endpoint) {
      cpu->ram[address] = val;
      address++;
    }
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
  cpu->FL = 0;
}

/**
 * Cleanup the CPU struct
 */
void cpu_destroy(struct cpu *cpu)
{
  free(cpu->registers);
  free(cpu->ram);
  reset_terminal();
}

unsigned char cpu_ram_read(struct cpu *cpu, int address)
{
  return cpu->ram[address];
}

void cpu_ram_write(struct cpu *cpu, int address, int value)
{
  cpu->ram[address] = value;
}

/**
 * ALU
 */
void alu(struct cpu *cpu, unsigned char op, unsigned char regA, unsigned char regB)
{
  switch (op)
  {
  // generic ALU
  case ADD:
    cpu->registers[regA] = cpu->registers[regA] + cpu->registers[regB];
    break;
  case AND:
    cpu->registers[regA] = cpu->registers[regA] & cpu->registers[regB];
    break;
  case DEC:
    cpu->registers[regA]--;
    break;
  case DIV:
    cpu->registers[regA] = cpu->registers[regA] / cpu->registers[regB];
    break;
  case INC:
    cpu->registers[regA]++;
    break;
  case MOD:
    cpu->registers[regA] = cpu->registers[regA] % cpu->registers[regB];
    break;
  case MUL:
    cpu->registers[regA] = cpu->registers[regA] * cpu->registers[regB];
    break;
  case NOT:
    cpu->registers[regA] = ~cpu->registers[regA];
    break;
  case OR:
    cpu->registers[regA] = cpu->registers[regA] | cpu->registers[regB];
    break;
  case SHL:
    cpu->registers[regA] = cpu->registers[regA] << cpu->registers[regB];
    break;
  case SHR:
    cpu->registers[regA] = cpu->registers[regA] >> cpu->registers[regB];
    break;
  case SUB:
    cpu->registers[regA] = cpu->registers[regA] - cpu->registers[regB];
    break;
  case XOR:
    cpu->registers[regA] = cpu->registers[regA] ^ cpu->registers[regB];
    break;

  // comparison
  case CMP:
    if (cpu->registers[regA] > cpu->registers[regB])
    {
      cpu->FL = 2;
    }
    else if (cpu->registers[regA] == cpu->registers[regB])
    {
      cpu->FL = 1;
    }
    else
    {
      cpu->FL = 4;
    }
    break;
  }
}

void update_PC(struct cpu *cpu, unsigned char IR)
{
      // mask to redetermine number of operands
    unsigned char mask = 0b11000000;
    switch (IR & mask)
    {
    // 0 operands
    case 0b00000000:
      cpu->PC = cpu->PC + 1;
      break;
    // 1 operand
    case 0b01000000:
      cpu->PC = cpu->PC + 2;
      break;
    // 2 operands
    case 0b10000000:
      cpu->PC = cpu->PC + 3;
      break;
    }
}

/**
 * Run the CPU
 */
void cpu_run(struct cpu *cpu)
{
  int running = 1; // True until we get a HLT instruction
  int interrupted = 0; // set global interrupt state to false
  // set up interrupt timer
  struct timeval tv;
  gettimeofday(&tv, NULL);
  double next_time = ((double) tv.tv_sec + 1) + tv.tv_usec / 1000000;

  // Raw, noecho, non-blocking mode
  set_terminal();
  

  // set up stack pointer
  cpu->registers[7] = 0xF4;
  unsigned char *SP = cpu->registers + 7;

  // special ram positions
  unsigned char INTERRUPT_RAM = 0xF8;
  unsigned char KEY_RAM = 0xF4;

  // set up IS and IM register
  cpu->registers[5] = 0;
  unsigned char *IM = cpu->registers + 5;
  cpu->registers[6] = 0;
  unsigned char *IS = cpu->registers + 6;

  while (running)
  {
    // check interrupts
    // check timer interrupt
    gettimeofday(&tv, NULL);
    double new_time = (double) tv.tv_sec + (tv.tv_usec / 1000000);
    if (new_time >= next_time) {
      next_time = new_time + 1;
      // set 0th IS bit to 1
      *IS = *IS | 0b00000001;
    }

    // check keyboard interrupt
    int key = getchar();
    if (key != -1) {
      if (isprint(key)) {
        // save key press to RAM
        cpu_ram_write(cpu, KEY_RAM, key);
        
        // set 1st IS bit to 1
        *IS = * IS | 0b00000010;
      }
    }

    // trace(cpu);


    int IR = cpu_ram_read(cpu, cpu->PC);

    // mask to get first two values
    int mask = 0b11000000;
    int operandA;
    int operandB;
    switch (IR & mask)
    {
    // 1 operand case 
    case 0b01000000:
      operandA = cpu_ram_read(cpu, cpu->PC + 1);
      break;

    // 2 operand case
    case 0b10000000:
      operandA = cpu_ram_read(cpu, cpu->PC + 1);
      operandB = cpu_ram_read(cpu, cpu->PC + 2);
      break;
    }

    // first check for interrupt
    if (!interrupted) {
      unsigned char interrupts = (*IS & *IM);
      int interrupt_happened = 0;
      int interrupt_pos;
      for (int i = 0; i < 8; i++) {
        if (((interrupts >> i) & 1) == 1) {
          interrupt_happened = 1;
          interrupt_pos = i;
          break;
        }
      }
      if (interrupt_happened) {
        // set IS bit to 0
        unsigned char mask = ~(1 << interrupt_pos);
        *IS = *IS & mask;

        interrupted = 1;
        *SP = *SP - 1;
        // push PC onto stack
        cpu_ram_write(cpu, *SP, cpu->PC);

        // push FL onto stack
        *SP = *SP - 1;
        cpu_ram_write(cpu, *SP, cpu->FL);

        // push registers R0 - R6 onto stack
        for (int i = 0; i < 7; i++) {
          *SP = *SP - 1;
          cpu_ram_write(cpu, *SP, cpu->registers[i]);
        }

        // set PC to interrupt address
        cpu->PC = cpu_ram_read(cpu, INTERRUPT_RAM + interrupt_pos);
        continue;
      }
    }

    // handle alu operations
    mask = 0b00100000;
    // determine if ALU bit is true
    if ((IR & mask) == 0b00100000)
    {
      alu(cpu, IR, operandA, operandB);
    }
    else
    {
      // otherwise, non-alu. non-jump cases.
      // jump cases are handled further on as part of PC updation
      switch (IR)
      {
      case HLT:
        cpu_destroy(cpu);
        running = 0;
        break;
      // ALU Operations
      // memory
      case LDI:
        cpu->registers[operandA] = operandB;
        break;
      case LD:
        cpu->registers[operandA] = cpu_ram_read(cpu, cpu->registers[operandB]);
        break;
      
      case ST:
        cpu_ram_write(cpu, cpu->registers[operandA], cpu->registers[operandB]);
        break;

      case INT:


      // print
      case PRA:
        printf("%c\n", cpu->registers[operandA]);
        break;
      case PRN:
        printf("%d\n", cpu->registers[operandA]);
        break;

      // stack operations
      case PUSH:
        *SP = *SP - 1;
        cpu_ram_write(cpu, *SP, cpu->registers[operandA]);
        break;

      case POP:
        cpu->registers[operandA] = cpu_ram_read(cpu, *SP);
        *SP = *SP + 1;
        break;
      }


    }

    // handle PC updating
    switch (IR)
    {
      case CALL:
        *SP = *SP - 1;
        cpu_ram_write(cpu, *SP, cpu->PC + 2);
        cpu->PC = cpu->registers[operandA];
        break;
      
      case RET:
        cpu->PC = cpu_ram_read(cpu, *SP);
        *SP = *SP + 1;
        break;

      case IRET:
        // pop R6-RO off stack into registers
        for (int i = 0; i < 7; i++) {
          cpu->registers[6 - i] = cpu_ram_read(cpu, *SP);
          *SP = *SP + 1;
        }

        // pop FL off stack onto FL register
        cpu->FL = cpu_ram_read(cpu, *SP);
        *SP = *SP + 1;

        // pop PC off stack onto PC register
        cpu->PC = cpu_ram_read(cpu, *SP);
        *SP = *SP + 1;

        // set global interrupted flag to false
        interrupted = 0;
        break;

      case JEQ:
        if (cpu->FL == 1)
        {
          cpu->PC = cpu->registers[operandA];
          break;
        } else {
          update_PC(cpu, IR);
          break;
        }

      case JGE:
        if (cpu->FL == 1 || cpu->FL == 2)
        {
          cpu->PC = cpu->registers[operandA];
          break;
        } else {
          update_PC(cpu, IR);
          break;
        }

      case JGT:
        if (cpu->FL == 2)
        {
          cpu->PC = cpu->registers[operandA];
          break;
        } else {
          update_PC(cpu, IR);
          break;
        }

      case JLE:
        if (cpu->FL == 1 || cpu->FL == 4)
        {
          cpu->PC = cpu->registers[operandA];
          break;
        } else {
          update_PC(cpu, IR);
          break;
        }
      case JLT:
        if (cpu->FL == 4)
        {
          cpu->PC = cpu->registers[operandA];
          break;
        } else {
          update_PC(cpu, IR);
          break;
        }
      case JNE:
        if (cpu->FL == 2 || cpu->FL == 4)
        {
          cpu->PC = cpu->registers[operandA];
          break;
        } else {
          update_PC(cpu, IR);
          break;
        }
      case JMP:
        cpu->PC = cpu->registers[operandA];
        break;
      // if not a jump case, increment appropriately
      default:
        update_PC(cpu, IR);

    }
  }
}

// for testing
// int main() {
//   struct cpu cpu;

//   cpu_init(&cpu);
//   cpu_load(&cpu, "");
//   cpu_run(&cpu);
//   return 0;
// }