#include <stdio.h>
#include "cpu.h"
#include <stdlib.h>

/**
 * Main
 */
int main(int argc, char *argv[])
{
  if (argc < 2) {
    perror("Must provide a file path.");
    exit(1);
  }

  struct cpu cpu;


  cpu_init(&cpu);
  

  cpu_load(&cpu, argv[1]);
  cpu_run(&cpu);

  return 0;
}