As far as possible the original files from riscv-tests. 
The HZ macro is set to 24 MHz, the clock rate of Danzig.

Modifications:
  * test.ld:
    Added markers for the beginning end end of the .bss sections.
  * crt.S: 
    Added code to clear .bss sections. This is important, because the
    variable Reg in dhrystone_main.c lies in .sbss.
    Set stack pointer to end of available memory.
  * syscalls.c
    Implemented write syscall via bit banging UART.
    Added simple puts() that calls printf().
  * dhrystone.h
    Set HZ to 24'000'000 and NUMBER_OF_RUNS to 100'000