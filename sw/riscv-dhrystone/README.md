As far as possible the original files from riscv-tests. The HZ macro is left
unchanged at a value of 1 MHz.

Modifications:
  * test.ld:
    Added markers for the beginning end end of the .bss sections.
  * crt.S: 
    Added code to clear .bss sections. This is important, because the
    variable Reg in dhrystone_main.c lies in .sbss.
    Set stack pointer to end of available memory.
  * syscalls.c
    Implemented write syscall.
    Added simple puts() that calls printf().
