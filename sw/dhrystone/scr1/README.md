As far as possible the original files from the 
[Syntacore SCR1 repository](https://github.com/syntacore/scr1-sdk/tree/master/sw/tests/dhry21/)

Modifications:
  * Makefile:
    No separate tests_commmon/ directory, files are here in the same directory
    Do not delete .elf executable
  * crt-noreloc.S, tcm.ld:
    Completely replaced
  * sc_print.c:
    Implement printf_putch() via CSR 0xbc0
  * uart.c, uart.h:
    Not required any more
  * tests_common.mk:
    Remove linking uart.c
