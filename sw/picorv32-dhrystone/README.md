As far as possible the original files from picorv32.
Only the standalone version (USE_MYSTDLIB=1 in the Makefile) fits into the
available memory. The version with newlib is too large (80 KiB).

Modifications:
  * sections.lds:
    Set code start to 0 instead of 0x10000.
  * start.S: 
    Set stack pointer to end of available memory.
    Signal program termination to simulator via CSR 0x3ff.
  * stdlib.c:
    Implement printf_c() via CSR 0xbc0
