#ifndef _ENV_DANZIG_TEST_H
#define _ENV_DANZIG_TEST_H

#define RVTEST_RV32M
#define RVTEST_RV32U
#define TESTNUM x28     // register where test number is stored


#define RVTEST_CODE_BEGIN       \
    .section .text.crt;         \
    .global _start;             \
_start:                         \
    j _start2;                  \
    .text;                      \
_start2:

/*  If .text starts directly at address 0, the linker has problems with
        la x1, foo
    when foo is in the .data section with pc-relative addressing.
    The linker gives the following error:
        relocation truncated to fit: R_RISCV_PCREL_LO12_I
*/

#define RVTEST_CODE_END

#define RVTEST_PASS             \
        li x10, 0x03;           \
        lui x11, 0x10001;       \
        sw x10, 0(x11);         \
1:      j 1b;

#define RVTEST_FAIL             \
        li x10, 0xfd;           \
        lui x11, 0x10001;       \
        sw x10, 0(x11);         \
1:      j 1b;



#define RVTEST_DATA_BEGIN .balign 4;
#define RVTEST_DATA_END

#endif
