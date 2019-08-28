#ifndef _ENV_RUDOLV_TEST_H
#define _ENV_RUDOLV_TEST_H

#define RVTEST_RV32U
#define RVTEST_RV64U
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

#define RVTEST_EXIT             \
        li x10, 0x03;           \
        lui x11, 0x10001;       \
        sw x10, 0(x11);         \
1:      j 1b;


#define RVTEST_PASS             \
        la x11, result_ok;      \
        li x10, 0x6b6f;         \
        sw x10, 0(x11);         \
        RVTEST_EXIT

#define RVTEST_FAIL             \
        la x11, result_ok;      \
        li x10, 0x4c494146;     \
        sw x10, 0(x11);         \
        RVTEST_EXIT

#define RVTEST_DATA_BEGIN .balign 4;

#define RVTEST_DATA_END         \
        .section .signature;    \
        result_ok: .word 0

#endif
