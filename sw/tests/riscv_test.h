#ifndef _ENV_RUDOLV_TEST_H
#define _ENV_RUDOLV_TEST_H

#define TESTNUM x28     // register where test number is stored

#define RVTEST_RV32U
#define RVTEST_RV64U

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
    la t0, begin_signature      ;\
    csrw 0x3fd, t0              ;\
    la t0, end_signature        ;\
    csrw 0x3fe, t0              ;\
    csrw 0x3ff, 2               ;\
1:  j 1b                        ;

#define RVTEST_PASS             \
    la x11, begin_signature     ;\
    li x10, 0x6b6f              ;\
    sw x10, 0(x11)              ;\
    RVTEST_EXIT

#define RVTEST_FAIL             \
    la x11, begin_signature     ;\
    li x10, 0x4c494146          ;\
    sw x10, 0(x11)              ;\
    RVTEST_EXIT

#define RVTEST_DATA_BEGIN

#define RVTEST_DATA_END         \
    .align 4                    ;\
    .global begin_signature     ;\
    begin_signature:            ;\
    .word -1                    ;\
    .global end_signature       ;\
    end_signature:


#endif
