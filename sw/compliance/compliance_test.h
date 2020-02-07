#ifndef _COMPLIANCE_TEST_H
#define _COMPLIANCE_TEST_H

//#include "riscv_test.h"

#define RV_COMPLIANCE_HALT       \
    li t0, 1;                   ;\
    SWSIG (0, t0);              ;\
    la t0, begin_signature      ;\
    csrw 0x3fd, t0              ;\
    la t0, end_signature        ;\
    csrw 0x3fe, t0              ;\
    csrw 0x3ff, 2               ;\
1:  j 1b                        ;

/* The firts two lines are there because of an ugly bug in riscv-compliance.
   The call of the SWSIG macro is part of RVTEST_PASS which is called in the
   golden standard implementation of riscv-compliance within RV_COMPLIANCE_HALT.
   If the benchark includes "test_macros.h" or "riscv_test_macros.h", SWSIG is
   cleared and does nothing. But if "aw_test_macros.h" is included (for all
   benchmarks imported from riscv-tests), then WSSIG writes to the signature.
 */



/*
    li t1, 1                    ;\
    sw t1, 0(t0)                ;\
*/

#define RV_COMPLIANCE_RV32M

#define RV_COMPLIANCE_CODE_BEGIN \
    .section .text.crt;         \
    .global _start;             \
_start:                         \
    j _start2;                  \
    .text;                      \
_start2:                        ;\
    la t0, begin_signature;     ;
    
/*
    \
    li t1, 1                    ;\
    sw t1, 0(t0)                ;
*/

/*  If .text starts directly at address 0, the linker has problems with
        la x1, foo
    when foo is in the .data section with pc-relative addressing.
    The linker gives the following error:
        relocation truncated to fit: R_RISCV_PCREL_LO12_I
*/

#define RV_COMPLIANCE_CODE_END


#define RV_COMPLIANCE_DATA_BEGIN \
    .align 4; .global begin_signature; begin_signature:
#define RV_COMPLIANCE_DATA_END \
    .align 4; .global end_signature; end_signature:

#endif
