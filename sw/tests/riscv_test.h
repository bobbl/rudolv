#ifndef _ENV_RUDOLV_TEST_H
#define _ENV_RUDOLV_TEST_H

#define TESTNUM x28     // register where test number is stored

#define RVTEST_RV32U
#define RVTEST_RV64U

#define RVTEST_CODE_BEGIN       \
    .section .text.crt          ;\
    .global _start              ;\
_start:                         ;\
    .weak mtvec_handler         ;\
    la x11, mtvec_handler       ;\
    csrw mtvec, x11             ;



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

#define RVTEST_RV64M

// from encoding.h
#define CAUSE_MISALIGNED_FETCH 0
#define CAUSE_ILLEGAL_INSTRUCTION 2
#define CAUSE_BREAKPOINT 3
#define CAUSE_MISALIGNED_LOAD 4
#define CAUSE_MISALIGNED_STORE 6
#define CAUSE_USER_ECALL 8

#define MCONTROL_LOAD 1
#define MCONTROL_STORE 2
#define MCONTROL_EXECUTE 4
#define MCONTROL_M 64

#define MSTATUS_MIE 8
#define MSTATUS_MPP (3<<11)
#define MSTATUS_TVM (1<<20)
#define MSTATUS_TSR (1<<22)


#endif
