#*****************************************************************************
# rudolv_ecall.S
#-----------------------------------------------------------------------------
#
# Test user exceptions with ecall, ebreak and c.ebreak
#
# Copyright Jörg Mische <bobbl@gmx.de>
# SPDX-License-Identifier: ISC


#include "riscv_test.h"
#include "test_macros.h"



RVTEST_RV64U
RVTEST_CODE_BEGIN

# code are ecall/ebreak follwed by 3 instructions that should be ignored by the
# trap handler, if working correctly


#define TEST_DELAYSLOTS(testnum, cause, code... ) \
test_ ## testnum:                        \
        li      TESTNUM, testnum        ;\
        la      x10, 1f                 ;\
        li      x11, cause              ;\
        li      x12, 42                 ;\
1:      code                            ;\
        li      x4, 42                  ;\
        bne     x12, x4, fail           ;\
        lw      x4, 0(x9)               ;\
        bnez    x4, fail                ;



init:
        la      x4, trap_handler       # set interrupt handler
        csrw    mtvec, x4
        la      x7, fail
        li      x8, 0xC00A
        la      x9, tdat
        sw      zero, 0(x9)
        li      x4, 12345
        sw      x4, 4(x9)
        li      x4, 100
        csrw    mscratch, x4


        #-------------------------------------------------------------
        # Test 1-5: ecall
        #-------------------------------------------------------------

        li      x13, 0                  # MTVAL=0
        TEST_DELAYSLOTS(1, 11,           \
            ecall                       ;\
            add     x12, x12, 3         ;\
            lw      x12, 4(x9)          ;\
            sw      x8, 0(x9)           ;
        )
        lw      x4, 0(x9)               # test if sw was not executed
        bnez    x4, fail

        li      x13, 0                  # MTVAL=0
        TEST_DELAYSLOTS(2, 11,           \
            ecall                       ;\
            lw      x12, 4(x9)          ;\
            sw      x8, 0(x9)           ;\
            rdcycle x12                 ;
        )
        lw      x4, 0(x9)               # test if sw was not executed
        bnez    x4, fail

        li      x13, 0                  # MTVAL=0
        TEST_DELAYSLOTS(3, 11,           \
            ecall                       ;\
            sw      x8, 0(x9)           ;\
            rdcycle x12                 ;\
            csrw    mscratch, 3         ;
        )
        lw      x4, 0(x9)               # test if sw was not executed
        bnez    x4, fail
        csrr    x4, mscratch            # test if csrw was not executed
        li      x5, 100
        bne     x4, x5, fail

        li      x13, 0                  # MTVAL=0
        TEST_DELAYSLOTS(4, 11,           \
            ecall                       ;\
            rdcycle x12                 ;\
            csrw    mscratch, 3         ;\
            beqz    x12, fail           ;
        )
        csrr    x4, mscratch            # test if csrw was not executed
        li      x5, 100
        bne     x4, x5, fail

        li      x13, 0                  # MTVAL=0
        TEST_DELAYSLOTS(5, 11,           \
            ecall                       ;\
            csrw    mscratch, 3         ;\
            beqz    x12, fail           ;\
            j       fail                ;
        )
        csrr    x4, mscratch            # test if csrw was not executed
        li      x5, 100
        bne     x4, x5, fail


        #-------------------------------------------------------------
        # Test 5-10: ebreak
        #-------------------------------------------------------------

        la      x13, 1f                 # MTVAL=PC
        TEST_DELAYSLOTS(6, 3,            \
            ebreak                      ;\
            beqz    x12, fail           ;\
            j       fail                ;\
            jr      x7                  ;
        )

        la      x13, 1f                 # MTVAL=PC
        TEST_DELAYSLOTS(7, 3,            \
            ebreak                      ;\
            j       fail                ;\
            jr      x7                  ;\
            ecall                       ;
        )

        la      x13, 1f                 # MTVAL=PC
        TEST_DELAYSLOTS(8, 3,            \
            ebreak                      ;\
            jr      x7                  ;\
            ecall                       ;\
            mret                        ;
        )

        la      x13, 1f                 # MTVAL=PC
        TEST_DELAYSLOTS(9, 3,            \
            ebreak                      ;\
            ecall                       ;\
            mret                        ;\
            add     x12, x12, 97        ;
        )

        la      x13, 1f                 # MTVAL=PC
        TEST_DELAYSLOTS(10, 3,           \
            ebreak                      ;\
            mret                        ;\
            add     x12, x12, 97        ;\
            lw      x12, 4(x9)          ;
        )


        #-------------------------------------------------------------
        # Test 11-15: c.ebreak
        #-------------------------------------------------------------
        # CAUTION: code must be exactly 16 bytes or more if the
        #          instructions beyond 16 bytes are nops

        csrr    x13, misa               # dont check if RVC not supported
        andi    x13, x13, 4
        beqz    x13, pass

        .option push
        .option rvc
        c.nop

        la      x13, 1f                 # MTVAL=PC
        TEST_DELAYSLOTS(11, 3,            \
            c.ebreak                      ;\
            beqz    x12, fail           ;\
            j       fail                ;\
            jr      x7                  ;\
            bne     x8, x9, fail        ;\
            c.nop                       ;\
            c.nop                       ;
        )



        la      x13, 1f                 # MTVAL=PC
        TEST_DELAYSLOTS(12, 3,           \
            c.ebreak                    ;\
            j       fail                ;\
            jr      x7                  ;\
            ecall                       ;\
            bne     x8, x9, fail        ;\
            c.nop                       ;\
            c.nop                       ;
        )

        la      x13, 1f                 # MTVAL=PC
        TEST_DELAYSLOTS(13, 3,           \
            c.ebreak                    ;\
            jr      x7                  ;\
            ecall                       ;\
            mret                        ;\
            bne     x8, x9, fail        ;\
            c.nop                       ;\
            c.nop                       ;
        )

        la      x13, 1f                 # MTVAL=PC
        TEST_DELAYSLOTS(14, 3,           \
            c.ebreak                    ;\
            ecall                       ;\
            mret                        ;\
            add     x12, x12, 97        ;\
            c.nop                       ;\
            c.nop                       ;
        )

        la      x13, 1f                 # MTVAL=PC
        TEST_DELAYSLOTS(15, 3,           \
            c.ebreak                    ;\
            mret                        ;\
            add     x12, x12, 97        ;\
            lw      x12, 4(x9)          ;\
            bne     x8, x9, fail        ;\
            c.nop                       ;\
            c.nop                       ;
        )

        j       pass
        .option pop
        .align  2



        TEST_PASSFAIL




# x10 expected address for MEPC
# x11 expected value   for MCAUSE
trap_handler:
        csrr    x20, mepc
        bne     x20, x10, fail
        csrr    x21, mcause
        bne     x21, x11, fail
        csrr    x23, mtval
        bne     x23, x13, fail
        add     x20, x20, 16            # ignore 3 instructions after ecall/ebreak
        csrw    mepc, x20
        mret




RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

tdat:   .word 0, 0

RVTEST_DATA_END

