#*****************************************************************************
# rudolv_illegal.S
#-----------------------------------------------------------------------------
#
# Test illegal instruction exception
#
# Copyright Jörg Mische <bobbl@gmx.de>
# SPDX-License-Identifier: ISC


#include "riscv_test.h"
#include "test_macros.h"



RVTEST_RV64U
RVTEST_CODE_BEGIN


#define TEST_ILLEGAL(testnum, insn)      \
test_ ## testnum:                        \
        li      TESTNUM, testnum        ;\
        la      x20, -1                 ;\
        li      x21, 0                  ;\
        li      x22, 0                  ;\
1:      .word   insn                    ;\
        la      x4, 1b                  ;\
        bne     x20, x4, fail           ;\
        li      x4, 2                   ;\
        bne     x21, x4, fail           ;\
        li      x4, insn                ;\
        bne     x22, x4, fail           ;









init:
        la      x4, trap_handler       # set interrupt handler
        csrw    mtvec, x4



        #-------------------------------------------------------------
        # Test 1: illegal RVC instruction
        #-------------------------------------------------------------

        TEST_ILLEGAL(1, 0x0)           # illegal instruction by spec


        #-------------------------------------------------------------
        # Test 11: illegal opcode
        #-------------------------------------------------------------

        TEST_ILLEGAL(11, 0x6B)          # illegal opcode 1101011 (reserved)


        #-------------------------------------------------------------
        # Test 12: illegal funct7
        #-------------------------------------------------------------

        TEST_ILLEGAL(12, 0x516A8A33)    # sub x20, x21, x22   but bit 28 set


        #-------------------------------------------------------------
        # Test 13: illegal funct3
        #-------------------------------------------------------------

        TEST_ILLEGAL(13, 0x015A2063)    # beq x20, x21, .   but bit 13 set


        #-------------------------------------------------------------
        # Test 21: write to read-only counter
        #-------------------------------------------------------------

        TEST_ILLEGAL(21, 0xC00A1073)     # csrw cycle, x20


        #-------------------------------------------------------------
        # Test 22: read from illegal CSR
        #-------------------------------------------------------------

        TEST_ILLEGAL(22, 0x31F02A73)     # csrr x20, 0x31f





        TEST_PASSFAIL






trap_handler:
        csrr    x20, mepc               # read trap CSRs
        csrr    x21, mcause
        csrr    x22, mtval
        add     x10, x20, 4
        csrw    mepc, x10               # ignore instruction
        mret                            # and return




RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

tdat:   .word 0, 0

RVTEST_DATA_END

