# See LICENSE for license details.

#*****************************************************************************
# jal.S
#-----------------------------------------------------------------------------
#
# Test jal instruction.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

test_1:

        li      x6, 800                 # number of cycles to wait
        li      x7, 100



        rdcycle x8
        la      x5, trap_handler        # set interrupt handler
        csrrw   x4, mtvec, x5

wait_no_int:                            # wait, handler should not be called
        rdcycle x9
        sub     x9, x9, x8
        blt     x9, x6, wait_no_int
        mv      x10, x7

        rdcycle x8
        li      x7, 0x70
        csrrs   x5, mstatus, 8          # MSTATUS.IE=1 enable interrupts

wait_int:                               # wait, handler should modify x7
        rdcycle x9
        sub     x9, x9, x8
        blt     x9, x6, wait_int
        mv      x11, x7

        csrw    mstatus, x5             # restore cpu state
        csrw    mtvec, x4



        #-------------------------------------------------------------
        # Test 1: handler should not be called
        #-------------------------------------------------------------
        li      TESTNUM, 1
        li      x4, 100
        bne     x10, x4, fail

        #-------------------------------------------------------------
        # Test 2: handler should be called
        #-------------------------------------------------------------
test_2:
        li      TESTNUM, 2
        li      x4, 0x80000077          # cause: timer interrupt
        bne     x11, x4, fail



        TEST_PASSFAIL



trap_handler:
        csrr    x20, mepc           # increment return address
        addi    x20, x20, 4
        csrw    mepc, x20

        csrr    x20, mcause
        add     x7, x7, x20
        mret


RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

