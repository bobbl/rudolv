# See LICENSE for license details.

#*****************************************************************************
# rudolv_irq.S
#-----------------------------------------------------------------------------
#
# Test interrupt handling.
#   Tests 1-3: no trap
#   Tests 4-6: IE permanently on, toggle SWI
#   Tests 7-9: SWI permanently on, toggle IE
#
# Requires a special CSR configuration:
#   rdcycle
#   CSR BC1h: set software interrupt pin (CsrPinsOut)
#   CSR BC2h: timer interrupt controler (CsrTimerAdd)
#


#include "riscv_test.h"
#include "test_macros.h"

#define ENABLE_SOFTINT  csrs 0xbc1, 1
#define DISABLE_SOFTINT csrc 0xbc1, 1
#define ENABLE_INTS     csrs mstatus, 8
#define DISABLE_INTS    csrc mstatus, 8


RVTEST_RV64U
RVTEST_CODE_BEGIN

test_1:
        csrrc   x10, mstatus, 8         # MSTATUS.IE=0 disable interrupts
        la      x11, trap_disable_softint       # set interrupt handler
        csrrw   x11, mtvec, x11

        li      x6, 100                 # number of cycles to wait




        #-------------------------------------------------------------
        # Test 1: IE=0 SWI=0 => no trap
        #-------------------------------------------------------------
        li      TESTNUM, 1
        li      x7, 100
        DISABLE_SOFTINT
        jal     wait_for_int            # wait, handler should not be called
        li      x4, 100
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 2: IE=0 SWI=1 => no trap
        #-------------------------------------------------------------
test_2:
        li      TESTNUM, 2
        li      x7, 100
        ENABLE_SOFTINT
        jal     wait_for_int            # wait, handler should not be called
        li      x4, 100
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 3: IE=1 SWI=0 => no trap
        #-------------------------------------------------------------
test_3:
        li      TESTNUM, 3
        li      x7, 100
        DISABLE_SOFTINT
        fence.i
        ENABLE_INTS
        jal     wait_for_int            # wait, handler should not be called
        li      x4, 100
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 4: IE=1 then SWI=1 => trap (interrupt li sequence)
        #-------------------------------------------------------------
test_4:
        li      TESTNUM, 4
        li      x7, 0x70
        ENABLE_SOFTINT
        xor     x7, x7, 0x400
        xor     x7, x7, 0x200
        xor     x7, x7, 0x100
        xor     x7, x7, 0x080
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x800007f3          # cause: software interrupt
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 5: IE=1 then SWI=1 => trap (interrupt jal)
        #-------------------------------------------------------------
test_5:
        li      TESTNUM, 5
        li      x7, 0x70
        ENABLE_SOFTINT
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x80000073          # cause: software interrupt
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 6: IE=1 then SWI=1 => trap (interrupt multicycle div)
        #-------------------------------------------------------------
test_6:
        li      TESTNUM, 6
        li      x7, 0x400
        li      x10, 1000000000
        li      x11, 1000000
        ENABLE_SOFTINT
        div     x12, x10, x11
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x80000403          # cause: software interrupt
        bne     x7, x4, restore_fail
        li      x4, 1000                # division result correct?
        bne     x12, x4, restore_fail




        # alternative trap handler: clear IE after trap
        DISABLE_INTS
        la      x4, trap_disable_ie
        csrrw   x4, mtvec, x4
        fence.i
        ENABLE_SOFTINT
        fence.i

        #-------------------------------------------------------------
        # Test 7: SWI=1 then IE=1 => trap (interrupt li sequence)
        #-------------------------------------------------------------
test_7:
        li      TESTNUM, 12
        li      x7, 0x70
        ENABLE_INTS
        xor     x7, x7, 0x400
        xor     x7, x7, 0x200
        xor     x7, x7, 0x100
        xor     x7, x7, 0x080
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x800007f3          # cause: software interrupt
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 8: SWI=1 then IE=1 => trap (interrupt jal)
        #-------------------------------------------------------------
test_8:
        li      TESTNUM, 13
        li      x7, 0x70
        ENABLE_INTS
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x80000073          # cause: software interrupt
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 9: SWI=1 then IE=1 => trap (interrupt multicycle div)
        #-------------------------------------------------------------
test_9:
        li      TESTNUM, 9
        li      x7, 0x400
        li      x10, 1000000000
        li      x11, 1000000
        ENABLE_INTS
        div     x12, x10, x11
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x80000403          # cause: software interrupt
        bne     x7, x4, restore_fail
        li      x4, 1000                # division result correct?
        bne     x12, x4, restore_fail







restore_pass:
        DISABLE_INTS
        csrw    mtvec, x11              # restore trap handler
        csrw    mstatus, x10            # restore cpu state


        TEST_PASSFAIL


restore_fail:
        csrw    mtvec, x4
        j       fail






# Wait x6 cycles
# If am interrupt occurs, x7 will be clobbered
# Input: x5 saved mstatus, will be restored at end of subroutine

wait_for_int:
        rdcycle x8
loop:
        rdcycle x9
        sub     x9, x9, x8
        blt     x9, x6, loop
        ret


trap_disable_softint:
        DISABLE_SOFTINT
        csrr    x20, mcause
        xor     x7, x7, x20
        mret


trap_disable_ie:
        li      x20, 0x80       # MSTATUS.MPIE=0 disable ints after return
        csrc    mstatus, x20
        csrr    x20, mcause
        xor     x7, x7, x20
        mret




RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

