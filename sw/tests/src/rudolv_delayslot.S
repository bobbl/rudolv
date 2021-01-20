#*****************************************************************************
# rudolv_delayslot.S
#-----------------------------------------------------------------------------
#
# Test if instructions in branch delay slots are correctly killed
#   Test 1: load after branch


#include "riscv_test.h"
#include "test_macros.h"



RVTEST_RV64U
RVTEST_CODE_BEGIN

        la      x4, trap_handler       # set interrupt handler
        csrw    mtvec, x4
        la      x9, tdat


        #-------------------------------------------------------------
        # Test 1: load after branch
        #-------------------------------------------------------------
test_1:
        li      TESTNUM, 1
        li      x4, -17
        li      x5, 0
        li      x10, 1
        bnez    x10, target_1
        lw      x4, 0(x9)
        lw      x5, 4(x9)
        li      x4, -1
        li      x5, -2
        li      x4, -3
        li      x5, -4
target_1:
        # The instructions in the delay slots can corrupt the fetching.
        # Therefore check, if the 4 instructions at the branch target
        # are correctly executed.
        add     x4, x4, 2
        add     x4, x4, 3
        add     x4, x4, 5
        add     x4, x4, 7
        bnez    x4, fail
        bnez    x5, fail


        #-------------------------------------------------------------
        # Test 2: load after branch
        #-------------------------------------------------------------
        # CAUTION: Be careful when modifying this code!
        #          Test 1 and 2 are identical, but when fetching 64 bit, 
        #          test 1 is at a even address and test 2 is at an odd one.
test_2:
        li      TESTNUM, 2
        li      x4, 0
        li      x5, -33
        li      x10, 1
        bnez    x10, target_2
        lw      x4, 0(x9)
        lw      x5, 4(x9)
        li      x4, -1
        li      x5, -2
        li      x4, -3
        li      x5, -4
target_2:
        # The instructions in the delay slots can corrupt the fetching.
        # Therefore check, if the 4 instructions at the branch target
        # are correctly executed.
        add     x5, x5, 4
        add     x5, x5, 11
        add     x5, x5, 6
        add     x5, x5, 12
        bnez    x4, fail
        bnez    x5, fail




        #-------------------------------------------------------------
        # Test 3: csr after branch
        #-------------------------------------------------------------
test_3:
        li      TESTNUM, 3
        li      x4, 0
        li      x5, 0
        li      x6, 6
        li      x7, 7
        csrw    mscratch, 4
        li      x10, 1
        bnez    x10, target_3
        csrrw   x4, mscratch, x6
        csrrw   x5, mscratch, x7
        li      x4, -1
        li      x5, -2
target_3:
        bnez    x4, fail
        bnez    x5, fail
        csrr    x4, mscratch
        li      x5, 4
        bne     x4, x5, fail



        #-------------------------------------------------------------
        # Test 4: mul after branch
        #-------------------------------------------------------------
test_4:
        li      TESTNUM, 4
        li      x4, -17
        li      x5, 0
        li      x6, 8
        li      x7, 9
        li      x10, 1
        bnez    x10, target_4
        mul     x4, x6, x7
        mul     x5, x7, x6
        li      x4, -1
        li      x5, -2
        li      x4, -3
        li      x5, -4
target_4:
        # The instructions in the delay slots can corrupt the fetching.
        # Therefore check, if the 4 instructions at the branch target
        # are correctly executed.
        add     x4, x4, 2
        add     x4, x4, 3
        add     x4, x4, 5
        add     x4, x4, 7
        bnez    x4, fail
        bnez    x5, fail



        #-------------------------------------------------------------
        # Test 5: ecall after branch
        #-------------------------------------------------------------
test_5:
        li      TESTNUM, 5
        li      x4, -17
        li      x5, 0
        li      x6, 8
        li      x7, 9
        li      x10, 1
        bnez    x10, target_5
        ecall
        ecall
        li      x4, -1
        li      x5, -2
        li      x4, -3
        li      x5, -4
target_5:
        # The instructions in the delay slots can corrupt the fetching.
        # Therefore check, if the 4 instructions at the branch target
        # are correctly executed.
        add     x4, x4, 2
        add     x4, x4, 3
        add     x4, x4, 5
        add     x4, x4, 7
        bnez    x4, fail
        bnez    x5, fail




        TEST_PASSFAIL




trap_handler:
        li      x4, 99
        li      x5, 98
        li      x4, 97
        li      x5, 96
        mret





RVTEST_CODE_END
  .data
RVTEST_DATA_BEGIN

  TEST_DATA
.align 9

tdat:   .word 47, 11

RVTEST_DATA_END

