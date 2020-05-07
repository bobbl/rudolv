#*****************************************************************************
# rudolv_misalign.S
#-----------------------------------------------------------------------------
#
# Test if misaligned memory access exceptions set MEPC correctly
#   Test 1: load address misaligned exception 4
#   Test 2: store/AMO address misaligned exception 6
#   Test 3: instruction address misaligned exception 0


#include "riscv_test.h"
#include "test_macros.h"



RVTEST_RV64U
RVTEST_CODE_BEGIN

test_1:
        la      x4, trap_handler       # set interrupt handler
        csrw    mtvec, x4
        la      x9, tdat+4



        #-------------------------------------------------------------
        # Test 1: misaligned load address
        #-------------------------------------------------------------
        li      TESTNUM, 1
        la      x10, load_1             # expected MEPC
        li      x11, 4                  # expected MCAUSE
        add     x12, x9, 1              # expected MTVAL
load_1:
        lw      x4, 1(x9)


        #-------------------------------------------------------------
        # Test 2: misaligned store address
        #-------------------------------------------------------------
        li      TESTNUM, 2
        la      x10, store_2            # expected MEPC
        li      x11, 6                  # expected MCAUSE
        add     x12, x9, 3              # expected MTVAL
store_2:
        sw      x0, 3(x9)


        #-------------------------------------------------------------
        # Test 3: misaligned jump address
        #-------------------------------------------------------------
        li      TESTNUM, 3
        la      x10, jump_2             # expected MEPC
        li      x11, 0                  # expected MCAUSE
        add     x12, x9, 2              # expected MTVAL
jump_2:
        jr      2(x9)



        TEST_PASSFAIL




# x10 expected address for MEPC
# x11 expected value   for MCAUSE
# x12 expected address for MTVAL
trap_handler:
        csrr    x20, mepc
        bne     x20, x10, fail
        csrr    x21, mcause
        bne     x21, x11, fail
        csrr    x22, mtval
        bne     x22, x12, fail
        add     x20, x20, 4
        csrw    mepc, x20
        mret




RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

tdat:   .word 0, 0

RVTEST_DATA_END

