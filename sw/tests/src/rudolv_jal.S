#*****************************************************************************
# rudolv_jal.S
#-----------------------------------------------------------------------------
#
# Additional tests for the jal instruction.
#
# Copyright Jörg Mische <bobbl@gmx.de>
# SPDX-License-Identifier: ISC


#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  #-------------------------------------------------------------
  # Test 2: Misuse of JAL to read PC
  #-------------------------------------------------------------

test_2:
  li  TESTNUM, 2
  li  ra, 0

  jal x4, target_2
target_2:
  la  x2, target_2
  bne x2, x4, fail

  #-------------------------------------------------------------
  # Test 3: jump into delay slot
  #-------------------------------------------------------------

  TEST_CASE( 3, x4, 10, \
    nop; \
    li  x4, 1; \
    jal ra, 1f; \
1:  add x4, x4, 6; \
    add x4, x4, 1; \
    add x4, x4, 1; \
    add x4, x4, 1; \
  )

  #-------------------------------------------------------------
  # Tests 4-7: counter read in delay slot
  #-------------------------------------------------------------

  TEST_CASE( 4, x4, 2, \
    li  x4, 1; \
    jal ra, 1f; \
    rdcycle x4; \
    addi x4, x4, 1; \
    addi x4, x4, 1; \
1:  addi x4, x4, 1; \
  )

  TEST_CASE( 5, x4, 2, \
    li  x4, 1; \
    jal ra, 1f; \
    addi x4, x4, 1; \
    rdcycle x4; \
    addi x4, x4, 1; \
1:  addi x4, x4, 1; \
  )

  TEST_CASE( 6, x4, 2, \
    li  x4, 1; \
    jal ra, 1f; \
    addi x4, x4, 1; \
    addi x4, x4, 1; \
    rdcycle x4; \
1:  addi x4, x4, 1; \
  )

  TEST_CASE( 7, x4, 2, \
    li  x4, 1; \
    jal ra, 1f; \
    rdcycle x4; \
    rdcycle x4; \
    rdcycle x4; \
    rdcycle x4; \
1:  addi x4, x4, 1; \
  )

  #-------------------------------------------------------------
  # Tests 8-11: csr read in delay slot
  #-------------------------------------------------------------

  TEST_CASE( 8, x4, 2, \
    li  x4, 1; \
    jal ra, 1f; \
    csrr x4, mtvec; \
    addi x4, x4, 1; \
    addi x4, x4, 1; \
1:  addi x4, x4, 1; \
  )

  TEST_CASE( 9, x4, 2, \
    li  x4, 1; \
    jal ra, 1f; \
    addi x4, x4, 1; \
    csrr x4, mtvec; \
    addi x4, x4, 1; \
1:  addi x4, x4, 1; \
  )

  TEST_CASE(10, x4, 2, \
    li  x4, 1; \
    jal ra, 1f; \
    addi x4, x4, 1; \
    addi x4, x4, 1; \
    csrr x4, mtvec; \
1:  addi x4, x4, 1; \
  )

  TEST_CASE(11, x4, 2, \
    li  x4, 1; \
    jal ra, 1f; \
    csrr x4, mtvec; \
    csrr x4, mtvec; \
    csrr x4, mtvec; \
    csrr x4, mtvec; \
1:  addi x4, x4, 1; \
  )

  #-------------------------------------------------------------
  # Tests 12-15: aligned load in delay slot
  #-------------------------------------------------------------

  TEST_CASE(12, x4, 2, \
    li  x4, 1; \
    la  x5, tdat1; \
    jal ra, 1f; \
    lw  x4, 0(x5); \
    addi x4, x4, 1; \
    addi x4, x4, 1; \
1:  addi x4, x4, 1; \
  )

  TEST_CASE(13, x4, 2, \
    li  x4, 1; \
    la  x5, tdat1; \
    jal ra, 1f; \
    addi x4, x4, 1; \
    lb  x4, 0(x5); \
    addi x4, x4, 1; \
1:  addi x4, x4, 1; \
  )

  TEST_CASE(14, x4, 2, \
    li  x4, 1; \
    la  x5, tdat1; \
    jal ra, 1f; \
    addi x4, x4, 1; \
    addi x4, x4, 1; \
    lh  x4, 0(x5); \
1:  addi x4, x4, 1; \
  )

  TEST_CASE(15, x4, 2, \
    li  x4, 1; \
    la  x5, tdat1; \
    jal ra, 1f; \
    lw  x4, 0(x5); \
    lw  x4, 0(x5); \
    lw  x4, 0(x5); \
    lw  x4, 0(x5); \
1:  addi x4, x4, 1; \
  )

  #-------------------------------------------------------------
  # Test 16: JALR with a killed memory access before
  #-------------------------------------------------------------

test_16:
        li      TESTNUM, 16
        li      x4, 0
        la      ra, 2f 
        la      x5, tdat1
        beqz    x4, 1f
        bnez    x4, 1f          # never reached
        lw      x4, 0(x5)       # never reached, but in the delay slot
        bnez    x4, 1f          # never reached
1:      jalr    ra
        li      x4, 3           # should not be reached
2:      add     x4, x4, 1
        li      x5, 1
        bne     x4, x5, fail




  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

tdat1:  .word 0x11223344
tdat2:  .word 0x55667788

RVTEST_DATA_END
