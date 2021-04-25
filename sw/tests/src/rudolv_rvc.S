#*****************************************************************************
# rudolv_rvc.S
#-----------------------------------------------------------------------------
#
# Test unaligned 32 bit instructions
#
# Copyright Jörg Mische <bobbl@gmx.de>
# SPDX-License-Identifier: ISC


#include "riscv_test.h"
#include "test_macros.h"

#define RVC_NOP .2byte 0x0001

RVTEST_RV64U
RVTEST_CODE_BEGIN

  #-------------------------------------------------------------
  # Test 1: run for a while and read counters
  #-------------------------------------------------------------

test_1:
        addi    TESTNUM, x0, 1

        addi    x4, x0, 4
        li      x5, 5
        li      x6, 6
        RVC_NOP + 0x70
        li      x8, 8           # unaligned
        li      x9, 9           # unaligned
        RVC_NOP + 0xA0          # unaligned
        li      x11, 11         # realigned
        li      x12, 12
        RVC_NOP + 0xD0
        RVC_NOP + 0xE0          # unaligned
        li      x15, 15         # realigned
        li      x16, 16
        RVC_NOP + 0x110         # unaligned
        li      x18, 18         # unaligned
        li      x19, 19         # unaligned
        RVC_NOP + 0x140         # unaligned
        RVC_NOP + 0x150
        RVC_NOP + 0x160         # unaligned
        RVC_NOP + 0x170
        RVC_NOP + 0x180         # unaligned
        li      x4, 2           # realigned

        RVC_NOP + 0x1A0
1:
        RVC_NOP + 0x1B0         # unaligned
        add     x4, x4, -1      # unaligned
        RVC_NOP + 0x1D0         # unaligned
#        RVC_NOP + 0x1E0
        li      x5, 0x55        # unaligned
        bnez    x4, 1b          # unaligned

        RVC_NOP                 # unaligned
        li      x6, 0x66
        li      x7, 0x77



        j       pass

  #-------------------------------------------------------------
  # Test 2: cycle(t1) > cycle(t0)
  #-------------------------------------------------------------
  li  TESTNUM, 2
  beqz x4, fail

  #-------------------------------------------------------------
  # Test 3: cycle(t2) > cycle(t1)
  #-------------------------------------------------------------
  li  TESTNUM, 3
  bgeu x4, x5, fail

  #-------------------------------------------------------------
  # Test 4: instret(t1) > instret(t0)
  #-------------------------------------------------------------
  li  TESTNUM, 4
  beqz x6, fail

  #-------------------------------------------------------------
  # Test 5: instret(t2) > instret(t1)
  #-------------------------------------------------------------
  li  TESTNUM, 5
  bgeu x6, x7, fail


        j       pass
        j       fail
        j       fail
        j       fail
        j       fail
        j       fail

  TEST_PASSFAIL



RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END
