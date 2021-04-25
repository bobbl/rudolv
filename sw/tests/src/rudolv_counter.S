#*****************************************************************************
# rudolv_counter.S
#-----------------------------------------------------------------------------
#
# Test instructions to read CSR counters
#
# Copyright Jörg Mische <bobbl@gmx.de>
# SPDX-License-Identifier: ISC


#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  #-------------------------------------------------------------
  # Test 1: run for a while and read counters
  #-------------------------------------------------------------

test_1:
  li  TESTNUM, 1

  li  x4, 0
  li  x5, 0
  li  x6, 0
  li  x7, 0
  li  x8, 0
  li  x9, 0
  li  x10, 0
  li  x11, 0
  li  x12, 0
  li  x13, 0
  li  x14, 0
  li  x15, 0

  rdcycle x4
  mv x5, x4             # check forwarding
  rdinstret x6
  mv x7, x6

  jal delay

  rdcycle x8
  mv x9, x8
  rdinstret x10
  mv x11, x10

  jal delay

  rdcycle x12
  mv x13, x12
  rdinstret x14
  mv x15, x14

  sub x4, x9, x5        # cycle1 - cycle0
  sub x5, x13, x5       # cycle2 - cycle0 
  sub x6, x11, x7       # instret1 - instret0
  sub x7, x15, x7       # instret2 - instret0

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


  TEST_PASSFAIL

# subroutine should run for at least one cycle
delay:
  li x12, 4
  li x13, 1
  li x14, 1
fib_loop:
  add x13, x13, x14
  add x14, x13, x14
  add x12, x12, -1
  bnez x12, fib_loop
  ret
  nop
  nop



RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END
