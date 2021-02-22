# See LICENSE for license details.

#*****************************************************************************
# irq_wfi.S
#-----------------------------------------------------------------------------
#
# Test endless loop directly after WFI

#include "riscv_test.h"
#include "test_macros.h"


#define ENABLE_INTS     csrs mstatus, 8
#define DISABLE_INTS    csrc mstatus, 8
#define ENABLE_SOFTINT  csrs 0xbc1, 1
#define DISABLE_SOFTINT csrc 0xbc1, 1
#define ENABLE_EXTINT   csrs 0xbc1, 2
#define DISABLE_EXTINT  csrc 0xbc1, 2
#define CSR_TIMER       0xbc2


RVTEST_RV64U
RVTEST_CODE_BEGIN


        csrrc   x10, mstatus, 8         # MSTATUS.IE=0 disable interrupts
        la      x11, trap_irqbomb_response   # set interrupt handler
        csrrw   x11, mtvec, x11

test_1:
        li      TESTNUM, 1
        li      x6, 0
        li      x7, 0
        li      x8, 0
        li      x9, 0
        li      x13, 1
        li      x14, 2
        li      x15, 4
        li      x16, 8

        ENABLE_INTS

        li      x4, 20                  # IRQ bombing: duration 80 cycles
        csrw    0x3f8, x4               # IRQ bombing: set marker
        nop


        wfi

#        nop

1:      j       1b



        nop
        nop
        nop
        nop

trap_irqbomb_response:
        csrr    x20, mcause
        li      x21, 0x8000000B
        bne     x20, x21, fail

        csrw    0x3f9, 1                # IRQ bombing: response


        TEST_PASSFAIL





RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

tdat:   .word 11, 22, 33, 44, 55, 66, 77, 88

RVTEST_DATA_END

