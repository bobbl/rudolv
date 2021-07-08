# See LICENSE for license details.

#*****************************************************************************
# irq_1.S
#-----------------------------------------------------------------------------
#

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

        li      x4, 130                 # IRQ bombing: duration 80 cycles
        csrw    0x3f8, x4               # IRQ bombing: set marker
        nop

        ENABLE_INTS

# unconditional jumps

        j       3f

1:      j       2f
2:      j       2f
2:      j       2f
2:      j       4f

3:      j       1b
4:

#

        DISABLE_INTS
        nop
        nop
        nop
        ENABLE_INTS


# from here is identical to irq_1.S


# conditional branches

        nop
        bnez    x4, 7f

5:      bnez    x4, 6f
6:      bnez    x4, 6f
6:      bnez    x4, 8f

7:      bnez    x4, 5b
8:



# loads
        la      x5, tdat

        sw      x15, 8(x5)
        sw      x13, 0(x5)
        sw      x14, 4(x5)
        sw      x16, 12(x5)
        lw      x6, 0(x5)
        lw      x7, 4(x5)
        lw      x8, 8(x5)
        lw      x9, 12(x5)
        bne     x6, x13, fail
        bne     x7, x14, fail
        bne     x8, x15, fail
        bne     x9, x16, fail

        li      x4, 400
        li      x5, 50
        csrw    mscratch, x4
        csrrw   x6, mscratch, x5
        csrrw   x7, mscratch, x4
        csrrw   x8, mscratch, x4
        csrrw   x9, mscratch, x5
        bne     x6, x4, fail
        bne     x7, x5, fail
        bne     x8, x4, fail
        bne     x9, x4, fail


        li      x5, 10
9:      add     x5, x5, -1
        bnez    x5, 9b

        TEST_PASSFAIL



trap_irqbomb_response:
        csrr    x20, mcause
        li      x21, 0x8000000B
        bne     x20, x21, fail

        csrw    0x3f9, 1                # IRQ bombing: response
        mret







RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

tdat:   .word 11, 22, 33, 44, 55, 66, 77, 88

RVTEST_DATA_END

