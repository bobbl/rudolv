/* *********************************************************************
 * rudolv_irq.S
 * *********************************************************************
 *
 * Test interrupt handling.
 *   Tests 1-3: no trap
 *   Tests 11-14: IE permanently on, toggle SWI
 *   Tests 21-24: SWI permanently on, toggle IE
 *   Tests 31-32: timer interrupt and WFI
 *   Tests 41-43: WFI in branch delay slot
 *
 * Requires a special CSR configuration:
 *   rdcycle
 *   CSR BC1h: set software interrupt pin (CsrPinsOut)
 *   CSR BC2h: timer interrupt controler (CsrTimerAdd)
 *
 * Copyright Jörg Mische <bobbl@gmx.de>
 * SPDX-License-Identifier: ISC
 */


#include "riscv_test.h"
#include "test_macros.h"


#define ENABLE_INTS     csrs mstatus, 8
#define DISABLE_INTS    csrc mstatus, 8
#define ENABLE_SOFTINT  csrs 0xbc1, 1
#define DISABLE_SOFTINT csrc 0xbc1, 1
#define CSR_TIMER       0xbc2


RVTEST_RV64U
RVTEST_CODE_BEGIN

        #-------------------------------------------------------------
        # Test 1: clear and set MSTATUS.MIE
        #-------------------------------------------------------------
test_1:
        li      TESTNUM, 1
        li      x10, 8
        csrrs   x4, mstatus, 8
        csrrc   x5, mstatus, 8
        csrrs   x6, mstatus, x10
        csrrc   x7, mstatus, x10
        andi    x5, x5, 8
        beqz    x5, fail
        andi    x6, x6, 8
        bnez    x6, fail
        andi    x7, x7, 8
        beqz    x7, fail

        # now IE is cleared => iterrupts disabled


        #-------------------------------------------------------------
        # Test 2: clear and set MSTATUS.MPIE
        #-------------------------------------------------------------
test_2:
        li      TESTNUM, 2
        li      x10, 0x80
        csrrs   x4, mstatus, x10
        csrrc   x5, mstatus, x10
        csrrs   x6, mstatus, x10
        csrrc   x7, mstatus, x10
        andi    x5, x5, 0x80
        beqz    x5, fail
        andi    x6, x6, 0x80
        bnez    x6, fail
        andi    x7, x7, 0x80
        beqz    x7, fail


        #-------------------------------------------------------------
        # Test 3: set MSTATUS.MIE via MRET
        #-------------------------------------------------------------
test_3:
        li      TESTNUM, 3
        li      x10, 0x80
        csrc    mstatus, 8          # MIE=0
        csrs    mstatus, x10        # MPIE=1
        la      x4, return_3
        csrw    mepc, x4
        mret
        j       fail
return_3:
        csrr    x4, mstatus         # after MRET:
        andi    x4, x4, 0x88        # MIE = previous MPIE = 1
        li      x5, 0x88            # MPIE = 1
        bne     x4, x5, fail


        #-------------------------------------------------------------
        # Test 4: clear MSTATUS.MIE via MRET
        #-------------------------------------------------------------
test_4:
        li      TESTNUM, 4
        li      x4, 0x80
        csrc    mstatus, x4     # MPIE := 0 (MIE still 1)
        la      x4, return_4
        csrw    mepc, x4
        mret
        j       fail
return_4:
        csrr    x4, mstatus     # after MRET:
        andi    x4, x4, 0x88    # MIE = previous MPIE = 0
        li      x5, 0x80        # MPIE = 1
        bne     x4, x5, fail

        # MSTATUS.MIE=0 interrupts disabled




        #-------------------------------------------------------------
        # Test 11: IE=0 SWI=0 => no trap
        #-------------------------------------------------------------
test_11:
        li      TESTNUM, 11
        la      x11, trap_disable_softint       # set interrupt handler
        csrrw   x11, mtvec, x11

        li      x6, 100                 # number of cycles to wait
        li      x7, 100
        DISABLE_SOFTINT
        jal     wait_for_int            # wait, handler should not be called
        li      x4, 100
        bne     x7, x4, restore_fail

        #-------------------------------------------------------------
        # Test 12: IE=0 SWI=1 => no trap
        #-------------------------------------------------------------
test_12:
        li      TESTNUM, 12
        li      x7, 100
        ENABLE_SOFTINT
        jal     wait_for_int            # wait, handler should not be called
        li      x4, 100
        bne     x7, x4, restore_fail

        #-------------------------------------------------------------
        # Test 13: IE=1 SWI=0 => no trap
        #-------------------------------------------------------------
test_13:
        li      TESTNUM, 13
        li      x7, 100
        DISABLE_SOFTINT
        fence.i
        ENABLE_INTS
        jal     wait_for_int            # wait, handler should not be called
        li      x4, 100
        bne     x7, x4, restore_fail






        #-------------------------------------------------------------
        # Test 21: IE=1 then SWI=1 => trap (interrupt li sequence)
        #-------------------------------------------------------------
test_21:
        li      TESTNUM, 21
        li      x7, 0x70
        ENABLE_SOFTINT
        xor     x7, x7, 0x400
        xor     x7, x7, 0x200
        xor     x7, x7, 0x100
        xor     x7, x7, 0x080
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x800007f4          # cause: software interrupt
        bne     x7, x4, restore_fail

        #-------------------------------------------------------------
        # Test 22: IE=1 then SWI=1 => trap (interrupt jal)
        #-------------------------------------------------------------
test_22:
        li      TESTNUM, 22
        li      x7, 0x70
        ENABLE_SOFTINT
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x80000074          # cause: software interrupt
        bne     x7, x4, restore_fail

        #-------------------------------------------------------------
        # Test 23: IE=1 then SWI=1 => trap (interrupt multicycle div)
        #-------------------------------------------------------------
test_23:
        li      TESTNUM, 23
        li      x7, 0x400
        li      x10, 1000000000
        li      x11, 1000000
        ENABLE_SOFTINT
        div     x12, x10, x11
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x80000404          # cause: software interrupt
        bne     x7, x4, restore_fail
        li      x4, 1000                # division result correct?
        bne     x12, x4, restore_fail

        #-------------------------------------------------------------
        # Test 24: IE=1 then SWI=1 => trap (interrupt wfi)
        #-------------------------------------------------------------
test_24:
        li      TESTNUM, 24
        li      x7, 0x90
        ENABLE_SOFTINT
        wfi
        j       1f                      # jump after wfi can be problematic
        nop
        nop
        j       restore_fail
1:      li      x4, 0x80000094          # cause: software interrupt
        bne     x7, x4, restore_fail




        # alternative trap handler: clear IE after trap
        DISABLE_INTS
        la      x4, trap_disable_ie
        csrw    mtvec, x4
        fence.i
        ENABLE_SOFTINT
        fence.i

        #-------------------------------------------------------------
        # Test 31: SWI=1 then IE=1 => trap (interrupt li sequence)
        #-------------------------------------------------------------
test_31:
        li      TESTNUM, 31
        li      x7, 0x70
        ENABLE_INTS
        xor     x7, x7, 0x400
        xor     x7, x7, 0x200
        xor     x7, x7, 0x100
        xor     x7, x7, 0x080
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x800007f4          # cause: software interrupt
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 32: SWI=1 then IE=1 => trap (interrupt jal)
        #-------------------------------------------------------------
test_32:
        li      TESTNUM, 32
        li      x7, 0x70
        ENABLE_INTS
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x80000074          # cause: software interrupt
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 33: SWI=1 then IE=1 => trap (interrupt multicycle div)
        #-------------------------------------------------------------
test_33:
        li      TESTNUM, 33
        li      x7, 0x400
        li      x10, 1000000000
        li      x11, 1000000
        ENABLE_INTS
        div     x12, x10, x11
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x80000404          # cause: software interrupt
        bne     x7, x4, restore_fail
        li      x4, 1000                # division result correct?
        bne     x12, x4, restore_fail


        #-------------------------------------------------------------
        # Test 34: SWI=1 then IE=1 => trap (interrupt wfi)
        #-------------------------------------------------------------
test_34:
        li      TESTNUM, 34
        li      x7, 0x70
        ENABLE_INTS
        wfi
        j       1f                      # jump after wfi can be problematic
        nop
        nop
        j       restore_fail
1:      li      x4, 0x80000074          # cause: software interrupt
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 35: SWI=1 then mret to set IE=1 => trap back-to-back to mret
        #-------------------------------------------------------------
test_35:
        li      TESTNUM, 35
        li      x7, 0x70
        li      x4, 0x80
        csrs    mstatus, x4             # MSTATUS.MPIE=1
        la      x4, target_35
        csrw    mepc, x4
        mret                            # set IE to MPIE => trap
        li      x7, 0x666
        nop
        nop
        nop

target_35:
        li      x4, 0x80000074          # cause: software interrupt
        bne     x7, x4, restore_fail







        # alternative trap handler: clear timer int after trap
        DISABLE_SOFTINT
        la      x4, trap_disable_timer
        csrw    mtvec, x4

        #-------------------------------------------------------------
        # Test 41: IE=1 then timer int => trap (interrupt loop)
        #-------------------------------------------------------------
test_41:
        li      TESTNUM, 41
        li      x7, 0x60

        ENABLE_INTS
        li      x4, 50
        csrw    CSR_TIMER, x4

        li      x6, 200
        jal     wait_for_int            # wait, handler should modify x7
        li      x4, 0x80000068          # cause: timer interrupt
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 42: IE=1 then timer int => trap (interrupt wfi)
        #-------------------------------------------------------------
test_42:
        li      TESTNUM, 42
        li      x7, 0x50
        li      x8, 0x50
        rdcycle x5
        add     x5, x5, 100

        ENABLE_INTS
        li      x4, 50
        csrw    CSR_TIMER, x4

1:      wfi
        j       2f                      # jump after wfi can be problematic
        nop
        nop
        j       restore_fail

2:      # If wfi is implemented as nop, wfi must be called multiple times to
        # wait long enough. Therefore repeat while there is no trap (x7
        # unchanged) and no timeout (rdcycle < start+100)
        bne     x7, x8, 3f
        rdcycle x6
        blt     x6, x5, 1b

3:      li      x4, 0x80000058          # cause: timer interrupt
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 43: IE=1 then timer int => trap (interrupt wfi in subroutine)
        #-------------------------------------------------------------
test_43:
        li      TESTNUM, 43
        li      x7, 0x50
        li      x8, 0x50
        rdcycle x5
        add     x5, x5, 100

        ENABLE_INTS
        li      x4, 50
        csrw    CSR_TIMER, x4

4:      jal     sub_wfi

        # If wfi is implemented as nop, wfi must be called multiple times to
        # wait long enough. Therefore repeat while there is no trap (x7
        # unchanged) and no timeout (rdcycle < start+100)
        bne     x7, x8, 5f
        rdcycle x6
        blt     x6, x5, 4b

5:      li      x4, 0x80000058          # cause: timer interrupt
        bne     x7, x4, restore_fail


        #-------------------------------------------------------------
        # Test 44: IE=0 then timer int => exit wfi but do not trap
        #-------------------------------------------------------------
test_44:
        li      TESTNUM, 44
        li      x7, 0x50
        li      x8, 0x50
        rdcycle x5
        add     x5, x5, 100

        DISABLE_INTS
        li      x4, 50
        csrw    CSR_TIMER, x4

6:      wfi

        bne     x7, x8, fail            # trap not allowed during wfi

        # TODO: check MIP.MTIP (not yet supported by RudolV)
        # alternative: enable ints briefly to service a pending int
        ENABLE_INTS
        nop
        DISABLE_INTS
        bne     x7, x8, 7f              # trap was pending => exit loop

        rdcycle x6                      # wait 100 cycles if wfi is a nop
        blt     x6, x5, 6b

7:      li      x4, 0x80000058          # cause: timer interrupt
        bne     x7, x4, restore_fail











        #-------------------------------------------------------------
        # Test 51: wfi in branch delay slot of branch => no trap
        #-------------------------------------------------------------
test_51:
        li      TESTNUM, 51
        li      x7, 0x50

        ENABLE_INTS
        li      x4, 50
        csrw    CSR_TIMER, x4

        nop
        nop
        nop
        li      x4, 3
        li      x5, 3
        beq     x4, x5, test_52
        wfi
        wfi
        j       restore_fail
        nop
        nop


        #-------------------------------------------------------------
        # Test 52: wfi in branch delay slot of jal => no trap
        #-------------------------------------------------------------
test_52:
        li      TESTNUM, 52
        j       test_53
        wfi
        wfi
        j       restore_fail
        nop
        nop


        #-------------------------------------------------------------
        # Test 53: wfi in branch delay slot of jalr => no trap
        #-------------------------------------------------------------
test_53:
        li      TESTNUM, 53
        la      x4, check_53
        jr      x4
        wfi
        wfi
        j       restore_fail
        nop
        nop

check_53:
        li      x4, 0x50
        bne     x7, x4, restore_fail








restore_pass:
        DISABLE_INTS
        csrw    mtvec, x11              # restore trap handler
        csrw    mstatus, x10            # restore cpu state


        TEST_PASSFAIL


restore_fail:
        csrw    mtvec, x11
        j       fail




# Wait x6 cycles
# If am interrupt occurs, x7 will be clobbered

wait_for_int:
        rdcycle x8
        la      x9, tdat
        sw      x8, 0(x9)       # check if SW clears IE
loop:
        rdcycle x9
        sub     x9, x9, x8
        blt     x9, x6, loop
        ret


sub_wfi:
        wfi
        ret
        j       restore_fail


trap_disable_softint:
        DISABLE_SOFTINT
        csrr    x20, mcause
        xor     x7, x7, x20
        add     x7, x7, 1
        j       skip_wfi

trap_disable_ie:
        li      x20, 0x80       # MSTATUS.MPIE=0 disable ints after return
        csrc    mstatus, x20
        csrr    x20, mcause
        xor     x7, x7, x20
        add     x7, x7, 1

# If trap response is so fast that execution is interruptet before the wfi,
# the wfi will wait forever. Skip wfi in this case to avoid that.

skip_wfi:
        csrr    x21, mepc
        lw      x22, 0(x21)             # read insn at return address
        li      x23, 0x10500073         # wfi opcode
        bne     x22, x23, 1f
        add     x21, x21, 4             # if wfi, jump to next insn
        csrw    mepc, x21
1:      mret


trap_disable_timer:
        csrc    CSR_TIMER, 1
        csrr    x20, mcause
        xor     x7, x7, x20
        add     x7, x7, 1
        mret




RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

tdat:   .word 0

RVTEST_DATA_END

