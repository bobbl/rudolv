/* *********************************************************************
 * rudolv_misalign.S
 * *********************************************************************
 *
 * Test misaligned memory accesses
 *
 * Copyright Jörg Mische <bobbl@gmx.de>
 * SPDX-License-Identifier: ISC
 */


#include "riscv_test.h"
#include "test_macros.h"



RVTEST_RV64U
RVTEST_CODE_BEGIN

        la      x4, fail                # fail if exception is raised
        csrw    mtvec, x4
        la      x9, tdat+4



        #-------------------------------------------------------------
        # Test 1: misaligned lw, ordered
        #-------------------------------------------------------------
test_1:
        li      TESTNUM, 1
        li      x10, 0
        li      x11, 0
        li      x12, 0
        li      x13, 0
        li      x14, 0
        li      x15, 0
        li      x16, 0
        li      x17, 0
        li      x19, 0

        li      x20, 0x44332211
        li      x21, 0x55443322
        li      x22, 0x66554433
        li      x23, 0x77665544
        li      x24, 0x88776655
        li      x25, 0x99887766
        li      x26, 0xAA998877
        li      x27, 0xBBAA9988
        /* x28 is TESTNUM */
        li      x29, 0xCCBBAA99

        lw      x10, -4(x9)
        lw      x11, -3(x9)
        lw      x12, -2(x9)
        lw      x13, -1(x9)
        lw      x14, 0(x9)
        lw      x15, 1(x9)
        lw      x16, 2(x9)
        bne     x16, x26, fail
        lw      x17, 3(x9)
        bne     x17, x27, fail
        lw      x19, 4(x9)
        bne     x10, x20, fail
        bne     x11, x21, fail
        bne     x12, x22, fail
        bne     x13, x23, fail
        bne     x14, x24, fail
        bne     x15, x25, fail
        bne     x19, x29, fail



        #-------------------------------------------------------------
        # Test 2: misaligned lw, unordered
        #-------------------------------------------------------------
test_2:
        li      TESTNUM, 2
        li      x10, 0
        li      x11, 0
        li      x12, 0
        li      x13, 0
        li      x14, 0
        li      x15, 0
        li      x16, 0
        li      x17, 0
        li      x19, 0

        lw      x12, -2(x9)
        lw      x11, -3(x9)
        bne     x12, x22, fail
        lw      x13, -1(x9)
        lw      x15, 1(x9)
        lw      x17, 3(x9)
        bne     x11, x21, fail
        bne     x13, x23, fail
        lw      x10, -4(x9)
        lw      x16, 2(x9)
        lw      x14, 0(x9)
        lw      x19, 4(x9)
        bne     x10, x20, fail
        bne     x14, x24, fail
        bne     x15, x25, fail
        bne     x16, x26, fail
        bne     x17, x27, fail
        bne     x19, x29, fail



        #-------------------------------------------------------------
        # Test 3: misaligned lh, ordered
        #-------------------------------------------------------------
test_3:
        li      TESTNUM, 3
        li      x10, 0
        li      x11, 0
        li      x12, 0
        li      x13, 0
        li      x14, 0
        li      x15, 0
        li      x16, 0
        li      x17, 0
        li      x19, 0

        li      x20, 0x00002211
        li      x21, 0x00003322
        li      x22, 0x00004433
        li      x23, 0x00005544
        li      x24, 0x00006655
        li      x25, 0x00007766
        li      x26, 0xFFFF8877
        li      x27, 0xFFFF9988
        /* x28 is TESTNUM */
        li      x29, 0xFFFFAA99

        lh      x10, -4(x9)
        lh      x11, -3(x9)
        bne     x10, x20, fail
        bne     x11, x21, fail
        lh      x12, -2(x9)
        lh      x13, -1(x9)
        bne     x12, x22, fail
        bne     x13, x23, fail
        lh      x14, 0(x9)
        lh      x15, 1(x9)
        lh      x16, 2(x9)
        bne     x14, x24, fail
        bne     x15, x25, fail
        bne     x16, x26, fail
        lh      x17, 3(x9)
        lh      x19, 4(x9)
        bne     x17, x27, fail
        bne     x19, x29, fail

        # special bug in RudolV: signed/unsigned bit taken from next instruction
        lh      x10, 3(x9)
        xori    x6, x6, 6       # insn does not matter, but bit 14 must be set
        bne     x10, x17, fail




        #-------------------------------------------------------------
        # Test 4: misaligned lhu, unordered
        #-------------------------------------------------------------
test_4:
        li      TESTNUM, 4
        li      x10, 0
        li      x11, 0
        li      x12, 0
        li      x13, 0
        li      x14, 0
        li      x15, 0
        li      x16, 0
        li      x17, 0
        li      x19, 0

/*
        li      x20, 0x00002211
        li      x21, 0x00003322
        li      x22, 0x00004433
        li      x23, 0x00005544
        li      x24, 0x00006655
        li      x25, 0x00007766
*/
        li      x26, 0x00008877
        li      x27, 0x00009988
        /* x28 is TESTNUM */
        li      x29, 0x0000AA99

        lhu     x17, 3(x9)
        lhu     x12, -2(x9)
        lhu     x19, 4(x9)
        lhu     x13, -1(x9)
        bne     x13, x23, fail
        lhu     x11, -3(x9)
        bne     x11, x21, fail
        lhu     x15, 1(x9)
        bne     x15, x25, fail
        lhu     x10, -4(x9)
        bne     x10, x20, fail
        lhu     x14, 0(x9)
        bne     x14, x24, fail
        lhu     x16, 2(x9)

        bne     x12, x22, fail
        bne     x16, x26, fail
        bne     x17, x27, fail
        bne     x19, x29, fail




        .option push
        .option rvc


        #-------------------------------------------------------------
        # Test 5: misaligned c.lw ordered
        #-------------------------------------------------------------
test_5:
        li      TESTNUM, 5
        mv      x31, x9
        addi    x8, x31, -3
        addi    x9, x31, -2
        addi    x10, x31, -1

        li      x11, 0
        li      x12, 0
        li      x13, 0
        li      x14, 0
        li      x15, 0
        li      x16, 0
        li      x17, 0
        li      x19, 0

        li      x21, 0x55443322
        li      x22, 0x66554433
        li      x23, 0x77665544
        li      x24, 0x99887766
        li      x25, 0xBBAA9988
        li      x26, 0x88776655
        li      x27, 0x99887766
        /* x28 is TESTNUM */
        li      x29, 0xAA998877

        c.lw    x11, 0(x8)
        c.lw    x12, 0(x9)
        c.lw    x13, 0(x10)
        c.lw    x14, 4(x8)
        c.lw    x15, 4(x10)
        lw      x16, 0(x31)
        lw      x17, 1(x31)
        lw      x19, 2(x31)

        bne     x11, x21, fail
        bne     x12, x22, fail
        bne     x13, x23, fail
        bne     x14, x24, fail
        bne     x15, x25, fail
        bne     x16, x26, fail
        bne     x17, x27, fail
        bne     x19, x29, fail




        #-------------------------------------------------------------
        # Test 6: misaligned c.lw unordered
        #-------------------------------------------------------------
test_6:
        li      TESTNUM, 6

        li      x11, 0
        li      x12, 0
        li      x13, 0
        li      x14, 0
        li      x15, 0
        li      x16, 0
        li      x17, 0
        li      x19, 0

        c.lw    x11, 0(x8)
        lw      x16, 0(x31)
        c.lw    x12, 0(x9)
        lw      x17, 1(x31)
        c.lw    x13, 0(x10)
        bne     x11, x21, fail
        c.lw    x15, 4(x10)
        c.lw    x14, 4(x8)
        lw      x19, 2(x31)

        bne     x12, x22, fail
        bne     x13, x23, fail
        bne     x14, x24, fail
        bne     x15, x25, fail
        bne     x16, x26, fail
        bne     x17, x27, fail
        bne     x19, x29, fail



        #-------------------------------------------------------------
        # Test 11: misaligned stores
        #-------------------------------------------------------------
test_11:
        li      TESTNUM, 11

        la      x8, tdat
        addi    x9, x8, 1
        addi    x10, x8, 2
        addi    x11, x8, 3
        li      x12, 0x55443322
        li      x13, 0x99887766
        li      x16, 0xDDCCBBAA
        li      x17, 0
        li      x18, 0

        c.sw    x12, 0(x8)      # +0  2345 .... .... .... ....
        sw      x16, 0(x9)      # +1  .ABC D... .... .... ....
        c.sw    x12, 4(x11)     # +7  .... ...2 345. .... ....
        c.li    x17, 7
        sh      x12, 4(x9)      # +5  .... .23. .... .... ....
        c.sw    x13, 12(x8)     # +12 .... .... .... 6789 ....
        c.sw    x13, 16(x8)     # +16 .... .... .... .... 6789
        c.sw    x12, 12(x10)    # +14 .... .... .... ..23 45..
        c.li    x18, 8
        sh      x16, 8(x11)     # +11 .... .... ...A B... ....
                                #     2ABC D232 345A B723 4589

        li      x19, 0xCCBBAA22
        li      x20, 0x223322DD
        li      x21, 0xAA554433
        li      x22, 0x332277BB
        li      x23, 0x99885544

        c.lw    x9, 0(x8)
        c.lw    x10, 4(x8)
        bne     x9, x19, fail
        c.lw    x11, 8(x8)
        bne     x10, x20, fail
        c.lw    x12, 12(x8)
        c.lw    x13, 16(x8)
        bne     x11, x21, fail
        bne     x12, x22, fail
        bne     x13, x23, fail







        j       pass

        .option pop
        .align 4

        TEST_PASSFAIL



RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

        .align 4
tdat:   .word 0x44332211, 0x88776655, 0xCCBBAA99

RVTEST_DATA_END

