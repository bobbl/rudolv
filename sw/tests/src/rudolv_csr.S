/* *********************************************************************
 * rudolv_csr.S
 * *********************************************************************
 *
 * Test CSR access via external interface (CsrScratch required)
 *
 * Using the same test patterns as riscv-compliance
 *
 * Copyright Jörg Mische <bobbl@gmx.de>
 * SPDX-License-Identifier: ISC
 */


#include "riscv_test.h"
#include "test_macros.h"



RVTEST_RV64U
RVTEST_CODE_BEGIN


/* CSR under test */
#define CSRUT 0xBC3



init:
        # constants
        li      x20, 0x000F0000
        li      x21, 1
        li      x22, 0x800F0000
        li      x23, 0x7FF0FFFF
        li      x24, 0x80000000
        li      x25, -1
        li      x26, -2
        li      x27, 0x7FFFFFFF


        # random numbers (change as you like)
        li      x29, 0xa3d95b33
        li      x30, 0x72cfa435
        li      x31, 0x10b05fb5



        #-------------------------------------------------------------
        # Test 1: CSRRW
        #-------------------------------------------------------------

test_1:
        li      TESTNUM, 1

        csrrw   x0, CSRUT, x0
        csrrw   x1, CSRUT, x21
        csrrw   x2, CSRUT, x0
        csrrw   x3, CSRUT, x22
        csrrw   x4, CSRUT, x23
        csrrw   x5, CSRUT, x24
        csrrw   x6, CSRUT, x25
        csrrw   x7, CSRUT, x26
        csrrw   x8, CSRUT, x27
        csrrw   x9, CSRUT, x0

        bne     x1, x0, fail
        bne     x2, x21, fail
        bne     x3, x0, fail
        bne     x4, x22, fail
        bne     x5, x23, fail
        bne     x6, x24, fail
        bne     x7, x25, fail
        bne     x8, x26, fail
        bne     x9, x27, fail

        csrrw   x0, CSRUT, x29
        csrrw   x1, CSRUT, x30
        csrrw   x2, CSRUT, x1
        csrrw   x3, CSRUT, x2
        csrrw   x4, CSRUT, x3
        csrrw   x5, CSRUT, x4
        csrrw   x6, CSRUT, x0

        bne     x1, x29, fail
        bne     x2, x30, fail
        bne     x3, x29, fail
        bne     x4, x30, fail
        bne     x5, x29, fail
        bne     x6, x30, fail

        csrrw   x0, CSRUT, x29
        csrrw   x0, CSRUT, x0
        csrrw   x0, CSRUT, x0
        csrrw   x1, CSRUT, x0
        bne     x1, x0, fail

        mv      x2, x29
        mv      x3, x30
        mv      x4, x31
        csrrw   x0, CSRUT, x2
        csrrw   x3, CSRUT, x3
        csrrw   x4, CSRUT, x4
        csrrw   x5, CSRUT, x0

        bne     x3, x29, fail
        bne     x4, x30, fail
        bne     x5, x31, fail





        #-------------------------------------------------------------
        # Test 2: CSRRC
        #-------------------------------------------------------------

test_2:
        li      TESTNUM, 2
        li      x1, 1
        li      x2, 0
        li      x3, 0x7FF0FFFF
        li      x4, 0x80000000
        li      x5, -1
        csrrw   x0, CSRUT, x25
        csrrc   x1, CSRUT, x1
        csrrw   x1, CSRUT, x25
        csrrc   x2, CSRUT, x2
        csrrw   x2, CSRUT, x25
        csrrc   x3, CSRUT, x3
        csrrw   x3, CSRUT, x25
        csrrc   x4, CSRUT, x4
        csrrw   x4, CSRUT, x25
        csrrc   x5, CSRUT, x5
        csrrw   x5, CSRUT, x0

        bne     x1, x26, fail          # 0xFFFFFFFE
        bne     x2, x25, fail          # 0xFFFFFFFF
        bne     x3, x22, fail          # 0x800F0000
        bne     x4, x27, fail          # 0x7FFFFFFF
        bne     x5, x0, fail           # 0x00000000




        li      x11, 1
        li      x12, 0
        li      x13, 0x7FF0FFFF
        li      x14, 0x80000000
        li      x15, -1
        csrrw   x0, CSRUT, x15
        csrrc   x11, CSRUT, x11
        csrrc   x12, CSRUT, x12
        csrrc   x13, CSRUT, x13
        csrrc   x14, CSRUT, x14
        csrrc   x15, CSRUT, x15
        csrrc   x16, CSRUT, x0

        bne     x11, x25, fail          # 0xFFFFFFFF
        bne     x12, x26, fail          # 0xFFFFFFFE
        bne     x13, x26, fail          # 0xFFFFFFFE
        bne     x14, x22, fail          # 0x800F0000
        bne     x15, x20, fail          # 0x000F0000
        bne     x16, x0, fail           # 0x00000000



        mv      x1, x30                 # random number
        xori    x11, x1, -1             # imitate bit clear

        csrrw   x0, CSRUT, x25
        csrrc   x2, CSRUT, x1
        csrrc   x1, CSRUT, x2
        csrrw   x3, CSRUT, x25
        csrrc   x4, CSRUT, x3
        csrrc   x5, CSRUT, x0

        bne     x1, x11, fail
        bne     x2, x25, fail          # 0xFFFFFFFE
        bne     x3, x0, fail           # 0x00000000
        bne     x4, x25, fail          # 0xFFFFFFFF
        bne     x5, x25, fail          # 0xFFFFFFFF



        csrrw   x0, CSRUT, x29
        csrrc   x0, CSRUT, x0
        csrrc   x0, CSRUT, x0
        csrrc   x6, CSRUT, x0
        bne     x6, x29, fail

        mv      x7, x29                 # random number
        mv      x8, x31                 # random number
        xori    x18, x7, -1             # imitate bit clear
        xori    x19, x8, -1
        and     x19, x18, x19           # imitate bit clear

        csrrw   x0, CSRUT, x25
        csrrc   x7, CSRUT, x7
        csrrc   x8, CSRUT, x8
        csrrc   x9, CSRUT, x0

        bne     x7, x25, fail
        bne     x8, x18, fail
        bne     x9, x19, fail




        #-------------------------------------------------------------
        # Test 3: CSRRS
        #-------------------------------------------------------------

test_3:
        li      TESTNUM, 3

        li      x1, 1
        li      x2, 0
        li      x3, 0x7FF0FFFF
        li      x4, 0x80000000
        li      x5, -1

        csrrw   x0, CSRUT, x0
        csrrs   x1, CSRUT, x1
        csrrw   x1, CSRUT, x0
        csrrs   x2, CSRUT, x2
        csrrw   x2, CSRUT, x0
        csrrs   x3, CSRUT, x3
        csrrw   x3, CSRUT, x0
        csrrs   x4, CSRUT, x4
        csrrw   x4, CSRUT, x0
        csrrs   x5, CSRUT, x5
        csrrw   x5, CSRUT, x0

        bne     x1, x21, fail           # 0x00000001
        bne     x2, x0, fail            # 0x00000000
        bne     x3, x23, fail           # 0x7FF0FFFF
        bne     x4, x24, fail           # 0x80000000
        bne     x5, x25, fail           # 0xFFFFFFFF




        li      x1, 1
        li      x2, 0
        li      x3, 0x7FF0FFFF
        li      x4, 0x80000000
        li      x5, -1
        li      x15, 0xFFF0FFFF

        csrrw   x0, CSRUT, x0
        csrrs   x1, CSRUT, x1
        csrrs   x2, CSRUT, x2
        csrrs   x3, CSRUT, x3
        csrrs   x4, CSRUT, x4
        csrrs   x5, CSRUT, x5
        csrrs   x6, CSRUT, x0

        bne     x1, x0, fail           # 0x00000000
        bne     x2, x21, fail          # 0x00000001
        bne     x3, x21, fail          # 0x00000001
        bne     x4, x23, fail          # 0x7FF0FFFF
        bne     x5, x15, fail          # 0xFFF0FFFF
        bne     x6, x25, fail          # 0xFFFFFFFF



        mv      x1, x29
        csrrw   x0, CSRUT, x0

        csrrs   x2, CSRUT, x1
        csrrs   x3, CSRUT, x2
        csrrw   x3, CSRUT, x0
        csrrs   x4, CSRUT, x3
        csrrs   x5, CSRUT, x0

        bne     x1, x29, fail
        bne     x2, x0, fail
        bne     x3, x29, fail
        bne     x4, x0, fail
        bne     x5, x29, fail



        csrrw   x0, CSRUT, x31
        csrrs   x0, CSRUT, x0
        csrrs   x0, CSRUT, x0
        csrrs   x6, CSRUT, x0

        bne     x6, x31, fail



        mv      x7, x31                 # random number
        mv      x8, x30                 # random number
        or      x19, x7, x8             # imitate bit set
        csrrw   x0, CSRUT, x0

        csrrs   x7, CSRUT, x7
        csrrs   x8, CSRUT, x8
        csrrs   x9, CSRUT, x9

        bne     x7, x0, fail
        bne     x8, x31, fail
        bne     x9, x19, fail








        #-------------------------------------------------------------
        # Test 4: CSRRWI
        #-------------------------------------------------------------

        li      x14, 0x0F
        li      x15, 0x10
        li      x16, 0x1F
        li      x17, 0xFFFFFFE0
        li      x18, 0xFFFFFFEF
        li      x19, 0xFFFFFFF0


test_4:
        li      TESTNUM, 4

        csrrw   x0, CSRUT, x25       # 0xFFFFFFFF

        csrrwi  x1, CSRUT, 1
        csrrwi  x2, CSRUT, 0
        csrrwi  x3, CSRUT, 0x1F
        csrrwi  x4, CSRUT, 0x0F
        csrrwi  x5, CSRUT, 0x10
        csrrwi  x6, CSRUT, 0

        bne     x1, x25, fail
        bne     x2, x21, fail
        bne     x3, x0, fail
        bne     x4, x16, fail
        bne     x5, x14, fail
        bne     x6, x15, fail




        #-------------------------------------------------------------
        # Test 5: CSRRCI
        #-------------------------------------------------------------

test_5:
        li      TESTNUM, 5

        csrrw   x0, CSRUT, x25

        csrrci  x1, CSRUT, 1
        csrrw   x1, CSRUT, x25
        csrrci  x2, CSRUT, 0
        csrrw   x2, CSRUT, x25
        csrrci  x3, CSRUT, 0x1F
        csrrw   x3, CSRUT, x25
        csrrci  x4, CSRUT, 0x10
        csrrw   x4, CSRUT, x25
        csrrci  x5, CSRUT, 0xF
        csrrw   x5, CSRUT, x25

        csrrw   x0, CSRUT, x25

        csrrci  x6, CSRUT, 1
        csrrci  x7, CSRUT, 0
        csrrci  x8, CSRUT, 0x1F
        csrrci  x9, CSRUT, 0x10
        csrrci  x10, CSRUT, 0xF
        csrrci  x11, CSRUT, 0

        bne     x1, x26, fail           # 0xFFFFFFFE
        bne     x2, x25, fail           # 0xFFFFFFFF
        bne     x3, x17, fail           # 0xFFFFFFE0
        bne     x4, x18, fail           # 0xFFFFFFEF
        bne     x5, x19, fail           # 0xFFFFFFF0

        bne     x6, x25, fail           # 0xFFFFFFFF
        bne     x7, x26, fail           # 0xFFFFFFFE
        bne     x8, x26, fail           # 0xFFFFFFFE
        bne     x9, x17, fail           # 0xFFFFFFE0
        bne     x10, x17, fail          # 0xFFFFFFE0
        bne     x11, x17, fail          # 0xFFFFFFE0





        #-------------------------------------------------------------
        # Test 6: CSRRSI
        #-------------------------------------------------------------

test_6:
        li      TESTNUM, 6

        csrrw   x0, CSRUT, x0

        csrrsi  x1, CSRUT, 1
        csrrw   x1, CSRUT, x0
        csrrsi  x2, CSRUT, 0
        csrrw   x2, CSRUT, x0
        csrrsi  x3, CSRUT, 0x1F
        csrrw   x3, CSRUT, x0
        csrrsi  x4, CSRUT, 0x10
        csrrw   x4, CSRUT, x0
        csrrsi  x5, CSRUT, 0xF
        csrrw   x5, CSRUT, x0

        csrrw   x0, CSRUT, x0

        csrrsi  x6, CSRUT, 1
        csrrsi  x7, CSRUT, 0
        csrrsi  x8, CSRUT, 0x1F
        csrrsi  x9, CSRUT, 0x10
        csrrsi  x10, CSRUT, 0xF
        csrrsi  x11, CSRUT, 0

        bne     x1, x21, fail           # 0x00000001
        bne     x2, x0, fail            # 0x00000000
        bne     x3, x16, fail           # 0x0000001F
        bne     x4, x15, fail           # 0x00000010
        bne     x5, x14, fail           # 0x0000000F

        bne     x6, x0, fail            # 0x00000000
        bne     x7, x21, fail           # 0x00000001
        bne     x8, x21, fail           # 0x00000001
        bne     x9, x16, fail           # 0x0000001F
        bne     x10, x16, fail          # 0x0000001F
        bne     x11, x16, fail          # 0x0000001F




        TEST_PASSFAIL




RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END

