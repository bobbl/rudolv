.section ".text.init"
.globl _start
_start:



#ifdef UART_BITBANG
# ----------------------------------------------------------------------

        li      s0, 0           # s0 = length of image
        li      s1, 0           # s1 = image target address
        li      a6, 10          # a6 = const 10
        jal     ra, uart_receive

        add     a0, a0, -0x30   # ra points here
        blt     a0, zero, get_image
        bge     a0, a6, get_image

        sll     a1, s0, 2       # length = 10*length + char
        add     s0, s0, a1
        add     s0, s0, s0
        add     s0, s0, a0

        # dirty fallthrough: ra still points to the beginning of this loop

# return char in a0
# clobber a1-a6
uart_receive:
        csrr    a1, 0x7c0
        srl     a1, a1, 1       # a1 = UART period
1:      csrr    a2, 0x7c0       # a2 = UART RX
        and     a2, a2, 1       # wait for low
        bne     a2, zero, 1b

        rdcycle a3
        srl     a4, a1, 1
        sub     a3, a3, a4      # a3 = timestamp - period/2

        li      a2, 0           # a2 = bit position
        li      a4, 0           # a4 = bit pattern
bit_loop:
        csrr    a5, 0x7c0
        and     a5, a5, 1
        sll     a5, a5, a2
        or      a4, a4, a5

2:      rdcycle a5              # while (cycle-timestamp < period)
        sub     a5, a5, a3
        blt     a5, a1, 2b

        add     a3, a3, a1      # timestamp += period
        add     a2, a2, 1
        blt     a2, a6, bit_loop

        srl     a0, a4, 2
        and     a0, a0, 0xff
        jr      ra

get_image:
        jal     ra, uart_receive
        sb      a0, 0(s1)
        add     s1, s1, 1
        bltu    s1, s0, get_image
        jr      zero




#endif
#ifdef UART_CHAR
# ----------------------------------------------------------------------

        li      s0, 0           # s0 = length of image
        li      s1, 0           # s1 = image target address
        li      a4, 10          # a4 = const 10
        li      a5, 0x100       # a5 = mask for UART receive state
        jal     ra, uart_receive

        add     a0, a0, -0x30   # ra points here
        blt     a0, zero, get_image
        bge     a0, a4, get_image

        sll     a1, s0, 2       # length = 10*length + char
        add     s0, s0, a1
        add     s0, s0, s0
        add     s0, s0, a0

        # dirty fallthrough: ra still points to the beginning of this loop

# return char in a0
uart_receive:
        csrrs   a0, 0xbc0, a5
        and     a1, a0, a5
        bnez    a1, uart_receive
        and     a0, a0, 0xff
        jr      ra

get_image:
        jal     ra, uart_receive
        sb      a0, 0(s1)
        add     s1, s1, 1
        bltu    s1, s0, get_image
        jr      zero




#endif
#ifdef UART_MIV
# ----------------------------------------------------------------------

        li      s0, 0           # s0 = length of image
        lui     s1, 0x80000     # s1 = image target address
        li      a4, 10          # a4 = const 10
        lui     a5, 0x70000     # a5 = const address of UART
        jal     ra, uart_receive

        add     a0, a0, -0x30   # ra points here
        blt     a0, zero, get_image
        bge     a0, a4, get_image

        sll     a1, s0, 2       # length = 10*length + char
        add     s0, s0, a1
        add     s0, s0, s0
        add     s0, s0, a0

        # dirty fallthrough: ra still points to the beginning of this loop

# return char in a0
uart_receive:
        lb      a0, 16(a5)
        and     a0, a0, 2
        beqz    a0, uart_receive
        lb      a0, 4(a5)
        jr      ra

get_image:
        add     s0, s0, s1
1:
        jal     ra, uart_receive
        sb      a0, 0(s1)
        add     s1, s1, 1
        bltu    s1, s0, 1b

        lui     ra, 0x80000
        jr      ra

#endif
