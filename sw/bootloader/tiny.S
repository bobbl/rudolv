.section ".text.init"
.globl _start
_start:
        li      a6, 10          # a6 = 10 const
        li      t0, 0           # t0 = length of image
        li      t1, 0           # t1 = image target address
        jal     ra, uart_receive

        add     a0, a0, -0x30   # ra points here
        blt     a0, zero, get_image
        bge     a0, a6, get_image

        sll     a1, t0, 2       # length = 10*length + char
        add     t0, t0, a1
        add     t0, t0, t0
        add     t0, t0, a0

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
        sb      a0, 0(t1)
        add     t1, t1, 1
        bltu    t1, t0, get_image
        jr      zero

