.section ".text.init"
.globl _start
_start:
        li      s0, 0           # s0 = length of image
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
# clobber t2
uart_receive:
        lb      a0, 16(a5)
        and     a0, a0, 2
        beqz    a0, uart_receive
        lb      a0, 4(a5)
        jr      ra

get_image:
        lui     s1, 0x80000     # s1 = image target address
        add     a1, s1, s0
1:
        jal     ra, uart_receive
        sb      a0, 0(s1)
        add     s1, s1, 1
        bltu    s1, a1, 1b

        lui     s1, 0x80000
        add     a1, s1, s0
2:      lw      a0, 0(s1)
        sw      a0, 0(s1)
        add     s1, s1, 4
        bltu    s1, a1, 2b

        lui     ra, 0x80000
        jr      ra

