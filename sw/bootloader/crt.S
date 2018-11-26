  .section ".text.init"
  .globl _start
_start:
        # set stack pointer
        # write to a2 first, to guarantee an atomic write to sp
        li      a2, 0xfff0
        mv      sp, a2
        jal     main
1:      j       1b


