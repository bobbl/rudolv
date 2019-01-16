  .section ".text.init"
  .globl _start
_start:
        # set stack pointer
        auipc   a0, 0
        add     sp, a0, 0x200
        jal     main
1:      j       1b


