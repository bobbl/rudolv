#include "encoding.h"

  .section ".text.init"
  .globl _start
_start:
      j _start2

.text

_start2:
        auipc   a2, 0
        la      a3, _entrypc
        sw      a2, 0(a3)

        # set stack pointer
        # write to a2 first, to guarantee an atomic write to sp
        li      a2, 0x80041ff0
        mv      sp, a2

        # clear bss
        li      a2, 0
        la      a3, _bss_begin
        la      a4, _bss_end
        beq     a3, a4, 2f 
1:      sw      a2, (a3)
        sw      a2, 4(a3)
        sw      a2, 8(a3)
        sw      a2, 12(a3)
        add     a3, a3, 16
        blt     a3, a4, 1b

        # arguments 
        # a0 = 0 = mhartid
        # a1 = 1 = number of cores
2:      jal     main


.section ".tdata.begin"
.globl _tdata_begin
_tdata_begin:

.section ".tdata.end"
.globl _tdata_end
_tdata_end:

.section ".tbss.end"
.globl _tbss_end
_tbss_end:

.section ".tohost","aw",@progbits
.align 6
.globl tohost
tohost: .dword 0
.align 6
.globl fromhost
fromhost: .dword 0

.data
.global _entrypc
_entrypc: .word 0
