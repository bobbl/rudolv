OUTPUT_ARCH("riscv")
ENTRY(_start)
SECTIONS
{
    . = 0x00000000;
    .text.crt : {
        *(.text.crt)
    }
    .text : {
        *(.text)
      }

    . = ALIGN(0x1000);
        /* .data must start at 4K or higher, because otherwise `la` might be
           optimised by the assembler to a single `li` instead of a sequence of
           `auipc` and `addi`. That would change the offsets and the checks for
           MEPC and MTVAL fail. */

    .data : {
        *(.data)
        *(.bss) 
    }
}

