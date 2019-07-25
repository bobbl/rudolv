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
    .data : {
        *(.data)
        *(.bss) 
    }
    . = 0x00007f00;
    .signature : { 
        *(.signature) 
    }
}

