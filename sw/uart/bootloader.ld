OUTPUT_ARCH( "riscv" )
ENTRY(_start)

SECTIONS
{
    . = 0;
    .text.init : { *(.text.init) }
    .text : { *(.text) }
    .data : { *(.data) }
}

