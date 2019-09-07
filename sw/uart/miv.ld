OUTPUT_ARCH( "riscv" )
ENTRY(_start)

SECTIONS
{
    . = 0x80000000;
    .text.init : { *(.text.init) }
    .text : { *(.text) }
    .data : { *(.data) }

    . = ALIGN(16);
_bss_begin = .;
    .sbss : {
        *(.sbss .sbss.* .gnu.linkonce.sb.*)
        *(.scommon)
     }
    .bss : { *(.bss) }
    . = ALIGN(16);
_bss_end = .;
_end = .;
}

