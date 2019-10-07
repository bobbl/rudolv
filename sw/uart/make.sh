#!/bin/sh

cat << EOF
Building 
    bootloader  large bootloader with prompt and LED output
    bl          minimal bootloader
    test        UART test program to be used as firmware instead of bootloader
for different UART interfaces
    bitbang     directly control TX and RX pins by software
    char        RudolV's CSR mapped character interface
    miv         Mi-V compatible memory mapped character interface
EOF

rv32i_prefix=riscv32-unknown-elf-



# Build all programs for a single UART type
#   $1 macro name
#   $2 executable name suffix
build() {
    ${rv32i_prefix}gcc -O2 -fPIC -march=rv32i -mabi=ilp32 -I. -nostartfiles \
        -Tbootloader.ld -D$1 crt.S \
        bootloader.c -o build/bootloader_$2.elf
    ${rv32i_prefix}gcc -O2 -fPIC -march=rv32i -mabi=ilp32 -I. -nostartfiles \
        -Tbootloader.ld -D$1 crt.S \
        test.c -o build/test_$2.elf
    ${rv32i_prefix}gcc -nostartfiles -Tbootloader.ld -D$1 \
        bl.S -o build/bl_$2.elf
}



mkdir -p build
build UART_BITBANG bitbang
build UART_CHAR char
build UART_MIV miv

# build .bin and .hex images
for elf in build/*.elf
do
    bin=$(echo $elf | sed 's/elf$/bin/')
    hex=$(echo $elf | sed 's/elf$/hex/')

    ${rv32i_prefix}objcopy -O binary $elf $bin
    printf "@0 " > $hex
    od -An -tx4 -w4 -v $bin | cut -b2- >> $hex
done
