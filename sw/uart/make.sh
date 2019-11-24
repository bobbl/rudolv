#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 all"
    echo
    echo "Build 4 firmware images for 3 uart interfaces"
    echo
    echo "firmware"
    echo "  bootloader  large bootloader with prompt and LED output"
    echo "  bl          minimal bootloader"
    echo "  grubby      slightly larger bootloader with sw instead of sb"
    echo "  test        UART test program to be used instead of bootloader"
    echo
    echo "interface"
    echo "  bitbang     directly control TX and RX pins by software"
    echo "  char        RudolV's CSR mapped character interface"
    echo "  miv         Mi-V compatible memory mapped character interface"
    exit 1
fi

if [ $# -ne 1 ] || [ "$1" != "all" ]
then
    echo "Only target all supported"
fi 

# Read configuration for external tools
. ../../config_default.sh ; [ ! -e ../../config.sh ] || . ../../config.sh



# Build all programs for a single UART type
#   $1 macro name
#   $2 executable name suffix
build() {
    ${RV_PREFIX}gcc -O2 -fPIC -march=rv32i -mabi=ilp32 -I. -nostartfiles \
        -Tbootloader.ld -D$1 crt.S \
        bootloader.c -o build/bootloader_$2.elf
    ${RV_PREFIX}gcc -O2 -fPIC -march=rv32i -mabi=ilp32 -I. -nostartfiles \
        -Tbootloader.ld -D$1 crt.S \
        test.c -o build/test_$2.elf
    ${RV_PREFIX}gcc -nostartfiles -Tbootloader.ld -D$1 \
        bl.S -o build/bl_$2.elf
    ${RV_PREFIX}gcc -nostartfiles -Tbootloader.ld -D$1 \
        grubby.S -o build/grubby_$2.elf
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

    ${RV32I_PREFIX}objcopy -O binary $elf $bin
    printf "@0 " > $hex
    od -An -tx4 -w4 -v $bin | cut -b2- >> $hex
done
