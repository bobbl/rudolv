#!/bin/sh

if [ $# -ne 2 ] 
then
    echo "Extract binary image from an ELF file and send it to the RudolV bootloader"
    echo "Usage: $0 <tty> <image.elf>"
    echo "Example: $0 /dev/ttyUSB0 example.elf"
    exit
fi

. ../../config_default.sh
[ ! -e ../../config.sh ] || . ../../config.sh

bin=send_elf.tmp

${RV_PREFIX}objcopy -O binary "$2" $bin
size=$(wc -c < $bin)
echo "Sending $size bytes to UART"

stty -F "$1" 115200
echo $size >> "$1"
cat $bin >> "$1"
rm $bin
