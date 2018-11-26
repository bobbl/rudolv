#!/bin/sh

if [ $# -ne 2 ] 
then
    echo "Send file to Danzig booloader"
    echo "Usage: $0 <tty> <image>"
    echo "Example: $0 /dev/ttyUSB0 example.bin"
    exit
fi

stty -F "$1" 115200
#stty -F "$1" 9600
echo $(wc -c < "$2") >> "$1"
cat "$2" >> "$1"
