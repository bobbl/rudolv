#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <targets> ..."
    echo "  synth   Synthesize for Genesys-2"
    echo "  prog    Program Genesys-2"
    echo "          Set VIVADO_HW_TARGET in config.sh for reliable programming"
    echo '          e.g. VIVADO_HW_TARGET="*/xilinx_tcf/Digilent/100200300400Z"'
    exit 1
fi

. ../../config_default.sh
[ ! -e ../../config.sh ] || . ../../config.sh

while [ $# -ne 0 ]
do
    case $1 in
        synth)
            # CAUTION: offset is a word (4 byte) address, not a byte address
            sed -e 's/@0 /@3f80 /' ../../sw/uart/build/bootloader_char.hex > bootloader.hex

            $VIVADO -nojournal -mode batch -source genesys2_synth.tcl
            ;;
        prog)
            $VIVADO -nojournal -mode batch -source prog.tcl
            ;;
        *)
            echo "Unknown target $1. Stop."
            exit 2
            ;;
    esac
    shift
done
