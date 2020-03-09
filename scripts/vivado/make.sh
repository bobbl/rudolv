#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <device> <target> ..."
    echo
    echo "<device>"
    echo "  genesys2    Digilent Genesys 2 board"
    echo
    echo "<target>"
    echo "  synth       Synthesize with Vivado"
    echo "  prog        Program the device"
    echo "              Set VIVADO_HW_TARGET in config.sh for reliable programming"
    echo '              e.g. VIVADO_HW_TARGET="*/xilinx_tcf/Digilent/100200300400Z"'
    exit 1
fi

# Read configuration for external tools
. ../../config_default.sh ; [ ! -e ../../config.sh ] || . ../../config.sh



device=$1
case $1 in
    genesys2)    ;;
    *)          echo "Currently only the genesys2 board is supported."
                echo "Unknown device $1. Stop."
                exit 2
                ;;
esac
shift



while [ $# -ne 0 ]
do
    case $1 in
        synth)
            # CAUTION: offset is a word (4 byte) address, not a byte address
#            sed -e 's/@0 /@3f80 /' ../../sw/uart/build/bootloader_char.hex > bootloader.hex

            od -An -tx1 -v -w4 ../../sw/uart/build/bootloader_char.bin > tmp.bytes.hex
            for i in 0 1 2 3
            do
                printf "@3f80 " > bootloader.byte$i.hex
                cut -d' ' -f$((i + 2)) tmp.bytes.hex >> bootloader.byte$i.hex
            done

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
