#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <device> <command> ..."
    echo
    echo "<device>"
    echo "  genesys2    Digilent Genesys 2 board"
    echo
    echo "<command>"
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


target_bootloader() {
    back=$(pwd)
    cd $(dirname $0)/../../sw/uart
    ./make.sh all
    cd $back
    sed -e 's/@0 /@3f80 /' ../../sw/uart/build/bootloader_char.hex \
        > bootloader_3f80.hex
}


report() {
    printf "\033[33m"
    awk '/1. Slice Logic/,/1.1 Summary of Registers by Type/ { \
          if ($2=="Slice") { \
            if ($3=="LUTs") {lut=$5} \
            if ($3=="Registers") {ff=$5} \
          } \
        } \
        /^\| Slice   / {slice=$4} \
        /^Slack \(MET\)/ {slack=$4} \
        /  Requirement:  / {period=$2} \
        END { printf("SYNTHESIS RESULTS: %d slices, %d LUTs, %d FFs, %s clock period, %s slack\n", \
                     slice, lut, ff, period, slack)}' \
        < vivado.log
    printf "\033[0m"
}


while [ $# -ne 0 ]
do
    case $1 in
        report)
            report
            ;;
        synth)
            target_bootloader
            $VIVADO -nojournal -mode batch -source genesys2_synth.tcl
            report
            ;;
        prog)
            $VIVADO -nojournal -mode batch -source prog.tcl
            ;;
        *)
            echo "Unknown command $1. Stop."
            exit 3
            ;;
    esac
    shift
done


# SPDX-License-Identifier: ISC
