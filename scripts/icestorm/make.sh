#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <device> <target> ..."
    echo
    echo "<device>"
    echo "  hx8k        Lattice HX8K (e.g. breakout board)"
    echo "  uwg30       Lattice UP5K package UWG30 (e.g. iCE40 UltraPlus MDP)"
    echo "  sg48        Lattice UP5K package SG48"
    echo
    echo "<target>"
    echo "  bootloader  Build bootloader"
    echo "  synth       Synthesize with yosys"
    echo "  miv-synth   Synthesize Mi-V compatible core (only UP5K)"
    echo "  pnr         Place and route wirh nextpnr"
    echo "  prog        Program the device with iceprog"
    exit 1
fi

# Read configuration for external tools
. ../../config_default.sh ; [ ! -e ../../config.sh ] || . ../../config.sh

verilog_files="../../src/regset33.v ../../src/csr.v ../../pipeline.v"



device=$1
case $1 in
    hx8k)       chip="hx8k" ; args="--hx8k --package ct256" ;;
    uwg30)      chip="up5k" ; args="--up5k --package uwg30" ;;
    sg48)       chip="up5k" ; args="--up5k --package sg48" ;;
    *)          echo "Unknown device $1. Stop."
                exit 2
                ;;
esac
shift



while [ $# -ne 0 ]
do
    case $1 in
        bootloader)
            back=$(pwd)
            cd $(dirname $0)/../../sw/uart
            ./make.sh all
            cd $back
            sed -e 's/@0 /@780 /' ../../sw/uart/build/bootloader_char.hex \
                > hx8k_bootloader.hex
            ;;
        synth)
            $YOSYS -ql tmp.log -p 'synth_ice40 -top top -json tmp.json' \
                $verilog_files $chip.v
            ;;
        miv-synth)
            $YOSYS -ql tmp.log -p 'synth_ice40 -top top -json tmp.json' \
                $verilog_files ${chip}_miv.v ../../src/memory.v
            ;;
        pnr)
            $NEXTPNR_ICE40 $args --json tmp.json --pcf $chip.pcf --asc \
                tmp.asc
            $ICEPACK tmp.asc $device.bin
            ;;
        prog)
            iceprog $device.bin
            ;;
        *)
            echo "Unknown target $1. Stop."
            exit 2
            ;;
    esac
    shift
done

