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
    echo "  synth       Synthesize using yosys"
    echo "  gy-synth    Synthesize with gruppy detection"
    echo "  pnr         Place and route with nextpnr"
    echo "  prog        Program the device with iceprog"
    exit 1
fi

# Read configuration for external tools
. ../../config_default.sh ; [ ! -e ../../config.sh ] || . ../../config.sh

verilog_files="../../src/regset.v ../../src/memory.v ../../src/csr.v ../../pipeline.v"




report_yosys() {
    printf "\033[33m"
    awk '/^     SB_CARRY / {carry=$2} \
         /^     SB_DFF/    {dff+=$2} \
         /^     SB_LUT4 /  {lut=$2} \
        END { printf("SYNTHESIS RESULTS: %d LUTs, %d DFFs, %s carries\n", \
                     lut, dff, carry)}' \
        < tmp.yosys.log
    printf "\033[0m"
}


report_nextpnr() {
    printf "\033[33m"
    awk '/ LCs used as LUT4 only$/    {lut=$2} \
         / LCs used as LUT4 and DFF$/ {lutdff=$2} \
         / LCs used as DFF only$/     {dff=$2} \
         / LCs used as CARRY only$/   {carry=$2} \
         / ICESTORM_LC: /             {lc=substr($3, 1, length($3)-1)} \
         /^Info: Max frequency for clock / {mhz=$7} # last is best\
        END { printf("SYNTHESIS RESULTS: %s LCs, %d LUTs, %d DFFs, %d carries, %s MHz\n", \
                     lc, lut+lutdff, lutdff+dff, carry, mhz)}' \
        < tmp.nextpnr.log
    printf "\033[0m"
}





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
            $YOSYS -ql tmp.yosys.log -p 'synth_ice40 -top top -json tmp.json' \
                $verilog_files $chip.v
            report_yosys
            ;;
        synth2)
            $YOSYS -ql tmp.yosys.log -p 'synth_ice40 -abc2 -top top -json tmp.json' \
                $verilog_files $chip.v
            report_yosys
            ;;
        pnr)
            $NEXTPNR_ICE40 $args -ql tmp.nextpnr.log --json tmp.json \
                --pcf $chip.pcf --asc tmp.asc --placer sa
            $ICEPACK tmp.asc $device.bin
            report_nextpnr
            ;;
        prog)
            iceprog $device.bin
            ;;
        *)
            echo "Unknown target $1. Stop."
            exit 3
            ;;
    esac
    shift
done


# SPDX-License-Identifier: ISC

