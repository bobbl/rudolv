#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <targets> ..."
    echo "  riscv      build Dhrystone benchmark implementation from riscv.org"
    echo "  picorv32   build Dhrystone benchmark implementation from PicoRV32"
    echo "  scr1       build Dhrystone benchmark implementation from Syntacore"
    echo "  all        build all variants"
    echo
    echo "The .elf images are placed in build/"
    exit 1
fi

# Read configuration for external tools
. ../../config_default.sh ; [ ! -e ../../config.sh ] || . ../../config.sh



# ---------------------------------------------
# targets
# ---------------------------------------------


# build dhryrstone in the subdir and copy .elf file
# $1 subdir
build_and_copy() 
{
    mkdir -p build
    make -C $1 clean all
    cp $1/dhrystone.elf build/$1.elf
}


target_scr1()
{
    mkdir -p build
    make CROSS_PREFIX="$RV32I_PREFIX" ITERATIONS=50 OPT=3lto -C scr1 clean all
    cp scr1/dhry21-o3lto.elf build/scr1.elf
}


# ---------------------------------------------
# main loop: process one target after the other
# ---------------------------------------------

while [ $# -ne 0 ]
do
    case $1 in
        riscv)
            build_and_copy riscv
            ;;
        picorv32)
            build_and_copy picorv32
            ;;
        scr1)
            target_scr1
            ;;
        all)
            build_and_copy riscv
            build_and_copy picorv32
            target_scr1
            ;;
        clean)
            rm -rf build
            make -C riscv clean
            make -C picorv32 clean
            make -C scr1 clean
            ;;
        *)
            echo "Unknown target $1. Stop."
            exit 2
            ;;
    esac
    shift
done
