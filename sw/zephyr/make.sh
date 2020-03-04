#!/bin/sh

BUILDDIR=build



target_zephyr() {
    #pip3 install --user west
    west init zephyrproject --mr v2.2.0-rc3
    cd zephyrproject
    west update

    pip3 install --user -r zephyr/scripts/requirements.txt
    ZEPHYR_TOOLCHAIN_VARIANT=zephyr
    export ZEPHYR_TOOLCHAIN_VARIANT
    ZEPHYR_BASE=$(pwd)/zephyr
    export ZEPHYR_BASE
    cd ..
}

# $1 path to app directory
west_build() {
    west build -p -b rudolv $1 -- -DBOARD_ROOT=$(pwd)

    mkdir -p elf
    cp build/zephyr/zephyr.elf elf/$(basename $1).elf
}





# ---------------------------------------------
# check dependencies
# ---------------------------------------------

if [ $# -eq 0 ] 
then
    echo "Please give a make target:"
    echo "  zephyr     clone, patch and build Zephyr RTOS 2.2.0-rc3"
    echo "  elf <dir>  build Zephyr application in <dir> and copy ELF to ./elf/"
#    echo "  ripe     build all 5 ripe attack scenarios for the RISC-V IoT contest"
    exit 1
fi

if [ -z "$ZEPHYR_SDK_INSTALL_DIR" ]
then
    echo "ZEPHYR_SDK_INSTALL_DIR must be set"
    exit 2
fi

if [ -z "$ZEPHYR_TOOLCHAIN_VARIANT" ]
then
    if [ -d zephyrproject/zephr ]
    then
        echo "Zephyr not found. Set ZEPHYR_BASE or install with $0 zephyr"
        exit 3
    fi
    ZEPHYR_TOOLCHAIN_VARIANT=zephyr
    export ZEPHYR_TOOLCHAIN_VARIANT
    ZEPHYR_BASE=$(pwd)/zephyrproject/zephyr
    export ZEPHYR_BASE
fi



# ---------------------------------------------
# main loop: process one target after the other
# ---------------------------------------------

while [ $# -ne 0 ]
do
    case $1 in
        zephyr)         target_zephyr ;;
        elf)
            west_build $2
            shift
            ;;
        clean)
            rm -rf elf
            rm -rf build
            ;;
        *)
            echo "Unknown target $1. Stop."
            exit 2
            ;;
    esac
    shift
done
