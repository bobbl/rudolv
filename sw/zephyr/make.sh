#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <targets> ..."
    echo "  zephyr     clone and build Zephyr RTOS 2.2.0"
    echo "  elf <dir>  build Zephyr application in <dir> and copy ELF to ./elf/"
    echo "  ripe       build all 5 ripe attack scenarios for the RISC-V IoT contest"
    echo
    echo "<dir>"
    echo "  zephyrproject/zephyr/samples/hello_world"
    echo "  zephyrproject/zephyr/samples/synchronization"
    echo "  zephyrproject/zephyr/samples/philosophers"
    echo "  apps/ripe"
    exit 1
fi

# Read configuration for external tools
. ../../config_default.sh ; [ ! -e ../../config.sh ] || . ../../config.sh



# ---------------------------------------------
# targets
# ---------------------------------------------

target_zephyr() {
    #pip3 install --user west
    west init zephyrproject --mr v2.2.0
    cd zephyrproject
    west update

    pip3 install --user -r zephyr/scripts/requirements.txt
    ZEPHYR_TOOLCHAIN_VARIANT=zephyr
    export ZEPHYR_TOOLCHAIN_VARIANT
    ZEPHYR_BASE=$(pwd)/zephyr
    export ZEPHYR_BASE
    cd ..
}

# $1 attack number 1...5
target_ripe() {
    srcfile=apps/ripe/src/ripe_attack_generator.c

    # set attack number
    sed -i.bak "s/^#define ATTACK_NR   .*/#define ATTACK_NR   $1/" ${srcfile}

    west build -p -b rudolv apps/ripe/ -- -DBOARD_ROOT=$(pwd)
    cp build/zephyr/zephyr.elf elf/ripe$1.elf
    mv ${srcfile}.bak ${srcfile}
}

# $1 path to app directory
target_elf() {
    west build -p -b rudolv $1 -- -DBOARD_ROOT=$(pwd)

    mkdir -p elf
    cp build/zephyr/zephyr.elf elf/$(basename $1).elf
}



# ---------------------------------------------
# check dependencies
# ---------------------------------------------

if [ -z "$ZEPHYR_SDK_INSTALL_DIR" ]
then
    echo "ZEPHYR_SDK_INSTALL_DIR must be set"
    exit 2
fi

if [ -z "$ZEPHYR_TOOLCHAIN_VARIANT" ]
then
    if [ -d zephyrproject/zephyr ]
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
        zephyr)
            target_zephyr
            ;;
        elf)
            target_elf $2
            shift
            ;;
        ripe)
            target_ripe 1
            target_ripe 2
            target_ripe 3
            target_ripe 4
            target_ripe 5
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
