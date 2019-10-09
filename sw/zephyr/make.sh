#!/bin/sh

BUILDDIR=build



target_zephyr() {
    pip3 install --user west
    west init zephyrproject --mr v1.14.1-rc1
    cd zephyrproject
    west update

    # patch: hub was renamed to git-spindle
    sed -i 's/hub==/git-spindle==/' zephyr/scripts/requirements.txt 

    pip3 install --user -r zephyr/scripts/requirements.txt
    ZEPHYR_TOOLCHAIN_VARIANT=zephyr
    export ZEPHYR_TOOLCHAIN_VARIANT
    ZEPHYR_BASE=$(pwd)/zephyr
    export ZEPHYR_BASE
    cd ..
}

# $1 attack number 1..5
target_ripe() {
    srcfile=src/ripe_attack_generator.c
    mkdir -p $BUILDDIR
    cd apps/ripe

    # set attack number
    sed -i.bak "s/^#define ATTACK_NR   .*/#define ATTACK_NR   $1/" ${srcfile}

    rm -rf build
    west build -p -b m2gl025_miv
    cp build/zephyr/zephyr.elf ../../$BUILDDIR/ripe$1.elf
    mv ${srcfile}.bak ${srcfile}
    cd ../..
}

# $1 app name = subdir
west_build() {
    mkdir -p $BUILDDIR
    cd apps/$1
    rm -rf build

    west build -p -b m2gl025_miv
    cp build/zephyr/zephyr.elf ../../build/$1.elf
    cd ../..
}





# ---------------------------------------------
# main loop: process one target after the other
# ---------------------------------------------

if [ $# -eq 0 ] 
then
    echo "Please give a make target:"
    echo "  zephyr   clone, patch and build Zephyr RTOS 1.14.1-rc1"
    echo "  ripe     build all 5 ripe attack scenarios for the RISC-V IoT contest"
    exit 1
fi

if [ -z "$ZEPHYR_SDK_INSTALL_DIR" ]
then
    echo "ZEPHY_SDK_INSTALL_DIR must be set"
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



apps=$(cd apps; echo * | sed 's/SOURCE//')

while [ $# -ne 0 ]
do
    case $1 in
        ripe)
            target_ripe 1
            target_ripe 2
            target_ripe 3
            target_ripe 4
            target_ripe 5
            ;;
        test)            west_build $1 ;;
        hello_world)     west_build $1 ;;
        synchronization) west_build $1 ;;
        philosophers)    west_build $1 ;;
        zephyr)          target_zephyr ;;
        all)
#            target_zephyr
            for a in $apps ; do west_build $a ; done
            ;;
        clean)
            for p in $apps ; do rm -rf $a/build ; done
            ;;
        *)
            echo "Unknown target $1. Stop."
            exit 2
            ;;
    esac
    shift
done

