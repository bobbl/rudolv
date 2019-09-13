#!/bin/sh

BOARD=m2gl025_miv
BUILDDIR=build
RV32IM_PREFIX="${ZEPHYR_SDK_INSTALL_DIR}/riscv32-zephyr-elf/bin/riscv32-zephyr-elf-"


# convert $1.elf to $1.bin and $1.hex
# $1 filename
elf_to_hex() {
    ${RV32IM_PREFIX}objcopy -O binary $1.elf $1.bin
    printf "@0 " > $1.hex
    od -An -tx4 -w4 -v $1.bin | cut -b2- >> $1.hex
}



target_zephyr() {
    pip3 install --user west
    west init zephyrproject --mr v1.14.1-rc1
    cd zephyrproject
    west update

    # patch: hub was renamed to git-spindle
    sed -i 's/hub==/git-spindle==/' zephyr/scripts/requirements.txt 

    pip3 install --user -r zephyr/scripts/requirements.txt
    export ZEPHYR_BASE=$(pwd)/zephyr
    export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
    cd ..
}

# $1 attack number 1..5
target_ripe() {
    SRCFILE=src/ripe_attack_generator.c
    mkdir -p $BUILDDIR
    cd apps/ripe

    # set attack number
    sed -i.bak "s/^#define ATTACK_NR   .*/#define ATTACK_NR   $1/" ${SRCFILE}

    rm -rf build
    west build -p -b m2gl025_miv
    cp build/zephyr/zephyr.bin ../../$BUILDDIR/ripe$1.rv32im.bin
    mv ${SRCFILE}.bak ${SRCFILE}
    cd ../..
}

# $1 app name = subdir
west_build() {
    mkdir -p $BUILDDIR
    cd apps/$1
    rm -rf build

    west build -p -b m2gl025_miv
    cp build/zephyr/zephyr.elf ../../build/$1.rv32im.elf
    cd ../..

    elf_to_hex $BUILDDIR/$1.rv32im
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
        test)                   west_build $1 ;;
        hello_world)            west_build $1 ;;
        synchronization)        west_build $1 ;;
        philosophers)           west_build $1 ;;
        zephyr)                 target_zephyr ;;
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

