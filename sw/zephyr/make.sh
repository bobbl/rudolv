#!/bin/sh

BOARD=m2gl025_miv
BUILDDIR=build


target_zephyr() {
    pip3 install --user west
    west init zephyrproject --mr v1.14.1-rc1
    cd zephyrproject
    west update
    pip3 install --user -r zephyr/scripts/requirements.txt
    export ZEPHYR_BASE=$(pwd)/zephyr
    cd ..
}


# $1 app name = subdir
west_build() {
    mkdir -p $BUILDDIR
    cd apps/$1

    rm -rf build

#    export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
    export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
#    export ZEPHYR_BASE=/home/bobbl/git/github/contest2/zephyrproject/zephyr
    export ZEPHYR_BASE=${zephyr_base}

    west build -p -b m2gl025_miv
    cp build/zephyr/zephyr.elf ../../build/$1.unp_rv32imc.elf

    cd ../..
}





# ---------------------------------------------
# main loop: process one target after the other
# ---------------------------------------------

if [ $# -eq 0 ] 
then
    echo "No make target given. Supported targets:"
    echo "    zephyr  clone, patch and build Zephyr RTOS 1.14.1"
    exit 1
fi

apps=$(cd apps; echo * | sed 's/SOURCE//')
zephyr_base=$(pwd)/zephyrproject/zephyr

while [ $# -ne 0 ]
do
    case $1 in
        ripe)                   west_build $1 ;;
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

