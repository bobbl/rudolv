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


    # the recommended patch has no effect, therefore use unpatched zephyr
    export ZEPHYR_BASE=${zephyr_base}


    # When building for rv32i, the SDK gcc cannot be used, because its libgcc
    # contains rv32ima instructions and is linked to the executable. Therefore
    # use special cross compiler.
    export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
    west build -p -b m2gl025_miv
    cp build/zephyr/zephyr.elf ../../build/$1.a.elf

    # For rv32ima the zephyr SDK can be used
    # To change the ISA, CONFIG_COMPILER_OPT in prj.conf must be modified, too.
#    export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
#    west build -p -b m2gl025_miv
#    cp build/zephyr/zephyr.elf ../../build/$1.b.elf

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
        ripe)
            west_build ripe1
            west_build ripe2
            west_build ripe3
            west_build ripe4
            west_build ripe5
            ;;
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

