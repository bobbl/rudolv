#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 [install] <tool> ..."
    echo
    echo "Build the listed tools locally "
    echo "Tools after 'install' are also installed to the system"
    echo
    echo "<tool>"
    echo "  ubuntu              Install required packages with apt"
    echo "  riscv32i"
    echo "  icestorm            Always installed to /usr/local/"
    echo "  arachne-pnr"
    echo "  nextpnr"
    echo "  yosys"
    exit 1
fi

# Read configuration for external tools
#. ../config_default.sh ; [ ! -e ../config.sh ] || . ../config.sh


install=false

while [ $# -ne 0 ]
do
    case $1 in
        install)
            install=true
            ;;

        ubuntu)
            # for IceStorm
            sudo apt install build-essential clang bison flex libreadline-dev \
                gawk tcl-dev libffi-dev git mercurial graphviz xdot pkg-config \
                python python3 libftdi-dev qt5-default python3-dev \
                libboost-all-dev cmake libeigen3-dev
            ;;

        riscv32i)
            git clone --recursive https://github.com/riscv/riscv-gnu-toolchain
            if [ $install = true ]
            then
                destdir=/opt/riscv32i
                sudo mkdir $destdir
                sudo chown $USER $destdir
            else
                destdir=$(pwd)/riscv32i
            fi
            cd riscv-gnu-toolchain
            mkdir build
            cd build
            ../configure --prefix=$destdir --with-arch=rv32i
            make -j$(nproc)
            echo "Add the following lines to ../config.sh:"
            echo
            echo "    RV32I_PREFIX=$destdir/riscv32-unknown-elf-"
            echo "    export RV32I_PREFIX"
            cd ../..
            ;;

        icestorm) # must be istalled to /usr/local
            git clone https://github.com/YosysHQ/icestorm.git
            cd icestorm
            make -j$(nproc)
            sudo make install
            cd ..
            ;;

        arachne-pnr)
            git clone https://github.com/cseed/arachne-pnr.git
            cd arachne-pnr
            make -j$(nproc)
            if [ $install = true ]
            then
                sudo make install
            else
                echo "Add the following lines to ../config.sh:"
                echo
                echo "    ARACHNE=$(pwd)/arachne-pnr/bin/arachne-pnr"
                echo "    export ARACHNE"
            fi
            cd ..
            ;;

        nextpnr)
            git clone https://github.com/YosysHQ/nextpnr nextpnr
            cd nextpnr
            cmake -DARCH=ice40 -DCMAKE_INSTALL_PREFIX=/usr/local .
            make -j$(nproc)
            if [ $install = true ]
            then
                sudo make install
            else
                echo "Add the following lines to ../config.sh:"
                echo
                echo "    NEXTPNR=$(pwd)/nextpnr-ice40"
                echo "    export NEXTPNR"
            fi
            cd ..
            ;;


        *)
            echo "Unknown tool $1. Stop."
            exit 3
            ;;
    esac
    shift
done


# SPDX-License-Identifier: ISC

