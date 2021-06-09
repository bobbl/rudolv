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
    echo "  rv32i"
    echo "  rv32im"
    echo "  rv32imc"
    echo "  icestorm            Always installed to /usr/local/"
    echo "  arachne-pnr"
    echo "  nextpnr"
    echo "  yosys"
    exit 1
fi

# Read configuration for external tools
#. ../config_default.sh ; [ ! -e ../config.sh ] || . ../config.sh



# Build GNU toolchain for specific ISA
# $1 ISA (rv32i, rv32im, ...
riscv_gnu() {
    isa=$1
    destdir="$HOME/.local/share/riscv/$isa"

    # dependencies
    sudo apt-get install autoconf automake autotools-dev curl libmpc-dev \
        libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo \
        gperf libtool patchutils bc zlib1g-dev git libexpat1-dev

    mkdir -p "$destdir"

    if [ ! -d riscv-gnu-toolchain/.git/ ]
    then
        git clone --recursive https://github.com/riscv/riscv-gnu-toolchain
        cd riscv-gnu-toolchain
    else
        cd riscv-gnu-toolchain
        git pull
        git submodule update --init --recursive
    fi

    git checkout master
    #git checkout 411d134

    mkdir build
    cd build
    ../configure --with-arch=$isa --prefix="$destdir"
    make -j$(nproc)

    upper=$(echo $isa | tr [a-z] [A-Z])
    echo "Finished."
    echo
    echo "Add the following lines to ../config.sh:"
    echo
    echo "    ${upper}_PREFIX=$destdir/bin/riscv32-unknown-elf-"
    echo "    export ${upper}_PREFIX"
    cd ../..
}







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

        rv32i|rv32im|rv32imc)
            riscv_gnu $1
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

