#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <target> ..."
    echo
    echo "  clone      Clone coremark source code from github"
    echo "  mhz <MHz>  Build image to measure total CoreMark of an FPGA implementation"
    echo "             (Lattice: 24 MHz, Genesys: 200 Mhz, DE2-115: 50 MHz, ...)"
    echo "  elf <Iter> Build image to measure CoreMark / MHz on an FPGA"
    echo "             CoreMark should run at least 10 seconds. Therefore <Iter>"
    echo "             should be larger than 10 * CoreMark/MHz * MHz"
    echo "  icarus     Simulate 1 iteration with Icarus verilog"
    exit 1
fi

# Read configuration for external tools
. ../../config_default.sh ; [ ! -e ../../config.sh ] || . ../../config.sh



clone() {
    git clone https://github.com/eembc/coremark.git
    cp -r portme/* coremark/
}



while [ $# -ne 0 ]
do
    case $1 in
        clone)
            clone
            ;;

        mhz)
            if [ $# -lt 2 ] ; then echo "MHz expected" ; exit 2 ; fi
            if [ ! -d coremark/.git/ ] ; then clone ; fi
            mkdir -p build

            mhz=$2
            iter=$(( $mhz * 30 ))
            # 10 seconds with up to 3.0 CoreMark per MHz
            echo "#define CYCLES_PER_SEC ${mhz}000000" > coremark/rudolv/cycles_per_sec.h

            make -C coremark ITERATIONS=$iter PORT_DIR=rudolv clean link
            cp coremark/coremark.elf build/coremark_mhz_${mhz}.elf
            shift
            ;;

        elf)
            if [ $# -lt 2 ] ; then echo "Iterations expected" ; exit 3 ; fi
            if [ ! -d coremark/.git/ ] ; then clone ; fi
            mkdir -p build

            echo "#define CYCLES_PER_SEC 1000000" > coremark/rudolv/cycles_per_sec.h

            make -C coremark ITERATIONS=$2 PORT_DIR=rudolv clean link
            cp coremark/coremark.elf build/coremark_iter_$2.elf
            shift
            ;;

        icarus)
            if [ ! -d coremark/.git/ ] ; then clone ; fi
            mkdir -p build

            echo "#define CYCLES_PER_SEC 1000000" > coremark/rudolv/cycles_per_sec.h
            make -C coremark ITERATIONS=1 PORT_DIR=rudolv clean run

            cm=$(grep "Iterations/Sec" coremark/run1.log | cut -c 20-)
            echo
            echo "CoreMark/MHz: $cm"
            ;;

        *)
            echo "Unknown target $1. Stop."
            exit 2
            ;;
    esac
    shift
done

