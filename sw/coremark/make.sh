#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <target> ..."
    echo
    echo "  clone      Clone coremark source code from github"
    echo "  elf <MHz>  Build image to measure total CoreMark of an FPGA implementation"
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

        elf)
            if [ $# -lt 2 ] ; then echo "Iterations expected" ; exit 3 ; fi
            if [ ! -d coremark/.git/ ] ; then clone ; fi
            mkdir -p build
            make -C coremark ITERATIONS=$2 PORT_DIR=rudolv clean link
            cp coremark/coremark.elf build/coremark_iter_$2.elf
            shift
            ;;

        icarus)
            if [ ! -d coremark/.git/ ] ; then clone ; fi
            mkdir -p build
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

