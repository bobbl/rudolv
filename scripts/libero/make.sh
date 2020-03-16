#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <device> <target> ..."
    echo
    echo "<device>"
    echo "  m2gl025     Future Electronics Creative Board"
    echo
    echo "<target>"
    echo "  synth       Synthesize using Libero"
    echo "  gy-synth    Synthesize with grubby support"
    echo "  prog        Program the device"
    echo "  gui <args>  Start Libero SoC GUI"
    exit 1
fi

# Read configuration for external tools
. ../../config_default.sh ; [ ! -e ../../config.sh ] || . ../../config.sh

device=$1
case $1 in
    m2gl025)    ;;
    *)          echo "Currently only the m2gl025 board is supported."
                echo "Unknown device $1. Stop."
                exit 2
                ;;
esac
shift



# check requirements
check() {
    if [ "$BASH_VERSION" = '' ]
    then
        echo "/bin/sh is not bash, which is a problem for some Libero scripts. Use"
        echo "    sudo dpkg-reconfigure dash"
        echo "to switch to bash."
        exit 4
    fi

    if ps -e | grep -q lmgrd 
    then
        echo "Microsemi license manager found"
    else
        echo "Microsemi license manager not found. Please start lmgrd first."
        exit 5
    fi
}


report() {
    printf "\033[33m"
    awk '/\^| 4LUT / {lut=$4} \
         /\^| DFF /  {dff=$4} \
         /\^| Logic Element /  {le=$5} \
        END { printf("SYNTHESIS RESULTS: %d LEs, %d LUTs, %d DFFs\n", \
                     le, lut, dff)}' \
        ${device}/designer/top/top_layout_log.log
    printf "\033[0m"
}


# start libero with environment variables for the license server
launch_libero() {
    LM_LICENSE_FILE=1702@localhost \
    SNPSLMD_LICENSE_FILE=1702@localhost \
    LD_LIBRARY_PATH=/usr/lib/i386-linux-gnu/:/usr/lib/x86_64-linux-gnu/:/usr/lib \
    ${LIBEROSOC} "$@"
}


while [ $# -ne 0 ]
do
    case $1 in
        report)
            report
            ;;
        synth)
            check
            rm -rf ${device}
            launch_libero SCRIPT:${device}.tcl
            report
            ;;
        gy-synth)
            ;;
        timing)
            cd $device
            launch_libero SCRIPT:../verify_timing.tcl
            cd ..
            ;;
        prog)
            ;;
        gui)
            check
            launch_libero "$@"
            exit
            ;;
        *)
            echo "Unknown target $1. Stop."
            exit 3
            ;;
    esac
    shift
done


# SPDX-License-Identifier: ISC
