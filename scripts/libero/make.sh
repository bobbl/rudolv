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
    echo "  timing      Verify timing"
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
# /bin/sh==bash is no more required with Libero 2021.1
#    if [ "$BASH_VERSION" = '' ]
#    then
#        echo "/bin/sh is not bash, which is a problem for some Libero scripts. Use"
#        echo "    sudo dpkg-reconfigure dash"
#        echo "to switch to bash."
#        exit 4
#    fi

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
        END { printf("SYNTHESIS RESULTS: %d LEs, %d LUTs, %d DFFs", le, lut, dff)}' \
        ${device}/designer/top/top_layout_log.log

    # Another file with lots of useful information about the resource usage:
    #   ${device}/designer/top/top_compile_netlist_resources.rpt
    # e.g. more than 1000 LUTs and DFFs are used for block RAM interfacing

    # Report on the block RAM usage:
    #   ${device}/designer/top/top_ram_rpt.txt

    awk '/Performance Summary/,/Clock Relationships/ { \
           if ($9=="generated") {mhz=$4} \
         } \
        END { printf(", %s MHz\n", mhz) }' \
        < ${device}/synthesis/top.srr

    printf "\033[0m"
}


# start libero with environment variables for the license server
launch_libero() {
    LM_LICENSE_FILE=1702@localhost \
    SNPSLMD_LICENSE_FILE=1702@localhost \
    LD_LIBRARY_PATH=/usr/lib/i386-linux-gnu/:/usr/lib/x86_64-linux-gnu/:/usr/lib \
    ${LIBEROSOC} "$@"
}


# create tempory .tcl file to run one step in an existing project
# $1 tool name
run_tool() {
    echo "open_project -file {RudolV.prjx}" > tmp.tcl
    echo "run_tool -name {$1}" >> tmp.tcl
    cd $device
    launch_libero SCRIPT:../tmp.tcl
    cd ..
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
            launch_libero SCRIPT:${device}.tcl SCRIPT_ARGS:"MHZ=60"
            report
            ;;
        gy-synth)
            check
            rm -rf ${device}
            launch_libero SCRIPT:${device}.tcl SCRIPT_ARGS:"MHZ=60 ENABLE_GRUBBY"
            report
            ;;
        timing)
            run_tool VERIFYTIMING
            cat ${device}/designer/top/top_has_violations
            # don't know, why the log does not print an error, if there is a
            # violation
            ;;
        prog)
            run_tool PROGRAMDEVICE
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
