#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <device> <target> ..."
    echo
    echo "<device>"
    echo "  de2-115     terasIC DE2-115 board"
    echo
    echo "<target>"
    echo "  bootloader  Build bootloader"
    echo "  synth       Synthesize with Quartus"
    echo "  prog        Program the device"
    echo "  jtag        If programming does not work, try this"
    echo
    echo "Set QUARTUS_BIN (with trailing slash) if Quartus is not in the search path"
    exit 1
fi

# Read configuration for external tools
. ../../config_default.sh ; [ ! -e ../../config.sh ] || . ../../config.sh




one_column()
{
    printf "@0 "   > $2
    yes "00" | head -n 16256 >> $2
    cut -d' ' -f $1 < tmp.mem32.hex >> $2
}


target_bootloader()
{
    # build bootloader from sources if necessary
    back=$(pwd)
    cd $(dirname $0)/../../sw/uart
    if [ ! -e build/bootloader_char.bin ] || \
        [ $(find bootloader.c -newer build/bootloader_char.bin) ]
    then
        ./make.sh all
    fi
    cd $back

    # 4 columns of the bytes
    od -An -tx1 -v -w4 ../../sw/uart/build/bootloader_char.bin > tmp.mem32.hex
    i=$(wc -l < tmp.mem32.hex)

    # align to 512 bytes
    while [ $i -lt 128 ]
    do
        echo " 00 00 00 00" >> tmp.mem32.hex
        i=$((i+1))
    done

    mkdir -p build
    one_column 2 build/bootloader.byte0.hex
    one_column 3 build/bootloader.byte1.hex
    one_column 4 build/bootloader.byte2.hex
    one_column 5 build/bootloader.byte3.hex
}


report() {
    printf "\033[33m"

    awk '/^Total logic elements : / {le=$5} \
         /^    Total combinational functions : / {lut=$5} \
         /^    Dedicated logic registers : / {dff=$5} \
         END {
           gsub(/,/, "", le);
           gsub(/,/, "", lut);
           gsub(/,/, "", dff);
           printf("SYNTHESIS RESULTS: %s slices, %s LUTs, %s DFFs, ", \
             le, lut, dff)}' \
        < build/output_files/${device}.fit.summary

    awk '/; Slow 1200mV 85C Model Fmax Summary/,/; Slow 1200mV 85C Model Setup Summary/ {
           if ($8=="CLOCK_50") {mhz=$2} \
         } \
         END {
           printf("%s MHz\n", mhz)}' \
        < build/output_files/${device}.sta.rpt

    printf "\033[0m"
}

# $1 arguments to quartus_map
quartus()
{
    mkdir -p build
    cp ${device}.qsf build/
    cd build

    ${QUARTUS_BIN}quartus_map $1 ${device}.qsf && \
    ${QUARTUS_BIN}quartus_fit --read_settings_files=off \
        -write_settings_files=off ${device} -c ${device} && \
    ${QUARTUS_BIN}quartus_sta ${device} -c ${device} && \
    ${QUARTUS_BIN}quartus_asm ${device}
    cd ..
}





device=$1
case $1 in
    de2-115)    ;;
    *)          echo "Currently only the de2-115 board is supported."
                echo "Unknown device $1. Stop."
                exit 2
                ;;
esac
shift



while [ $# -ne 0 ]
do
    case $1 in
        report)
            report
            ;;
        bootloader)
            target_bootloader
            ;;
        synth)
            target_bootloader
            quartus
            report
            ;;
        gy-synth)
            target_bootloader
            quartus "--verilog_macro=ENABLE_GRUBBY"
            report
            ;;
        prog)
            ${QUARTUS_BIN}/quartus_pgm -m jtag -o "p;build/output_files/${device}.sof"
            ;;
        jtag)
            # try if programming does not work
            sudo killall jtagd
            sudo ${QUARTUS_BIN}/jtagconfig
            ;;
        *)
            echo "Unknown target $1. Stop."
            exit 2
            ;;
    esac
    shift
done
