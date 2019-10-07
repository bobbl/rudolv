#!/bin/sh

QUARTUS_BIN=/home/bobbl/install/altera/16.0/quartus/bin
PROJ=de2-115


one_column()
{
    printf "@0 "   > $2
    yes "00" | head -n 16256 >> $2
#    awk '{print $1}' mem32.tmp >> $2
    cut -d' ' -f $1 < mem32.tmp >> $2
}


# 4 columns of the bytes
od -An -tx1 -v -w4 ../../sw/uart/build/bootloader_char.bin > mem32.tmp
i=$(wc -l < mem32.tmp)

# align to 512 bytes
while [ $i -lt 128 ]
do
    echo " 00 00 00 00" >> mem32.tmp
    i=$((i+1))
done


one_column 2 mem0.hex
one_column 3 mem1.hex
one_column 4 mem2.hex
one_column 5 mem3.hex




mkdir -p build
cp ${PROJ}.qsf build/

cd build


## CAUTION: offset is a word (4 byte) address, not a byte address
#sed -e 's/@0 /@3f80 /' ../../../sw/bootloader/bootloader.hex > bootloader.hex


${QUARTUS_BIN}/quartus_map ${PROJ}.qsf
${QUARTUS_BIN}/quartus_fit --read_settings_files=off -write_settings_files=off ${PROJ} -c ${PROJ}
${QUARTUS_BIN}/quartus_sta ${PROJ} -c ${PROJ}
${QUARTUS_BIN}/quartus_asm ${PROJ}

grep -A3 "Total logic elements" output_files/${PROJ}.fit.summary
grep -B1 "Slack" output_files/${PROJ}.sta.summary


# programmer

# sudo killall jtagd
# sudo ${QUARTUS_BIN}/jtagconfig

${QUARTUS_BIN}/quartus_pgm -m jtag -o "p;output_files/${PROJ}.sof"




cd ..
