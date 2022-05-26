#!/bin/sh
# Cannot be done in make.sh, because it can be called from any directory

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <elffile> <sigfile>"
    echo
    echo "Simulate the executable <elffile> and write its signature to <sigfile>"
    exit 1
fi

rootdir=$(dirname "$0")/../..
# can be called from anywhere

. $rootdir/config_default.sh
[ ! -e $rootdir/config.sh ] || . $rootdir/config.sh

verilog_files="$rootdir/src/regset.v $rootdir/src/csr.v $rootdir/pipeline.v"




${RV_PREFIX}objcopy -O binary "$1" tmp.bin
printf "@0 " > tmp.hex
od -An -tx4 -w4 -v tmp.bin | cut -b2- >> tmp.hex
$IVERILOG -o tmp.vvp -DCODE=\"tmp.hex\" $rootdir/scripts/sim/testbench.v \
    $verilog_files || exit 1
$VVP -N tmp.vvp > tmp.stdout
cat tmp.stdout | sed -e '/^-/d' > "$2"


