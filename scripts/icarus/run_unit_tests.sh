#!/bin/sh
# Compile and run the riscv-tests and riscv-compliance tests
# Output in TAP (Test Anything Protocol) format

PATH_TESTS=../../sw/tests
PATH_COMPLIANCE=../../sw/compliance
VERILOG_FILES="../../pipeline.v"


# run binary image and compare signature
#   $1 filename of binary image
#   $2 filename of expected signature
check_sig() {
    NAME=$(basename $1 .hex)
    iverilog -o tmp.vvp -DCODE=\"$1\" tb_tests.v \
        ../../src/memory32.v ../../src/regset32.v ${VERILOG_FILES}
    vvp -N tmp.vvp | sed -e '/^xxxxxxxx$/d' > tmp.sig

    diff --strip-trailing-cr $2 tmp.sig > tmp.diff
    if [ $? -ne 0 ]
    then
        printf "not "
        FAILED=$((${FAILED} + 1))
    fi
    echo "ok - ${NAME}"
}




# build binaries
make -s -C ${PATH_TESTS}
make -s -C ${PATH_COMPLIANCE}

# count tests
echo "TAP version 13"
COUNT_TESTS=$(ls -afq ${PATH_TESTS}/build/*.hex | wc -l)
COUNT_COMPLIANCE=$(ls -afq ${PATH_COMPLIANCE}/build/*.hex | wc -l)
TOTAL=$(( $COUNT_TESTS + $COUNT_COMPLIANCE ))
echo "1..${TOTAL}"
FAILED=0


for TEST in ${PATH_TESTS}/build/*.hex
do
    check_sig ${TEST} ok.sig
done

for TEST in ${PATH_COMPLIANCE}/build/*.hex
do
    NAME=$(basename ${TEST} .hex)
    check_sig ${TEST} ${PATH_COMPLIANCE}/references/${NAME}.reference_output
done 


if [ "${FAILED}" -eq 0 ]
then
    echo "# All ${TOTAL} tests passed."
else
    echo "# Failed ${FAILED} of ${TOTAL} tests"
fi
