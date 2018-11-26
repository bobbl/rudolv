#!/bin/sh
# Compile and run the compliance tests
# Output in TAP (Test Anything Protocol) format

PATH_COMPLIANCE=../../sw/compliance
VERILOG_FILES=../../pipeline.v

make -s -C ${PATH_COMPLIANCE}

echo "TAP version 13"
TOTAL=$(ls -afq ${PATH_COMPLIANCE}/build/*.hex | wc -l)
echo "1..${TOTAL}"
FAILED=0
for TEST in ${PATH_COMPLIANCE}/build/*.hex
do
    NAME=$(basename ${TEST} .hex)
    iverilog -o tmp.vvp -DCODE=\"${TEST}\" tb_compliance.v ${VERILOG_FILES}
    vvp -N tmp.vvp | sed -e '/xxxxxxxx/d' > tmp.sig
    diff --strip-trailing-cr \
        ${PATH_COMPLIANCE}/references/${NAME}.reference_output tmp.sig > tmp.diff
    if [ $? = 0 ]
    then
        echo "ok - ${NAME}"
    else
        echo "not ok - ${NAME}"
        FAILED=$((${FAILED} + 1))
    fi
done

if [ ${FAILED} = 0 ] 
then
    echo "# All tests passed."
else
    echo "# Failed ${FAILED} of ${TOTAL} tests"
fi
