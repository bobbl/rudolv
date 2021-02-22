#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <command> ..."
    echo
    echo "Choose simulator"
    echo "  icarus                       Use Icarus Verilog for simulation (default)"
    echo "  verilator                    Use Verilator for simulation"
    echo "  verilator-vcd                Use Verilator ans generate .vcd file"
    echo
    echo "Simulation"
    echo "  run <file.elf>               Simulate without logging"
    echo "  debug <file.elf>             Simulate with extensive logging"
    echo "  irq-debug <file.elf> <cycle> Raise IRQ in <cycle>, log"
    echo "  gy-run <file.elf>            Enable grubby, no log"
    echo "  gy-debug <file.elf>          Enable grubby, log"
    echo "  tests                        Run tests (riscv-tests, riscv-compliance, RudolV-specific)"
    echo "  irqbomb                      Run IRQ tests"
    echo
    echo "<file.elf>"
    echo "  ../../sw/dhrystone/build/riscv.elf      Benchmark from RISC-V repo"
    echo "  ../../sw/zephyr/elf/philosophers.elf    Philosophers sample from Zephyr"
    echo "  ../../sw/compliance/build/I-XOR-01.elf  Compliance test for XOR"
    exit 1
fi

. ../../config_default.sh
[ ! -e ../../config.sh ] || . ../../config.sh

verilog_files="../../src/memory.v ../../src/regset.v ../../src/csr.v ../../pipeline.v"
path_tests="../../sw/tests"
path_compliance="../../sw/compliance"
path_irqbomb="../../sw/irqbomb"






# compile Icarus Verilog simulation
#   $1 ELF filename
#   $2 arguments for iverilog
compile() {
    # build memory content
    ${RV_PREFIX}objcopy -O binary $1 tmp.bin
    printf "@0 " > tmp.hex
    od -An -tx4 -w4 -v tmp.bin | cut -b2- >> tmp.hex

    case $simulator in
        icarus)
            $IVERILOG -o tmp.vvp -DCODE=\"tmp.hex\" $2 testbench.v $verilog_files || exit 1
            ;;

        verilator)
            $VERILATOR --cc --exe --top-module top \
                -DCODE=\"tmp.hex\" $2 $verilog_files \
                testbench.v verilator.cpp --Mdir tmp.verilator
            make -C tmp.verilator -f Vtop.mk
            ;;

        verilator-vcd)
            $VERILATOR --cc --exe -trace --top-module top \
                -DCODE=\"tmp.hex\" $2 $verilog_files \
                testbench.v verilator-vcd.cpp --Mdir tmp.verilator
            make -C tmp.verilator -f Vtop.mk
            ;;
    esac
}


# run Icarus Verilog simulation
run() {
    case $simulator in
        icarus)
            $VVP -N tmp.vvp
            ;;

        verilator)
            ./tmp.verilator/Vtop
            ;;

        verilator-vcd)
            ./tmp.verilator/Vtop
            ;;
    esac
}




# run binary image and compare signature
#   $1 filename of ELF image
#   $2 filename of expected signature
check_sig() {
    compile $1 "" > /dev/null
    run | sed -e '/^-/d' > tmp.sig

    diff --strip-trailing-cr $2 tmp.sig > tmp.diff
    if [ $? -ne 0 ]
    then
        printf "not "
        failed=$(($failed + 1))
    fi
    name=$(basename $1 .elf)
    echo "ok - $name"
}


# Compile and run the riscv-tests and riscv-compliance tests
# Output in TAP (Test Anything Protocol) format
target_tests() {

    # build binaries
    make -s -C ${path_tests} || exit 1
    make -s -C ${path_compliance} || exit 1

    # count tests
    echo "TAP version 13"
    count_tests=$(ls -afq ${path_tests}/build/*.elf | wc -l)
    count_compliance=$(ls -afq ${path_compliance}/build/*.elf | wc -l)
    total=$(( $count_tests + $count_compliance ))
    echo "1..${total}"
    failed=0

    for test in ${path_tests}/build/*.elf
    do
        check_sig ${test} ${path_tests}/ok.sig
    done

    for test in ${path_compliance}/build/*.elf
    do
        name=$(basename $test .elf)
        check_sig $test ${path_compliance}/references/${name}.reference_output
    done 

    if [ "${failed}" -eq 0 ]
    then
        echo "# All ${total} tests passed."
    else
        echo "# Failed ${failed} of ${total} tests"
    fi
}


# run binary image several times and raise IRQ in different cycles
#   $1 filename of ELF image
check_irqs() {
    compile $1 "" > /dev/null

    run > tmp.log

    cat tmp.log | grep 'IRQBOMB marker' > tmp.marker
    marker=$(cat tmp.marker | cut -d' ' -f5)
    count=$(cat tmp.marker | cut -d' ' -f7)

    echo "TAP version 13"
    echo "1..$count"
    i=$marker
    lasti=$(($marker + $count))
    while [ $i -lt $lasti ]
    do
        compile $1 "-DIRQBOMB=$i"
        run > tmp.log

        ok=$(cat tmp.log | grep IRQBOMB -v)
        response=$(cat tmp.log | grep 'IRQBOMB response' | cut -d' ' -f5)
        delta=$(($response - $i))

        if [ "$ok" = '00006b6f' ]
        then
            echo "ok - $(basename $1) request $i response $response time $delta"
        else
            echo "not ok - $1 request $i execution failed"
            exit 1
        fi

        i=$(($i + 1))
    done
}

irqbomb() {

    # build binaries
    make -s -C ${path_irqbomb} || exit 1

    for test in ${path_irqbomb}/build/*.elf
    do
        check_irqs $test
    done
}





simulator=icarus

while [ $# -ne 0 ]
do
    case $1 in
        icarus|verilator|verilator-vcd)
            simulator = $1
            ;;
        run)
            compile $2 ""
            run
            shift
            ;;
        debug)
            compile $2 "-DDEBUG"
            run
            shift
            ;;
        irq-debug)
            compile $2 "-DDEBUG -DIRQBOMB=$3"
            run
            shift
            shift
            ;;
        gy-run)
            compile $2 "-DENABLE_GRUBBY"
            run
            shift
            ;;
        gy-debug)
            compile $2 "-DENABLE_GRUBBY -DDEBUG"
            run
            shift
            ;;
        tests)
            target_tests
            ;;
        irqbomb)
            irqbomb
            ;;
        *)
            echo "Unknown target $1. Stop."
            exit 2
            ;;
    esac
    shift
done
