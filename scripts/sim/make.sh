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
    echo "  rvfi <file.elf>              Simulate with RVFI logging"
    echo "  irq-debug <file.elf> <cycle> Raise IRQ in <cycle>, log"
    echo "  gy-run <file.elf>            Enable grubby, no log"
    echo "  gy-debug <file.elf>          Enable grubby, log"
    echo "  tests                        Run ISA tests"
    echo "  irqbomb                      Run IRQ tests"
    echo
    echo "<file.elf>"
    echo "  ../../sw/dhrystone/build/riscv.elf           Benchmark from RISC-V repo"
    echo "  ../../sw/coremark/build/coremark_iter50.elf  CoreMark benchmark"
    echo "  ../../sw/zephyr/elf/philosophers.elf         Philosophers sample from Zephyr"
    echo "  ../../sw/tests/build/xor.elf                 riscv-test for XOR"
    exit 1
fi

. ../../config_default.sh
[ ! -e ../../config.sh ] || . ../../config.sh

verilog_files="../../src/regset.v ../../src/csr.v ../../pipeline.v"
path_tests="../../sw/tests"
path_compliance="../../sw/compliance"
path_irqbomb="../../sw/irqbomb"

esc_green="\033[32m"
esc_red="\033[31m"
esc="\033[0m"



# generate a hex file `tmp.hex` with the memory content
#   $1 ELF filename
elf() {
    ${RV_PREFIX}objcopy -O binary $1 tmp.bin
    printf "@0 " > tmp.hex
    od -An -tx4 -w4 -v tmp.bin | cut -b2- >> tmp.hex
}


# compile simulation (works for Icarus Verilog and Verilator)
#   $1 arguments to compiler
compile() {
    case $simulator in
        icarus)
            $IVERILOG -o tmp.vvp -DCODE=\"dontcare.hex\" $1 testbench.v $verilog_files || exit 1
            # The memory file name will be overwritten by the plusarg when
            # running the simulation. Therefore -DCODE= doesn't care. 
            # But -DCODE= is still supported, because the plusarhs don't work
            # yet with Verlilator
            ;;

        verilator)
            $VERILATOR --cc --exe --top-module top \
                -DCODE=\"tmp.hex\" $1 $verilog_files \
                testbench.v verilator.cpp --Mdir tmp.verilator
            make -C tmp.verilator -f Vtop.mk
            ;;

        verilator-vcd)
            $VERILATOR --cc --exe -trace --top-module top \
                -DCODE=\"tmp.hex\" $1 $verilog_files \
                testbench.v verilator-vcd.cpp --Mdir tmp.verilator
            make -C tmp.verilator -f Vtop.mk
            ;;
    esac
}


# run simulation (works for Icarus Verilog and Verilator)
run() {
    case $simulator in
        icarus)
            $VVP -N tmp.vvp +memfile=tmp.hex
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
    name=$(basename $1 .elf)
    elf $1 > /dev/null
    run | sed -e '/^-/d' > tmp.sig

    diff --strip-trailing-cr $2 tmp.sig > tmp.diff
    if [ $? -ne 0 ]
    then
        printf "not ok - ${esc_red}FAIL${esc} $name\n"
        failed=$(($failed + 1))
    else
        printf "ok -     ${esc_green}PASS${esc} $name\n"
    fi
}


# Compile and run the riscv-tests and riscv-compliance tests
# Output in TAP (Test Anything Protocol) format
target_tests() {

    # build binaries
    make -s -C ${path_tests} || exit 1
    make -s -C ${path_compliance} || exit 1

    # build simulator
    compile "" || exit

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
        printf "# ${esc_green}All ${total} tests passed.${esc}\n"
    else
        printf "# ${esc_red}Failed ${failed} of ${total} tests${esc}\n"
    fi
}


# run binary image several times and raise IRQ in different cycles
#   $1 filename of ELF image
check_irqs() {
    elf $1 > /dev/null
    compile "" > /dev/null

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
        elf $1 > /dev/null
        compile "-DIRQBOMB=$i"
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
            simulator=$1
            ;;
        run)
            elf $2
            compile ""
            run
            shift
            ;;
        debug)
            elf $2
            compile "-DDEBUG"
            run
            shift
            ;;
        rvfi)
            elf $2
            compile "-DRISCV_FORMAL"
            run
            shift
            ;;
        irq-debug)
            elf $2
            compile "-DDEBUG -DIRQBOMB=$3"
            run
            shift
            shift
            ;;
        uart-run)
            elf $2
            compile "-DTEST_UART"
            run
            shift
            ;;
        uart-debug)
            elf $2
            compile "-DTEST_UART -DDEBUG"
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
