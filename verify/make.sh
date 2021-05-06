#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <command> ..."
    echo
    echo "  clone           Clone riscv-formal source code from github"
    echo "  verify          Run riscv-formal checks"
    echo "  stat            Show failed and passed checks"
    echo "  wave <check>    Show counter example with gtkwave"
    echo "  disasm <check>  Show instructions of the counter example"
    exit 1
fi

# Read configuration for external tools
. ../config_default.sh ; [ ! -e ../config.sh ] || . ../config.sh

esc_green="\033[32m"
esc_red="\033[31m"
esc="\033[0m"



clone() {
    git clone https://github.com/YosysHQ/riscv-formal.git
    mkdir riscv-formal/cores/rudolv
    cp checks.cfg riscv-formal/cores/rudolv/
}


while [ $# -ne 0 ]
do
    case $1 in
        clone)
            clone
            ;;

        verify)
            cd riscv-formal/cores/rudolv
            rm -r checks/
            python3 ../../checks/genchecks.py
            time make -C checks/ -j$(($(nproc)/2))
                # using only half the available cores works better for me
            cd ../../..
            ;;

        stat)
            total=$(set -- riscv-formal/cores/rudolv/checks/*.sby ; echo $#)
                # count files by counting arguments
            echo "TAP version 13"
            echo "1..$total"
            passed=0
            failed=0
            for t in riscv-formal/cores/rudolv/checks/*.sby
            do
                name=$(basename "$t" .sby)
                dirname=riscv-formal/cores/rudolv/checks/$name
                if [ -e $dirname/PASS ]
                then
                    printf "ok -     ${esc_green}PASS${esc} $name\n"
                    passed=$(($passed + 1))
                fi
                if [ -e $dirname/FAIL ]
                then
                    printf "not ok - ${esc_red}FAIL${esc} $name\n"
                    failed=$(($failed + 1))
                fi
            done
            if [ $passed -eq $total ]
            then
                printf "# ${esc_green}All $total tests passed.${esc}\n"
            else
                printf "# ${esc_red}Failed ${failed} of ${total} tests${esc}\n"
            fi
            ;;

        wave)
            gtkwave riscv-formal/cores/rudolv/checks/$2/engine_0/trace.vcd &
            shift
            ;;

        disasm)
            python3 disasm.py \
                riscv-formal/cores/rudolv/checks/$2/engine_0/trace.vcd
            ${RV_PREFIX}gcc -c -o tmp.o disasm.s
            ${RV_PREFIX}objdump -d -M numeric,no-aliases tmp.o
            shift
            ;;

        *)
            echo "Unknown command $1. Stop."
            exit 1
            ;;
    esac
    shift
done

