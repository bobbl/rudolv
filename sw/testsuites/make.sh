#!/bin/sh

if [ $# -eq 0 ] 
then
    echo "Usage: $0 <target> ..."
    echo
    echo "  clone                Clone all test suites from github"
    echo "  riscv-arch-test      Run tests from the RISC-V Architecture Test SIG"
#    echo "  imperas-riscv-tests  Run tests from Imperas"
    exit 1
fi

# Read configuration for external tools
. ../../config_default.sh ; [ ! -e ../../config.sh ] || . ../../config.sh



clone() {
#    git clone https://github.com/riscv-non-isa/riscv-arch-test.git
    git clone https://github.com/bobbl/riscv-arch-test-2.x.git
        # my modified version that fits in 64K of memory

#    git clone https://github.com/riscv-ovpsim/imperas-riscv-tests.git
}



while [ $# -ne 0 ]
do
    case $1 in
        clone)
            clone
            ;;

        riscv-arch-test|A)
            if [ ! -d riscv-arch-test/.git/ ] ; then clone ; fi
            cd riscv-arch-test-2.x

            # write configuration
            echo "export XLEN ?= 32" > Makefile.include
            echo "export RISCV_TARGET ?= rudolv" >> Makefile.include
            echo "export TARGETDIR ?= $(cd ../arch_test_target; pwd)" >> Makefile.include
                # must be an absolute path
            echo "export RISCV_PREFIX ?= $RV32I_PREFIX" >> Makefile.include
            echo "JOBS ?= -j1" >> Makefile.include

            make RISCV_DEVICE=I verify
            make RISCV_DEVICE=M verify
            make RISCV_DEVICE=Zifencei verify
            make RISCV_DEVICE=privilege verify
            make RISCV_DEVICE=C verify

            cd ..
            ;;

        *)
            echo "Unknown command $1. Stop."
            exit 2
            ;;
    esac
    shift
done

