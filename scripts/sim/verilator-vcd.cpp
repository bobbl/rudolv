// same as verilator.cc, but with .vcd generation
#include "Vtop.h"
#include "verilated_vcd_c.h"

unsigned long timestamp;

// required for $time
double sc_time_stamp()
{
    return timestamp;
}

int main(int argc, char **argv, char **env)
{
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);
    Vtop* top = new Vtop;

    VerilatedVcdC* tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("tmp.vcd");
    top->clk = 0;
    top->rstn = 0;
    timestamp = 0;
    while (!Verilated::gotFinish()) {
        if (timestamp > 200) top->rstn = 1;
        top->clk = !top->clk;
        top->eval();
        tfp->dump(timestamp);
        timestamp += 5;
    }
    tfp->close();
    delete top;
    return 0;
}

