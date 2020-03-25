#include "Vtop.h"

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
    top->clk = 0;
    top->rstn = 0;
    timestamp = 0;
    while (!Verilated::gotFinish()) {
        if (timestamp > 200) top->rstn = 1;
        top->clk = !top->clk;
        top->eval();
        timestamp += 5;
    }
    delete top;
    return 0;
}

