#include <stdint.h>

#define read_csr(reg) ({ unsigned long __tmp; \
  asm volatile ("csrr %0, " #reg : "=r"(__tmp)); \
  __tmp; })

#define write_csr(reg, val) ({ \
  asm volatile ("csrw " #reg ", %0" :: "rK"(val)); })

#define swap_csr(reg, val) ({ unsigned long __tmp; \
  asm volatile ("csrrw %0, " #reg ", %1" : "=r"(__tmp) : "rK"(val)); \
  __tmp; })

#define set_csr(reg, bit) ({ unsigned long __tmp; \
  asm volatile ("csrrs %0, " #reg ", %1" : "=r"(__tmp) : "rK"(bit)); \
  __tmp; })

#define clear_csr(reg, bit) ({ unsigned long __tmp; \
  asm volatile ("csrrc %0, " #reg ", %1" : "=r"(__tmp) : "rK"(bit)); \
  __tmp; })

#define rdtime() read_csr(time)
#define rdcycle() read_csr(cycle)
#define rdinstret() read_csr(instret)


#define CLOCK_RATE 12000000
#define BAUD_RATE 9600

#define leds (*(volatile char *)0x10001000)
#define uart_rx (*(volatile char *)0x10002000)
#define uart_tx (*(volatile char *)0x10003000)
#define uart_period (*(volatile uint32_t *)0x10004000)


#define read_pc() ({ unsigned long __tmp; \
  asm volatile ("auipc %0, 0" : "=r"(__tmp)); \
  __tmp; })



// send an idle bit before any send for proper synchronization
static void uart_sync()
{
    uart_tx = 1;
    unsigned long wait = rdcycle();
    while (rdcycle() - wait < uart_period);
}


static void uart_send(unsigned ch)
{
    unsigned pattern = ((unsigned)ch<<1) | 0x200;
    unsigned i;
    unsigned long period    = uart_period;
    unsigned long timestamp = rdcycle();
    for (i=0; i<10; i++) {
        uart_tx = pattern & 1;
        pattern = pattern >> 1;
        while (rdcycle() - timestamp < period);
        timestamp = timestamp + period;
    }
}

int uart_nonblocking_receive()
{
    unsigned long timestamp = read_csr(cycle);
    if ((uart_rx & 1) != 0) return -1;

    unsigned i;
    unsigned pattern = 0;
    unsigned long period = uart_period;
    timestamp += period/2;

    for (i=0; i<10; i++) {
        pattern = pattern | ((uart_rx & 1) << i);
        while (rdcycle() < timestamp);
        timestamp = timestamp + period;
    }
    return (pattern >> 2) & 0xff;
}



void print_str(char *s)
{
    char ch;
    while (ch=*s++) uart_send(ch);
}

void print_int32(int32_t l)
{
    if (l<0) {
        uart_send('-');
        l = -l;
    }

    int i;
    int32_t base = 1000000000;
    for (i=0; i<9; i++) {
        int64_t leading = l / base;
        base = base / 10;
        if (leading>0)
            uart_send((leading % 10) + '0');
    }
    uart_send((l % 10) + '0');
}


void print_hex32(uint32_t l)
{
    int i;
    for (i=28; i>=0; i-=4) {
        int digit = (l>>i) & 15;
        uart_send(digit + ((digit>9) ? ('A'-10) : ('0')));
    }
}


extern unsigned long _entrypc;

int main(int argc, char **argv)
{
    char ff_leds = 0;
    unsigned long wait;
    int ch;

    uart_sync();

    print_str("Starting\r\n");
    leds = ff_leds = 0xaa;
    while (1) {
        leds = ff_leds = ff_leds ^ 0xa4;
        print_hex32(read_pc());
        print_str(" Hello2\r\n");

        wait = rdcycle();
        while (rdcycle() - wait < 115200*uart_period) { // one second
            ch = uart_nonblocking_receive();
            if (ch>0) uart_send(ch+1);
        }
    }
}