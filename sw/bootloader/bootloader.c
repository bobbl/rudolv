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




static int uart_nonblocking_receive()
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


static int uart_blocking_receive()
{
    int ch;
    do {
        ch = uart_nonblocking_receive();
    } while (ch<0);
    return ch;
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


int main(int argc, char **argv)
{
    unsigned long i;
    unsigned long length = 0;

    uart_send('?');

    while (1) {
        int ch = uart_nonblocking_receive();
        if (ch>='0' && ch<='9') {
            length = (((length<<2) + length)<<1) + ch -'0';
        } else if (ch >= 0) break;
    }

    char *p = 0;
    for (i=0; i<length; i++) {
        *p++ = uart_blocking_receive();
    }

    uart_send(13);
    uart_send(10);

    asm volatile ("jalr 0(x0)");
}