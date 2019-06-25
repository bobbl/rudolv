#ifndef _UART_H
#define _UART_H

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

#define read_uart_rx()      (read_csr(0x7c0) & 1)
#define read_uart_period()  (read_csr(0x7c0) >> 1)
#define write_uart_tx(v)    write_csr(0x7c0, v)


int uart_nonblocking_receive()
{
    if (read_uart_rx() != 0) return -1;
    unsigned long period = read_uart_period();
    unsigned long timestamp = read_csr(cycle) - (period>>1);
    unsigned pattern = 0;
    unsigned i;
    for (i=0; i<10; i++) {
        pattern = pattern | (read_uart_rx() << i);
        while (read_csr(cycle) - timestamp < period);
        timestamp = timestamp + period;
    }
    return (pattern >> 2) & 0xff;
}

int uart_blocking_receive()
{
    int ch;
    do {
        ch = uart_nonblocking_receive();
    } while (ch<0);
    return ch;
}

void uart_send(unsigned ch)
{
    unsigned pattern = ((unsigned)ch<<1) | 0x200;
    unsigned long period    = read_uart_period();
    unsigned long timestamp = read_csr(cycle);
    unsigned i;
    for (i=0; i<10; i++) {
        write_uart_tx(pattern); // everything but lowest bit is ignored
        pattern = pattern >> 1;
        while (read_csr(cycle) - timestamp < period);
        timestamp = timestamp + period;
    }
}

#endif
