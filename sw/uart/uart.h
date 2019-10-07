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

#define set_csr(r, v) \
    ({long t; asm volatile ("csrrs %0," #r ",%1" : "=r"(t) : "rK"(v)); t;})
#define clear_csr(r, v) \
    ({long t; asm volatile ("csrrc %0," #r ",%1" : "=r"(t) : "rK"(v)); t;})




#ifdef UART_BITBANG
// bitbanging interface: read and set RX and TX pins by software

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




#ifdef UART_CHAR
// CSR mapped character interface, RudolV specific

#define UART_CSR 0xbc0
#define UART_RECV_BUF_EMPTY 0x100
#define UART_SEND_BUF_FULL 0x200

int uart_nonblocking_receive()
{
    return (read_csr(0xbc0) & UART_RECV_BUF_EMPTY) ? -1 : set_csr(0xbc0, UART_RECV_BUF_EMPTY);
}

int uart_blocking_receive()
{
/*
    int ch;
    do {
        ch = uart_nonblocking_receive();
    } while (ch<0);
    return ch;
*/
    unsigned long ch;
    do {
        ch = set_csr(0xbc0, UART_RECV_BUF_EMPTY);
    } while (ch & UART_RECV_BUF_EMPTY);
    return ch & 0xff;
}

void uart_send(unsigned ch)
{
    while (swap_csr(0xbc0, ch) & UART_SEND_BUF_FULL);
}

#endif




#ifdef UART_MIV
// memory mapped interface, compatible to Mi-V (Zephyr supported)

#define uart_send_char (*(volatile char *)0x70000000)
#define uart_recv_char (*(volatile char *)0x70000004)
#define uart_state     (*(volatile char *)0x70000010)
#define uart_send_ready (uart_state & 1)
#define uart_recv_full (uart_state & 2)

int uart_nonblocking_receive()
{
    return uart_recv_full ? uart_recv_char : -1;
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
    while (!uart_send_ready);
    uart_send_char = ch;
}

#endif



#endif
