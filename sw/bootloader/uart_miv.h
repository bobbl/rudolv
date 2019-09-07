#ifndef _UART_MIV_H
#define _UART_MIV_H

#include <stdint.h>

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
