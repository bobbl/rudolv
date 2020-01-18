#include "uart.h"

static inline void set_leds(unsigned long leds)
{
    write_csr(0xbc1, leds);
}

int main(int argc, char **argv)
{
    unsigned long i;
    unsigned long length = 0;

    set_leds(0x02);

    // Microsemi: UART needs some time until it is ready
    unsigned long start = read_cycle();
    while (read_cycle() - start < 19000); // 22000 okay, 16384 not


    uart_send('?');

    while (1) {
        int ch = uart_blocking_receive();
        if (ch>='0' && ch<='9') {
            length = (((length<<2) + length)<<1) + ch -'0';
        } else if (ch >= 0) break;
    }
    set_leds(0x03);

    char *p = 0;
    for (i=0; i<length; i++) {
        *p++ = uart_blocking_receive();
    }

    set_leds(0x04);
    uart_send(13);
    uart_send(10);

    asm volatile ("jalr 0(x0)");
}

// SPDX-License-Identifier: ISC
