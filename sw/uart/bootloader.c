#include "uart.h"

static inline void set_leds(unsigned long leds)
{
    write_csr(0xbc1, leds);
}

void main(int argc, char **argv) __attribute__ ((noreturn));
// _Noreturn gives a warning with main()

void main(int argc, char **argv)
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

/*
    char *p = 0;
    for (i=0; i<length; i++) {
        *p++ = uart_blocking_receive();
    }
*/
    unsigned long *p = 0;
    unsigned long word = 0;
    for (i=0; i<length; i++) {
        word = (word >> 8) | (uart_blocking_receive() << 24);
        if ((i&3)==3) *p++ = word;
    }
    if ((i&3)!=0) {
        // length is not a multiple of 4
        uart_send('!');
        // fill last word correctly (not done in grubby.S)
        *p = word >> (8*(4-(i&3)));
    }

    set_leds(0x04);
    uart_send(13);
    uart_send(10);

    asm volatile ("jr x0");
    while (1); // avoid warning
}

// SPDX-License-Identifier: ISC
