#include "uart.h"

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
