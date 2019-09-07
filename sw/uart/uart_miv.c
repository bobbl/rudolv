#include "../bootloader/uart_miv.h"

#define read_pc() ({ unsigned long __tmp; \
  asm volatile ("auipc %0, 0" : "=r"(__tmp)); \
  __tmp; })

static inline void set_leds(unsigned leds)
{
    write_csr(0x7c1, leds);
}

void print_str(char *s)
{
    char ch;
    while (ch=*s++) uart_send(ch);
}

static void print_int32(int32_t l)
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

//    set_leds(1);
    print_str("Starting\r\n");
//    set_leds(ff_leds = 0xaa);

    while (1) {
//        set_leds(ff_leds = ff_leds ^ 0xa4);
        print_hex32(read_pc());
        print_str(" Hello5\r\n");

        wait = read_csr(cycle);
        while (read_csr(cycle) - wait < 115200*read_uart_period()) { // one second
            ch = uart_nonblocking_receive();
            if (ch>0) uart_send(ch+1);
        }
    }
/*
    char hex[16] = "0123456789ABCDEF";
    while (1) {
        ch = uart_nonblocking_receive();
        if (ch>255) uart_send('*');
        else if (ch>=0) {
            uart_send(hex[ch>>4]);
            uart_send(hex[ch&15]);
        }
    }
*/
}
