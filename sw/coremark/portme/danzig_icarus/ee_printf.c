#include <stdarg.h>

void uart_sync();
void uart_send_char(unsigned ch);


static void print_num(char leading, unsigned width, unsigned base, unsigned long long x)
{
    char buf[20];
    unsigned i=0;
    unsigned long long base_ll = base;

    do {
        unsigned long long rem = x % base;
        x = x / base;
        buf[i++] = (rem>9) ? (rem + 'a' - 10) : (rem + '0');
    } while (x!=0);

    for (; i<width; width--) uart_send_char(leading);
    for (; i; i--) uart_send_char(buf[i-1]);
}


int ee_printf(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    uart_sync();

    char ch = *fmt++;
    while (ch) {
        if (ch!='%') {
            uart_send_char(ch);
        } else {
            ch = *fmt++;

            char leading = ' ';
            if (ch=='0') {
                leading = '0';
                ch = *fmt++;
            }

            unsigned width = 0;
            while (ch>='0' && ch<='9') {
                width = width*10 + (ch-'0');
                ch = *fmt++;
            }

            unsigned longs = 0;
            while (ch=='l') {
                longs++;
                ch = *fmt++;
            }

            if (ch=='c') {
                uart_send_char(va_arg(ap, int));
            } else if (ch=='s') {
                char *s = va_arg(ap, char *);
                if (s!=0) {
                    while ( (ch=*s++) != 0 ) uart_send_char(ch);
                }

            } else if (ch=='f') {
                // ignore width and precision
                double x = va_arg(ap, double);
                long long ll = x; // truncate
                if (ll<0) {
                    uart_send_char('-');
                    ll = -ll;
                }
                print_num('0', 0, 10, ll);
                uart_send_char('.');
                print_num('0', 6, 10, (x - (double)ll)*1000000.0);

            } else {
                unsigned base = 10;
                unsigned long long x = 0;

                if (ch=='d' || ch=='i') {
                    if (longs==0) x = va_arg(ap, int);
                    else if (longs==1) x = va_arg(ap, long int);
                    else x = va_arg(ap, long long int);
                    if ((long long)x<0) {
                        uart_send_char('-');
                        x = -x;
                        if (width!=0) width--;
                    }
                } else {
                    if (ch=='o') base = 8;
                    else if (ch=='x') base = 16;
                    else if (ch!='u') {
                        uart_send_char(ch);
                        continue;
                    }
                    if (longs==0) x = va_arg(ap, unsigned int);
                    else if (longs==1) x = va_arg(ap, unsigned long int);
                    else x = va_arg(ap, unsigned long long int);
                }

                print_num(leading, width, base, x);
            }
        }
        ch = *fmt++;
    }

    va_end(ap);
    return 0;
}
