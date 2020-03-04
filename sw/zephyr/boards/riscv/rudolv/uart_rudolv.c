/* RudolV UART driver for zephyr
 *
 * Copyright (c) 2020 Jörg Mische <bobbl@gmx.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/uart.h>


#define UART_CSR 0xbc0
#define UART_RECV_BUF_EMPTY 0x100
#define UART_SEND_BUF_FULL 0x200

#define read_csr(r) ({ unsigned long t; \
  __asm__ volatile ("csrr %0, %1" : "=r"(t) : "i"(r)); t; })

#define swap_csr(r, v) ({ unsigned long t; \
  __asm__ volatile ("csrrw %0, %1, %2" : "=r"(t) : "i"(r), "rK"(v)); t; })

#define set_csr(r, v) ({ unsigned long t; \
  __asm__ volatile ("csrrs %0, %1, %2" : "=r"(t) : "i"(r), "rK"(v)); t; })


static void uart_rudolv_poll_out(struct device *dev, unsigned char c)
{
    while (swap_csr(UART_CSR, c) & UART_SEND_BUF_FULL);
}


static int uart_rudolv_poll_in(struct device *dev, unsigned char *c)
{
    if (read_csr(UART_CSR) & UART_RECV_BUF_EMPTY) return -1;
    *c = set_csr(UART_CSR, UART_RECV_BUF_EMPTY);
    return 0;
}


static int uart_rudolv_init(struct device *dev)
{
    return 0;
}


static const struct uart_driver_api uart_rudolv_driver_api = {
    .poll_in          = uart_rudolv_poll_in,
    .poll_out         = uart_rudolv_poll_out,
    .err_check        = NULL,
};


DEVICE_AND_API_INIT(
    uart_rudolv_0,
    "uart_0",
    uart_rudolv_init,
    NULL,
    NULL,
    PRE_KERNEL_1,
    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
    (void *)&uart_rudolv_driver_api);
