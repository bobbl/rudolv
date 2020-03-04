/* RudolV timer driver for zephyr
 *
 * Copyright (c) 2020 Jörg Mische <bobbl@gmx.de>
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/timer/system_timer.h>
#include <sys_clock.h>
#include <spinlock.h>
#include <soc.h>


#define CSR_TIMER_DELAY 0xbc2
#define CSR_CYCLE       0xc00
#define CSR_CYCLEH      0xc80
#define CSR_KHZ         0xfc0

#define read_csr(r) ({ unsigned long t; \
  __asm__ volatile ("csrr %0, %1" : "=r"(t) : "i"(r)); t; })

#define write_csr(r, v) ({ \
  __asm__ volatile ("csrw %0, %1" :: "i"(r), "rK"(v)); })



static void set_delay(unsigned long delay)
{
    write_csr(CSR_TIMER_DELAY, delay);
}


#define MIN_DELAY 1000

static unsigned long cycles_per_tick;
static struct k_spinlock lock;


#ifdef CONFIG_TIMER_RUDOLV_32BIT
/* Use only a 32 bit timer and a loop to compute the delay for programming the
 * timer interrupt.
 *
 * If the interrupt service routine is invoked regularly (which is typically
 * true), the timer must only be a few bits wider than cycles_per_tick and the
 * next timestamp can be computed by a few additions instead of a costly
 * division.
 * Much faster if RV64M is not available or only RV32.
 *
 * Does not work if the ISR is often delayed for many ticks or
 * if cycles_per_tick > 1'000'000'000.
 */


static u32_t last_tick;

/* last_tick is the timestamp of the last invocation of timer_isr
 * To get the timestamp for the next invocation, increment the timestamp
 * until it is in the future. Then program the timer with the difference
 * up to this timestamp.
 */
static void set_next_tick()
{
    u32_t now = read_csr(CSR_CYCLE) + MIN_DELAY;
    do {
        last_tick += cycles_per_tick;
    } while (last_tick <= now);
    set_delay(last_tick - now + MIN_DELAY);
}



#else
/* Use 64 bit timer and the  remainder of a division to compute the delay for
 * programming the timer interrupt.
 *
 * To force the compiler to replace the slow division by a bit operation, 
 * (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
 * should be a power of 2.
 * Otherwise the interrupt service routine might be very slow.
 */


static u64_t read_cycles(void)
{
    u32_t lo, hi;
    do {
        hi = read_csr(CSR_CYCLEH);
        lo = read_csr(CSR_CYCLE);
    } while (read_csr(CSR_CYCLEH) != hi);
    return (((u64_t)hi) << 32) | lo;
}


static void set_next_tick()
{
    u64_t now = read_cycles() + MIN_DELAY;
    set_delay(MIN_DELAY + cycles_per_tick - (now % cycles_per_tick));
}


#endif


static void timer_isr(void *arg)
{
    ARG_UNUSED(arg);

    // No lock neccessary, because timer is not programmed by absolute
    // timestamp but a relative delay
    //k_spinlock_key_t key = k_spin_lock(&lock);
    set_next_tick();
    //k_spin_unlock(&lock, key);
    z_clock_announce(1);
}



int z_clock_driver_init(struct device *device)
{
    extern int z_clock_hw_cycles_per_sec;

    IRQ_CONNECT(RISCV_MACHINE_TIMER_IRQ, 0, timer_isr, NULL, 0);

#ifdef CONFIG_TIMER_RUDOLV_32BIT
    last_tick                 = read_csr(CSR_CYCLE);
#endif
    unsigned long hz          = 1000 * read_csr(CSR_KHZ);
    cycles_per_tick           = hz / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    z_clock_hw_cycles_per_sec = hz;
    set_delay(cycles_per_tick);

    irq_enable(RISCV_MACHINE_TIMER_IRQ);
    return 0;
}


void z_clock_set_timeout(s32_t ticks, bool idle);


u32_t z_clock_elapsed()
{
    return 0;
}


u32_t z_timer_cycle_get_32()
{
    return read_csr(CSR_CYCLE);
}
