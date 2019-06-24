#include "coremark.h"
#include "core_portme.h"

#if VALIDATION_RUN
	volatile ee_s32 seed1_volatile=0x3415;
	volatile ee_s32 seed2_volatile=0x3415;
	volatile ee_s32 seed3_volatile=0x66;
#endif
#if PERFORMANCE_RUN
	volatile ee_s32 seed1_volatile=0x0;
	volatile ee_s32 seed2_volatile=0x0;
	volatile ee_s32 seed3_volatile=0x66;
#endif
#if PROFILE_RUN
	volatile ee_s32 seed1_volatile=0x8;
	volatile ee_s32 seed2_volatile=0x8;
	volatile ee_s32 seed3_volatile=0x8;
#endif
	volatile ee_s32 seed4_volatile=ITERATIONS;
	volatile ee_s32 seed5_volatile=0;

ee_u32 default_num_contexts=1;

void portable_init(core_portable *p, int *argc, char *argv[])
{
    p->portable_id = 1;
}

void portable_fini(core_portable *p)
{
    p->portable_id = 0;
}



#define read_csr(reg) ({ unsigned long t; \
    asm volatile ("csrr %0, " #reg : "=r"(t)); t; })

#define write_csr(reg, val) ({ \
    asm volatile ("csrw " #reg ", %0" :: "rK"(val)); })

static unsigned timestamp_start, timestamp_stop;

void start_time()
{
    timestamp_start = read_csr(cycle);
}

void stop_time()
{
    timestamp_stop = read_csr(cycle);
}

CORE_TICKS get_time()
{
    return timestamp_stop - timestamp_start;
}

secs_ret time_in_secs(CORE_TICKS ticks)
{
    return ticks / CYCLES_PER_SEC;
}



#define read_uart_rx()      (read_csr(0x7c0) & 1)
#define read_uart_period()  (read_csr(0x7c0) >> 1)
#define write_uart_tx(v)    write_csr(0x7c0, v)

void uart_send_char(unsigned ch)
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
