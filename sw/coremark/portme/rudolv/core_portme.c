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

#define write_csr(reg, val) ({ \
  asm volatile ("csrw " #reg ", %0" :: "rK"(val)); })

#define swap_csr(r, v) \
({ long t; asm volatile ("csrrw %0, " #r ", %1" : "=r"(t) : "rK"(v)); t; })

#define read_csr(r) \
    ({ unsigned long t; asm volatile ("csrr %0, " #r : "=r"(t)); t; })


static inline unsigned long long rdcycle()
{
    unsigned long h1, h2, l;
    do {
        h1 = read_csr(cycleh);
        l  = read_csr(cycle);
        h2 = read_csr(cycleh);
    } while (h1 != h2);
    return ((unsigned long long)h2 << 32) | l;
}

void portable_init(core_portable *p, int *argc, char *argv[])
{
    p->portable_id = 1;
}

void portable_fini(core_portable *p)
{
    p->portable_id = 0;
    write_csr(0x3ff, 1); // end simulation
    while (1);
}

static unsigned long long timestamp_start, timestamp_stop;

void start_time()
{
    timestamp_start = rdcycle();
}

void stop_time()
{
    timestamp_stop = rdcycle();
}

CORE_TICKS get_time()
{
    return (timestamp_stop - timestamp_start) / 1000;
        // divide because result must fit in 32 bits
}

secs_ret time_in_secs(CORE_TICKS ticks)
{
    return (secs_ret)ticks / (secs_ret)read_csr(0xFC0);
        // read clock frequency in KHz from CSR 0xFC0 (RudolV feature)
}

void uart_send_char(unsigned ch)
{
    while (swap_csr(0xbc0, ch) & 0x200);
}
