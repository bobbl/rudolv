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


#define read_csr(reg) ({ unsigned long __tmp; \
  asm volatile ("csrr %0, " #reg : "=r"(__tmp)); \
  __tmp; })

#define rdcycle() read_csr(cycle)

#define uart_tx (*(volatile char *)0x10003000)
#define uart_period (*(volatile ee_u32 *)0x10004000)



static unsigned timestamp_start, timestamp_stop;

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
    return timestamp_stop - timestamp_start;
}

secs_ret time_in_secs(CORE_TICKS ticks)
{
    return (secs_ret)ticks / (secs_ret)CYCLES_PER_SEC;
}

void uart_send_char(unsigned ch)
{
    *(volatile char *)0x10000000 = ch;
}
