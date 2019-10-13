#ifndef CORE_PORTME_H
#define CORE_PORTME_H

#include <stddef.h>
#include <stdint.h>

typedef int16_t         ee_s16;
typedef uint16_t        ee_u16;
typedef int32_t         ee_s32;
typedef uint32_t        ee_u32;
typedef double          ee_f32;
typedef unsigned char   ee_u8;
typedef size_t          ee_size_t;
typedef intptr_t        ee_ptr_int;



// set to 1 Mhz to print CoreMark/MHz
//#define CYCLES_PER_SEC 1000000
#include "cycles_per_sec.h"


#define SEED_METHOD SEED_VOLATILE
#define MEM_METHOD MEM_STACK

#define MULTITHREAD 1
#define USE_PTHREAD 0
#define USE_FORK 0
#define USE_SOCKET 0

#define MAIN_HAS_NOARGC 0       // main() has argc/argv
#define MAIN_HAS_NORETURN 0     // main() returns int
#define HAS_FLOAT 1             // softfloat support
#define HAS_STDIO 0             // stdio.h not required
#define HAS_PRINTF 0            // own printf()



// align matrix entries to 4
#define align_mem(x) (void *)(4 + (((intptr_t)(x) - 1) & ~3))

extern ee_u32 default_num_contexts;

typedef ee_u32 CORE_TICKS;

typedef struct CORE_PORTABLE_S {
    ee_u8 portable_id;
} core_portable;

void portable_init(core_portable *p, int *argc, char *argv[]);
void portable_fini(core_portable *p);
int ee_printf(const char *fmt, ...);




#ifndef COMPILER_VERSION 
 #ifdef __GNUC__
 #define COMPILER_VERSION "GCC"__VERSION__
 #else
 #define COMPILER_VERSION "Please put compiler version here (e.g. gcc 4.1)"
 #endif
#endif
#ifndef COMPILER_FLAGS 
 #define COMPILER_FLAGS FLAGS_STR /* "Please put compiler flags here (e.g. -o3)" */
#endif
#ifndef MEM_LOCATION 
 #define MEM_LOCATION "STACK"
#endif


#if !defined(PROFILE_RUN) && !defined(PERFORMANCE_RUN) && !defined(VALIDATION_RUN)
#if (TOTAL_DATA_SIZE==1200)
#define PROFILE_RUN 1
#elif (TOTAL_DATA_SIZE==2000)
#define PERFORMANCE_RUN 1
#else
#define VALIDATION_RUN 1
#endif
#endif


#endif
