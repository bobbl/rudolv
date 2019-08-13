/* RIPE was originally developed by John Wilander (@johnwilander)
 * and was debugged and extended by Nick Nikiforakis (@nicknikiforakis)
 *
 * RISC-V port developed by John Merrill
 *
 * Released under the MIT license (see file named LICENSE)
 *
 * This program is part the paper titled
 * RIPE: Runtime Intrusion Prevention Evaluator 
 * Authored by: John Wilander, Nick Nikiforakis, Yves Younan,
 *              Mariam Kamkar and Wouter Joosen
 * Published in the proceedings of ACSAC 2011, Orlando, Florida
 *
 * Please cite accordingly.
 */

/**
 * @author John Wilander
 * 2007-01-16
 */

#ifndef RIPE_ATTACK_PARAMETERS_H
#define RIPE_ATTACK_PARAMETERS_H

#define ATTACK_IMPOSSIBLE -900
#define ATTACK_NOT_IMPLEMENTED -909

/* Enumerations for typing of attack form parameters                        */
/* Each enumeration has its own integer space to provide better type safety */
enum techniques    {DIRECT=100, INDIRECT};
enum inject_params {INJECTED_CODE_NO_NOP=200, RETURN_INTO_LIBC,
			RETURN_ORIENTED_PROGRAMMING, DATA_ONLY};

enum code_ptrs     {RET_ADDR=300, FUNC_PTR_STACK_VAR, FUNC_PTR_STACK_PARAM,
		    FUNC_PTR_HEAP, FUNC_PTR_BSS, FUNC_PTR_DATA,
		    LONGJMP_BUF_STACK_VAR, LONGJMP_BUF_STACK_PARAM,
            LONGJMP_BUF_HEAP, LONGJMP_BUF_BSS, LONGJMP_BUF_DATA,
		    STRUCT_FUNC_PTR_STACK,STRUCT_FUNC_PTR_HEAP,
		    STRUCT_FUNC_PTR_DATA,STRUCT_FUNC_PTR_BSS, VAR_BOF, VAR_IOF, VAR_LEAK};
enum locations     {STACK=400, HEAP, BSS, DATA};
enum functions     {MEMCPY=500, STRCPY, STRNCPY, SPRINTF, SNPRINTF,
                    STRCAT, STRNCAT, SSCANF, HOMEBREW};

/* 2 overflow techniques */
size_t nr_of_techniques = 2;
char *opt_techniques[] = {"direct", "indirect"};

/* 4 types of injection parameters */
size_t nr_of_inject_params = 4;
char *opt_inject_params[] = {"shellcode", "returnintolibc", "rop", "dataonly"};

/* 18 code pointers to overwrite */
size_t nr_of_code_ptrs = 18;
char *opt_code_ptrs[] = {"ret", "funcptrstackvar", "funcptrstackparam",
			 "funcptrheap", "funcptrbss", "funcptrdata",
			 "longjmpstackvar", "longjmpstackparam",
			 "longjmpheap", "longjmpbss", "longjmpdata",
			 "structfuncptrstack","structfuncptrheap",
             "structfuncptrdata","structfuncptrbss", "bof", "iof", "leak"};

/* 4 memory locations */
size_t nr_of_locations = 4;
char *opt_locations[] = {"stack", "heap", "bss", "data"};

/* 9 vulnerable functions */
size_t nr_of_funcs = 10;
char *opt_funcs[] = {"memcpy", "strcpy", "strncpy", "sprintf", "snprintf",
		     "strcat", "strncat", "sscanf", "homebrew"};

#endif /* !RIPE_ATTACK_PARAMETERS_H */
