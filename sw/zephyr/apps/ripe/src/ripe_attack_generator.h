/* RIPE was originally developed by John Wilander (@johnwilander)
 * and was debugged and extended by Nick Nikiforakis (@nicknikiforakis)
 *
 * The RISC-V port of RIPE was developed by John Merrill.
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

#ifndef RIPE_ATTACK_GENERATOR_H
#define RIPE_ATTACK_GENERATOR_H
/*
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <limits.h>
#include <stdint.h>
#include <setjmp.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
*/
// ASA
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <setjmp.h>

#include <zephyr.h>
#include <misc/printk.h>

#include "ripe_attack_parameters.h"
#define fprintf(dest,str...) printk(str)
#define printf(str...) printk(str)
// END ASA

typedef int boolean;
enum booleans {FALSE=0, TRUE};

const char *bin4b[16] = {"0000", "0001", "0010", "0011",
		     	 "0100", "0101", "0110", "0111",
		      	 "1000", "1001", "1010", "1011",
	 	      	 "1100", "1101", "1110", "1111"};

typedef struct attack_form ATTACK_FORM;
struct attack_form {
        enum techniques technique;
        enum inject_params inject_param;
        enum code_ptrs code_ptr;
        enum locations location;
        enum functions function;
};

typedef struct char_payload CHARPAYLOAD;
struct char_payload {
        enum inject_params inject_param;
        size_t size;
        void *overflow_ptr; /* Points to code pointer (direct attack) */
                            /* or general pointer (indirect attack)   */
        char *buffer;

        jmp_buf *jmp_buffer;

        long stack_jmp_buffer_param;
        size_t offset_to_copied_base_ptr;
        size_t offset_to_fake_return_addr;
        long *fake_return_addr;
        long *ptr_to_correct_return_addr;
};

struct attackme {
        char buffer[256];
        int (*func_ptr)(const char *, int);
};

/**
 * main
 * -t technique
 * -i injection parameter (code + NOP / return-into-libc / param to system())
 * -c code pointer
 * -l memory location
 * -f function to overflow with
 * -d output debug info
 * -o set output stream
 */
// ASA int main(int argc, char **argv);
void main(void);

void perform_attack(
             		int (*stack_func_ptr_param)(const char *),
                    jmp_buf stack_jmp_buffer_param);

/* BUILD_PAYLOAD()                                                  */
/*                                                                  */
/* Simplified example of payload (exact figures are just made up):  */
/*                                                                  */
/*   size      = 31 (the total payload size)                        */
/*   size_sc   = 12 (size of shellcode incl NOP)                    */
/*   size_addr = 4  (size of address to code)                       */
/*   size_null = 1  (size of null termination)                      */
/*                                                                  */
/*    ------------ ----------------- ------------- -                */
/*   | Shell code | Padded bytes    | Address     |N|               */
/*   | including  |                 | back to     |u|               */
/*   | optional   |                 | NOP sled or |l|               */
/*   | NOP sled   |                 | shell code  |l|               */
/*    ------------ ----------------- ------------- -                */
/*    |          | |               | |           | |                */
/*    0         11 12             25 26         29 30               */
/*              /   \             /   \             \               */
/*     size_sc-1     size_sc     /     \             size-size_null */
/*                              /       \                           */
/*  (size-1)-size_addr-size_null         size-size_addr-size_null   */
/*                                                                  */
/* This means that we should pad with                               */
/* size - size_sc - size_addr - size_null = 31-12-4-1 = 14 bytes    */
/* and start the padding at index size_sc                           */
boolean build_payload(CHARPAYLOAD *payload);

void set_technique(char *choice);
void set_inject_param(char *choice);
void set_code_ptr(char *choice);
void set_location(char *choice);
void set_function(char *choice);

int dummy_function(const char *str) {
        printf("Dummy function\n");
        return 0;
}

boolean is_attack_possible();
void homebrew_memcpy(void *dst, const void *src, size_t len);

/*
RIPE shellcode uses the following instructions:
la <reg>, <addr of shellcode_func()>
jalr <reg>

The first la instruction is disassembled to:
lui <reg>, <upper 20 bits>
addi <reg>, <reg>, <lower 12 bits>

Thus, the shellcode follows the pattern
shown in the following encodings:

LUI: xxxx xxxx xxxx xxxx xxxx xxxx x011 0111
     \                  / \    /\      /
             imm value         reg#  opcode


ADDI: xxxx xxxx xxxx xxxx x000 xxxx x011 0011
      \        / \    /    \    /\      /
        imm value     reg#      reg#  opcode


JALR: 0000 0000 0000 xxxx x000 0000 1110 0111
                     \    /          \      /
                      reg#            opcode

The shellcode is formatted so that:
  1. All instructions are stored to a single string
  1. Byte order is converted to little-endian
*/
void build_shellcode(char *shellcode);
void hex_to_string(char *str, size_t val);
void format_instruction(char *dest, size_t insn);

const char *hex_to_bin(char c) {
	if (c >= '0' && c <= '9') return bin4b[c - '0'];
	if (c >= 'a' && c <= 'f') return bin4b[10 + c - 'a'];
	return NULL;
}

#endif /* !RIPE_ATTACK_GENERATOR_H */
