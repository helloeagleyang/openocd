/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_PROGRAM_H
#define OPENOCD_TARGET_RISCV_PROGRAM_H

#include "riscv.h"

#define RISCV013_MAX_PROGBUF_SIZE 16

/* Maximum group width implemented by the bulk memory layouts. */
#define RISCV_PROGBUF_MAX_MEMORY_WORDS 4

enum riscv_progbuf_exec_result {
	RISCV_PROGBUF_EXEC_RESULT_NOT_EXECUTED,
	RISCV_PROGBUF_EXEC_RESULT_UNKNOWN,
	RISCV_PROGBUF_EXEC_RESULT_EXCEPTION,
	RISCV_PROGBUF_EXEC_RESULT_UNKNOWN_ERROR,
	RISCV_PROGBUF_EXEC_RESULT_SUCCESS
};

/* The various RISC-V debug specifications all revolve around setting up
 * program buffers and executing them on the target.  This structure contains a
 * single program, which can then be executed on targets.  */
struct riscv_program {
	struct target *target;

	uint32_t progbuf[RISCV013_MAX_PROGBUF_SIZE];

	/* Number of populated 32-bit Program Buffer words. */
	unsigned int instruction_count;

	/* execution result of the program */
	/* TODO: remove this field. We should make it a parameter to riscv_program_exec */
	enum riscv_progbuf_exec_result execution_result;
};

/* Initializes a program with the header. */
int riscv_program_init(struct riscv_program *p, struct target *t);

/* Write the program to the program buffer. */
int riscv_program_write(struct riscv_program *program);

/* Executes a program, returning 0 if the program successfully executed.  Note
 * that this may cause registers to be saved or restored, which could result to
 * calls to things like riscv013_reg_save which itself could require a
 * program to execute.  That's OK, just make sure this eventually terminates.
 * */
int riscv_program_exec(struct riscv_program *p, struct target *t);

/* A lower level interface, you shouldn't use this unless you have a reason. */
int riscv_program_insert(struct riscv_program *p, riscv_insn_t i);

/* Helpers to assemble various instructions.  Return 0 on success.  These might
 * assemble into a multi-instruction sequence that overwrites some other
 * register, but those will be properly saved and restored. */
int riscv_program_ldr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int16_t o);
int riscv_program_lwr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int16_t o);
int riscv_program_lhr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int16_t o);
int riscv_program_lbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int16_t o);
int riscv_program_load(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int16_t o,
		unsigned int s);

int riscv_program_sdr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int16_t o);
int riscv_program_swr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int16_t o);
int riscv_program_shr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int16_t o);
int riscv_program_sbr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int16_t o);
int riscv_program_store(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int16_t o,
		unsigned int s);

int riscv_program_csrrsi(struct riscv_program *p, enum gdb_regno d, uint8_t z, enum gdb_regno csr);
int riscv_program_csrrci(struct riscv_program *p, enum gdb_regno d, uint8_t z, enum gdb_regno csr);
int riscv_program_csrr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno csr);
int riscv_program_csrw(struct riscv_program *p, enum gdb_regno s, enum gdb_regno csr);

int riscv_program_fence_i(struct riscv_program *p);
int riscv_program_fence_rw_rw(struct riscv_program *p);
int riscv_program_ebreak(struct riscv_program *p);

int riscv_program_addi(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, int16_t i);

/* Select a bulk layout using 32-bit Program Buffer entries and data words.
 * autoexec contains the implemented ABSTRACTAUTO autoexecdata bits; only the
 * last data register in a group triggers execution. The caller has already
 * limited data_words to the usable hart-visible memory window.
 *
 * Try wider groups first, using ordinary instructions when they fit that
 * width. Compressed instructions are worthwhile here only if they permit a
 * wider group. Return 1 to retain the existing single-word implementation;
 * this helper does not construct or change that implementation. */
static inline unsigned int riscv_program_memory_words(unsigned int progbufsize,
		bool impebreak, bool has_c, unsigned int data_words, uint32_t autoexec,
		bool *compressed)
{
	*compressed = false;
	for (unsigned int words = MIN(data_words, RISCV_PROGBUF_MAX_MEMORY_WORDS); words >= 2; --words) {
		if (!(autoexec & BIT(words - 1)))
			continue;
		/* Two instructions per data word, then addi and termination.
		 * An implicit ebreak can occupy the entry just beyond the buffer. */
		if (2 * words + 2 <= progbufsize + impebreak)
			return words;
		if (!has_c)
			continue;
		/* Each load/store pair is 6 bytes, followed by a 2-byte c.addi.
		 * Add c.ebreak and round up to complete 32-bit entries. It may
		 * be omitted only when the body ends exactly at implicit ebreak. */
		const unsigned int body_bytes = 6 * words + 2;
		if (DIV_ROUND_UP(body_bytes + 2, 4) <= progbufsize ||
				(impebreak && body_bytes == 4 * progbufsize)) {
			*compressed = true;
			return words;
		}
	}
	return 1;
}

#endif /* OPENOCD_TARGET_RISCV_PROGRAM_H */
