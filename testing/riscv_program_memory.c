/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "config.h"
#include <stdio.h>
#include "target/riscv/program.h"

#define CHECK(condition) do { \
	if (!(condition)) { \
		fprintf(stderr, "check failed at line %u: %s\n", __LINE__, #condition); \
		return 1; \
	} \
} while (0)

int main(void)
{
	static const struct {
		unsigned int progbufsize;
		bool impebreak;
		bool has_c;
		unsigned int data_words;
		uint32_t autoexec_mask;
		unsigned int expected_words;
		bool expected_compressed;
	} cases[] = {
		/* Common 2/4/8-entry buffers. */
		{2, true, false, 4, 0xe, 1, false},
		{2, true, true, 4, 0xe, 1, false},
		{4, false, false, 4, 0xe, 1, false},
		{4, true, false, 4, 0xe, 1, false},
		{4, false, true, 4, 0xe, 2, true},
		{8, false, false, 4, 0xe, 3, false},
		{8, true, false, 4, 0xe, 3, false},
		{8, false, true, 4, 0xe, 4, true},
		{8, true, true, 4, 0xe, 4, true},
		/* Smaller data windows and missing optional triggers. */
		{8, true, true, 3, 0xe, 3, false},
		{8, true, true, 2, 0xe, 2, false},
		{8, true, true, 1, 0xe, 1, false},
		{8, true, true, 0, 0xe, 1, false},
		{8, true, true, 4, 0x6, 3, false},
		{8, true, true, 4, 0x2, 2, false},
		{8, true, false, 4, 0x8, 1, false},
		{8, true, true, 4, 0x0, 1, false},
		/* Exact buffer boundaries with and without implicit termination. */
		{5, false, true, 4, 0xe, 2, true},
		{5, true, true, 4, 0xe, 3, true},
		{6, false, true, 4, 0xe, 3, true},
		{7, true, false, 4, 0xe, 3, false},
		{9, true, false, 4, 0xe, 4, false},
		{10, false, false, 4, 0xe, 4, false},
	};
	for (unsigned int i = 0; i < ARRAY_SIZE(cases); ++i) {
		bool compressed;
		CHECK(riscv_program_memory_words(cases[i].progbufsize, cases[i].impebreak,
				cases[i].has_c, cases[i].data_words, cases[i].autoexec_mask, &compressed) == cases[i].expected_words);
		CHECK(compressed == cases[i].expected_compressed);
	}

	/* Independently tabulated instruction storage requirements for each
	 * emitted layout, including explicit termination and padding. */
	const unsigned int ordinary[] = {0, 0, 6, 8, 10};
	const unsigned int compact[] = {0, 0, 4, 6, 7};
	unsigned int count = 0;
	for (unsigned int size = 0; size <= 16; ++size) {
		for (unsigned int flags = 0; flags < 4; ++flags) {
			const bool implicit = flags & 1;
			const bool c = flags & 2;
			for (unsigned int data = 0; data <= 4; ++data) {
				for (uint32_t mask = 0; mask < 16; ++mask) {
					bool compressed;
					unsigned int words = riscv_program_memory_words(size, implicit,
							c, data, mask, &compressed);
					CHECK(words >= 1 && words <= 4);
					if (words > 1) {
						CHECK(words <= data && (mask & BIT(words - 1)));
						if (compressed) {
							CHECK(c && (compact[words] <= size ||
									(words == 3 && size == 5 && implicit)));
							CHECK(ordinary[words] > size + implicit);
						} else {
							CHECK(ordinary[words] <= size + implicit);
						}
					} else {
						CHECK(!compressed);
					}
					for (unsigned int wider = words + 1; wider <= MIN(data, 4); ++wider) {
						if (!(mask & BIT(wider - 1)))
							continue;
						CHECK(ordinary[wider] > size + implicit);
						CHECK(!c || (compact[wider] > size &&
								!(wider == 3 && size == 5 && implicit)));
					}
					++count;
				}
			}
		}
	}
	printf("%u capability combinations and %zu layout cases passed\n", count, ARRAY_SIZE(cases));
	return 0;
}
