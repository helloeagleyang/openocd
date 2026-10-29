# SPDX-License-Identifier: GPL-2.0-or-later

SUBDIRS = tcl_commands
DIST_SUBDIRS = tcl_commands

check_PROGRAMS = riscv-program-memory
TESTS = riscv-program-memory
riscv_program_memory_SOURCES = riscv_program_memory.c
AM_CPPFLAGS = -I$(top_builddir) -I$(top_srcdir)/src -I$(top_builddir)/src
if INTERNAL_JIMTCL
AM_CPPFLAGS += -I$(top_srcdir)/jimtcl -I$(top_builddir)/jimtcl
else
AM_CPPFLAGS += $(JIMTCL_CFLAGS)
endif
