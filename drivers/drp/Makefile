# SPDX-License-Identifier: GPL-2.0
#
# Makefile for DRP/DRP.
#
# enable <stddef.h> for lock_drp.o
CFLAGS_lock_drp.o += -isystem $(shell $(CC) -print-file-name=include) -mno-outline-atomics
obj-$(CONFIG_DRP)	+= drp-core.o drp-if.o lock_drp.o


