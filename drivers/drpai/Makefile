# SPDX-License-Identifier: GPL-2.0
#
# Makefile for DRP-AI/DRP.
#

ifdef CONFIG_ARCH_R9A09G056
CFLAGS_lock_drpai.o += -isystem $(shell $(CC) -print-file-name=include) -mno-outline-atomics
obj-$(CONFIG_DRPAI)	+= drpai-core.o drpai-if.o lock_drpai.o
else
obj-$(CONFIG_DRPAI)	+= drpai-core.o drpai-if.o
endif
