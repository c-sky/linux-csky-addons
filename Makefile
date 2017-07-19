#
# Makefile for the Linux kernel addon device drivers.
#
KBUILD_CFLAGS   += -O0

obj-y += drivers/
obj-y += sound/soc/csky/
obj-y += sound/soc/codecs/
