# C-SKY mailbox drivers
ccflags-$(CONFIG_DEBUG_MAILBOX) += -O0
ccflags-$(CONFIG_DEBUG_MAILBOX)	+= -DDEBUG

obj-$(CONFIG_MAILBOX_CSKY)	+= mailbox-csky.o
obj-$(CONFIG_MAILBOX_CSKY)	+= mailbox-client-csky.o
obj-$(CONFIG_TTY_MAILBOX_CSKY)	+= tty-mailbox-csky.o
obj-$(CONFIG_TTY_MAILBOX_CSKY)	+= tty-mailbox-client-csky.o

