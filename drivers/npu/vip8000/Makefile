#
# This enables the HDMI driver for C-SKY.
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

#EXTRA_CFLAGS += -Werror -O0 -g
EXTRA_CFLAGS += -DLINUX -DDRIVER
EXTRA_CFLAGS += -DDBG=0
EXTRA_CFLAGS += -DNO_DMA_COHERENT
EXTRA_CFLAGS += -DVIVANTE_PROFILER=1
EXTRA_CFLAGS += -DVIVANTE_PROFILER_CONTEXT=1
EXTRA_CFLAGS += -DUSE_PLATFORM_DRIVER=1
EXTRA_CFLAGS += -DgcdENABLE_3D=1
EXTRA_CFLAGS += -DgcdNONPAGED_MEMORY_BUFFERABLE=1
EXTRA_CFLAGS += -DgcdFPGA_BUILD=1
EXTRA_CFLAGS += -DgcdENABLE_TRUST_APPLICATION=1

obj-$(CONFIG_DMA_SHARED_BUFFER) += gc_hal_kernel_allocator_dmabuf.o
obj-$(CONFIG_IOMMU_SUPPORT) += gc_hal_kernel_iommu.o
obj-$(CONFIG_DRM) += gc_hal_kernel_drm.o
obj-$(CONFIG_SYNC_FILE) += gc_hal_kernel_sync.o
ifeq ($(CONFIG_CSKY_NPU),m)
obj-m += vip8000_galcore.o
else ifeq ($(CONFIG_CSKY_NPU),y)
obj-m += vip8000_galcore.o
endif
#obj-$(CONFIG_CSKY_NPU) += vip8000_galcore.o
vip8000_galcore-y := gc_hal_kernel_device.o \
		     gc_hal_kernel_linux.o \
		     gc_hal_kernel_math.o \
		     gc_hal_kernel_os.o \
		     gc_hal_kernel_debugfs.o \
		     gc_hal_kernel_allocator.o \
		     gc_hal_kernel_allocator_user_memory.o \
		     gc_hal_kernel_allocator_dma.o \
		     gc_hal_kernel_allocator_gfp.o \
		     gc_hal_kernel_allocator_reserved_mem.o \
		     gc_hal_kernel_driver.o \
		     gc_hal_kernel.o \
		     gc_hal_kernel_command.o \
		     gc_hal_kernel_async_command.o \
		     gc_hal_kernel_db.o \
		     gc_hal_kernel_debug.o \
		     gc_hal_kernel_event.o \
		     gc_hal_kernel_heap.o \
		     gc_hal_kernel_mmu.o \
		     gc_hal_kernel_video_memory.o \
		     gc_hal_kernel_power.o \
		     gc_hal_kernel_security_v1.o \
		     gc_hal_kernel_context.o \
		     gc_hal_kernel_hardware.o \
		     gc_hal_kernel_security_channel_emulator.o \
		     gc_hal_kernel_platform_default.o \
		     gc_hal_ta.o \
		     gc_hal_ta_hardware.o \
		     gc_hal_ta_mmu.o \
		     gc_hal_ta_emulator.o \
		     gc_hal_kernel_recorder.o
