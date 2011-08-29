LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	kexec/kexec.c kexec/ifdown.c kexec/kexec-elf.c \
	kexec/kexec-elf-exec.c kexec/kexec-elf-core.c \
	kexec/kexec-iomem.c kexec/firmware_memmap.c \
	kexec/crashdump.c kexec/crashdump-xen.c \
	kexec/phys_arch.c kexec/proc_iomem.c \
	kexec/virt_to_phys.c kexec/add_segment.c \
	kexec/add_buffer.c kexec/arch_reuse_initrd.c \
	kexec/arch/arm/kexec-elf-rel-arm.c kexec/arch/arm/kexec-zImage-arm.c \
	kexec/arch/arm/crashdump-arm.c \
	kexec/arch/arm/kexec-arm.c kexec/purgatory.c

LOCAL_CFLAGS :=  -g -O0 -fno-strict-aliasing -Wall -Wstrict-prototypes
LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/kexec/arch/arm/include/  \
	$(LOCAL_PATH)/include/ \
	$(KERNEL_HEADERS)

LOCAL_MODULE := kexec-tool
include $(BUILD_EXECUTABLE)

