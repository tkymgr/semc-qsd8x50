#ifndef CRASHDUMP_ARM_H
#define CRASHDUMP_ARM_H

struct kexec_info;
int load_crashdump_segments(struct kexec_info *info, char *mod_cmdline,
				unsigned long max_addr, unsigned long min_base);
#if 0
#define PAGE_OFFSET	0x10000000 /*For MSM*/
#define __pa(x)		((unsigned long)(x)-PAGE_OFFSET)

#define __VMALLOC_RESERVE       (128 << 20)
#define MAXMEM                  (-PAGE_OFFSET-__VMALLOC_RESERVE)
#endif

#define	PAGE_OFFSET	0x10000000
#define MAX_MEMORY_RANGES 64
#define COMMAND_LINE_SIZE 1024
#define CRASH_MAX_MEMMAP_NR	(KEXEC_MAX_SEGMENTS + 1)
#define CRASH_MAX_MEMORY_RANGES	(MAX_MEMORY_RANGES + 2)

/* Backup Region, None For ARM. */
#define BACKUP_SRC_START	0x00000000
#define BACKUP_SRC_END		0x00000000
#define BACKUP_SRC_SIZE	(BACKUP_SRC_END - BACKUP_SRC_START + 1)

#endif /* CRASHDUMP_ARM_H */
