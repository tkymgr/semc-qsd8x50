/*
 * - 08/21/2007 ATAG support added by Uli Luckas <u.luckas@road.de>
 *
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <getopt.h>
#include <arch/options.h>
#include <asm/page.h>
#include "../../kexec.h"
#include "../../kexec-syscall.h"

#define COMMAND_LINE_SIZE 1024
#define BOOT_PARAMS_SIZE 1536

struct tag_header {
	uint32_t size;
	uint32_t tag;
};

/* The list must start with an ATAG_CORE node */
#define ATAG_CORE       0x54410001

struct tag_core {
	uint32_t flags;	    /* bit 0 = read-only */
	uint32_t pagesize;
	uint32_t rootdev;
};

/* it is allowed to have multiple ATAG_MEM nodes */
#define ATAG_MEM	0x54410002

struct tag_mem32 {
	uint32_t   size;
	uint32_t   start;  /* physical start address */
};

/* describes where the compressed ramdisk image lives (virtual address) */
/*
 * this one accidentally used virtual addresses - as such,
 * it's deprecated.
 */
#define ATAG_INITRD     0x54410005

/* describes where the compressed ramdisk image lives (physical address) */
#define ATAG_INITRD2    0x54420005

/* 5MB offset from crash memory base */
#define INITRD_OFFSET       0x500000

/* 4KB offset from crash memory base */
#define ATAG_OFFSET         0x1000

/* kernel entry point is always 0x8000 offset from PHYS_OFFSET */
#define KERNEL_START_OFFSET 0x8000

struct tag_initrd {
	uint32_t start;    /* physical start address */
	uint32_t size;     /* size of compressed ramdisk image in bytes */
};

/* command line: \0 terminated string */
#define ATAG_CMDLINE    0x54410009

struct tag_cmdline {
	char    cmdline[1];     /* this is the minimum size */
};

/* The list ends with an ATAG_NONE node. */
#define ATAG_NONE       0x00000000

struct tag {
	struct tag_header hdr;
	union {
		struct tag_core	 core;
		struct tag_mem32	mem;
		struct tag_initrd       initrd;
		struct tag_cmdline      cmdline;
	} u;
};

#define tag_next(t)     ((struct tag *)((uint32_t *)(t) + (t)->hdr.size))
#define byte_size(t)    ((t)->hdr.size << 2)
#define tag_size(type)  ((sizeof(struct tag_header) + sizeof(struct type) + 3) >> 2)

int zImage_arm_probe(const char *buf, off_t len)
{
	/*
	 * Only zImage loading is supported. Do not check if
	 * the buffer is valid kernel image
	 */
	return 0;
}

void zImage_arm_usage(void)
{
	printf(	"     --command-line=STRING Set the kernel command line to STRING.\n"
		"     --append=STRING       Set the kernel command line to STRING.\n"
		"     --initrd=FILE         Use FILE as the kernel's initial ramdisk.\n"
		"     --ramdisk=FILE        Use FILE as the kernel's initial ramdisk.\n"
		);
}

static
struct tag * atag_read_tags(void)
{
	static unsigned long buf[BOOT_PARAMS_SIZE];
	const char fn[]= "/proc/atags";
	FILE *fp;
	fp = fopen(fn, "r");
	if (!fp) {
		fprintf(stderr, "Cannot open %s: %s\n",
			fn, strerror(errno));
		return NULL;
	}

	fread(buf, sizeof(buf[1]), BOOT_PARAMS_SIZE, fp);
	if (ferror(fp)) {
		fprintf(stderr, "Cannot read %s: %s\n",
			fn, strerror(errno));
		fclose(fp);
		return NULL;
	}

	fclose(fp);
	return (struct tag *) buf;
}


static
int atag_arm_load(struct kexec_info *info, unsigned long base,
	const char *command_line, off_t command_line_len,
	const char *initrd, off_t initrd_len)
{
	struct tag *saved_tags = atag_read_tags();
	char *buf, *cmdline_buf;
	off_t len, cmdline_len = 0;
	struct tag *params;
	uint32_t *initrd_start;
	int i;

	buf = xmalloc(getpagesize());
	if (!buf) {
		fprintf(stderr, "Compiling ATAGs: out of memory\n");
		return -1;
	}

	cmdline_buf = xmalloc(COMMAND_LINE_SIZE);
	if (!cmdline_buf) {
		fprintf(stderr, "Compiling Command line: out of memory\n");
		return -1;
	}

	memset(buf, 0xff, getpagesize());
	memset((void *)cmdline_buf, 0, COMMAND_LINE_SIZE);
	params = (struct tag *)buf;
	if (saved_tags) {
		// Copy tags
		saved_tags = (struct tag *) saved_tags;
		while(byte_size(saved_tags)) {
			switch (saved_tags->hdr.tag) {
			case ATAG_INITRD:
			case ATAG_INITRD2:
			case ATAG_NONE:
				// skip these tags
				break;
			case ATAG_CMDLINE:
				cmdline_len = strlen(saved_tags->u.cmdline.cmdline) + 1;
				if(cmdline_len > COMMAND_LINE_SIZE)
					die("Command line overflow\n");
				strncpy(cmdline_buf, saved_tags->u.cmdline.cmdline, COMMAND_LINE_SIZE);
				cmdline_buf[COMMAND_LINE_SIZE - 1] = '\0';
				break;
			default:
				// copy all other tags
				memcpy(params, saved_tags, byte_size(saved_tags));
				params = tag_next(params);
			}
			saved_tags = tag_next(saved_tags);
		}
	} else {
		params->hdr.size = 2;
		params->hdr.tag = ATAG_CORE;
		params = tag_next(params);
	}

	if (initrd) {
		params->hdr.size = tag_size(tag_initrd);
		params->hdr.tag = ATAG_INITRD2;
		initrd_start = &params->u.initrd.start;
		params->u.initrd.size = initrd_len;
		params = tag_next(params);
	}

	if (command_line) {
		command_line_len = command_line_len + cmdline_len -1;
		if(command_line_len > COMMAND_LINE_SIZE)
			die("Command line overflow\n");
		strncat(cmdline_buf, command_line, COMMAND_LINE_SIZE - 1);
		params->hdr.size = (sizeof(struct tag_header) + command_line_len + 3) >> 2;
		params->hdr.tag = ATAG_CMDLINE;
		memcpy(params->u.cmdline.cmdline, cmdline_buf,
			command_line_len);
		params->u.cmdline.cmdline[command_line_len - 1] = '\0';
		params = tag_next(params);
	}

	params->hdr.size = 0;
	params->hdr.tag = ATAG_NONE;
	len = ((char *)params - buf) + sizeof(struct tag_header);

	add_segment(info, buf, len, base, len);

	if (initrd) {
	/* Load initrd at 4MB offset from the start of the capture kernel */
		*initrd_start = base - ATAG_OFFSET + INITRD_OFFSET;
		add_segment(info, initrd, initrd_len, *initrd_start, initrd_len);
	}

	return 0;
}

int zImage_arm_load(int argc, char **argv, const char *buf, off_t len,
	struct kexec_info *info)
{
	unsigned long base;
	unsigned int atag_offset = ATAG_OFFSET;              /* 4k offset from memory start */
	unsigned int offset = KERNEL_START_OFFSET;           /* 32k offset from memory start */
	const char *command_line;
	off_t command_line_len;
	const char *ramdisk;
	char *ramdisk_buf;
	char *modified_cmdline;
	off_t ramdisk_length;
	int opt, result;
  #define OPT_APPEND	'a'
#define OPT_RAMDISK	'r'
	static const struct option options[] = {
		KEXEC_ARCH_OPTIONS
		{ "command-line",	1, 0, OPT_APPEND },
		{ "append",		1, 0, OPT_APPEND },
		{ "initrd",		1, 0, OPT_RAMDISK },
		{ "ramdisk",		1, 0, OPT_RAMDISK },
		{ 0, 			0, 0, 0 },
	};
	static const char short_options[] = KEXEC_ARCH_OPT_STR "a:r:";

	/*
	 * Parse the command line arguments
	 */
	command_line = 0;
	command_line_len = 0;
	ramdisk = 0;
	ramdisk_buf = 0;
	ramdisk_length = 0;
	while((opt = getopt_long(argc, argv, short_options, options, 0)) != -1) {
		switch(opt) {
		default:
			/* Ignore core options */
			if (opt < OPT_ARCH_MAX) {
				break;
			}
		case '?':
			usage();
			return -1;
		case OPT_APPEND:
			command_line = optarg;
			break;
		case OPT_RAMDISK:
			ramdisk = optarg;
			break;
		}
	}
	if (command_line) {
		command_line_len = strlen(command_line) + 1;
		if (command_line_len > COMMAND_LINE_SIZE)
			command_line_len = COMMAND_LINE_SIZE;
	}
	if (ramdisk) {
		ramdisk_buf = slurp_file(ramdisk, &ramdisk_length);
	}

	/* This code is just to load the elf headers for /proc/vmcore */
	if (info->kexec_flags & KEXEC_ON_CRASH) {
		modified_cmdline = xmalloc(COMMAND_LINE_SIZE);
		memset((void *)modified_cmdline, 0, COMMAND_LINE_SIZE);
		if (command_line) {
			strncpy(modified_cmdline, command_line,
					COMMAND_LINE_SIZE);
			modified_cmdline[COMMAND_LINE_SIZE - 1] = '\0';
		}

		/* If panic kernel is being loaded, additional segments need
		 * to be created. load_crashdump_segments will take care of
		 * loading the segments as high in memory as possible, hence
		 * in turn as away as possible from kernel to avoid being
		 * stomped by the kernel.
		 */
		if (load_crashdump_segments(info, modified_cmdline, -1, 0) < 0)
			return -1;

		/* Use new command line buffer */
		command_line = modified_cmdline;
		command_line_len = strlen(command_line) + 1;
	}

	//base = locate_hole(info,len+offset,0,0,ULONG_MAX,INT_MAX);
        result = get_crashkernel_mem_range(&base, NULL);
        if(result < 0) {
            die("get_crashkernel_mem_range failed");
        }
	if (base == ULONG_MAX)
		return -1;

	if (atag_arm_load(info, base + atag_offset,
			 command_line, command_line_len,
			 ramdisk_buf, ramdisk_length)    == -1)
		return -1;

	add_segment(info, buf, len, base + offset, len);
	info->entry = (void*)base + offset;
	return 0;
}
