/*
 * kdump: Linux Dumps Linux
 *
 * Created by: Vivek Pallantla (Vivek.Pallantla@sonyericsson.com)
 * Copyright (C) IBM Corporation, 2005. All rights reserved
 * Copyright (C) Sony Ericsson Mobile Communications Japan Inc, 2008
 * All rights reserved
 *
 * Copied from crashdump-x86.c
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License).
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include "../include/elf.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "../../kexec.h"
#include "../../kexec-elf.h"
#include "../../kexec-syscall.h"
#include "../../crashdump.h"
#include "crashdump-arm.h"

/* Forward Declaration. */
static int exclude_crash_reserve_region(int *nr_ranges);

/* Stores a sorted list of RAM memory ranges for which to create elf headers.
 * A separate program header is created for backup region */
static struct memory_range crash_memory_range[CRASH_MAX_MEMORY_RANGES];

/* Memory region reserved for storing panic kernel and other data. */
static struct memory_range crash_reserved_mem;

/* Reads the appropriate file and retrieves the SYSTEM RAM regions for whom to
 * create Elf headers. Keeping it separate from get_memory_ranges() as
 * requirements are different in the case of normal kexec and crashdumps.
 *
 * Normal kexec needs to look at all of available physical memory irrespective
 * of the fact how much of it is being used by currently running kernel.
 * Crashdumps need to have access to memory regions actually being used by
 * running  kernel. Expecting a different file/data structure than /proc/iomem
 * to look into down the line. May be something like /proc/kernelmem or may
 * be zone data structures exported from kernel.
 */
static int get_crash_memory_ranges(struct memory_range **range, int *ranges)
{
	const char *iomem = proc_iomem();
	int memory_ranges = 0;
	char line[MAX_LINE];
	FILE *fp;
	unsigned long start, end;
	int i;

	fp = fopen(iomem, "r");
	if (!fp) {
		fprintf(stderr, "Cannot open %s: %s\n",
			iomem, strerror(errno));
		return -1;
	}

	while (fgets(line, sizeof(line), fp) != 0) {
		char *str;
		int type, consumed, count;
		if (memory_ranges >= CRASH_MAX_MEMORY_RANGES)
			break;

		count = sscanf(line, "%lx-%lx : %n",
			&start, &end, &consumed);
		if (count != 2)
			continue;

		str = line + consumed;
		printf("%8lx-%8lx : %s",
			start, end, str);
		/* Only Dumping memory of type System RAM. */
		if (memcmp(str, "System RAM\n", 11) == 0) {
			type = RANGE_RAM;
		} else if (memcmp(str, "Crash kernel\n", 13) == 0) {
				/* Reserved memory region. New kernel can
				 * use this region to boot into. */
				crash_reserved_mem.start = start;
				crash_reserved_mem.end = end;
				crash_reserved_mem.type = RANGE_RAM;
				continue;
		} else {
			continue;
		}

		crash_memory_range[memory_ranges].start = start;
		crash_memory_range[memory_ranges].end = end;
		crash_memory_range[memory_ranges].type = type;
		memory_ranges++;
	}

	fclose(fp);
	if (exclude_crash_reserve_region(&memory_ranges) < 0)
		return -1;

	*range = crash_memory_range;
	*ranges = memory_ranges;
	printf("CRASH MEMORY RANGES\n");
	for (i = 0; i < memory_ranges; i++) {
		start = crash_memory_range[i].start;
		end = crash_memory_range[i].end;
		printf("%8lx-%8lx\n", start, end);
	}

	return 0;
}

/* Removes crash reserve region from list of memory chunks for whom elf program
 * headers have to be created. Assuming crash reserve region to be a single
 * continuous area fully contained inside one of the memory chunks */
static int exclude_crash_reserve_region(int *nr_ranges)
{
	int i, j, tidx = -1;
	unsigned long long cstart, cend;
	struct memory_range temp_region = { };

	/* Crash reserved region. */
	cstart = crash_reserved_mem.start;
	cend = crash_reserved_mem.end;
	for (i = 0; i < (*nr_ranges); i++) {
		unsigned long long mstart, mend;
		mstart = crash_memory_range[i].start;
		mend = crash_memory_range[i].end;
		if (cstart < mend && cend > mstart) {
			if (cstart != mstart && cend != mend) {
				/* Split memory region */
				crash_memory_range[i].end = cstart - 1;
				temp_region.start = cend + 1;
				temp_region.end = mend;
				temp_region.type = RANGE_RAM;
				tidx = i+1;
			} else if (cstart != mstart)
				crash_memory_range[i].end = cstart - 1;
			else
				crash_memory_range[i].start = cend + 1;
		}
	}

	/* Insert split memory region, if any. */
	if (tidx >= 0) {
		printf("\n Warning !! Something wrong ??\n");
		if (*nr_ranges == CRASH_MAX_MEMORY_RANGES) {
			/* No space to insert another element. */
			fprintf(stderr, "Error: Number of crash memory ranges"
					" excedeed the max limit\n");
			return -1;
		}

		for (j = (*nr_ranges - 1); j >= tidx; j--)
			crash_memory_range[j+1] = crash_memory_range[j];
		crash_memory_range[tidx] = temp_region;
		(*nr_ranges)++;
	}

	return 0;
}

/* Converts unsigned long to ascii string. */
static void ultoa(unsigned long i, char *str)
{
	int j = 0, k;
	char tmp;

	do {
		str[j++] = i % 10 + '0';
	} while ((i /= 10) > 0);

	str[j] = '\0';
	/* Reverse the string. */
	for (j = 0, k = strlen(str) - 1; j < k; j++, k--) {
		tmp = str[k];
		str[k] = str[j];
		str[j] = tmp;
	}

}

/* Adds the elfcorehdr= command line parameter to command line. */
static int cmdline_add_elfcorehdr(char *cmdline, unsigned long addr)
{
	int cmdlen, len, align = 1024;
	char str[30], *ptr;

	/* Passing in elfcorehdr=xxxK format. Saves space required in cmdline.
	 * Ensure 1K alignment*/
	if (addr%align)
		return -1;

	addr = addr/align;
	ptr = str;
	strcpy(str, " elfcorehdr=");
	ptr += strlen(str);
	ultoa(addr, ptr);
	strcat(str, "K");
	len = strlen(str);
	cmdlen = strlen(cmdline) + len;
	if (cmdlen > (COMMAND_LINE_SIZE - 1))
		die("Command line overflow\n");

	strcat(cmdline, str);
	printf("Command line after adding elfcorehdr\n");
	printf("%s\n", cmdline);
	return 0;
}

static struct crash_elf_info elf_info32 =
{
	class: ELFCLASS32,
	data: ELFDATA2LSB,
	machine: EM_ARM,
	backup_src_start: BACKUP_SRC_START,
	backup_src_end: BACKUP_SRC_END,
	page_offset: PAGE_OFFSET,
	lowmem_limit: 0,
	get_note_info: 0,
};

/* Loads additional segments in case of a panic kernel is being loaded.
 * for storing elf headers for crash memory image.
 */
int load_crashdump_segments(struct kexec_info *info, char* mod_cmdline,
				unsigned long max_addr, unsigned long min_base)
{
	void *tmp;
	unsigned long sz, elfcorehdr;
	int nr_ranges, align = 1024;
	struct memory_range *mem_range, *memmap_p;

	if (get_crash_memory_ranges(&mem_range, &nr_ranges) < 0)
		return -1;

	/* Assuming ELF32 for ARM */
	/* Create elf header segment and store crash image data. */
	if (crash_create_elf32_headers(info, &elf_info32,
				       crash_memory_range, nr_ranges,
				       &tmp, &sz,
				       ELF_CORE_HEADER_ALIGN) < 0) {

		printf("\n Failed to create Elf Core headers \n");
		return -1;
	}

	/* Hack: With some ld versions (GNU ld version 2.14.90.0.4 20030523),
	 * vmlinux program headers show a gap of two pages between bss segment
	 * and data segment but effectively kernel considers it as bss segment
	 * and overwrites the any data placed there. Hence bloat the memsz of
	 * elf core header segment to 16K to avoid being placed in such gaps.
	 * This is a makeshift solution until it is fixed in kernel.
	 */
	elfcorehdr = add_buffer(info, tmp, sz, 16*1024, align, min_base,
							max_addr, -1);
	printf("Created elf header segment at 0x%lx\n", elfcorehdr);
	cmdline_add_elfcorehdr(mod_cmdline, elfcorehdr);
	return 0;
}

int is_crashkernel_mem_reserved(void)
{
	uint64_t start, end;

	return parse_iomem_single("Crash kernel\n", &start, &end) == 0 ?
	  (start != end) : 0;
}

int get_crashkernel_mem_range(uint64_t *start, uint64_t *end)
{
	return parse_iomem_single("Crash kernel\n", start, end);
}
