/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Stefan Teodorescu <stefanl.teodorescu@gmail.com>
 *
 * Copyright (c) 2020, University Politehnica of Bucharest. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * THIS HEADER MAY NOT BE EXTRACTED OR MODIFIED IN ANY WAY.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>

#include <uk/arch/limits.h>
#include <uk/plat/memory.h>
#include <uk/bitmap.h>
#include <uk/print.h>
#include <uk/assert.h>
#include <page_allocator.h>


static int libc_to_internal_prot(int prot)
{
	int page_prot = PAGE_PROT_NONE;

	if (prot & PROT_READ)
		page_prot |= PAGE_PROT_READ;
	if (prot & PROT_WRITE)
		page_prot |= PAGE_PROT_WRITE;
	if (prot & PROT_EXEC)
		page_prot |= PAGE_PROT_EXEC;

	return page_prot;
}

#define PAGE_ALIGN_UP(vaddr)             ALIGN_UP(vaddr, __PAGE_SIZE)
#define PAGE_ALIGNED(vaddr)              (!((vaddr) & (__PAGE_SIZE - 1)))

//#define MYMMAPPRINT 1
#if MYMMAPPRINT
static void mmap_dbg_print(void *addr, size_t length, int prot, int flags,
		int fd, off_t offset, void *res_addr)
{
	fprintf(stderr, "mmap(");
	if (addr)
		fprintf(stderr, "%p", addr);
	else
		fprintf(stderr, "NULL");
	fprintf(stderr, ", %d, ", length);
	{
		int smth = 0;
		if (prot & PROT_READ) {
			fprintf(stderr, "PROT_READ");
			smth = 1;
		}
		if (prot & PROT_WRITE) {
			if (smth)
				fprintf(stderr, "|");
			fprintf(stderr, "PROT_WRITE");
			smth = 1;
		}
		if (prot & PROT_EXEC) {
			if (smth)
				fprintf(stderr, "|");
			fprintf(stderr, "PROT_EXEC");
			smth = 1;
		}
		if (!smth)
			fprintf(stderr, "PROT_NONE");
		fprintf(stderr, ", ");
	}
	{
		int smth = 0;
		if (flags & MAP_PRIVATE) {
			if (smth)
				fprintf(stderr, "|");
			fprintf(stderr, "MAP_PRIVATE");
			smth = 1;
		}
		if (flags & MAP_FIXED) {
			if (smth)
				fprintf(stderr, "|");
			fprintf(stderr, "MAP_FIXED");
			smth = 1;
		}
		if (flags & MAP_ANONYMOUS) {
			if (smth)
				fprintf(stderr, "|");
			fprintf(stderr, "MAP_ANONYMOUS");
			smth = 1;
		}
		fprintf(stderr, ", ");
	}
	fprintf(stderr, "%d, %d) = %p\n", fd, offset, res_addr);
}
#endif

void *mmap(void *addr, size_t length, int prot, int flags,
		int fd, off_t offset)
{
	void *res_addr;
	unsigned long myprot;
	int rc;

	if (flags & MAP_ANONYMOUS) {
		if (fd != -1 || offset) {
			errno = EINVAL;
			return MAP_FAILED;
		}
	} else {
		/* TODO: We don't currently support mapping files */
		errno = ENOTSUP;
		return MAP_FAILED;
	}


	/* At least one of MAP_SHARED or MAP_PRIVATE has to be specified */
	if (!(flags & MAP_SHARED) && !(flags & MAP_PRIVATE)) {
		errno = EINVAL;
		return MAP_FAILED;
	}

	if (!length) {
		errno = EINVAL;
		return MAP_FAILED;
	}

	length = PAGE_ALIGN_UP(length);
	if (!length) {
		errno = ENOMEM;
		return MAP_FAILED;
	}

	if (flags & MAP_FIXED) {
		/* Discard any overlapping mappings */
		if (munmap(addr, length)) {
			errno = EINVAL;
			return MAP_FAILED;
		}

		res_addr = page_allocator_alloc_fixed(addr, length / __PAGE_SIZE);
		UK_ASSERT(res_addr == addr);

	} else {
		res_addr = page_allocator_alloc(length / __PAGE_SIZE);
		if (!res_addr) {
			errno = ENOMEM;
			return MAP_FAILED;
		}
	}

	if (flags & MAP_ANONYMOUS) {
		/* MAP_ANONYMOUS pages are zeroed out */

		rc = page_allocator_set_prot(res_addr, length / __PAGE_SIZE, PAGE_PROT_WRITE);
		UK_ASSERT(rc == 0);

		/*
		 * XXX: there is a bug when building with performance
		 * optimizations flag that make this memset loop infintely.
		 * Using for loop for now.
		 */
		/* memset((void *) area_to_map, 0, length); */
		for (unsigned long i = 0; i < (length / sizeof(unsigned long)); i++)
			*((unsigned long *) res_addr + i) = 0;

	} else {
		/* TODO: file mapping */
	}

	myprot = libc_to_internal_prot(prot);
	rc = page_allocator_set_prot(res_addr, length / __PAGE_SIZE, myprot);
	UK_ASSERT(rc == 0);

	return res_addr;
}

int munmap(void *addr, size_t length)
{
	if (!PAGE_ALIGNED((unsigned long) addr)) {
		errno = EINVAL;
		return -1;
	}

	if (!length)
		return 0;

	length = PAGE_ALIGN_UP(length);

	return page_allocator_free(addr, length / __PAGE_SIZE);
}

int mprotect(void *addr, size_t length, int prot)
{
	unsigned long myprot;
	int rc;

	if (PAGE_ALIGNED((unsigned long) addr)) {
		errno = EINVAL;
		return -1;
	}

	if (!length)
		return 0;

	if ((prot & PROT_NONE) && (prot != PROT_NONE)) {
		errno = EINVAL;
		return -1;
	}

	length = PAGE_ALIGN_UP(length);
	myprot = libc_to_internal_prot(prot);
	rc = page_allocator_set_prot(addr, length / __PAGE_SIZE, myprot);

	return rc;
}

int msync(void *addr __unused, size_t length __unused, int flags __unused)
{
	errno = ENOTSUP;
	return -1;
}
