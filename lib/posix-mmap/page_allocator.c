/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Costin Lupu <costin.lupu@cs.pub.ro>
 *
 * Copyright (c) 2021, University Politehnica of Bucharest. All rights reserved.
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
 */

#include <stdio.h>
#include <uk/arch/limits.h>
#include <uk/plat/memory.h>
#include <uk/libparam.h>
#include <uk/bitmap.h>
#include <uk/mutex.h>
#include <page_allocator.h>

static __u64 percent;
UK_LIB_PARAM(percent, __u64);

struct page_allocator {
	void *start_addr;
	void *end_addr;
	unsigned long pages_num;
	unsigned long allocated;
	long search_start_pos;
	long search_last_free_pos;
	long search_last_free_num;
	void *bm;
	struct uk_mutex m;
};

struct page_allocator page_allocator;

#define KB    (1024)
#define MB    (1024 * KB)
#define GB    (1024 * MB)

static void memsz_to_str(__sz memsz, char *buf, int bufsz)
{
	unsigned long val;

	if (memsz > GB) {
		val = memsz / GB;
		snprintf(buf, bufsz, "%luGB", val);
	} else if (memsz > MB) {
		val = memsz / MB;
		snprintf(buf, bufsz, "%luMB", val);
	} else if (memsz > KB) {
		val = memsz / KB;
		snprintf(buf, bufsz, "%luKB", val);
	} else {
		val = memsz;
		snprintf(buf, bufsz, "%luB", val);
	}
}

#define BITS_PER_PAGE (__PAGE_SIZE * BITS_PER_BYTE)

static unsigned long __page_allocator_init(struct page_allocator *a,
		void *membase, __sz memsz)
{
	unsigned long mymemsz = 0, pages_num, bm_pages_num, pages_in_2MB;
	char buf[32];

	memsz_to_str(memsz, buf, sizeof(buf));
	uk_pr_info("Total memory size: %s mmap.percent=%lu\n", buf, percent);

	if (!memsz)
		goto out;
	if (memsz < MB)
		goto out;

	mymemsz = memsz * percent / 100;

	pages_num = (mymemsz * BITS_PER_PAGE) / (1 + BITS_PER_PAGE * __PAGE_SIZE);
	pages_in_2MB = (2 * MB) / __PAGE_SIZE;
	if (pages_num % pages_in_2MB)
		pages_num = (pages_num / pages_in_2MB) * pages_in_2MB;

	a->start_addr = (membase + memsz) - pages_num * __PAGE_SIZE;
	bm_pages_num = DIV_ROUND_UP(pages_num, BITS_PER_PAGE);
	a->bm = a->start_addr - bm_pages_num * __PAGE_SIZE;
	a->pages_num = pages_num;
	a->end_addr = a->start_addr + a->pages_num * __PAGE_SIZE;
	uk_bitmap_zero(a->bm, a->pages_num);
	a->allocated = 0;

	a->search_start_pos = 0;
	a->search_last_free_pos = -1;
	a->search_last_free_num = 0;

	uk_mutex_init(&a->m);

	mymemsz = (pages_num + bm_pages_num) * __PAGE_SIZE;
	memsz_to_str(mymemsz, buf, sizeof(buf));
	uk_pr_info("Memory size used of page allocations: %s @ %p\n",
			buf, a->start_addr);

	memsz_to_str(bm_pages_num * __PAGE_SIZE, buf, sizeof(buf));
	uk_pr_info("Bitmap size %s @ %p\n", buf, a->bm);
	uk_pr_info("percent=%lu\n", 100 * mymemsz / memsz);

out:
	return mymemsz;
}

static void *__page_allocator_alloc(struct page_allocator *a,
		unsigned long pages_num)
{
	unsigned long pg_idx;
	void *addr = NULL;

	uk_mutex_lock(&a->m);

	if (pages_num > a->pages_num - a->allocated) {
		uk_pr_err("Not enough pages available: %ld < %ld\n",
				a->pages_num - a->allocated, pages_num);
		goto out;
	}

	if (a->search_last_free_pos >= 0 && pages_num == (unsigned long) a->search_last_free_num) {
		pg_idx = a->search_last_free_pos;
		a->search_last_free_pos = -1;

	} else {
		pg_idx = uk_bitmap_find_next_zero_area(
				a->bm, a->pages_num,
				a->search_start_pos,
				pages_num, /* nr */
				pages_num - 1); /* align_mask TODO */
		if (pg_idx >= a->pages_num) {
			pg_idx = uk_bitmap_find_next_zero_area(
					a->bm, a->pages_num,
					0,
					pages_num, /* nr */
					pages_num - 1); /* align_mask */
			if (pg_idx >= a->pages_num) {
				uk_pr_err("Not enough contiguous pages available: %ld < %lu\n",
						a->pages_num - a->allocated, pages_num);
				goto out;
			}
		}
		a->search_start_pos = pg_idx + pages_num;
	}

	addr = a->start_addr + pg_idx * __PAGE_SIZE;
	uk_bitmap_set(a->bm, pg_idx, pages_num);
	a->allocated += pages_num;

out:
	uk_mutex_unlock(&a->m);
	return addr;
}

static void *__page_allocator_alloc_fixed(struct page_allocator *a,
		void *fixed_addr, unsigned long pages_num)
{
	unsigned long pg_idx;
	void *addr = NULL;

	if (!(a->start_addr <= fixed_addr && fixed_addr < a->end_addr))
		goto out;

	if (!(fixed_addr + pages_num * __PAGE_SIZE <= a->end_addr))
		goto out;

	pg_idx = (fixed_addr - a->start_addr) / __PAGE_SIZE;

	uk_mutex_lock(&a->m);
	for (unsigned long i = pg_idx; i < pg_idx + pages_num; i++) {
		if (uk_test_bit(i, a->bm)) {
			uk_mutex_unlock(&a->m);
			goto out;
		}
	}

	addr = a->start_addr + pg_idx * __PAGE_SIZE;
	uk_bitmap_set(a->bm, pg_idx, pages_num);
	a->allocated += pages_num;

	uk_mutex_unlock(&a->m);
out:
	return addr;
}

static int __page_allocator_free(struct page_allocator *a,
		void *addr, unsigned long pages_num)
{
	unsigned long pg_idx;
	int rc = 0;

	if (!(a->start_addr <= addr && addr < a->end_addr)) {
		rc = -EINVAL;
		goto out;
	}

	if (!(addr + pages_num * __PAGE_SIZE <= a->end_addr)) {
		rc = -EINVAL;
		goto out;
	}

	pg_idx = (addr - a->start_addr) / __PAGE_SIZE;

	uk_mutex_lock(&a->m);

	uk_bitmap_clear(a->bm, pg_idx, pages_num);//TODO check if already freed
	a->allocated -= pages_num;

	a->search_last_free_pos = pg_idx;
	a->search_last_free_num = pages_num;

	uk_mutex_unlock(&a->m);
out:
	return rc;
}

static int __page_allocator_set_prot(struct page_allocator *a,
		void *addr, unsigned long pages_num, unsigned long prot)
{
	unsigned long pg_idx;
	int rc = 0;

	if (!(a->start_addr <= addr && addr < a->end_addr)) {
		fprintf(stderr, "%s:%d\n", __FUNCTION__, __LINE__);
		rc = -EINVAL;
		goto out;
	}

	if (!(addr + pages_num * __PAGE_SIZE <= a->end_addr)) {
		rc = -EINVAL;
		goto out;
	}

	pg_idx = (addr - a->start_addr) / __PAGE_SIZE;

	uk_mutex_lock(&a->m);
	for (unsigned long i = pg_idx; i < pg_idx + pages_num; i++) {
		if (!uk_test_bit(i, a->bm)) {
			fprintf(stderr, "%s:%d\n", __FUNCTION__, __LINE__);
			rc = -EINVAL;
			uk_mutex_unlock(&a->m);
			goto out;
		}
	}

	ukplat_set_prot(addr, pages_num * __PAGE_SIZE, prot);
	uk_mutex_unlock(&a->m);
out:
	return rc;
}

static void __page_allocator_show_stats(struct page_allocator *a)
{
	long total, used, free;
	char buf1[32], buf2[32];

	total = a->pages_num * __PAGE_SIZE;
	memsz_to_str(total, buf1, sizeof(buf1));

	used = a->allocated * __PAGE_SIZE;
	memsz_to_str(used, buf1, sizeof(buf1));

	free = (a->pages_num - a->allocated) * __PAGE_SIZE;
	memsz_to_str(free, buf2, sizeof(buf2));

	uk_pr_info("Used: %s (%ld%%)  Free: %s (%ld%%)\n",
			buf1, 100 * used / total,
			buf2, 100 * free / total);

}

unsigned long page_allocator_init(void *membase, __sz memsz)
{
	return __page_allocator_init(&page_allocator, membase, memsz);
}

void *page_allocator_alloc(unsigned long pages_num)
{
	return __page_allocator_alloc(&page_allocator, pages_num);
}

void *page_allocator_alloc_fixed(void *addr, unsigned long pages_num)
{
	return __page_allocator_alloc_fixed(&page_allocator, addr, pages_num);
}

int page_allocator_free(void *addr, unsigned long pages_num)
{
	return __page_allocator_free(&page_allocator, addr, pages_num);
}

int page_allocator_set_prot(void *addr, unsigned long pages_num, unsigned long prot)
{
	return __page_allocator_set_prot(&page_allocator, addr, pages_num, prot);
}

void page_allocator_show_stats(void)
{
	__page_allocator_show_stats(&page_allocator);
}
