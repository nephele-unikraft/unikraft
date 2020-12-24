/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Costin Lupu <costin.lupu@cs.pub.ro>
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
 */

#include <stdlib.h>
#include <stdint.h>
#include <hashtable.h>
#include <uk/init.h>
#include <uk/plat/clone.h>
#include <common/hypervisor.h>
#include <common/gnttab.h>
#if defined(__x86_64__)
#include <xen-x86/irq.h>
#include <xen-x86/mm.h>
#include <xen-x86/setup.h>
#else
#error "Unsupported architecture"
#endif
#include <xen/clone.h>
#include <xenbus/xs.h>


int ukplat_clone(unsigned int nr_children, unsigned short *child_ids)
{
    struct clone_op op;
    unsigned long flags;
    int rc;

    op.start_info_mfn = virt_to_mfn(HYPERVISOR_start_info);
    op.nr_children = nr_children;
    set_xen_guest_handle(op.child_list, child_ids);

    local_irq_save(flags);
    rc = HYPERVISOR_clone(CLONEOP_clone, &op);
    local_irq_restore(flags);

    return rc;
}

int ukplat_get_domain_id(void)
{
	return xs_get_self_id();
}

static unsigned int uint_hash_from_key_fn(void *k)
{
	unsigned long value = (unsigned long) k;
	char *p = (char *) &value;
	unsigned int hash = 5381;
	char c;
	int i;

	for (i = 0; i < (int) sizeof(value); i++) {
		c = *p++;
		hash = ((hash << 5) + hash) + (unsigned int) c;
	}

	return hash;
}

static int ulong_keys_equal_fn(void *key1, void *key2)
{
	return ((unsigned long) key1 == (unsigned long) key2);
}

static struct hashtable *shmem_map;

static int shmem_init(void)
{
	int rc = 0;

	if (shmem_map) {
		rc = -EBUSY;
		goto out;
	}

	shmem_map = create_hashtable(16, uint_hash_from_key_fn,
			ulong_keys_equal_fn);
	if (!shmem_map) {
		rc = -ENOMEM;
		goto out;
	}
out:
	return rc;
}
uk_plat_initcall(shmem_init);

struct shmem {
	void *map;
	unsigned long pages_num;
	grant_ref_t grefs[];
};

static void uk_plat_shmem_destroy(struct shmem *shmem)
{
	int i, rc;

	if (shmem->map) {
		for (i = 0; i < (int) shmem->pages_num; i++) {
			UK_ASSERT(shmem->grefs[i] != GRANT_INVALID_REF);
			rc = gnttab_end_access(shmem->grefs[i]);
			UK_ASSERT(rc);
		}
		uk_pfree(uk_alloc_get_default(), shmem->map, shmem->pages_num);
	}
	uk_free(uk_alloc_get_default(), shmem);
}

void *uk_plat_shmem_alloc(int capacity)
{
	struct shmem *shmem;
	unsigned long pages_num;
	void *map = NULL;
	int rc;

	if (!capacity)
		goto out;

	pages_num = DIV_ROUND_UP(capacity, PAGE_SIZE);

	shmem = uk_calloc(uk_alloc_get_default(),
			1, sizeof(struct shmem) + pages_num * sizeof(grant_ref_t));
	if (!shmem)
		goto out;
	shmem->pages_num = pages_num;

	rc = gnttab_alloc_and_grant_multi(&shmem->map, shmem->pages_num,
			uk_alloc_get_default(), DOMID_CHILD, 0, shmem->grefs);
	if (!shmem->map) {
		uk_plat_shmem_destroy(shmem);
		goto out;
	}

	rc = hashtable_insert(shmem_map, shmem->map, shmem);
	if (rc == 0) {
		uk_plat_shmem_destroy(shmem);
		goto out;
	}

	map = shmem->map;
out:
	return map;
}

int uk_plat_shmem_free(void *map)
{
	struct shmem *shmem;
	int rc = 0;

	if (!map) {
		rc = -EINVAL;
		goto out;
	}
	if ((unsigned long) map & ~(PAGE_SIZE - 1)) {
		rc = -EINVAL;
		goto out;
	}

	shmem = hashtable_remove(shmem_map, map);
	if (!shmem) {
		rc = -EINVAL;
		goto out;
	}

	uk_plat_shmem_destroy(shmem);
out:
	return rc;
}
