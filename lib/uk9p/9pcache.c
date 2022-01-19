/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Costin Lupu <costin.lupu@cs.pub.ro>
 *
 * Copyright (c) 2022, University Politehnica of Bucharest. All rights reserved.
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

#include <string.h>
#include <errno.h>
#include <uk/assert.h>
#include <uk/list.h>
#include <uk/init.h>
#include <uk/alloc.h>
#include <hashtable.h>
#include <uk/bitmap.h>
#include <uk/thread.h>
#include <uk/arch/spinlock.h>
#include <uk/9pcache.h>
#include <profile.h>


struct p9_fid_cache_key {
	struct vnode *dvp;
	char *name;
};

static unsigned int p9_fcache_hash_from_key_fn(void *k)
{
	struct p9_fid_cache_key *key = k;
	char *p;
	unsigned int hash = 5381;
	char c;

	p = (char *) key->dvp;
	for (int i = 0; i < (int) sizeof(key->dvp); i++) {
		c = *p++;
		hash = ((hash << 5) + hash) + (unsigned int) c;
	}

	p = (char *) key->name;
	for (int i = 0; i < (int) strlen(key->name); i++) {
		c = *p++;
		hash = ((hash << 5) + hash) + (unsigned int) c;
	}

	return hash;
}

static int p9_fcache_keys_equal_fn(void *key1, void *key2)
{
	struct p9_fid_cache_key *mykey1 = key1;
	struct p9_fid_cache_key *mykey2 = key2;

	return (mykey1->dvp == mykey2->dvp && !strcmp(mykey1->name, mykey2->name));
}

struct p9_fid_cache_entry {
	struct p9_fid_cache_key key;
	struct uk_9pfid *fid;
	struct uk_9p_stat stat;
};

static struct p9_fid_cache {
	spinlock_t spinlock;
	struct hashtable *name2fidmap;
	struct hashtable *fid2namemap;
	long count;
	struct p9_fid_cache_entry *last_accessed_entry;
} p9_fcache;

struct uk_9pfid *p9_fid_cache_get(struct vnode *dvp, char *name,
		struct uk_9p_stat *stat)
{
	struct p9_fid_cache_key key;
	struct p9_fid_cache_entry *e;
	struct uk_9pfid *fid = NULL;

	key.dvp = dvp;
	key.name = name;

	ukarch_spin_lock(&p9_fcache.spinlock);

	e = p9_fcache.last_accessed_entry;
	if (e && p9_fcache_keys_equal_fn(&e->key, &key))
		fid = e->fid;

	else {
		e = hashtable_search(p9_fcache.name2fidmap, &key);
		if (e) {
			UK_ASSERT(e->key.dvp == dvp);
			UK_ASSERT(!strcmp(e->key.name, key.name));
			p9_fcache.last_accessed_entry = e;
			fid = e->fid;
		}
	}

	if (e && stat)
		memcpy(stat, &e->stat, sizeof(*stat));

	ukarch_spin_unlock(&p9_fcache.spinlock);

	return fid;
}

bool p9_fid_cache_contains(struct uk_9pfid *fid)
{
	struct p9_fid_cache_entry *e;
	bool ret = false;

	ukarch_spin_lock(&p9_fcache.spinlock);

	e = p9_fcache.last_accessed_entry;
	if (e && e->fid == fid)
		ret = true;

	else {
		e = hashtable_search(p9_fcache.fid2namemap, fid);
		if (e) {
			UK_ASSERT(e->fid == fid);
			p9_fcache.last_accessed_entry = e;
			ret = true;
		}
	}

	ukarch_spin_unlock(&p9_fcache.spinlock);

	return ret;
}

static void p9_vnode_cache_add_entry(struct p9_fid_cache_entry *e)
{
	hashtable_insert(p9_fcache.name2fidmap, &e->key, e);
	hashtable_insert(p9_fcache.fid2namemap, e->fid, e);
	p9_fcache.count++;
}

static void p9_vnode_cache_remove_entry(struct p9_fid_cache_entry *e)
{
	UK_ASSERT(e != NULL);
//TODO	uk_9pfid_put(e->fid_clone);
	hashtable_remove(p9_fcache.name2fidmap, &e->key);
	hashtable_remove(p9_fcache.fid2namemap, e->fid);
	p9_fcache.count--;

	if (p9_fcache.last_accessed_entry == e)
		p9_fcache.last_accessed_entry = NULL;
}

//TODO max opened files

int p9_fid_cache_add(struct vnode *dvp, char *name, struct uk_9pfid *fid,
		struct uk_9p_stat *stat)
{
	struct p9_fid_cache_entry *e;
	int rc = 0, len;

	if (!dvp || !name) {
		rc = -EINVAL;
		goto out;
	}

	{
		struct p9_fid_cache_key k;

		k.dvp = dvp;
		k.name = name;
		UK_ASSERT(hashtable_search(p9_fcache.name2fidmap, &k) == NULL);
	}

	len = strlen(name);

	ukarch_spin_lock(&p9_fcache.spinlock);

	if (p9_fcache.count == hashtable_loadlimit(p9_fcache.name2fidmap)) {
		uk_pr_err("Too many opened files!");
		rc = -ENOMEM;
		goto out_unlock;
	}

	e = uk_malloc(uk_alloc_get_default(), sizeof(*e) + len + 1);
	if (!e) {
		rc = -ENOMEM;
		goto out_unlock;
	}
	e->key.dvp = dvp;
	e->key.name = (char *) (e + 1);
	strncpy(e->key.name, name, len + 1);
	e->fid = fid;
	if (stat)
		memcpy(&e->stat, stat, sizeof(*stat));

	p9_vnode_cache_add_entry(e);

out_unlock:
	ukarch_spin_unlock(&p9_fcache.spinlock);
out:
	return rc;
}

int p9_fid_cache_del(struct uk_9pfid *fid)
{
	struct p9_fid_cache_entry *e;
	int rc = 0;

	if (!fid) {
		rc = -EINVAL;
		goto out;
	}

	ukarch_spin_lock(&p9_fcache.spinlock);

	e = hashtable_search(p9_fcache.fid2namemap, fid);
	if (e) {
		UK_ASSERT(e->fid->fid == fid->fid);
		p9_vnode_cache_remove_entry(e);
	}

	ukarch_spin_unlock(&p9_fcache.spinlock);
out:
	return rc;
}

//TODO hash from unsigned long (fid value)?

static unsigned int p9_fcache_hash_from_fid_fn(void *k)
{
	struct uk_9pfid *fid = (struct uk_9pfid *) k;
	char *p = (char *) &fid->fid;
	unsigned int hash = 5381;
	char c;

	for (int i = 0; i < (int) sizeof(fid->fid); i++) {
		c = *p++;
		hash = ((hash << 5) + hash) + (unsigned int) c;
	}

	return hash;
}

static int p9_fcache_fid_equal_fn(void *key1, void *key2)
{
	struct uk_9pfid *mykey1 = key1;
	struct uk_9pfid *mykey2 = key2;

	return (mykey1->fid == mykey2->fid);
}

static int p9_fid_cache_init(void)
{
	int rc = 0;

	if (p9_fcache.name2fidmap) {
		rc = -EBUSY;
		goto out;
	}

	p9_fcache.name2fidmap = create_hashtable(16, p9_fcache_hash_from_key_fn,
			p9_fcache_keys_equal_fn);
	if (!p9_fcache.name2fidmap) {
		rc = -ENOMEM;
		goto out;
	}

	p9_fcache.fid2namemap = create_hashtable(16, p9_fcache_hash_from_fid_fn,
			p9_fcache_fid_equal_fn);
	if (!p9_fcache.fid2namemap) {
		rc = -ENOMEM;
		goto out;
	}

	p9_fcache.count = 0;

	ukarch_spin_lock_init(&p9_fcache.spinlock);
out:
	return rc;
}




struct p9_wrfid_key {
	unsigned long fid;
	unsigned long tid;
};

static unsigned int p9_wcache_hash_from_key_fn(void *k)
{
	char *p = (char *) k;
	unsigned int hash = 5381;
	char c;

	for (int i = 0; i < (int) sizeof(struct p9_wrfid_key); i++) {
		c = *p++;
		hash = ((hash << 5) + hash) + (unsigned int) c;
	}

	return hash;
}

static int p9_wcache_keys_equal_fn(void *key1, void *key2)
{
	struct p9_wrfid_key *mykey1 = key1;
	struct p9_wrfid_key *mykey2 = key2;

	return (mykey1->fid == mykey2->fid && mykey1->tid == mykey2->tid);
}

struct p9_wrfid_cache_entry {
	struct p9_wrfid_key key;
	struct uk_9pfid *fid_clone;
	unsigned long idx;
	struct timespec last_accessed;
	struct uk_list_head list;
};

#define WRITE_FID_POOL_SIZE 32

struct p9_wrfid_pool {
	struct p9_wrfid_cache_entry entries[WRITE_FID_POOL_SIZE];
	unsigned long bm[UK_BITS_TO_LONGS(WRITE_FID_POOL_SIZE)];
    long last_freed;
    long search_start;
	struct uk_list_head list_head;
};

static struct p9_wrfid_cache {
	spinlock_t spinlock;
	struct hashtable *ht;
	long count;
	struct p9_wrfid_pool pool;
	struct p9_wrfid_cache_entry *last_accessed_entry;
} p9_wcache;

struct uk_9pfid *p9_wrfid_cache_get(struct uk_9pfid *orig)
{
	struct p9_wrfid_key key;
	struct p9_wrfid_cache_entry *e;
	struct uk_9pfid *fid = NULL;

	key.fid = (unsigned long) orig;
	key.tid = (unsigned long) uk_thread_current();

	ukarch_spin_lock(&p9_wcache.spinlock);

	e = p9_wcache.last_accessed_entry;
	if (e && p9_wcache_keys_equal_fn(&e->key, &key))
		fid = e->fid_clone;

	else {
		e = hashtable_search(p9_wcache.ht, &key);
		if (e) {
			UK_ASSERT(e->key.fid == (unsigned long) key.fid);
			UK_ASSERT(e->key.tid == (unsigned long) key.tid);
			UK_ASSERT(clock_gettime(CLOCK_MONOTONIC, &e->last_accessed) == 0);
			/* add to end of list */
			uk_list_del(&e->list);
			uk_list_add_tail(&e->list, &p9_wcache.pool.list_head);
			p9_wcache.last_accessed_entry = e;
			fid = e->fid_clone;
		}
	}

	ukarch_spin_unlock(&p9_wcache.spinlock);

	return fid;
}

static void p9_wrfid_cache_add_entry(struct p9_wrfid_cache_entry *e,
		struct uk_9pfid *orig, struct uk_9pfid *clone)
{
	UK_ASSERT(clock_gettime(CLOCK_MONOTONIC, &e->last_accessed) == 0);
	e->key.fid = (unsigned long) orig;
	e->key.tid = (unsigned long) uk_thread_current();
	e->fid_clone = clone;
	orig->was_cloned = true;
	uk_bitmap_set(p9_wcache.pool.bm, e->idx, 1);
	uk_list_add(&e->list, &p9_wcache.pool.list_head);
	hashtable_insert(p9_wcache.ht, &e->key, e);
	p9_wcache.count++;
}

static void p9_wrfid_cache_remove_entry(struct p9_wrfid_cache_entry *e)
{
	UK_ASSERT(e != NULL);
	uk_9pfid_put(e->fid_clone);
	hashtable_remove(p9_wcache.ht, &e->key);
	uk_list_del(&e->list);
	uk_bitmap_clear(p9_wcache.pool.bm, e->idx, 1);
	p9_wcache.pool.last_freed = e->idx;
	p9_wcache.count--;
	((struct uk_9pfid *) e->key.fid)->was_cloned = false;

	if (p9_wcache.last_accessed_entry == e)
		p9_wcache.last_accessed_entry = NULL;
}

int p9_wrfid_cache_add(struct uk_9pfid *orig, struct uk_9pfid *clone)
{
	long idx;
	struct p9_wrfid_cache_entry *e;

	if (!orig || !clone)
		return -EINVAL;

	ukarch_spin_lock(&p9_wcache.spinlock);

	if (p9_wcache.count == hashtable_loadlimit(p9_wcache.ht)) {
		struct timespec now;
		struct p9_wrfid_cache_entry *n;

		/* remove oldest entry */
		e = uk_list_first_entry(&p9_wcache.pool.list_head,
				struct p9_wrfid_cache_entry, list);
		p9_wrfid_cache_remove_entry(e);

		UK_ASSERT(clock_gettime(CLOCK_MONOTONIC, &now) == 0);
		uk_list_for_each_entry_safe(e, n, &p9_wcache.pool.list_head, list) {
			if (timespec_diff_sec(&e->last_accessed, &now) > 10)
				p9_wrfid_cache_remove_entry(e);
		}
	}

	if (p9_wcache.pool.last_freed >= 0) {
		idx = p9_wcache.pool.last_freed;
		p9_wcache.pool.last_freed = -1;

	} else {
		idx = uk_find_next_zero_bit(p9_wcache.pool.bm,
			WRITE_FID_POOL_SIZE, p9_wcache.pool.search_start);
		if (idx >= WRITE_FID_POOL_SIZE) {
			idx = uk_find_next_zero_bit(p9_wcache.pool.bm,
				WRITE_FID_POOL_SIZE, 0);
			UK_ASSERT(idx < p9_wcache.pool.search_start);
		}
		p9_wcache.pool.search_start = idx + 1;
	}

	e = &p9_wcache.pool.entries[idx];
	p9_wrfid_cache_add_entry(e, orig, clone);

	ukarch_spin_unlock(&p9_wcache.spinlock);
	return 0;
}

int p9_wrfid_cache_del(struct uk_9pfid *orig)
{
	struct p9_wrfid_key key;
	struct p9_wrfid_cache_entry *e;
	int rc = 0;

	if (!orig) {
		rc = -EINVAL;
		goto out;
	}

	if (!orig->was_cloned)
		goto out;

	key.fid = (unsigned long) orig;
	key.tid = (unsigned long) uk_thread_current();

	ukarch_spin_lock(&p9_wcache.spinlock);

	e = hashtable_search(p9_wcache.ht, &key);
	if (e) {
		UK_ASSERT(e->key.fid == (unsigned long) key.fid);
		UK_ASSERT(e->key.tid == (unsigned long) key.tid);
		p9_wrfid_cache_remove_entry(e);
	}

	ukarch_spin_unlock(&p9_wcache.spinlock);
out:
	return rc;
}

void p9_cache_del(struct uk_9pfid *fid)
{
	p9_wrfid_cache_del(fid);
	p9_fid_cache_del(fid);
}

static int p9_wrfid_cache_init(void)
{
	int rc = 0;

	if (p9_wcache.ht) {
		rc = -EBUSY;
		goto out;
	}

	/* init hash table */
	p9_wcache.ht = create_hashtable(16, p9_wcache_hash_from_key_fn,
			p9_wcache_keys_equal_fn);
	if (!p9_wcache.ht) {
		rc = -ENOMEM;
		goto out;
	}
	p9_wcache.count = 0;

	/* init entries pool */
	for (int i = 0; i < WRITE_FID_POOL_SIZE; i++)
		p9_wcache.pool.entries[i].idx = i;
	uk_bitmap_zero(p9_wcache.pool.bm, WRITE_FID_POOL_SIZE);
	p9_wcache.pool.last_freed = -1;
	p9_wcache.pool.search_start = 0;
	UK_INIT_LIST_HEAD(&p9_wcache.pool.list_head);

	ukarch_spin_lock_init(&p9_wcache.spinlock);
out:
	return rc;
}

static int p9_caches_init(void)
{
	int rc = 0;

	rc = p9_fid_cache_init();
	if (rc) {
		uk_pr_err("Error initializing 9p fid cache (rc=%d)\n", rc);
		goto out;
	}

	rc = p9_wrfid_cache_init();
	if (rc) {
		uk_pr_err("Error initializing 9p write fid cache (rc=%d)\n", rc);
		goto out;
	}
out:
	return rc;
}
uk_plat_initcall(p9_caches_init);
