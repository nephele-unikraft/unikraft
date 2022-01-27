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

#define __XEN_TOOLS__ 1
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <hashtable.h>
#include <uk/init.h>
#include <uk/plat/clone.h>
#include <uk/list.h>
#include <uk/mutex.h>
#include <uk/print.h>
#include <common/hypervisor.h>
#include <common/gnttab.h>
#include <common/events.h>
#if defined(__x86_64__)
#include <xen-x86/irq.h>
#include <xen-x86/mm.h>
#include <xen-x86/setup.h>
#else
#error "Unsupported architecture"
#endif
#include <xen/domctl.h>
#include <xen/sched.h>
#include <xen/clone.h>
#include <xenbus/xs.h>

static int mydomid = -1;

struct child_domain {
	domid_t domid;
	struct uk_list_head child_list;
};
static UK_LIST_HEAD(child_domain_list);
static struct uk_mutex child_domain_list_lock =
		UK_MUTEX_INITIALIZER(child_domain_list_lock);

static int child_domains_add(domid_t domid)
{
	struct child_domain *child;
	int rc;

	child = malloc(sizeof(struct child_domain));
	if (!child) {
		rc = -ENOMEM;
		goto out;
	}

	child->domid = domid;

	uk_mutex_lock(&child_domain_list_lock);
	uk_list_add_tail(&child->child_list, &child_domain_list);
	uk_mutex_unlock(&child_domain_list_lock);

	rc = 0;
out:
	return rc;
}

static void child_domains_remove_last_n(unsigned int nr)
{
	struct child_domain *p, *n;
	unsigned int count = 0;

	uk_mutex_lock(&child_domain_list_lock);
	uk_list_for_each_entry_safe_reverse(p, n, &child_domain_list, child_list) {
		if (count == nr)
			break;

		uk_list_del(&p->child_list);
		free(p);

		count++;
	}
	uk_mutex_unlock(&child_domain_list_lock);
}

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

	if (rc == 0) {
		/* parent */
		for (unsigned int i = 0; i < nr_children; i++) {
			rc = child_domains_add(child_ids[i]);
			if (rc) {
				uk_pr_err("Error calling child_domain_add()=%d\n", rc);
				child_domains_remove_last_n(i);
			}
		}
	} else {
		/* child */
		/* reset domid in order to reinit it*/
		mydomid = -1;
	}

	return rc;
}

struct xen_dominfo {
	uint32_t domid;
	unsigned int dying:1, crashed:1, shutdown:1, paused:1,
		blocked:1, running:1, hvm:1, hap:1;
	unsigned int shutdown_reason; /* only meaningful if shutdown==1 */
};

static int get_domain_info(unsigned short domid, struct xen_dominfo *info)
{
	struct xen_domctl domctl;
	int rc;

	domctl.cmd = XEN_DOMCTL_getdomaininfo;
	domctl.interface_version = XEN_DOMCTL_INTERFACE_VERSION;
	domctl.domain = domid;
	memset(&domctl.u, 0, sizeof(domctl.u));

	rc = HYPERVISOR_domctl((unsigned long) &domctl);
	if (rc) {
		if (rc != -ESRCH)
			uk_pr_err("Error calling HYPERVISOR_domctl()=%d\n", rc);
		goto out;
	}

	info->domid = domctl.domain;

	info->dying    = !!(domctl.u.getdomaininfo.flags & XEN_DOMINF_dying);
	info->shutdown = !!(domctl.u.getdomaininfo.flags & XEN_DOMINF_shutdown);
	info->paused   = !!(domctl.u.getdomaininfo.flags & XEN_DOMINF_paused);
	info->blocked  = !!(domctl.u.getdomaininfo.flags & XEN_DOMINF_blocked);
	info->running  = !!(domctl.u.getdomaininfo.flags & XEN_DOMINF_running);
	info->hvm      = !!(domctl.u.getdomaininfo.flags & XEN_DOMINF_hvm_guest);
	info->hap      = !!(domctl.u.getdomaininfo.flags & XEN_DOMINF_hap);

	info->shutdown_reason =
	    (domctl.u.getdomaininfo.flags >> XEN_DOMINF_shutdownshift) &
	    XEN_DOMINF_shutdownmask;

	if (info->shutdown && (info->shutdown_reason == SHUTDOWN_crash)) {
		info->shutdown = 0;
		info->crashed  = 1;
	}

out:
	return rc;
}

static int ukplat_wait_any_once(struct ukplat_wait_result *result, bool *pfound)
{
	struct child_domain *p, *n;
	struct xen_dominfo info;
	bool found = false;
	int scanned = 0, wstatus, rc;

	uk_mutex_lock(&child_domain_list_lock);
	uk_list_for_each_entry_safe(p, n, &child_domain_list, child_list) {
		rc = get_domain_info(p->domid, &info);
		if (rc && rc != -ESRCH) {
			uk_pr_err("Error calling get_domain_info()=%d domid=%d\n",
					rc, p->domid);
			break;
		}
		scanned++;

		wstatus = 0;
		if (rc == -ESRCH || info.shutdown) {
			WAIT_STATUS_EXIT_VALUE_SET(wstatus, 0);
			WAIT_STATUS_EXITED_SET(wstatus);
			rc = 0;

		} else if (info.crashed) {

		} else
			continue;

		result->child_id = p->domid;
		result->wstatus = wstatus;

		uk_list_del(&p->child_list);
		free(p);

		found = true;
		break;
	}
	uk_mutex_unlock(&child_domain_list_lock);

	if (scanned == 0) {
		/* This domain has no No children */
		UK_ASSERT(found == false);
		rc = -ECHILD;
	}

	if (pfound)
		*pfound = found;

	return rc;
}

int ukplat_wait_any(struct ukplat_wait_result *result, int options)
{
	int rc;

	if (options & WAIT_OPT_NOHANG) {
		rc = ukplat_wait_any_once(result, NULL);

	} else {
		bool found = false;

		while (!found) {
			rc = ukplat_wait_any_once(result, &found);
			if (rc)
				break;
		}
	}

	return rc;
}

int ukplat_get_domain_id(void)
{
	if (mydomid == -1)
		mydomid = xs_get_self_id();
	return mydomid;
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

struct ukplat_notifier {
	evtchn_port_t evtchn;
	struct uk_waitq wq;
	bool spurious_handled;
	bool triggered;
};

static void notifier_handler(evtchn_port_t evtchn,
			    struct __regs *regs __unused,
			    void *arg)
{
	struct ukplat_notifier *ntfr = arg;

	UK_ASSERT(ntfr);
	UK_ASSERT(ntfr->evtchn == evtchn);

	if (!ntfr->spurious_handled) {
		//TODO mask_evtchn(ntfr->evtchn);
		ntfr->spurious_handled = true;
		return;
	}

	UK_WRITE_ONCE(ntfr->triggered, true);
	uk_waitq_wake_up(&ntfr->wq);
}

struct ukplat_notifier *ukplat_notifier_create(void)
{
	struct ukplat_notifier *ntfr = NULL;
	int rc = 0;

	ntfr = malloc(sizeof(struct ukplat_notifier));
	if (!ntfr)
		goto out;

	rc = evtchn_alloc_unbound(DOMID_CHILD, notifier_handler, ntfr,
			&ntfr->evtchn);
	if (rc) {
		uk_pr_err("Error calling evtchn_alloc_unbound() rc=%d\n", rc);
		goto out;
	}

	uk_waitq_init(&ntfr->wq);
	ntfr->spurious_handled = false;
	ntfr->triggered = false;
	unmask_evtchn(ntfr->evtchn);

out:
	if (rc) {
		free(ntfr);
		ntfr = NULL;
	}
	return ntfr;
}

int ukplat_notifier_destroy(struct ukplat_notifier *ntfr)
{
	int rc = 0;

	if (!ntfr) {
		rc = -EINVAL;
		goto out;
	}

	unbind_evtchn(ntfr->evtchn);
	free(ntfr);
out:
	return rc;
}

int ukplat_wait(struct ukplat_notifier *ntfr)
{
	int rc = 0;

	if (!ntfr) {
		rc = -EINVAL;
		goto out;
	}

	unmask_evtchn(ntfr->evtchn);
	uk_waitq_wait_event(&ntfr->wq, UK_READ_ONCE(ntfr->triggered));
	mask_evtchn(ntfr->evtchn);
	UK_WRITE_ONCE(ntfr->triggered, false);
out:
	return rc;
}

int ukplat_add_waiter(struct ukplat_notifier *ntfr,
		struct uk_waitq_entry *waiter)
{
	unsigned long flags;
	int rc = 0;

	if (!ntfr) {
		rc = -EINVAL;
		goto out;
	}

	flags = ukplat_lcpu_save_irqf();
	uk_waitq_add(&ntfr->wq, waiter);
	ukplat_lcpu_restore_irqf(flags);
	unmask_evtchn(ntfr->evtchn);//TODO
out:
	return rc;
}

int ukplat_remove_waiter(struct ukplat_notifier *ntfr,
		struct uk_waitq_entry *waiter)
{
	unsigned long flags;
	int rc = 0;

	if (!ntfr) {
		rc = -EINVAL;
		goto out;
	}

	flags = ukplat_lcpu_save_irqf();
	uk_waitq_remove(&ntfr->wq, waiter);
	ukplat_lcpu_restore_irqf(flags);
out:
	return rc;
}

int ukplat_notify(struct ukplat_notifier *ntfr)
{
	int rc = 0;

	if (!ntfr) {
		rc = -EINVAL;
		goto out;
	}

	rc = notify_remote_via_evtchn(ntfr->evtchn);
	if (rc) {
		uk_pr_err("Error calling notify_remote_via_evtchn() rc=%d\n", rc);
		goto out;
	}

	uk_waitq_wake_up(&ntfr->wq);
out:
	return rc;
}
