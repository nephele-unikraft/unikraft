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

#include <stdbool.h>
#include <errno.h>
#include <uk/alloc.h>
#include <uk/wait.h>
#include <uk/print.h>


int ukplat_clone(unsigned int nr_children, unsigned short *child_ids)
{
	WARN_STUBBED();
	return -1;
}

int ukplat_wait_any(struct ukplat_wait_result *result, int options)
{
	WARN_STUBBED();
	return -1;
}

int ukplat_get_domain_id(void)
{
	WARN_STUBBED();
	return 1;
}

void *uk_plat_shmem_alloc(int capacity)
{
	return uk_malloc(uk_alloc_get_default(), capacity);
}

int uk_plat_shmem_free(void *map)
{
	int rc = 0;

	if (!map) {
		rc = -EINVAL;
		goto out;
	}

	uk_free(uk_alloc_get_default(), map);
out:
	return rc;
}

struct ukplat_notifier {
	struct uk_waitq wq;
	bool triggered;
};

struct ukplat_notifier *ukplat_notifier_create(void)
{
	struct ukplat_notifier *ntfr = NULL;

	ntfr = malloc(sizeof(struct ukplat_notifier));
	if (!ntfr)
		goto out;

	uk_waitq_init(&ntfr->wq);
	ntfr->triggered = false;
out:
	return ntfr;
}

int ukplat_notifier_destroy(struct ukplat_notifier *ntfr)
{
	int rc = 0;

	if (!ntfr) {
		rc = -EINVAL;
		goto out;
	}

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

	uk_waitq_wait_event(&ntfr->wq, UK_READ_ONCE(ntfr->triggered));
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

	UK_WRITE_ONCE(ntfr->triggered, true);
	uk_waitq_wake_up(&ntfr->wq);
out:
	return rc;
}
