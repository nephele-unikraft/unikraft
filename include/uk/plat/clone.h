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

#ifndef __UKPLAT_CLONE__
#define __UKPLAT_CLONE__

#include <uk/wait.h>

int ukplat_clone(unsigned int nr_children, unsigned short *child_ids);

/* Wait options */
#define WAIT_OPT_NOHANG                0x1

/* Wait status: Exit value */
#define WAIT_STATUS_VALUE_POS          0
#define WAIT_STATUS_VALUE_BITS         8
#define WAIT_STATUS_VALUE_MASK \
	(((1 << WAIT_STATUS_VALUE_BITS) - 1) << WAIT_STATUS_VALUE_POS)

#define WAIT_STATUS_EXIT_VALUE_SET(wstatus, value) \
	   (wstatus) |= (value) & WAIT_STATUS_VALUE_MASK
#define WAIT_STATUS_EXIT_VALUE_GET(wstatus) \
	   ((wstatus) & WAIT_STATUS_VALUE_MASK)

/* Wait status: Exited */
#define WAIT_STATUS_EXITED_POS         (WAIT_STATUS_VALUE_POS + WAIT_STATUS_VALUE_BITS)
#define WAIT_STATUS_EXITED_BITS        1
#define WAIT_STATUS_EXITED_VALUE_MASK \
	(((1 << WAIT_STATUS_EXITED_BITS) - 1) << WAIT_STATUS_EXITED_POS)

#define WAIT_STATUS_EXITED_SET(wstatus) \
	   (wstatus) |= WAIT_STATUS_EXITED_VALUE_MASK
#define WAIT_STATUS_EXITED_GET(wstatus) \
	   ((wstatus) & WAIT_STATUS_EXITED_VALUE_MASK)

struct ukplat_wait_result {
	unsigned int child_id;
	int wstatus;
};

int ukplat_wait_any(struct ukplat_wait_result *result, int options);

int ukplat_get_domain_id(void);


void *uk_plat_shmem_alloc(int capacity);
int uk_plat_shmem_free(void *map);


struct ukplat_notifier *ukplat_notifier_create(void);
int ukplat_notifier_destroy(struct ukplat_notifier *ntfr);

int ukplat_wait(struct ukplat_notifier *ntfr);
int ukplat_add_waiter(struct ukplat_notifier *ntfr,
		struct uk_waitq_entry *waiter);
int ukplat_remove_waiter(struct ukplat_notifier *ntfr,
		struct uk_waitq_entry *waiter);
int ukplat_notify(struct ukplat_notifier *ntfr);

#endif /* __UKPLAT_CLONE__ */
