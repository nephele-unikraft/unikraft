/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Simon Kuenzer <simon.kuenzer@neclab.eu>
 *
 *
 * Copyright (c) 2017, NEC Europe Ltd., NEC Corporation. All rights reserved.
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

#include <inttypes.h>
#include <string.h>
#include <uk/arch/lcpu.h>
#include <uk/plat/bootstrap.h>
#include <uk/errptr.h>
#include <errno.h>

#include <xen/xen.h>
#include <common/console.h>
#include <xenbus/xs.h>
#include <xenbus/client.h>

#if defined __X86_32__
#include <xen-x86/hypercall32.h>
#elif defined __X86_64__
#include <xen-x86/hypercall64.h>
#include <xen-x86/setup.h>
#include <xen-x86/mm.h>
#elif (defined __ARM_32__) || (defined __ARM_64__)
#include <xen-arm/hypercall.h>
#endif

void ukplat_terminate(enum ukplat_gstate request)
{
	int reason;

	switch (request) {
	case UKPLAT_HALT:
		reason = SHUTDOWN_poweroff;
		break;
	case UKPLAT_RESTART:
		reason = SHUTDOWN_reboot;
		break;
	default: /* UKPLAT_CRASH */
		reason = SHUTDOWN_crash;
		break;
	}

	flush_console();

	for (;;) {
		struct sched_shutdown sched_shutdown = { .reason = reason };

		HYPERVISOR_sched_op(SCHEDOP_shutdown, &sched_shutdown);
	}
}

#ifdef CONFIG_MIGRATION
__weak void app_shutdown(enum ukplat_gstate reason);

extern start_info_t *HYPERVISOR_start_info;

static struct xenbus_watch *watch;
static int end_shutdown_thread;

int ukplat_suspend(void)
{
	int ret;

	uk_pr_info("Unikraft will suspend ...\n");

	pre_suspend();
	arch_pre_suspend();

	/*
	 * This hypercall returns 1 if the suspend
	 * was cancelled and 0 if resuming in a new domain
	 */

	ret = HYPERVISOR_suspend(virt_to_mfn(HYPERVISOR_start_info));

	arch_post_suspend(ret);
	post_suspend(ret);

	if (ret)
		uk_pr_info("Unikraft suspend canceled!\n");
	else
		uk_pr_info("Unikraft resumed from suspend!\n");

	return ret;
}

static void shutdown_thread(void *ign __unused)
{
	const char *path = "control/shutdown";
	int err, reason;
	char *shutdown;

	uk_pr_info("Shutdown thread started!\n");

	watch = xs_watch_path(XBT_NIL, path);
	if (PTRISERR(watch))
		UK_CRASH("Could not create shutdown watch\n");

	for ( ;; ) {
		err = xenbus_watch_wait_event(watch);
		UK_ASSERT(!err);

		if (end_shutdown_thread)
			break;

		shutdown = xs_read(XBT_NIL, path, NULL);
		if (PTRISERR(shutdown))
			UK_CRASH("Error reading from xenstore\n");

		uk_pr_debug("Message on control/shutdown: %s\n", shutdown);

		if (!strcmp(shutdown, "")) {
			free(shutdown);
			continue;
		} else if (!strcmp(shutdown, "poweroff"))
			reason = SHUTDOWN_poweroff;
		else if (!strcmp(shutdown, "reboot"))
			reason = SHUTDOWN_reboot;
		else if (!strcmp(shutdown, "suspend"))
			reason = SHUTDOWN_suspend;
		else
			reason = SHUTDOWN_crash;
		free(shutdown);

		/* Acknowledge shutdown request */
		err = xs_write(XBT_NIL, path, NULL, "");
		UK_ASSERT(!err);

		uk_pr_info("Shutdown requested: %d\n", reason);
		app_shutdown(reason);

		if (reason == SHUTDOWN_suspend)
			ukplat_suspend();
		else
			HYPERVISOR_shutdown(reason);
	}
}

/*
 * This function can be overridden by the application
 * we are linked against.
 *
 **/
__weak void app_shutdown(enum ukplat_gstate reason)
{
	uk_pr_debug("weak %s called. Reason: %d\n", __func__, reason);
}

void ukplat_shutdown_init(void)
{
	end_shutdown_thread = 0;
	uk_thread_create("shutdown", shutdown_thread, NULL);
}

void ukplat_shutdown_fini(void)
{
	int err;

	end_shutdown_thread = 1;

	err = xenbus_watch_notify_event(watch);
	UK_ASSERT(!err);

	err = xs_unwatch(XBT_NIL, watch);
	UK_ASSERT(!err);
}
#endif
