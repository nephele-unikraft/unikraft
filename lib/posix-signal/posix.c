/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Mihai Pogonaru <pogonarumihai@gmail.com>
 *          Costin Lupu <costin.lupu@cs.pub.ro>
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
/* adapted from OSv */

#include <errno.h>
#include <signal.h>
#include <uk/alloc.h>
#include <uk/sched.h>
#include <uk/thread.h>
#include <uk/process.h>
#include <uk/signal.h>


/* TODO: We do not support any sa_flags besides SA_SIGINFO */
int sigaction(int signum, const struct sigaction *act, struct sigaction *oldact)
{
	struct sigaction *myact;
	struct uk_thread_signal_state *tss;
	struct uk_signal *signal;

	if (!signum_is_valid(signum)
			|| signum == SIGKILL || signum == SIGSTOP) {
		errno = EINVAL;
		return -1;
	}

	myact = &pss.sigaction[signum - 1];

	if (oldact)
		*oldact = *myact;

	if (act) {
		/* TODO: SA_NODEFER */
		*myact = *act;
		sigaddset(&myact->sa_mask, signum);

		if (IS_SIG_IGN(act)) {
			uk_list_for_each_entry(tss, &pss.tss_list, tss_list_node) {
				signal = uk_thread_signal_state_remove_pending(tss, signum);
				if (signal)
					uk_signal_destroy(signal);
			}
			signal = uk_process_signal_state_remove_pending(signum);
			if (signal)
				uk_signal_destroy(signal);
		}
	}

	return 0;
}

static sighandler_t __signal(int signum, sighandler_t handler, int sa_flags)
{
	struct sigaction old, act = {
		.sa_handler = handler,
		.sa_flags = sa_flags
	};
	sighandler_t old_handler = NULL;
	int rc;

	rc = sigaction(signum, &act, &old);
	if (rc < 0) {
		old_handler = SIG_ERR;
		goto out;
	}

	if (!(old.sa_flags & SA_SIGINFO))
		/* one-param handler */
		old_handler = old.sa_handler;
out:
	return old_handler;
}

sighandler_t signal(int signum, sighandler_t handler)
{
	/* SA_RESTART <- BSD signal semantics */
	return __signal(signum, handler, SA_RESTART);
}

int sigpending(sigset_t *set)
{
	struct uk_thread_signal_state *tss;
	int rc = 0;

	if (!set) {
		rc = -EINVAL;
		goto out;
	}

	tss = uk_thread_signal_state_current();
	sigorset(set, &pss.pending_mask, &tss->pending_mask);
out:
	return rc;
}

static int thread_sigprocmask(struct uk_thread_signal_state *tss,
		int how, const sigset_t *set, sigset_t *oldset)
{
	int rc = -1;

	if (oldset)
		*oldset = *&tss->blocked_mask;

	if (set) {
		switch (how) {
		case SIG_BLOCK:
			rc = uk_thread_signal_state_block(tss, set);
			break;
		case SIG_UNBLOCK:
			rc = uk_thread_signal_state_unblock(tss, set);
			break;
		case SIG_SETMASK:
			rc = uk_thread_signal_state_set_blocked(tss, set);
			break;
		default:
			errno = EINVAL;
			rc = -1;
			break;
		}
	}

	return rc;
}

int sigprocmask(int how, const sigset_t *set, sigset_t *oldset)
{
	return thread_sigprocmask(uk_thread_signal_state_current(), how, set, oldset);
}

struct uk_thread_signal_wait_state {
	/* awaited signals mask */
	sigset_t awaited_mask;
	/* awaited signal received */
	struct uk_signal *received_signal;
};

/*
 * Tries to deliver a pending signal to the current thread
 * Used only with a waiting thread
 *
 * Returns: 0 if no signal was delivered, 1 if a signal was delivered
 */
static int get_awaited_signal(struct uk_thread_signal_wait_state *ctx)
{
	struct uk_thread_signal_state *tss;
	struct uk_signal *signal;
	int rc = 0;

	tss = uk_thread_signal_state_current();

	/* try to deliver thread pending signal */
	signal = uk_thread_signal_state_remove_pending_mask(tss, &ctx->awaited_mask);
	if (signal) {
		ctx->received_signal = signal;
		rc = 1;
		goto out;
	}

	/* try to deliver process pending signal */
	signal = uk_process_signal_state_remove_pending_mask(&ctx->awaited_mask);
	if (signal) {
		ctx->received_signal = signal;
		rc = 1;
		goto out;
	}
out:
	return rc;
}

static int wait_for_signal(void)
{
	struct uk_thread_signal_state *tss;
	struct uk_thread_signal_wait_state ctx;

	tss = uk_thread_signal_state_current();

	/* we are waiting for any signal that is not blocked */
	sigreverseset(&ctx.awaited_mask, &tss->blocked_mask);

	while (1) {
		if (get_awaited_signal(&ctx))
			break;
		tss->explicit_waiting = true;
		uk_thread_block(uk_thread_current());
		uk_sched_yield();
		tss->explicit_waiting = false;
	}

	/* handle signal */
	uk_signal_handle(ctx.received_signal);

	/* handle other pending signals too */
	uk_thread_signal_state_current_handle_pending();

	errno = EINTR;
	return -1; /* always returns -1 and sets errno to EINTR */
}

/* POSIX: does not return if the signals are ignored. */
int sigsuspend(const sigset_t *mask)
{
	struct uk_thread_signal_state *tss;
	sigset_t new_blocked_mask, old_blocked_mask;
	int rc;

	tss = uk_thread_signal_state_current();

	sigcopyset(&new_blocked_mask, mask);
	sigset_clear_unmaskable(&new_blocked_mask);
	/* save blocked mask */
	sigcopyset(&old_blocked_mask, &tss->blocked_mask);
	/* change blocked mask */
	sigcopyset(&tss->blocked_mask, &new_blocked_mask);

	rc = wait_for_signal();

	/* restore blocked mask */
	sigcopyset(&tss->blocked_mask, &old_blocked_mask);

	return rc;
}

int pause(void)
{
	return wait_for_signal();
}

int sigwait(const sigset_t *set, int *signum)
{
	struct uk_thread_signal_wait_state ctx;
	struct uk_thread_signal_state *tss;
	int handled_num, rc = 0;

	/*
	 * If the signals are ignored, this doesn't return <- TODO: POSIX ??
	 *
	 * POSIX states that the signals in set must have been blocked before
	 * calling sigwait, otherwise behavior is undefined -> for us the
	 * behavior is not caring -> even if the signal is not blocked sigwait
	 * will still accept it
	 *
	 * NOTE: this function is not signal safe
	 */

	sigcopyset(&ctx.awaited_mask, set);
	sigset_clear_unmaskable(&ctx.awaited_mask);
	if (sigisemptyset(&ctx.awaited_mask)) {
		rc = EINVAL;
		goto out;
	}

	tss = uk_thread_signal_state_current();

	while (1) {
		if (get_awaited_signal(&ctx))
			break;

		/* sigwait() allows signals to be handled while waiting */
		handled_num = uk_thread_signal_state_current_handle_pending();
		if (handled_num) {
			/* we might received a signal while handling the others */
			if (get_awaited_signal(&ctx))
				break;
		}

		tss->explicit_waiting = true;
		uk_thread_block(uk_thread_current());
		uk_sched_yield();
		tss->explicit_waiting = false;
	}

	/* do not execute handler, set received signal */
	*signum = ctx.received_signal->siginfo.si_signo;

	/* handle other pending signals */
	uk_thread_signal_state_current_handle_pending();
out:
	return rc; /* returns positive errno */
}

/*
 * Search for a thread that does not have the signal blocked
 * If all of the threads have the signal blocked, add it to process
 * pending signals
 */
int kill(pid_t pid, int signum)
{
	int rc = -1;

	/*
	 * POSIX.1 requires that if a process sends a signal to itself, and the
	 * sending thread does not have the signal blocked, and no other thread
	 * has it unblocked or is waiting for it in sigwait(3), at least one
	 * unblocked signal must be delivered to the sending thread before the
	 * kill() returns.
	 *
	 * FIXME: we don't implement this ^
	 */

	if (pid != 1 && pid != 0 && pid != -1) {//TODO
		errno = ESRCH;
		goto out;
	}

	if (!signum_is_valid(signum)) {
		errno = EINVAL;
		goto out;
	}

	rc = uk_process_signal_state_notify_any(signum);
out:
	return rc;
}

int killpg(int pgrp, int sig)
{
	int rc;

	if (pgrp != UNIKRAFT_PGID) {
		rc = -1;
		errno = ESRCH;
		goto out;
	}

	rc = kill(UNIKRAFT_PID, sig);
out:
	return rc;
}

int raise(int signum)
{
	int rc = -1;

	if (!signum_is_valid(signum)) {
		errno = EINVAL;
		goto out;
	}

	rc = uk_thread_signal_state_notify(uk_thread_signal_state_current(), signum);
out:
	return rc;
}
