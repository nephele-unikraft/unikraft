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

#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <uk/alloc.h>
#include <uk/print.h>
#include <uk/ctors.h>
#include <uk/thread.h>
#include <uk/signal.h>


/*
 * Process signal state
 */

struct uk_process_signal_state pss;

static void uk_process_signal_state_init(void)
{
	int i;

	sigemptyset(&pss.pending_mask);
	for (i = 0; i < NSIG - 1; ++i) {
		/* TODO: set ign to the ones that should be ign */
		pss.sigaction[i].sa_handler = SIG_DFL;
		sigemptyset(&pss.sigaction[i].sa_mask);
		pss.sigaction[i].sa_flags = 0;
	}
	UK_INIT_LIST_HEAD(&pss.tss_list);
}
UK_CTOR(uk_process_signal_state_init);

int uk_process_signal_state_add_pending(struct uk_signal *signal)
{
	int signum, rc = 0;

	if (!signal) {
		rc = -EINVAL;
		goto out;
	}

	signum = signal->siginfo.si_signo;
	if (!sigismember(&pss.pending_mask, signum)) {
		pss.pending_signals[signum - 1] = signal;
		sigaddset(&pss.pending_mask, signum);
		rc = 1;
	}
out:
	return rc;
}

struct uk_signal *uk_process_signal_state_remove_pending(int signum)
{
	struct uk_signal *signal = NULL;

	if (sigismember(&pss.pending_mask, signum)) {
		signal = pss.pending_signals[signum - 1];
		pss.pending_signals[signum - 1] = NULL;
		sigdelset(&pss.pending_mask, signum);
	}

	return signal;
}

struct uk_signal *uk_process_signal_state_remove_pending_mask(sigset_t *set)
{
	sigset_t accepted_mask;
	struct uk_signal *signal = NULL;
	int signum;

	sigandset(&accepted_mask, &pss.pending_mask, set);
	if (sigisemptyset(&accepted_mask))
		goto out;

	for (signum = 1; signum < NSIG; ++signum) {
		if (sigismember(&accepted_mask, signum)) {
			signal = uk_process_signal_state_remove_pending(signum);
			break;
		}
	}
	UK_ASSERT(signal != NULL);
out:
	return signal;
}

static int uk_thread_signal_state_add_pending(struct uk_thread_signal_state *tss, struct uk_signal *signal);//TODO

int uk_process_signal_state_notify_any(int signum)
{
	struct uk_thread_signal_state *tss;
	struct uk_signal *signal;
	int rc;

	signal = uk_signal_create(signum);
	if (!signal) {
		uk_pr_warn("Could not create signal");
		rc = -1;
		goto out;
	}

	uk_list_for_each_entry(tss, &pss.tss_list, tss_list_node) {
		if (sigismember(&tss->blocked_mask, signum))
			continue;
		rc = uk_thread_signal_state_add_pending(tss, signal);
		if (rc) {
			if (rc == 1) {
				rc = 0; /* success */
			}
			goto out;
		}
	}

	/* didn't find any thread that could accept this signal */
	rc = uk_process_signal_state_add_pending(signal);
	if (rc == 1)
		/* success */
		rc = 0;
	else if (rc == 0)
		/* duplicated signal */
		uk_signal_destroy(signal);
out:
	return rc;
}

/*
 * Thread signal state
 */

int uk_thread_signal_state_init(struct uk_thread_signal_state *tss)
{
	struct uk_thread *current_thread;

	tss->explicit_waiting = false;

	current_thread = uk_thread_current();
	if (current_thread)
		/* inherit signal mask */
		sigcopyset(&tss->blocked_mask, &current_thread->tss.blocked_mask);
	else
		sigemptyset(&tss->blocked_mask);

	sigemptyset(&tss->pending_mask);
	UK_INIT_LIST_HEAD(&tss->pending_list);
	UK_INIT_LIST_HEAD(&tss->blocked_list);
	uk_list_add(&tss->tss_list_node, &pss.tss_list);
	return 0;
}

void uk_thread_signal_state_fini(struct uk_thread_signal_state *tss)
{
	struct uk_signal *signal, *next;

	uk_list_del(&tss->tss_list_node);

	uk_list_for_each_entry_safe(signal, next, &tss->pending_list, list_node) {
		uk_list_del(&signal->list_node);
		uk_signal_destroy(signal);
	}
	uk_list_for_each_entry_safe(signal, next, &tss->blocked_list, list_node) {
		uk_list_del(&signal->list_node);
		uk_signal_destroy(signal);
	}
}

struct uk_thread_signal_state *uk_thread_signal_state_current(void)
{
	return &uk_thread_current()->tss;
}

/*
 * returns:
 *  >0 - on success
 *  0  - if signal is ignored / already pending
 *  <0 - on failure
 */
static int uk_thread_signal_state_add_pending(struct uk_thread_signal_state *tss, struct uk_signal *signal)
{
	int signum = signal->siginfo.si_signo;
	struct uk_thread *t;

	/* skip if signal is ignored */
	if (IS_SIG_IGN(&pss.sigaction[signum - 1]))//TODO deal with ignored
		return 0;
	/* skip if signal is already pending */
	if (sigismember(&tss->pending_mask, signum))
		return 0;

	uk_list_add(&signal->list_node, &tss->pending_list);
	sigaddset(&tss->pending_mask, signum);

	/* wake thread if needed */
	t = __containerof(tss, struct uk_thread, tss);
	if (!is_runnable(t))
		uk_thread_wake(t);

	return 1;
}

struct uk_signal *uk_thread_signal_state_remove_pending(struct uk_thread_signal_state *tss, int signum)
{
	struct uk_signal *signal = NULL;

	if (!sigismember(&tss->pending_mask, signum))
		goto out;

	uk_list_for_each_entry(signal, &tss->pending_list, list_node) {
		if (signal->siginfo.si_signo == signum)
			break;
	}

	UK_ASSERT(signal != NULL);
	sigdelset(&tss->pending_mask, signum);
out:
	return signal;
}

struct uk_signal *uk_thread_signal_state_remove_pending_mask(struct uk_thread_signal_state *tss, sigset_t *set)
{
	sigset_t accepted_mask;
	struct uk_signal *signal = NULL;

	sigandset(&accepted_mask, &tss->pending_mask, set);
	if (sigisemptyset(&accepted_mask))
		goto out;

	uk_list_for_each_entry(signal, &tss->pending_list, list_node) {
		if (sigismember(&accepted_mask, signal->siginfo.si_signo))
			break;
	}

	UK_ASSERT(signal != NULL);
	sigdelset(&tss->pending_mask, signal->siginfo.si_signo);
	uk_list_del(&signal->list_node);
out:
	return signal;
}

int uk_thread_signal_state_block(struct uk_thread_signal_state *tss, const sigset_t *set)
{
	sigset_t newly_blocked_mask;
	struct uk_signal *signal;
	int rc = 0;

	if (!tss || !set) {
		rc = -EINVAL;
		goto out;
	}

	/* get newly blocked mask */
	sigreverseset(&newly_blocked_mask, &tss->blocked_mask);
	sigandset(&newly_blocked_mask, &newly_blocked_mask, set);

	/* update blocked mask */
	sigorset(&tss->blocked_mask, &tss->blocked_mask, set);
	sigset_clear_unmaskable(&tss->blocked_mask);

	/* move signals from pending list to blocked list */
	uk_list_for_each_entry(signal, &tss->pending_list, list_node) {
		if (sigismember(&newly_blocked_mask, signal->siginfo.si_signo)) {
			uk_list_del(&signal->list_node);
			uk_list_add_tail(&signal->list_node, &tss->blocked_list);
		}
	}
out:
	return rc;
}

static void uk_thread_signal_state_add_pending_from_process(struct uk_thread_signal_state *tss, sigset_t *set)
{
	sigset_t filtered_set;
	struct uk_signal *signal;
	int signum, rc;

	sigreverseset(&filtered_set, &tss->pending_mask);
	sigandset(&filtered_set, &filtered_set, set);
	sigandset(&filtered_set, &filtered_set, &pss.pending_mask);

	for (signum = 1; signum < NSIG && !sigisemptyset(&filtered_set); ++signum) {
		if (!sigismember(&filtered_set, signum))
			continue;
		signal = uk_process_signal_state_remove_pending(signum);
		UK_ASSERT(signal != NULL);
		rc = uk_thread_signal_state_add_pending(tss, signal);
		UK_ASSERT(rc > 0);
		sigdelset(&filtered_set, signum);
	}
}

int uk_thread_signal_state_unblock(struct uk_thread_signal_state *tss, const sigset_t *set)
{
	sigset_t unblocked_mask, reversed_set;
	struct uk_signal *signal;
	int rc = 0;

	if (!tss || !set) {
		rc = -EINVAL;
		goto out;
	}

	/* get newly unblocked mask */
	sigandset(&unblocked_mask, &tss->blocked_mask, set);

	/* update blocked mask */
	sigreverseset(&reversed_set, set);
	sigandset(&tss->blocked_mask, &tss->blocked_mask, &reversed_set);

	if (!sigisemptyset(&unblocked_mask)) {
		/* move signals from blocked list to pending list */
		uk_list_for_each_entry(signal, &tss->blocked_list, list_node) {
			if (sigismember(&unblocked_mask, signal->siginfo.si_signo)) {
				uk_list_del(&signal->list_node);
				uk_list_add_tail(&signal->list_node, &tss->pending_list);
				//TODO set pending_mask?
				sigdelset(&unblocked_mask, signal->siginfo.si_signo);
			}
		}
		/* move signals from process pending to pending list */
		uk_thread_signal_state_add_pending_from_process(tss, &unblocked_mask);
		/* handle the new pending signals */
		if (tss == uk_thread_signal_state_current())
			uk_thread_signal_state_current_handle_pending();
	}
out:
	return rc;
}

int uk_thread_signal_state_set_blocked(struct uk_thread_signal_state *tss, const sigset_t *set)
{
	sigset_t newly_blocked_mask, unblocked_mask, reversed_set;
	struct uk_signal *signal;
	int rc = 0;

	if (!tss || !set) {
		rc = -EINVAL;
		goto out;
	}

	/* get newly blocked mask */
	sigreverseset(&newly_blocked_mask, &tss->blocked_mask);
	sigandset(&newly_blocked_mask, &newly_blocked_mask, set);
	/* get newly unblocked mask */
	sigreverseset(&reversed_set, set);
	sigandset(&unblocked_mask, &tss->blocked_mask, &reversed_set);

	/* update blocked mask */
	sigcopyset(&tss->blocked_mask, set);
	sigset_clear_unmaskable(&tss->blocked_mask);

	/* move signals from pending list to blocked list */
	uk_list_for_each_entry(signal, &tss->pending_list, list_node) {
		if (sigismember(&newly_blocked_mask, signal->siginfo.si_signo)) {
			uk_list_del(&signal->list_node);
			uk_list_add_tail(&signal->list_node, &tss->blocked_list);
		}
	}

	if (!sigisemptyset(&unblocked_mask)) {
		/* move signals from blocked list to pending list */
		uk_list_for_each_entry(signal, &tss->blocked_list, list_node) {
			if (sigismember(&unblocked_mask, signal->siginfo.si_signo)) {
				uk_list_del(&signal->list_node);
				uk_list_add_tail(&signal->list_node, &tss->pending_list);
				sigdelset(&unblocked_mask, signal->siginfo.si_signo);
			}
		}
		/* move signals from process pending to pending list */
		uk_thread_signal_state_add_pending_from_process(tss, &unblocked_mask);
		/* handle the new pending signals */
		if (tss == uk_thread_signal_state_current())
			uk_thread_signal_state_current_handle_pending();
	}
out:
	return rc;
}

int uk_thread_signal_state_notify(struct uk_thread_signal_state *tss, int signum)
{
	struct uk_signal *signal;
	int rc;

	if (!signum_is_valid(signum)) {
		errno = EINVAL;
		return -1;
	}

	/* check if we are sending this to ourself */
	if (tss == uk_thread_signal_state_current()) {
		if (!sigismember(&tss->blocked_mask, signum)) {
			/* remove the signal from pending */
			signal = uk_thread_signal_state_remove_pending(tss, signum);
			if (!signal) {
				signal = uk_signal_create(signum);
				if (!signal) {
					uk_pr_warn("Could not create signal");
					rc = -1;
					goto out;
				}
			}
			/* handle the signal */
			uk_signal_handle(signal);
			return 0;
		}
	}

	signal = uk_signal_create(signum);
	if (!signal) {
		uk_pr_warn("Could not create signal");
		rc = -1;
		goto out;
	}

	rc = uk_thread_signal_state_add_pending(tss, signal);
	if (rc == 1)
		rc = 0; /* success */
out:
	return rc;
}

int uk_thread_signal_state_current_handle_pending(void)
{
	struct uk_thread_signal_state *tss;
	struct uk_signal *signal, *next;
	int handled_num = 0;

	tss = uk_thread_signal_state_current();
	if (tss->explicit_waiting)
		goto out;

	uk_list_for_each_entry_safe(signal, next, &tss->pending_list, list_node) {
		sigdelset(&tss->pending_mask, signal->siginfo.si_signo);//TODO and other masks
		uk_list_del(&signal->list_node);
		uk_signal_handle(signal);
		handled_num++;
	}
out:
	return handled_num;
}

struct uk_signal *uk_signal_create(int signum)
{
	struct uk_signal *signal = NULL;

	signal = uk_malloc(uk_alloc_get_default(), sizeof(*signal));
	if (!signal) {
		uk_pr_warn("Could not allocate signal");
		errno = ENOMEM;
		goto out;
	}

	signal->siginfo.si_signo = signum;
	signal->siginfo.si_code = 0; /* TODO */
	signal->siginfo.si_pid = getpid();
out:
	return signal;
}

int uk_signal_destroy(struct uk_signal *signal)
{
	int rc = 0;

	if (!signal) {
		rc = -EINVAL;
		goto out;
	}

	uk_free(uk_alloc_get_default(), signal);
out:
	return rc;
}

int uk_signal_handle(struct uk_signal *signal)
{
	struct sigaction *act;
	struct uk_thread_signal_state *tss;
	sigset_t saved_blocked_mask;
	int signum, rc = 0;

	if (!signal) {
		rc = -EINVAL;
		goto out;
	}

	signum = signal->siginfo.si_signo;
	act = &pss.sigaction[signum - 1];
	if (IS_SIG_IGN(act))
		UK_CRASH("Handling ignored signal %d\n", signum);
	/* our default handler is shutdown */
	if (IS_SIG_DFL(act))
		UK_CRASH("Uncaught signal %d. Powering off.\n", signum);

	tss = uk_thread_signal_state_current();

	/* save blocked mask */
	sigcopyset(&saved_blocked_mask, &tss->blocked_mask);
	/* also block the signals being handled */
	sigorset(&tss->blocked_mask, &tss->blocked_mask, &act->sa_mask);

	/* run the handler */
	if (act->sa_flags & SA_SIGINFO)
		act->sa_sigaction(signum, &signal->siginfo, NULL);
	else
		act->sa_handler(signum);

	/* check if we need to reset handler */
	if (act->sa_flags & SA_RESETHAND) {
		act->sa_flags = 0;
		act->sa_handler = SIG_DFL;
	}

	uk_signal_destroy(signal);

	/* restore blocked mask */
	sigcopyset(&tss->blocked_mask, &saved_blocked_mask);
out:
	return rc;
}

int __libc_current_sigrtmin()
{
	return 35;
}

int __libc_current_sigrtmax()
{
	return _NSIG - 1;
}
