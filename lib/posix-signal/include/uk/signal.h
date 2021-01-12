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

#ifndef __UK_SIGNAL_H__
#define __UK_SIGNAL_H__

#include <stdbool.h>
#include <signal.h>
#include <uk/list.h>

#ifdef __cplusplus
extern "C" {
#endif

int sigcopyset(sigset_t *dest, const sigset_t *src);
int sigreverseset(sigset_t *dest, const sigset_t *src);

#define IS_SIG_DFL(act)	\
    (!((act)->sa_flags & SA_SIGINFO) && (act)->sa_handler == SIG_DFL)
#define IS_SIG_IGN(act)	\
    (!((act)->sa_flags & SA_SIGINFO) && (act)->sa_handler == SIG_IGN)

static inline int signum_is_valid(int signum)
{
	return (signum < _NSIG && signum > 0);
}

static inline void sigset_clear_unmaskable(sigset_t *set)
{
	sigdelset(set, SIGKILL);
	sigdelset(set, SIGSTOP);
}

struct uk_signal {
	siginfo_t siginfo;
	struct uk_list_head list_node;
};

struct uk_signal *uk_signal_create(int signum);
int uk_signal_destroy(struct uk_signal *signal);
int uk_signal_handle(struct uk_signal *signal);

/* TODO: add synchronization */

/*
 * Process signal state
 */

struct uk_process_signal_state {
	/* pending signals mask */
	sigset_t pending_mask;
	/* pending signals array */
	struct uk_signal *pending_signals[_NSIG - 1];
	/* signal handlers */
	struct sigaction sigaction[_NSIG - 1];
	/* threads signal states list */
	struct uk_list_head tss_list;
};

extern struct uk_process_signal_state pss;

int uk_process_signal_state_add_pending(struct uk_signal *signal);
struct uk_signal *uk_process_signal_state_remove_pending(int signum);
struct uk_signal *uk_process_signal_state_remove_pending_mask(sigset_t *set);
int uk_process_signal_state_notify_any(int signum);

/*
 * Thread signal state
 */

struct uk_thread_signal_state {
	/* waiting status */
	bool explicit_waiting;
	/* blocked signals */
	sigset_t blocked_mask;
	/* pending signals */
	sigset_t pending_mask;
	/* list of pending signals */
	struct uk_list_head pending_list;
	/* list of pending, but blocked, signals */
	struct uk_list_head blocked_list;
	/* tss list node */
	struct uk_list_head tss_list_node;
};

int  uk_thread_signal_state_init(struct uk_thread_signal_state *tss);
void uk_thread_signal_state_fini(struct uk_thread_signal_state *tss);

struct uk_thread_signal_state *uk_thread_signal_state_current(void);

struct uk_signal *uk_thread_signal_state_remove_pending(struct uk_thread_signal_state *tss, int signum);
struct uk_signal *uk_thread_signal_state_remove_pending_mask(struct uk_thread_signal_state *tss, sigset_t *set);

int uk_thread_signal_state_block(struct uk_thread_signal_state *tss, const sigset_t *set);
int uk_thread_signal_state_unblock(struct uk_thread_signal_state *tss, const sigset_t *set);
int uk_thread_signal_state_set_blocked(struct uk_thread_signal_state *tss, const sigset_t *set);

int uk_thread_signal_state_notify(struct uk_thread_signal_state *tss, int signum);

/* returns number of executed signal handlers */
int uk_thread_signal_state_current_handle_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* __UK_SIGNAL_H__ */
