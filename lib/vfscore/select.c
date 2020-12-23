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

#include <errno.h>
#include <sys/select.h>
#include <uk/plat/time.h>
#include <vfscore/file.h>
#include <vfscore/vnode.h>
#include <vfscore/fs.h>
#include <vfscore/poll.h>


/* TODO move somewhere else */
int timeval_to_nsec(struct timeval *tval, __nsec *nsec)
{
	if (!tval || !nsec)
		return -EINVAL;

	*nsec = ukarch_time_sec_to_nsec(tval->tv_sec) +
			ukarch_time_usec_to_nsec(tval->tv_usec);

	return 0;
}


struct poll_table {
	struct vfscore_poll entries[FDTABLE_MAX_FILES];
};

extern int vfs_can_read(struct vfscore_file *fp);
extern int vfs_can_write(struct vfscore_file *fp);
extern int vfs_poll_register(struct vfscore_file *fp,
		struct vfscore_poll *poll);
extern int vfs_poll_unregister(struct vfscore_file *fp,
		struct vfscore_poll *poll);

static int fds_scan(int nfds,
		fd_set *in_rd, fd_set *in_wr, fd_set *in_ex,
		fd_set *out_rd, fd_set *out_wr, fd_set *out_ex __unused)
{
	int fd, n = 0;
	struct vfscore_file *f;

	for (fd = 0; fd < nfds; fd++) {
		f = vfscore_get_file(fd);
		if (!f)
			continue;
		switch (f->f_dentry->d_vnode->v_type) {
		case VSOCK:
		case VFIFO:
			if (in_rd && FD_ISSET(fd, in_rd)) {
				if ((f->f_flags & UK_FREAD) == 0)
					return -EBADF;
				if (vfs_can_read(f)) {
					FD_SET(fd, out_rd);
					n++;
				}
			}
			if (in_wr && FD_ISSET(fd, in_wr)) {
				if ((f->f_flags & UK_FWRITE) == 0)
					return -EBADF;
				if (vfs_can_write(f)) {
					FD_SET(fd, out_wr);
					n++;
				}
			}
			break;
		default:
			if ((in_rd && FD_ISSET(fd, in_rd))
				|| (in_wr && FD_ISSET(fd, in_wr))
				|| (in_ex && FD_ISSET(fd, in_ex)))
				uk_pr_err("Unsupported type %d for fd %d\n",
					f->f_dentry->d_vnode->v_type, fd);
			break;
		}
		vfscore_put_file(f);
	}

	return n;
}

static int select_wait(int nfds,
		fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
		__nsec until)
{
	struct uk_thread *current_thread;
	struct poll_table *tbl;
	struct vfscore_poll *tble;
	struct vfscore_file *f;
	__nsec now;
	int fd, rc = 0;

	tbl = calloc(1, sizeof(struct poll_table));
	if (!tbl) {
		rc = -ENOMEM;
		goto out;
	}

	current_thread = uk_thread_current();

	/* add waiters */
	for (fd = 0; fd < nfds; fd++) {
		if ((!readfds || !FD_ISSET(fd, readfds))
				&& (!writefds || !FD_ISSET(fd, writefds))
				&& (!exceptfds || !FD_ISSET(fd, exceptfds)))
			continue;
		f = vfscore_get_file(fd);

		tble = &tbl->entries[fd];
		tble->events = 0;
		if (FD_ISSET(fd, readfds))
			tble->events |= POLLIN;
		else if (FD_ISSET(fd, writefds))
			tble->events |= POLLOUT;
		tble->revents = 0;
		uk_waitq_entry_init(&tble->waiter, current_thread);

		rc = vfs_poll_register(f, tble);
		vfscore_put_file(f);
		if (rc) {
			/* either event or error */
			nfds = fd;
			break;
		}
	}

	if (rc == 0) {
		if (until) {
			now = ukplat_monotonic_clock();
			if (until > now)
				uk_sched_thread_sleep(until - now);
		} else {
			uk_thread_block(current_thread);
			uk_sched_yield();
		}
	}

	/* remove waiters */
	for (fd = 0; fd < nfds; fd++) {
		if ((!readfds || !FD_ISSET(fd, readfds))
				&& (!writefds || !FD_ISSET(fd, writefds))
				&& (!exceptfds || !FD_ISSET(fd, exceptfds)))
			continue;
		f = vfscore_get_file(fd);
		tble = &tbl->entries[fd];
		rc = vfs_poll_unregister(f, tble);
		UK_ASSERT(rc == 0);
		vfscore_put_file(f);
	}

	free(tbl);
out:
	return rc;
}

int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
		struct timeval *timeout)
{
	fd_set out_rd, out_wr, out_ex;
	__nsec nsec, until = 0;
	int rc;

	if (nfds < 0 || nfds > FD_SETSIZE) {
		errno = EINVAL;
		rc = -1;
		goto out;
	}

	if (timeout) {
		timeval_to_nsec(timeout, &nsec);
		if (nfds == 0) {
			uk_sched_thread_sleep(nsec);
			timeout->tv_sec = 0;
			timeout->tv_usec = 0;
			rc = 0;
			goto out;
		} else
			until = ukplat_monotonic_clock() + nsec;
	}

	FD_ZERO(&out_rd);
	FD_ZERO(&out_wr);
	FD_ZERO(&out_ex);
	rc = fds_scan(nfds, readfds, writefds, exceptfds,
			&out_rd, &out_wr, &out_ex);
	if (rc)
		/* some events or error */
		goto out;

	if (timeout) {
		/* deadline expired? */
		if (ukplat_monotonic_clock() >= until) {
			timeout->tv_sec = 0;
			timeout->tv_usec = 0;
			rc = 0;
			goto out;
		}
	}

	rc = select_wait(nfds, readfds, writefds, exceptfds, until);
	if (rc) {
		uk_pr_err("Error calling select_wait() rc=%d\n", rc);
		goto out;
	}

	rc = fds_scan(nfds, readfds, writefds, exceptfds,
			&out_rd, &out_wr, &out_ex);
	if (rc)
		/* some events or error */
		goto out;

	if (!timeout) {
		errno = EINTR;
		rc = -1;
	}
out:
	if (rc >= 0) {
		if (readfds)
			*readfds = out_rd;
		if (writefds)
			*writefds = out_wr;
		if (exceptfds)
			*exceptfds = out_ex;
	}
	return rc;
}
