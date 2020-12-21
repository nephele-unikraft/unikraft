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

#include <sys/types.h>
#include <sys/socket.h>
#include <uk/print.h>
#include <vfscore/file.h>
#include <vfscore/vnode.h>


static int socket_is_net(int s)
{
	struct vfscore_file *file;
	int rc = 0;

	file = vfscore_get_file(s);
	if (!file)
		goto out;
	rc = (file->f_dentry->d_vnode->v_type == VSOCK);
	vfscore_put_file(file);
out:
	return rc;
}

extern int socket_recvmsg(int s, struct msghdr *msg, int flags);
extern int socket_sendmsg(int s, const struct msghdr *message, int flags);

extern ssize_t pwritev(int fd, const struct iovec *iov, int iovcnt, off_t offset);
extern ssize_t preadv(int fd, const struct iovec *iov, int iovcnt, off_t offset);

int sendmsg(int s, const struct msghdr *msg, int flags)
{
	int rc;

	if (socket_is_net(s))
		rc = socket_sendmsg(s, msg, flags);
	else {
		if (msg->msg_controllen > 0) {
			rc = pwrite(s, msg->msg_control, msg->msg_controllen, 0);
			if (rc != (int) msg->msg_controllen)
				goto out;
		}
		rc = pwritev(s, msg->msg_iov, msg->msg_iovlen, 0);
	}
out:
	return rc;
}

int recvmsg(int s, struct msghdr *msg, int flags)
{
	int rc;

	if (socket_is_net(s))
		rc = socket_recvmsg(s, msg, flags);
	else {
		if (msg->msg_controllen > 0) {
			rc = pread(s, msg->msg_control, msg->msg_controllen, 0);
			if (rc != (int) msg->msg_controllen)
				goto out;
		}
		rc = preadv(s, msg->msg_iov, msg->msg_iovlen, 0);
	}
out:
	return rc;
}
