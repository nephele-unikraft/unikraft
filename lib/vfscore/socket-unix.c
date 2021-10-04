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

#include <uk/config.h>
#include <stdlib.h>
#include <uk/plat/clone.h>
#include <vfscore/file.h>
#include <vfscore/vnode.h>
#include <vfscore/fs.h>
#include <vfscore/mount.h>
#if CONFIG_LWIP_SOCKET
#include <lwip/sockets.h>
#else
#error "Include sockets definitions header"
#endif
#include "pipe_buf.h"


struct sock_unix_file {
	struct pipe_buf *sender;
	struct pipe_buf *receiver;

	/* Write reference count */
	int w_refcount;
	/* Read reference count */
	int r_refcount;
	/* Flags */
	int flags;
};


struct sock_unix_file *
sock_unix_file_alloc(struct pipe_buf *sender, struct pipe_buf *receiver)
{
	struct sock_unix_file *su_file;

	su_file = malloc(sizeof(*su_file));
	if (!su_file)
		return NULL;

	su_file->sender = sender;
	su_file->sender->refcount++;

	su_file->receiver = receiver;
	su_file->receiver->refcount++;

	su_file->w_refcount = 1;
	su_file->r_refcount = 1;
	su_file->flags = 0;

	return su_file;
}

void sock_unix_file_free(struct sock_unix_file *su_file)
{
	su_file->sender->refcount--;
	pipe_buf_free(su_file->sender);

	su_file->receiver->refcount--;
	pipe_buf_free(su_file->receiver);

	free(su_file);
}

static int sock_unix_write(struct vnode *vnode,
		struct uio *buf, int ioflag __unused)
{
	struct sock_unix_file *su_file = vnode->v_data;
	// TODO bool nonblocking = false; /* TODO handle nonblocking */

	if (!su_file->r_refcount) {
		/* TODO before returning the error, send a SIGPIPE signal */
		return -EPIPE;
	}

	return pipe_buf_write(su_file->sender, buf, true);
}

static int sock_unix_read(struct vnode *vnode,
		struct vfscore_file *vfscore_file,
		struct uio *buf, int ioflag __unused)
{
	struct sock_unix_file *su_file = vnode->v_data;
	bool nonblocking = (vfscore_file->f_flags & O_NONBLOCK);

	return pipe_buf_read(su_file->receiver, buf, nonblocking);
}

static int sock_unix_can_read(struct vnode *vnode,
		struct vfscore_file *vfscore_file __unused)
{
	struct sock_unix_file *su_file = vnode->v_data;

	return pipe_buf_can_read(su_file->receiver);
}

static int sock_unix_can_write(struct vnode *vnode,
		struct vfscore_file *vfscore_file __unused)
{
	struct sock_unix_file *su_file = vnode->v_data;

	return pipe_buf_can_write(su_file->sender);
}

static int sock_unix_poll_register(struct vnode *vnode,
		struct vfscore_file *vfscore_file,
		struct vfscore_poll *poll)
{
	struct sock_unix_file *su_file = vnode->v_data;
	int rc;

	if (poll->events | POLLIN) {
		if ((vfscore_file->f_flags & UK_FREAD) == 0) {
			rc = -EINVAL;
			goto out;
		}
		rc = ukplat_add_waiter(su_file->receiver->rdntfr, &poll->waiter);

	} else if (poll->events | POLLOUT) {
		if ((vfscore_file->f_flags & UK_FWRITE) == 0) {
			rc = -EINVAL;
			goto out;
		}
		rc = ukplat_add_waiter(su_file->sender->wrntfr, &poll->waiter);

	} else
		rc = -EINVAL;
out:
	return rc;
}

static int sock_unix_poll_unregister(struct vnode *vnode,
		struct vfscore_file *vfscore_file,
		struct vfscore_poll *poll)
{
	struct sock_unix_file *su_file = vnode->v_data;
	int rc;

	if (poll->events | POLLIN) {
		if ((vfscore_file->f_flags & UK_FREAD) == 0) {
			rc = -EINVAL;
			goto out;
		}
		rc = ukplat_remove_waiter(su_file->receiver->rdntfr, &poll->waiter);

	} else if (poll->events | POLLOUT) {
		if ((vfscore_file->f_flags & UK_FWRITE) == 0) {
			rc = -EINVAL;
			goto out;
		}
		rc = ukplat_remove_waiter(su_file->sender->wrntfr, &poll->waiter);

	} else
		rc = -EINVAL;
out:
	return rc;
}

static int sock_unix_close(struct vnode *vnode,
		struct vfscore_file *vfscore_file)
{
	struct sock_unix_file *su_file = vnode->v_data;
	int open_buffers = 2;

	UK_ASSERT(vfscore_file->f_dentry->d_vnode == vnode);
	UK_ASSERT(vnode->v_refcnt == 1);

	if (vfscore_file->f_flags & UK_FREAD)
		su_file->r_refcount--;
	if (!su_file->r_refcount) {
		pipe_buf_close_read(su_file->receiver);
		open_buffers--;
	}

	if (vfscore_file->f_flags & UK_FWRITE)
		su_file->w_refcount--;
	if (!su_file->w_refcount) {
		pipe_buf_close_write(su_file->sender);
		open_buffers--;
	}

	if (open_buffers == 0)
		sock_unix_file_free(su_file);

	return 0;
}

static int sock_unix_seek(struct vnode *vnode __unused,
			struct vfscore_file *vfscore_file __unused,
			off_t off1 __unused, off_t off2 __unused)
{
	errno = ESPIPE;
	return -1;
}

static int sock_unix_ioctl(struct vnode *vnode,
		struct vfscore_file *vfscore_file __unused,
		unsigned long com, void *data)
{
	struct sock_unix_file *su_file = vnode->v_data;
	struct pipe_buf *pipe_buf = su_file->receiver;

	switch (com) {
	case FIONREAD:
		*((int *) data) = pipe_buf_get_available(pipe_buf);
		return 0;
	case FIOASYNC:
		return 0;
	default:
		return -EINVAL;
	}
}

#define sock_unix_open        ((vnop_open_t) vfscore_vop_einval)
#define sock_unix_fsync       ((vnop_fsync_t) vfscore_vop_nullop)
#define sock_unix_readdir     ((vnop_readdir_t) vfscore_vop_einval)
#define sock_unix_lookup      ((vnop_lookup_t) vfscore_vop_einval)
#define sock_unix_create      ((vnop_create_t) vfscore_vop_einval)
#define sock_unix_remove      ((vnop_remove_t) vfscore_vop_einval)
#define sock_unix_rename      ((vnop_rename_t) vfscore_vop_einval)
#define sock_unix_mkdir       ((vnop_mkdir_t) vfscore_vop_einval)
#define sock_unix_rmdir       ((vnop_rmdir_t) vfscore_vop_einval)
#define sock_unix_getattr     ((vnop_getattr_t) vfscore_vop_einval)
#define sock_unix_setattr     ((vnop_setattr_t) vfscore_vop_nullop)
#define sock_unix_inactive    ((vnop_inactive_t) vfscore_vop_einval)
#define sock_unix_truncate    ((vnop_truncate_t) vfscore_vop_nullop)
#define sock_unix_link        ((vnop_link_t) vfscore_vop_eperm)
#define sock_unix_cache       ((vnop_cache_t) NULL)
#define sock_unix_readlink    ((vnop_readlink_t) vfscore_vop_einval)
#define sock_unix_symlink     ((vnop_symlink_t) vfscore_vop_eperm)
#define sock_unix_fallocate   ((vnop_fallocate_t) vfscore_vop_nullop)

static struct vnops sock_unix_vnops = {
	.vop_open      = sock_unix_open,
	.vop_close     = sock_unix_close,
	.vop_read      = sock_unix_read,
	.vop_write     = sock_unix_write,
	.vop_can_read  = sock_unix_can_read,
	.vop_can_write = sock_unix_can_write,
	.vop_poll_register = sock_unix_poll_register,
	.vop_poll_unregister = sock_unix_poll_unregister,
	.vop_seek      = sock_unix_seek,
	.vop_ioctl     = sock_unix_ioctl,
	.vop_fsync     = sock_unix_fsync,
	.vop_readdir   = sock_unix_readdir,
	.vop_lookup    = sock_unix_lookup,
	.vop_create    = sock_unix_create,
	.vop_remove    = sock_unix_remove,
	.vop_rename    = sock_unix_rename,
	.vop_mkdir     = sock_unix_mkdir,
	.vop_rmdir     = sock_unix_rmdir,
	.vop_getattr   = sock_unix_getattr,
	.vop_setattr   = sock_unix_setattr,
	.vop_inactive  = sock_unix_inactive,
	.vop_truncate  = sock_unix_truncate,
	.vop_link      = sock_unix_link,
	.vop_cache     = sock_unix_cache,
	.vop_fallocate = sock_unix_fallocate,
	.vop_readlink  = sock_unix_readlink,
	.vop_symlink   = sock_unix_symlink
};

#define sock_unix_vget  ((vfsop_vget_t) vfscore_vop_nullop)

static struct vfsops sock_unix_vfsops = {
	.vfs_vget = sock_unix_vget,
	.vfs_vnops = &sock_unix_vnops
};

static uint64_t sock_unix_inode;

static struct mount sock_unix_mount = {
	.m_op = &sock_unix_vfsops
};


static int sock_unix_fd_alloc(struct sock_unix_file *su_file, int flags)
{
	int ret = 0;
	int vfs_fd;
	struct vfscore_file *vfs_file = NULL;
	struct dentry *su_dentry;
	struct vnode *su_vnode;

	/* Reserve file descriptor number */
	vfs_fd = vfscore_alloc_fd();
	if (vfs_fd < 0) {
		ret = -ENFILE;
		goto ERR_EXIT;
	}

	/* Allocate file, dentry, and vnode */
	vfs_file = calloc(1, sizeof(*vfs_file));
	if (!vfs_file) {
		ret = -ENOMEM;
		goto ERR_MALLOC_VFS_FILE;
	}

	ret = vfscore_vget(&sock_unix_mount, sock_unix_inode++, &su_vnode);
	UK_ASSERT(ret == 0); /* we should not find it in cache */

	if (!su_vnode) {
		ret = -ENOMEM;
		goto ERR_ALLOC_VNODE;
	}

	uk_mutex_unlock(&su_vnode->v_lock);

	su_dentry = dentry_alloc(NULL, su_vnode, "/");
	if (!su_dentry) {
		ret = -ENOMEM;
		goto ERR_ALLOC_DENTRY;
	}

	/* Fill out necessary fields. */
	vfs_file->fd = vfs_fd;
	vfs_file->f_flags = flags;
	vfs_file->f_count = 1;
	vfs_file->f_data = su_file;
	vfs_file->f_dentry = su_dentry;
	vfs_file->f_vfs_flags = UK_VFSCORE_NOPOS;

	su_vnode->v_data = su_file;
	su_vnode->v_type = VFIFO;

	/* Assign the file descriptors to the corresponding vfs_file. */
	ret = vfscore_install_fd(vfs_fd, vfs_file);
	if (ret)
		goto ERR_VFS_INSTALL;

	/* Only the dentry should hold a reference; release ours */
	vrele(su_vnode);

	return vfs_fd;

ERR_VFS_INSTALL:
	drele(su_dentry);
ERR_ALLOC_DENTRY:
	vrele(su_vnode);
ERR_ALLOC_VNODE:
	free(vfs_file);
ERR_MALLOC_VFS_FILE:
	vfscore_put_fd(vfs_fd);
ERR_EXIT:
	UK_ASSERT(ret < 0);
	return ret;
}

int socketpair_af_local(int type, int proto, int sv[2])
{
    struct pipe_buf *b1 = NULL, *b2 = NULL;
    struct sock_unix_file *s1 = NULL, *s2 = NULL;
	int fd1 = -1, fd2 = -1, rc = 0;

	if (type != SOCK_STREAM || proto != 0) {
		rc = -EINVAL;
		goto out_err;
	}

    b1 = pipe_buf_alloc(PIPE_MAX_SIZE);
    if (!b1) {
    	rc = -ENOMEM;
    	goto out_err;
    }
    b2 = pipe_buf_alloc(PIPE_MAX_SIZE);
    if (!b2) {
    	rc = -ENOMEM;
    	goto out_err;
    }

    s1 = sock_unix_file_alloc(b1, b2);
    if (!s1) {
    	rc = -ENOMEM;
    	goto out_err;
    }
    fd1 = sock_unix_fd_alloc(s1, UK_FREAD | UK_FWRITE);
	if (fd1 < 0) {
		rc = fd1;
		goto out_err;
	}

    s2 = sock_unix_file_alloc(b2, b1);
    if (!s2) {
    	rc = -ENOMEM;
    	goto out_err;
    }
	fd2 = sock_unix_fd_alloc(s2, UK_FREAD | UK_FWRITE);
	if (fd2 < 0) {
		rc = fd2;
		goto out_err;
	}

	sv[0] = fd1;
	sv[1] = fd2;

out_err:
	if (rc) {
		if (fd1 >= 0)
			vfscore_put_fd(fd1);
		if (fd2 >= 0)
			vfscore_put_fd(fd2);
		if (s1)
			sock_unix_file_free(s1);
		if (s2)
			sock_unix_file_free(s2);
		if (b1)
			pipe_buf_free(b1);
		if (b2)
			pipe_buf_free(b2);
	}
	return rc;
}
