/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Bogdan-George Lascu <lascu.bogdan96@gmail.com>
 *          Costin Lupu <costin.lupu@cs.pub.ro>
 *
 * Copyright (c) 2019, University Politehnica of Bucharest. All rights reserved.
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

#include <stdlib.h>
#include <sys/ioctl.h>
#include <vfscore/file.h>
#include <vfscore/fs.h>
#include <vfscore/mount.h>
#include <vfscore/vnode.h>
#include "pipe_buf.h"


struct pipe_file {
	/* Pipe buffer */
	struct pipe_buf *buf;
	/* Write reference count */
	int w_refcount;
	/* Read reference count */
	int r_refcount;
	/* Flags */
	int flags;
};

struct pipe_file *pipe_file_alloc(int capacity, int flags)
{
	struct pipe_file *pipe_file;

	pipe_file = malloc(sizeof(*pipe_file));
	if (!pipe_file)
		return NULL;

	pipe_file->buf = pipe_buf_alloc(capacity);
	if (!pipe_file->buf) {
		free(pipe_file);
		return NULL;
	}

	pipe_file->w_refcount = 1;
	pipe_file->r_refcount = 1;
	pipe_file->flags = flags;

	return pipe_file;
}

void pipe_file_free(struct pipe_file *pipe_file)
{
	pipe_buf_free(pipe_file->buf);
	free(pipe_file);
}

static int pipe_write(struct vnode *vnode,
		struct uio *buf, int ioflag __unused)
{
	struct pipe_file *pipe_file = vnode->v_data;
	struct pipe_buf *pipe_buf = pipe_file->buf;
	bool nonblocking = false; /* TODO handle nonblocking */

	if (!pipe_file->r_refcount) {
		/* TODO before returning the error, send a SIGPIPE signal */
		return -EPIPE;
	}

	return pipe_buf_write(pipe_buf, buf, nonblocking);
}

static int pipe_read(struct vnode *vnode,
		struct vfscore_file *vfscore_file,
		struct uio *buf, int ioflag __unused)
{
	struct pipe_file *pipe_file = vnode->v_data;
	struct pipe_buf *pipe_buf = pipe_file->buf;
	bool nonblocking = (vfscore_file->f_flags & O_NONBLOCK);

	return pipe_buf_read(pipe_buf, buf, nonblocking);
}

static int pipe_close(struct vnode *vnode,
		struct vfscore_file *vfscore_file)
{
	struct pipe_file *pipe_file = vnode->v_data;

	UK_ASSERT(vfscore_file->f_dentry->d_vnode == vnode);
	UK_ASSERT(vnode->v_refcnt == 1);

	if (vfscore_file->f_flags & UK_FREAD)
		pipe_file->r_refcount--;

	if (vfscore_file->f_flags & UK_FWRITE)
		pipe_file->w_refcount--;

	if (!pipe_file->r_refcount && !pipe_file->w_refcount)
		pipe_file_free(pipe_file);

	return 0;
}

static int pipe_seek(struct vnode *vnode __unused,
			struct vfscore_file *vfscore_file __unused,
			off_t off1 __unused, off_t off2 __unused)
{
	errno = ESPIPE;
	return -1;
}

static int pipe_ioctl(struct vnode *vnode,
		struct vfscore_file *vfscore_file __unused,
		unsigned long com, void *data)
{
	struct pipe_file *pipe_file = vnode->v_data;
	struct pipe_buf *pipe_buf = pipe_file->buf;

	switch (com) {
	case FIONREAD:
		uk_mutex_lock(&pipe_buf->rdlock);
		*((int *) data) = pipe_buf_get_available(pipe_buf);
		uk_mutex_unlock(&pipe_buf->rdlock);
		return 0;
	default:
		return -EINVAL;
	}
}

#define pipe_open        ((vnop_open_t) vfscore_vop_einval)
#define pipe_fsync       ((vnop_fsync_t) vfscore_vop_nullop)
#define pipe_readdir     ((vnop_readdir_t) vfscore_vop_einval)
#define pipe_lookup      ((vnop_lookup_t) vfscore_vop_einval)
#define pipe_create      ((vnop_create_t) vfscore_vop_einval)
#define pipe_remove      ((vnop_remove_t) vfscore_vop_einval)
#define pipe_rename      ((vnop_rename_t) vfscore_vop_einval)
#define pipe_mkdir       ((vnop_mkdir_t) vfscore_vop_einval)
#define pipe_rmdir       ((vnop_rmdir_t) vfscore_vop_einval)
#define pipe_getattr     ((vnop_getattr_t) vfscore_vop_einval)
#define pipe_setattr     ((vnop_setattr_t) vfscore_vop_nullop)
#define pipe_inactive    ((vnop_inactive_t) vfscore_vop_einval)
#define pipe_truncate    ((vnop_truncate_t) vfscore_vop_nullop)
#define pipe_link        ((vnop_link_t) vfscore_vop_eperm)
#define pipe_cache       ((vnop_cache_t) NULL)
#define pipe_readlink    ((vnop_readlink_t) vfscore_vop_einval)
#define pipe_symlink     ((vnop_symlink_t) vfscore_vop_eperm)
#define pipe_fallocate   ((vnop_fallocate_t) vfscore_vop_nullop)

static struct vnops pipe_vnops = {
	.vop_open      = pipe_open,
	.vop_close     = pipe_close,
	.vop_read      = pipe_read,
	.vop_write     = pipe_write,
	.vop_seek      = pipe_seek,
	.vop_ioctl     = pipe_ioctl,
	.vop_fsync     = pipe_fsync,
	.vop_readdir   = pipe_readdir,
	.vop_lookup    = pipe_lookup,
	.vop_create    = pipe_create,
	.vop_remove    = pipe_remove,
	.vop_rename    = pipe_rename,
	.vop_mkdir     = pipe_mkdir,
	.vop_rmdir     = pipe_rmdir,
	.vop_getattr   = pipe_getattr,
	.vop_setattr   = pipe_setattr,
	.vop_inactive  = pipe_inactive,
	.vop_truncate  = pipe_truncate,
	.vop_link      = pipe_link,
	.vop_cache     = pipe_cache,
	.vop_fallocate = pipe_fallocate,
	.vop_readlink  = pipe_readlink,
	.vop_symlink   = pipe_symlink
};

#define pipe_vget  ((vfsop_vget_t) vfscore_vop_nullop)

static struct vfsops pipe_vfsops = {
	.vfs_vget = pipe_vget,
	.vfs_vnops = &pipe_vnops
};

static uint64_t p_inode;

/*
 * Bogus mount point used by all sockets
 */
static struct mount p_mount = {
	.m_op = &pipe_vfsops
};

static int pipe_fd_alloc(struct pipe_file *pipe_file, int flags)
{
	int ret = 0;
	int vfs_fd;
	struct vfscore_file *vfs_file = NULL;
	struct dentry *p_dentry;
	struct vnode *p_vnode;

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

	ret = vfscore_vget(&p_mount, p_inode++, &p_vnode);
	UK_ASSERT(ret == 0); /* we should not find it in cache */

	if (!p_vnode) {
		ret = -ENOMEM;
		goto ERR_ALLOC_VNODE;
	}

	uk_mutex_unlock(&p_vnode->v_lock);

	p_dentry = dentry_alloc(NULL, p_vnode, "/");
	if (!p_dentry) {
		ret = -ENOMEM;
		goto ERR_ALLOC_DENTRY;
	}

	/* Fill out necessary fields. */
	vfs_file->fd = vfs_fd;
	vfs_file->f_flags = flags;
	vfs_file->f_count = 1;
	vfs_file->f_data = pipe_file;
	vfs_file->f_dentry = p_dentry;
	vfs_file->f_vfs_flags = UK_VFSCORE_NOPOS;

	p_vnode->v_data = pipe_file;
	p_vnode->v_type = VFIFO;

	/* Assign the file descriptors to the corresponding vfs_file. */
	ret = vfscore_install_fd(vfs_fd, vfs_file);
	if (ret)
		goto ERR_VFS_INSTALL;

	/* Only the dentry should hold a reference; release ours */
	vrele(p_vnode);

	return vfs_fd;

ERR_VFS_INSTALL:
	drele(p_dentry);
ERR_ALLOC_DENTRY:
	vrele(p_vnode);
ERR_ALLOC_VNODE:
	free(vfs_file);
ERR_MALLOC_VFS_FILE:
	vfscore_put_fd(vfs_fd);
ERR_EXIT:
	UK_ASSERT(ret < 0);
	return ret;
}

int pipe(int pipefd[2])
{
	int ret = 0;
	int r_fd, w_fd;
	struct pipe_file *pipe_file;

	/* Allocate pipe internal structure. */
	pipe_file = pipe_file_alloc(PIPE_MAX_SIZE, 0);
	if (!pipe_file) {
		ret = -ENOMEM;
		goto ERR_EXIT;
	}

	r_fd = pipe_fd_alloc(pipe_file, UK_FREAD);
	if (r_fd < 0)
		goto ERR_VFS_INSTALL;

	w_fd = pipe_fd_alloc(pipe_file, UK_FWRITE);
	if (w_fd < 0)
		goto ERR_W_FD;

	/* Fill pipefd fields. */
	pipefd[0] = r_fd;
	pipefd[1] = w_fd;

	return ret;

ERR_W_FD:
	vfscore_put_fd(r_fd);
ERR_VFS_INSTALL:
	pipe_file_free(pipe_file);
ERR_EXIT:
	UK_ASSERT(ret < 0);
	return ret;
}

/* TODO find a more efficient way to implement pipe2() */
int pipe2(int pipefd[2], int flags)
{
	int rc;

	rc = pipe(pipefd);
	if (rc)
		return rc;

	if (flags & O_CLOEXEC) {
		fcntl(pipefd[0], F_SETFD, FD_CLOEXEC);
		fcntl(pipefd[1], F_SETFD, FD_CLOEXEC);
	}
	if (flags & O_NONBLOCK) {
		fcntl(pipefd[0], F_SETFL, O_NONBLOCK);
		fcntl(pipefd[1], F_SETFL, O_NONBLOCK);
	}
	return 0;
}

/* TODO maybe find a better place for this when it will be implemented */
int mkfifo(const char *path __unused, mode_t mode __unused)
{
	WARN_STUBBED();
	errno = ENOTSUP;
	return -1;
}
