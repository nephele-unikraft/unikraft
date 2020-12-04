/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Bogdan-George Lascu <lascu.bogdan96@gmail.com>
 *          Costin Lupu <costin.lupu@cs.pub.ro>
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
#include <uk/mutex.h>
#include <uk/wait.h>
#include <vfscore/uio.h>

/* We use the default size in Linux kernel */
#define PIPE_MAX_SIZE	(1 << CONFIG_LIBVFSCORE_PIPE_SIZE_ORDER)

struct pipe_buf {
	/* The buffer */
	char *data;
	/* The buffer capacity, always a power of 2 */
	unsigned long capacity;
	/* Producer index */
	unsigned long prod;
	/* Consumer index */
	unsigned long cons;

	/* Read lock */
	struct uk_mutex rdlock;
	/* Write lock */
	struct uk_mutex wrlock;

	/* Readers queue */
	struct uk_waitq rdwq;
	/* Writers queue */
	struct uk_waitq wrwq;
	/* Reference counter */
	unsigned long refcount;
};


struct pipe_buf *pipe_buf_alloc(int capacity);
void pipe_buf_free(struct pipe_buf *pipe_buf);

static inline
unsigned long pipe_buf_get_available(struct pipe_buf *pipe_buf)
{
	unsigned long avail;

	uk_mutex_lock(&pipe_buf->wrlock);
	uk_mutex_lock(&pipe_buf->rdlock);
	avail = pipe_buf->prod - pipe_buf->cons;
	uk_mutex_unlock(&pipe_buf->rdlock);
	uk_mutex_unlock(&pipe_buf->wrlock);
	return avail;
}

static inline
unsigned long pipe_buf_get_free_space(struct pipe_buf *pipe_buf)
{
	return pipe_buf->capacity - pipe_buf_get_available(pipe_buf);
}

static inline
int pipe_buf_can_write(struct pipe_buf *pipe_buf)
{
	return pipe_buf_get_free_space(pipe_buf) > 0;
}

static inline
int pipe_buf_can_read(struct pipe_buf *pipe_buf)
{
	return pipe_buf_get_available(pipe_buf) > 0;
}

int pipe_buf_write(struct pipe_buf *pipe_buf, struct uio *buf,
		bool nonblocking);
int pipe_buf_read(struct pipe_buf *pipe_buf, struct uio *buf,
		bool nonblocking);
