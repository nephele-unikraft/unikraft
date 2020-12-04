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

#include <stdlib.h>
#include <string.h>
#include "pipe_buf.h"


#define PIPE_BUF_IDX(buf, n)    ((n) & ((buf)->capacity - 1))
#define PIPE_BUF_PROD_IDX(buf)  PIPE_BUF_IDX((buf), (buf)->prod)
#define PIPE_BUF_CONS_IDX(buf)  PIPE_BUF_IDX((buf), (buf)->cons)


struct pipe_buf *pipe_buf_alloc(int capacity)
{
	struct pipe_buf *pipe_buf;

	UK_ASSERT(POWER_OF_2(capacity));

	pipe_buf = malloc(sizeof(*pipe_buf));
	if (!pipe_buf)
		return NULL;

	pipe_buf->data = malloc(capacity);
	if (!pipe_buf->data) {
		free(pipe_buf);
		return NULL;
	}

	pipe_buf->capacity = capacity;
	pipe_buf->cons = 0;
	pipe_buf->prod = 0;
	uk_mutex_init(&pipe_buf->rdlock);
	uk_mutex_init(&pipe_buf->wrlock);
	uk_waitq_init(&pipe_buf->rdwq);
	uk_waitq_init(&pipe_buf->wrwq);

	return pipe_buf;
}

void pipe_buf_free(struct pipe_buf *pipe_buf)
{
	free(pipe_buf->data);
	free(pipe_buf);
}

static unsigned long pipe_buf_write_iovec(struct pipe_buf *pipe_buf,
		struct iovec *iovec, size_t iovec_off)
{
	unsigned long prod_idx, to_write;
	void *iovec_data = iovec->iov_base + iovec_off;
	size_t iov_len = iovec->iov_len - iovec_off;

	prod_idx = PIPE_BUF_PROD_IDX(pipe_buf);
	to_write = MIN(pipe_buf_get_free_space(pipe_buf), iov_len);
	if (to_write == 0)
		goto out;

	/* Copy in one piece */
	if (prod_idx + to_write <= pipe_buf->capacity)
		memcpy(pipe_buf->data + prod_idx, iovec_data, to_write);

	else {
		int first_copy_bytes, second_copy_bytes;

		/* Copy the first part */
		first_copy_bytes = pipe_buf->capacity - prod_idx;
		memcpy(pipe_buf->data + prod_idx,
				iovec_data,
				first_copy_bytes);

		/* Copy the second part */
		second_copy_bytes = prod_idx + to_write - pipe_buf->capacity;
		memcpy(pipe_buf->data,
				iovec_data + first_copy_bytes,
				second_copy_bytes);
	}

	/* Update producer */
	pipe_buf->prod += to_write;

out:
	return to_write;
}

static unsigned long pipe_buf_read_iovec(struct pipe_buf *pipe_buf,
		struct iovec *iovec, size_t iovec_off)
{
	unsigned long cons_idx, to_read;
	void *iovec_data = iovec->iov_base + iovec_off;
	size_t iov_len = iovec->iov_len - iovec_off;

	cons_idx = PIPE_BUF_CONS_IDX(pipe_buf);
	to_read = MIN(pipe_buf_get_available(pipe_buf), iov_len);
	if (to_read == 0)
		goto out;

	/* Copy in one piece */
	if (cons_idx + to_read <= pipe_buf->capacity)
		memcpy(iovec_data, pipe_buf->data + cons_idx, to_read);

	else {
		int first_copy_bytes;
		int second_copy_bytes;

		/* Copy the first part */
		first_copy_bytes = pipe_buf->capacity - pipe_buf->cons;
		memcpy(iovec_data,
				pipe_buf->data + cons_idx,
				first_copy_bytes);

		/* Copy the second part */
		second_copy_bytes = cons_idx + to_read - pipe_buf->capacity;
		memcpy(iovec_data + first_copy_bytes,
				pipe_buf->data,
				second_copy_bytes);
	}

	/* Update consumer */
	pipe_buf->cons += to_read;

out:
	return to_read;
}

int pipe_buf_write(struct pipe_buf *pipe_buf, struct uio *buf,
		bool nonblocking)
{
	bool data_available = true;
	int uio_idx = 0;

	uk_mutex_lock(&pipe_buf->wrlock);
	while (data_available && uio_idx < buf->uio_iovcnt) {
		struct iovec *iovec = &buf->uio_iov[uio_idx];
		unsigned long off = 0;

		while (off < iovec->iov_len) {
			unsigned long written_bytes;

			written_bytes = pipe_buf_write_iovec(pipe_buf, iovec, off);
			if (written_bytes == 0) {
				/* No data */
				if (nonblocking) {
					data_available = false;
					break;

				} else {
					/* Wait until data available */
					while (!pipe_buf_can_write(pipe_buf)) {
						uk_mutex_unlock(&pipe_buf->wrlock);
						uk_waitq_wait_event(&pipe_buf->wrwq,
							pipe_buf_can_write(pipe_buf));
						uk_mutex_lock(&pipe_buf->wrlock);
					}
				}

			} else {
				/* Update bytes written_bytes. */
				buf->uio_resid -= written_bytes;

				off += written_bytes;

				/* wake some readers */
				uk_waitq_wake_up(&pipe_buf->rdwq);
			}
		}

		uio_idx++;
	}
	uk_mutex_unlock(&pipe_buf->wrlock);

	return 0;
}

int pipe_buf_read(struct pipe_buf *pipe_buf, struct uio *buf, bool nonblocking)
{
	bool data_available = true;
	int uio_idx = 0;

	uk_mutex_lock(&pipe_buf->rdlock);
	if (nonblocking && !pipe_buf_can_read(pipe_buf)) {
		uk_mutex_unlock(&pipe_buf->rdlock);
		return EAGAIN;
	}

	while (data_available && uio_idx < buf->uio_iovcnt) {
		struct iovec *iovec = &buf->uio_iov[uio_idx];
		unsigned long off = 0;

		while (off < iovec->iov_len) {
			unsigned long read_bytes;

			read_bytes = pipe_buf_read_iovec(pipe_buf, iovec, off);
			if (read_bytes == 0) {
				/* No data */
				if (nonblocking) {
					data_available = false;
					break;

				} else {
					/* Wait until data available */
					while (!pipe_buf_can_read(pipe_buf)) {
						uk_mutex_unlock(&pipe_buf->rdlock);
						uk_waitq_wait_event(&pipe_buf->rdwq,
							pipe_buf_can_read(pipe_buf));
						uk_mutex_lock(&pipe_buf->rdlock);
					}
				}

			} else {
				/* Update bytes read */
				buf->uio_resid -= read_bytes;

				off += read_bytes;

				/* wake some writers */
				uk_waitq_wake_up(&pipe_buf->wrwq);
			}
		}

		uio_idx++;
	}
	uk_mutex_unlock(&pipe_buf->rdlock);

	return 0;
}
