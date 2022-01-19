/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Costin Lupu <costin.lupu@cs.pub.ro>
 *
 * Copyright (c) 2022, University Politehnica of Bucharest. All rights reserved.
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

#ifndef __UK_9PCACHE__
#define __UK_9PCACHE__

#include <vfscore/vnode.h>
#include <uk/9pfid.h>

#ifdef __cplusplus
extern "C" {
#endif

struct uk_9pfid *p9_fid_cache_get(struct vnode *dvp, char *name,
		struct uk_9p_stat *stat);
bool p9_fid_cache_contains(struct uk_9pfid *fid);

int p9_fid_cache_add(struct vnode *dvp, char *name, struct uk_9pfid *fid,
		struct uk_9p_stat *stat);
int p9_fid_cache_del(struct uk_9pfid *fid);


struct uk_9pfid *p9_wrfid_cache_get(struct uk_9pfid *orig);
int p9_wrfid_cache_add(struct uk_9pfid *orig, struct uk_9pfid *clone);
int p9_wrfid_cache_del(struct uk_9pfid *orig);

void p9_cache_del(struct uk_9pfid *fid);

#endif /* __UK_9PCACHE__ */
