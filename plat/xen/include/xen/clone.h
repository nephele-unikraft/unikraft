/******************************************************************************
 * clone.h
 *
 * Hypervisor interface for cloning support
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Copyright (c) 2020, Costin Lupu <costin.lupu@cs.pub.ro>
 */

#ifndef __XEN_PUBLIC_CLONE_H__
#define __XEN_PUBLIC_CLONE_H__

#include "xen.h"

/*
 * Cloning hypercall operations
 */

#define CLONEOP_enable 0
struct clone_enable_info {
    unsigned long ring_vaddr;
    unsigned long pages_num;
};
typedef struct clone_enable_info clone_enable_info_t;
DEFINE_XEN_GUEST_HANDLE(clone_enable_info_t);


#define CLONEOP_disable 1


DEFINE_XEN_GUEST_HANDLE(domid_t);

#define CLONEOP_clone 2
struct clone_op {
    /* IN parameters. */
    domid_t parent_id;
    unsigned long start_info_mfn;
    uint32_t nr_children;
    /* OUT parameters. */
    XEN_GUEST_HANDLE(domid_t) child_list;
};
typedef struct clone_op clone_op_t;
DEFINE_XEN_GUEST_HANDLE(clone_op_t);


#define CLONEOP_clone_completion 3
struct clone_completion {
    uint32_t id;
};
typedef struct clone_completion clone_completion_t;

struct clone_notification {
    uint32_t id;
    domid_t parent_id;
    domid_t child_id;
    unsigned long parent_start_info_mfn;
    unsigned long child_start_info_mfn;
};
typedef struct clone_notification clone_notification_t;


/*
 * Cloning notification ring definitions
 */

#define CLONING_RING_MAX_PAGES 32

struct clone_notification_ring_header {
    unsigned long entries_num;
    unsigned long prod_idx;
    unsigned long cons_idx;
};
typedef struct clone_notification_ring_header clone_notification_ring_header_t;

struct clone_notification_ring {
    clone_notification_ring_header_t hdr;
    clone_notification_t entries[];
};
typedef struct clone_notification_ring clone_notification_ring_t;

#define CLONE_RING_IDX(ring, idx) \
    ((idx) & ((ring)->hdr.entries_num - 1))

#define CLONE_RING_PROD_IDX(ring) \
    CLONE_RING_IDX((ring), (ring)->hdr.prod_idx)

#define CLONE_RING_CONS_IDX(ring) \
    CLONE_RING_IDX((ring), (ring)->hdr.cons_idx)

#endif /* __XEN_PUBLIC_CLONE_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
