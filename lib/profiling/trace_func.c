#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef __USE_GNU
#define __USE_GNU
#endif
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <link.h>
#include <uk/thread.h>
#include <uk/print.h>
#ifndef UK_DEBUG_TRACE
#define UK_DEBUG_TRACE 1
#endif
#include <uk/trace.h>
#include <hashtable.h>
#include "profile.h"


#ifndef INSTRUMENTED_LIB_PATH
#error "Define INSTRUMENTED_LIB_PATH"
#endif
#ifndef PROFILE_TRIGGER_INIT_VALUE
#define PROFILE_TRIGGER_INIT_VALUE 0
#endif

#if 0
__noinstrument int func_map_init(const char *filename);
__noinstrument const char *func_map_get(unsigned long address);

static unsigned long base_address;

static int __noinstrument
dl_iterate_phdr_callback(struct dl_phdr_info *info, size_t size, void *data)
{
    static int found = 0;

    if (found)
        goto out;

    if (!strcmp(info->dlpi_name, INSTRUMENTED_LIB_PATH)) {
        //TODO this was for libxl base_address = info->dlpi_addr + info->dlpi_phdr[0].p_vaddr;
        base_address = info->dlpi_addr;
        found = 1;
    }

out:
    return 0;
}
#endif


struct func_trace {
    struct timespec start;
    struct timespec stop;
	unsigned long func;
};

#define TRACE_NESTED_MAX 64
#define TRACE_NESTED_FILTER 32

static int started = 0;

static struct hashtable *threads_map;

struct thread_state {
	int current_level;
	int ignored_level;
	struct func_trace nested_trace[TRACE_NESTED_MAX];
};

int profile_trigger = PROFILE_TRIGGER_INIT_VALUE;

extern struct hashtable *trace_create_hashmap(void);


void __noinstrument __cyg_profile_func_enter(void *func, void *caller)
{
	struct uk_thread *t;
	struct thread_state *crnt;
    struct func_trace *trace;
    int rc;

    if (!profile_trigger)
        return;

    if (!started) {
#if 0
        rc = func_map_init(INSTRUMENTED_LIB_PATH);
        if (rc) {
            fprintf(stderr, "Error creating functions map\n");
            return;
        }
        dl_iterate_phdr(dl_iterate_phdr_callback, NULL);
#endif

        threads_map = trace_create_hashmap();
        if (!threads_map) {
            fprintf(stderr, "Error calling create_hashtable()\n");
            return;
        }

        started = 1;
    }

    t = uk_thread_current();
    crnt = hashtable_search(threads_map, t);
	if (!crnt) {
		/* new thread */
		crnt = malloc(sizeof(struct thread_state));
		assert(crnt);
		crnt->current_level = -1;
		crnt->ignored_level = 0;
		rc = hashtable_insert(threads_map, t, crnt);
		assert(rc);
	}

    if (crnt->current_level == TRACE_NESTED_FILTER) {
    	crnt->ignored_level++;
        return;
    }

    crnt->current_level++;
    assert(crnt->current_level < TRACE_NESTED_MAX);

    trace = &crnt->nested_trace[crnt->current_level];

    rc = clock_gettime(CLOCK_MONOTONIC, &trace->start);
    assert(rc == 0);

    trace->func = (unsigned long) func;

//    uk_pr_info("en %p %lx\n", t, trace->func);
}

#if 1
UK_TRACEPOINT(my_trace_func, "%ld.%09ld %lu %s", time_t, long, long, const char *);
#else
typedef __attribute__((unused)) char __assert_139[(sizeof("my_trace_func") < 255) ? 1 : -1];
typedef __attribute__((unused)) char __assert_139[(sizeof("%ld.%09ld %lu %s") < 255) ? 1 : -1];

__attribute((__section__(".uk_tracepoints_list")))
	 static struct {
	uint32_t magic;
	uint32_t size;
	uint64_t cookie;
	uint8_t args_nr;
	uint8_t name_len;
	uint8_t format_len;
	uint8_t sizes[4];
	uint8_t types[4];
	char name[sizeof("my_trace_func")];
	char format[sizeof("%ld.%09ld %lu %s")];
} __my_trace_func_regdata __attribute__((used))
= {
		0x65645054,
		sizeof(__my_trace_func_regdata),
		(uint64_t) &__my_trace_func_regdata,
		4,
		sizeof("my_trace_func"),
		sizeof("%ld.%09ld %lu %s"),
		{
				sizeof(time_t),
				sizeof(long),
				sizeof(long),
				sizeof(const char*)
		},
		{ (0), (0), (0), (0) },
		"my_trace_func",
		"%ld.%09ld %lu %s"
};

static inline void my_trace_func(time_t arg1, long arg2, long arg3, const char *arg4)
{
	unsigned long flags = ukplat_lcpu_save_irqf();
	size_t free __attribute__((unused));
	char *buff = __uk_trace_get_buff(&free);
	if (buff) {
		__uk_trace_save_arg(&buff, &free, (0), sizeof(arg1), (long) arg1);
		__uk_trace_save_arg(&buff, &free, (0), sizeof(arg2), (long) arg2);
		__uk_trace_save_arg(&buff, &free, (0), sizeof(arg3), (long) arg3);
		__uk_trace_save_arg(&buff, &free, (0), sizeof(arg4), (long) arg4);
		__uk_trace_finalize_buff(buff, &__my_trace_func_regdata);
	}
	ukplat_lcpu_restore_irqf(flags);
}
#endif

 
void __noinstrument __cyg_profile_func_exit(void *func, void *caller)
{
	struct uk_thread *t;
	struct thread_state *crnt;
    struct func_trace *trace;
    unsigned long duration;
    const char *func_name;
    int rc;

    if (!profile_trigger)
        return;

    t = uk_thread_current();
    crnt = hashtable_search(threads_map, t);
    if (!crnt) {
    	uk_pr_info("no info %lx\n", t);
    	return;
    	while (1);
    }
    assert(crnt);
    assert(crnt->current_level < TRACE_NESTED_MAX);

    if (crnt->current_level < 0)
        return;

    if (crnt->ignored_level) {
    	crnt->ignored_level--;
        return;
    }

    trace = &crnt->nested_trace[crnt->current_level];
    assert(trace->func == (unsigned long) func);

    rc = clock_gettime(CLOCK_MONOTONIC, &trace->stop);
    assert(rc == 0);

    duration = timespec_diff_usec(&trace->start, &trace->stop);
    if (duration >= 10000) {
#if 1
    	char addr[64];

//        uk_pr_info("trace->func=%lx addr=%lx\n", trace->func, addr);
    	snprintf(addr, sizeof(addr), "0x%08lx", trace->func);
    	func_name = addr;
#else
        func_name = func_map_get(trace->func - base_address);
#endif

#if 1
        uk_pr_info("%ld.%09ld %ld.%09ld %lu %s\n",
            trace->start.tv_sec, trace->start.tv_nsec,
            trace->stop.tv_sec, trace->stop.tv_nsec,
            (long) duration, func_name);
#else
        my_trace_func(trace->start.tv_sec, trace->start.tv_nsec, (long) duration, func_name);
#endif
    }

    crnt->current_level--;
}
