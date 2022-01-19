/******************************************************************************
 * profiling functions
 *
 * Copyright (c) 2020 Costin Lupu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __XENCLONE_PROFILE_H__
#define __XENCLONE_PROFILE_H__

#include <stdio.h>
#include <time.h>
#include <uk/print.h>

//#undef LIB_PROFILING

#define __noinstrument __attribute__((no_instrument_function))

#define NSECONDS_IN_SEC 1000000000
#define USECONDS_IN_SEC 1000000
#define MSECONDS_IN_SEC 1000

#define NSECONDS_IN_MSEC    (NSECONDS_IN_SEC / MSECONDS_IN_SEC)


static inline __noinstrument
time_t timespec_nsec(struct timespec *t)
{
	return t->tv_sec * NSECONDS_IN_SEC + t->tv_nsec;
}

static inline __noinstrument
unsigned long timespec_usec(struct timespec *t)
{
	return ((unsigned long) t->tv_sec) * USECONDS_IN_SEC + ((unsigned long) t->tv_nsec) / 1000;
}

static inline __noinstrument
time_t timespec_diff_nsec(struct timespec *start, struct timespec *end)
{
	return timespec_nsec(end) - timespec_nsec(start);
}

static inline __noinstrument
unsigned long timespec_diff_usec(struct timespec *start, struct timespec *end)
{
	unsigned long _end = timespec_usec(end);
	unsigned long _start = timespec_usec(start);
//	return (_end > _start) ? _end - _start : _start - _end;
	return _end - _start;
}

static inline __noinstrument
double timespec_diff_msec(struct timespec *start, struct timespec *end)
{
	return (double) timespec_diff_nsec(start, end) / NSECONDS_IN_MSEC;
}

static inline __noinstrument
double timespec_diff_sec(struct timespec *start, struct timespec *end)
{
	return (double) timespec_diff_nsec(start, end) / NSECONDS_IN_SEC;
}

#if LIB_PROFILING
#define PROFILE_PREFIX   "UNIKRAFT_TRACE "

extern __thread int __profile_lvl;


#define PROFILE_NESTED_TICK() \
	{ \
		struct timespec __profile_tick; \
		struct timespec __profile_tock; \
		double __profile_val; \
		__profile_lvl++; \
		clock_gettime(CLOCK_MONOTONIC, &__profile_tick); \

#define PROFILE_NESTED_TOCK_MSEC(_str) \
		clock_gettime(CLOCK_MONOTONIC, &__profile_tock); \
		__profile_val = timespec_diff_msec(&__profile_tick, &__profile_tock); \
		fprintf(stderr, PROFILE_PREFIX "%11.6lf %d %*s %s\n", \
			__profile_val, __profile_lvl, 2 * __profile_lvl, "", _str); \
		__profile_lvl--; \
	}

#define PROFILE_TS_SEC(fmt, ...) \
	do { \
		struct timespec __profile_ts; \
		clock_gettime(CLOCK_REALTIME, &__profile_ts); \
		fprintf(stderr, "%ld.%09ld " fmt "\n", \
            __profile_ts.tv_sec, __profile_ts.tv_nsec, ## __VA_ARGS__); \
	} while (0)


#if 1
struct profile {
	struct timespec start;
	struct timespec stop;
};

static inline int profile_start(struct profile *p)
{
	return clock_gettime(CLOCK_REALTIME, &p->start);
}

static inline int profile_stop(struct profile *p)
{
    return clock_gettime(CLOCK_REALTIME, &p->stop);
}

static inline double profile_msec(struct profile *p)
{
	return timespec_diff_msec(&p->start, &p->stop);
}

static inline time_t profile_sec(struct profile *p)
{
	return timespec_diff_sec(&p->start, &p->stop);
}

#define PROFILE_FILE "/root/xl.profile.out"

extern int libxl_domain_create_new_profile_trigger;

#define PROFILE_PRINT_MSEC(p, fmt, ...) \
    do { \
        if (libxl_domain_create_new_profile_trigger) { \
            FILE *fp = fopen(PROFILE_FILE, "a"); \
            fprintf(fp, "%.6lf" fmt "\n", profile_msec(p), ## __VA_ARGS__); \
            fclose(fp); \
        } \
    } while (0)

#endif

#else


#define profile_start(p)
#define profile_stop(p)

#define PROFILE_NESTED_TICK()
#define PROFILE_NESTED_TOCK_MSEC(_str)
#define PROFILE_PRINT_MSEC(p, fmt, ...)
#endif

#endif /* __XENCLONE_PROFILE_H__ */
