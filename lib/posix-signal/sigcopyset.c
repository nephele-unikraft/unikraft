#define _GNU_SOURCE
#include <signal.h>

#define SST_SIZE (_NSIG/8/sizeof(long))

int sigcopyset(sigset_t *dest, const sigset_t *src)
{
	unsigned long i = 0, *d = (void *) dest, *s = (void *) src;

	for (; i < SST_SIZE; i++)
		d[i] = s[i];
	return 0;
}
