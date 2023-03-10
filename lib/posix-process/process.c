/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Felipe Huici <felipe.huici@neclab.eu>
 *          Costin Lupu <costin.lupu@cs.pub.ro>
 *
 * Copyright (c) 2017, NEC Europe Ltd., NEC Corporation. All rights reserved.
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

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <uk/process.h>
#include <uk/print.h>
#include <uk/syscall.h>
#include <uk/plat/clone.h>


pid_t fork(void)
{
	unsigned short child_id;
	pid_t pid;
	int rc;

	rc = ukplat_clone(1, &child_id);
	if (rc == 0) /* parent */
		pid = child_id;
	else if (rc == 1) /* child */
		pid = 0;
	else {
		uk_pr_err("Error calling ukplat_clone() rc=%d\n", rc);
		pid = -1;
	}

	return pid;
}

int vfork(void)
{
	/* vfork() is not supported on this platform */
	errno = ENOSYS;
	return -1;
}

static void exec_warn_argv_variadic(const char *arg, va_list args)
{
	int i = 1;
	char *argi;

	uk_pr_warn(" argv=[%s", arg);

	argi = va_arg(args, char *);
	while (argi) {
		uk_pr_warn("%s%s", (i > 0 ? ", " : ""), argi);
		i++;
		argi = va_arg(args, char *);
	}
	uk_pr_warn("]\n");
}

static void __exec_warn_array(const char *name, char *const argv[])
{
	int i = 0;

	uk_pr_warn(" %s=[", name);

	if (argv) {
		while (argv[i]) {
			uk_pr_warn("%s%s", (i > 0 ? ", " : ""), argv[i]);
			i++;
		}
	}
	uk_pr_warn("]\n");
}
#define exec_warn_argv(values) __exec_warn_array("argv", values)
#define exec_warn_envp(values) __exec_warn_array("envp", values)

int execl(const char *path, const char *arg, ...
		/* (char  *) NULL */)
{
	va_list args;

	uk_pr_warn("%s(): path=%s\n", __func__, path);
	va_start(args, arg);
	exec_warn_argv_variadic(arg, args);
	va_end(args);
	errno = ENOSYS;
	return -1;
}

int execlp(const char *file, const char *arg, ...
		/* (char  *) NULL */)
{
	va_list args;

	uk_pr_warn("%s(): file=%s\n", __func__, file);
	va_start(args, arg);
	exec_warn_argv_variadic(arg, args);
	va_end(args);
	errno = ENOSYS;
	return -1;
}

int execle(const char *path, const char *arg, ...
		/*, (char *) NULL, char * const envp[] */)
{
	va_list args;
	char * const *envp;

	uk_pr_warn("%s(): path=%s\n", __func__, path);
	va_start(args, arg);
	exec_warn_argv_variadic(arg, args);
	envp = va_arg(args, char * const *);
	exec_warn_envp(envp);
	va_end(args);
	errno = ENOSYS;
	return -1;
}

int execve(const char *path, char *const argv[], char *const envp[])
{
	struct stat s;
	int rc;

	uk_pr_warn("%s(): path=%s\n", __func__, path);
	rc = stat(path, &s);
	if (rc) {
		errno = ENOENT;
		rc = -1;
		goto out;
	}
	if (!S_ISREG(s.st_mode)) {
		errno = ENOENT;
		rc = -1;
		goto out;
	}

	exec_warn_argv(argv);
	exec_warn_envp(envp);
	errno = ENOSYS;
out:
	return rc;
}

int execv(const char *path, char *const argv[])
{
	uk_pr_warn("%s(): path=%s\n", __func__, path);
	exec_warn_argv(argv);
	errno = ENOSYS;
	return -1;
}

int execvp(const char *file, char *const argv[])
{
	uk_pr_warn("%s(): file=%s\n", __func__, file);
	exec_warn_argv(argv);
	errno = ENOSYS;
	return -1;
}

int execvpe(const char *file, char *const argv[], char *const envp[])
{
	uk_pr_warn("%s(): file=%s\n", __func__, file);
	exec_warn_argv(argv);
	exec_warn_envp(envp);
	errno = ENOSYS;
	return -1;
}

int system(const char *command)
{
	uk_pr_warn("%s: %s\n", __func__, command);
	errno = ENOSYS;
	return -1;
}

FILE *popen(const char *command, const char *type __unused)
{
	uk_pr_warn("%s: %s\n", __func__, command);
	errno = ENOSYS;
	return NULL;
}

int pclose(FILE *stream __unused)
{
	errno = EINVAL;
	return -1;
}

int wait(int *status __unused)
{
	/* No children */
	errno = ECHILD;
	return -1;
}

pid_t waitpid(pid_t pid __unused, int *wstatus __unused, int options __unused)
{
	/* No children */
	errno = ECHILD;
	return -1;
}

pid_t wait3(int *wstatus, int options, struct rusage *rusage __unused)
{
	struct ukplat_wait_result result;
	pid_t ret = 0;
	int posix_wstatus, uk_options, exit_value, rc;

	uk_options = 0;
	if (options & WNOHANG)
		uk_options |= WAIT_OPT_NOHANG;

	rc = ukplat_wait_any(&result, uk_options);
	if (rc) {
		errno = -rc;
		ret = -1;
		goto out;
	}

	posix_wstatus = 0;
	if (WAIT_STATUS_EXITED_GET(result.wstatus)) {
		exit_value = WAIT_STATUS_EXIT_VALUE_GET(result.wstatus);
		posix_wstatus |= (exit_value << 8);

		ret = result.child_id;
	}

	*wstatus = posix_wstatus;

out:
	return ret;
}

pid_t wait4(pid_t pid __unused, int *wstatus __unused, int options __unused,
		struct rusage *rusage __unused)
{
	/* No children */
	errno = ECHILD;
	return -1;
}

UK_SYSCALL_R_DEFINE(int, getpid)
{
	return (pid_t) ukplat_get_domain_id();
}

UK_SYSCALL_R_DEFINE(pid_t, getppid)
{
	return UNIKRAFT_PPID;
}

pid_t setsid(void)
{
	/* We have a single "session" with a single "process" */
	errno = EPERM;
	return (pid_t) -1;
}

pid_t getsid(pid_t pid)
{
	if (pid != 0) {
		/* We support only calls for the only calling "process" */
		errno = ESRCH;
		return (pid_t) -1;
	}
	return UNIKRAFT_SID;
}

int setpgid(pid_t pid, pid_t pgid)
{
	if (pid != 0) {
		/* We support only calls for the only calling "process" */
		errno = ESRCH;
		return (pid_t) -1;
	}
	if (pgid != 0) {
		/* We have a single "group" with a single "process" */
		errno = EPERM;
		return (pid_t) -1;
	}
	return 0;
}

pid_t getpgid(pid_t pid)
{
	if (pid != 0) {
		/* We support only calls for the only calling "process" */
		errno = ESRCH;
		return (pid_t) -1;
	}
	return UNIKRAFT_PGID;
}

pid_t getpgrp(void)
{
	return UNIKRAFT_PGID;
}

int setpgrp(void)
{
	return setpgid(0, 0);
}

int tcsetpgrp(int fd __unused, pid_t pgrp)
{
	/* TODO check if fd is BADF */
	if (pgrp != UNIKRAFT_PGID) {
		errno = EINVAL;
		return -1;
	}
	return 0;
}

pid_t tcgetpgrp(int fd)
{
	/* We have a single "process group" */
	return UNIKRAFT_PGID;
}

int nice(int inc __unused)
{
	/* We don't support priority updates for unikernels */
	errno = EPERM;
	return -1;
}

int getpriority(int which, id_t who)
{
	int rc = 0;

	switch (which) {
	case PRIO_PROCESS:
	case PRIO_PGRP:
	case PRIO_USER:
		if (who == 0)
			/* Allow only for the calling "process" */
			rc = UNIKRAFT_PROCESS_PRIO;
		else {
			errno = ESRCH;
			rc = -1;
		}
		break;
	default:
		errno = EINVAL;
		rc = -1;
		break;
	}

	return rc;
}

int setpriority(int which, id_t who, int prio)
{
	int rc = 0;

	switch (which) {
	case PRIO_PROCESS:
	case PRIO_PGRP:
	case PRIO_USER:
		if (who == 0) {
			/* Allow only for the calling "process" */
			if (prio != UNIKRAFT_PROCESS_PRIO) {
				/* Allow setting only the default prio */
				errno = EACCES;
				rc = -1;
			}
		} else {
			errno = ESRCH;
			rc = -1;
		}
		break;
	default:
		errno = EINVAL;
		rc = -1;
		break;
	}

	return rc;
}

int prctl(int option, unsigned long arg2, unsigned long arg3,
                 unsigned long arg4, unsigned long arg5)
{
	WARN_STUBBED();
	return 0;
}
