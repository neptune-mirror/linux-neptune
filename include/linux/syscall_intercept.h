/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Collabora Ltd.
 */
#ifndef _SYSCALL_INTERCEPT_H
#define _SYSCALL_INTERCEPT_H

#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/thread_info.h>

#define SYSCALL_INTERCEPT_SECCOMP	0x1
#define SYSCALL_INTERCEPT_USER_DISPATCH	0x2

#ifdef TIF_SYSCALL_INTERCEPT

/* seccomp (at least) can modify TIF_SYSCALL_INTERCEPT from a different
 * thread, which means it can race with itself or with
 * syscall_user_dispatch. Therefore, TIF_SYSCALL_INTERCEPT and
 * syscall_intercept are synchronized by tsk->sighand->siglock.
 */

static inline void __set_tsk_syscall_intercept(struct task_struct *tsk, unsigned int type)
{
	tsk->syscall_intercept |= type;

	if (tsk->syscall_intercept)
		set_tsk_thread_flag(tsk, TIF_SYSCALL_INTERCEPT);
}

static inline void __clear_tsk_syscall_intercept(struct task_struct *tsk, unsigned int type)
{
	tsk->syscall_intercept &= ~type;

	if (!tsk->syscall_intercept)
		clear_tsk_thread_flag(tsk, TIF_SYSCALL_INTERCEPT);
}

static inline void set_tsk_syscall_intercept(struct task_struct *tsk, unsigned int type)
{
	spin_lock_irq(&tsk->sighand->siglock);
	__set_tsk_syscall_intercept(tsk, type);
	spin_unlock_irq(&tsk->sighand->siglock);
}

static inline void clear_tsk_syscall_intercept(struct task_struct *tsk, unsigned int type)
{
	spin_lock_irq(&tsk->sighand->siglock);
	__clear_tsk_syscall_intercept(tsk, type);
	spin_unlock_irq(&tsk->sighand->siglock);
}

#else
static inline void __set_tsk_syscall_intercept(struct task_struct *tsk, unsigned int type)
{
}
static inline void set_tsk_syscall_intercept(struct task_struct *tsk, unsigned int type)
{
}
static inline void __clear_tsk_syscall_intercept(struct task_struct *tsk, unsigned int type)
{
}
static inline void clear_tsk_syscall_intercept(struct task_struct *tsk, unsigned int type)
{
}
#endif

#endif

