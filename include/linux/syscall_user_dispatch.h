/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Collabora Ltd.
 */
#ifndef _SYSCALL_USER_DISPATCH_H
#define _SYSCALL_USER_DISPATCH_H

#ifdef CONFIG_SYSCALL_USER_DISPATCH
struct syscall_user_dispatch {
	char __user *selector;
	unsigned long offset;
	unsigned long len;
};

int set_syscall_user_dispatch(unsigned long mode, unsigned long offset,
			      unsigned long len, char __user *selector);
#else
struct syscall_user_dispatch {};

static inline int set_syscall_user_dispatch(unsigned long mode, unsigned long offset,
					    unsigned long len, char __user *selector)
{
	return -EINVAL;
}
#endif /* CONFIG_SYSCALL_USER_DISPATCH */

#endif /* _SYSCALL_USER_DISPATCH_H */
