/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _COMMON_H
#define _COMMON_H

#ifdef CONFIG_SYSCALL_USER_DISPATCH
int do_syscall_user_dispatch(struct pt_regs *regs);
#else
static inline int do_syscall_user_dispatch(struct pt_regs *regs)
{
	WARN_ON_ONCE(1);
	return 0;
}
#endif /* CONFIG_SYSCALL_USER_DISPATCH */

#endif
