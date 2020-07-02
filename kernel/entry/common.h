/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _COMMON_H
#define _COMMON_H

bool do_syscall_user_dispatch(struct pt_regs *regs);

static inline bool on_syscall_dispatch(void)
{
	if (unlikely(current->syscall_dispatch.on_dispatch)) {
		current->syscall_dispatch.on_dispatch = false;
		return true;
	}
	return false;
}

#endif
