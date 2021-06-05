// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * idle_sleep by André Almeida <andrealmeid@collabora.com>
 *
 * Do Androids Dream of Electric Sheep?
 *
 * Copyright 2021 Collabora Ltd.
 */

#include <linux/syscalls.h>
#include <linux/cpuidle.h>
#include <linux/idle_inject.h>

static struct idle_inject_device *ii_dev;

/**
 * sys_idle_sleep - syscall to inject idle cycles
 * @idle_time_ms: time to go idle in miliseconds
 */
SYSCALL_DEFINE1(idle_sleep, unsigned long long, idle_time_ms)
{
	int ret = 0;
	unsigned int idle_time_us = idle_time_ms * USEC_PER_MSEC;

	if (!ii_dev)
		return -ENODEV;

	idle_inject_set_latency(ii_dev, idle_time_us);
	idle_inject_set_time(ii_dev, ktime_add_us(ktime_get(), idle_time_us));

	ret = idle_inject_start(ii_dev);

	return ret;
}

static int __init init_idle_sleep(void)
{
	ii_dev = idle_inject_register((struct cpumask *) cpu_online_mask);
	if (!ii_dev) {
		pr_err("ii_dev not found\n");
		return -ENODEV;
	}

	return 0;
}

late_initcall(init_idle_sleep);
