// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * futex2 system call interface by André Almeida <andrealmeid@collabora.com>
 *
 * Copyright 2020 Collabora Ltd.
 */

#include <linux/freezer.h>
#include <linux/jhash.h>
#include <linux/sched/wake_q.h>
#include <linux/spinlock.h>
#include <linux/syscalls.h>
#include <linux/memblock.h>
#include <uapi/linux/futex.h>

/**
 * struct futex_waiter - List entry for a waiter
 * @key.address:  Memory address of userspace futex
 * @key.mm:       Pointer to memory management struct of this process
 * @key:          Stores information that uniquely identify a futex
 * @list:	  List node struct
 * @val:	  Expected value for this waiter
 * @flags:        Flags
 * @bucket:       Pointer to the bucket for this waiter
 * @index:        Index of waiter in futexv list
 */
struct futex_waiter {
	struct futex_key {
		uintptr_t address;
		struct mm_struct *mm;
	} key;
	struct list_head list;
	unsigned int val;
	unsigned int flags;
	struct futex_bucket *bucket;
	unsigned int index;
};

/**
 * struct futex_bucket - A bucket of futex's hash table
 * @waiters: Number of waiters in the bucket
 * @lock:    Bucket lock
 * @list:    List of waiters on this bucket
 */
struct futex_bucket {
	atomic_t waiters;
	spinlock_t lock;
	struct list_head list;
};

struct futexv {
	struct task_struct *task;
	int hint;
	struct futex_waiter objects[0];
};

struct futex_single_waiter {
	struct futexv parent;
	struct futex_waiter waiter;
} __packed;

struct futex_bucket *futex_table;

/* mask for futex2 flag operations */
#define FUTEX2_MASK (FUTEX_SIZE_MASK | FUTEX_SHARED_FLAG | \
		     FUTEX_CLOCK_REALTIME)

// mask for sys_futex_waitv
#define FUTEXV_MASK (FUTEX_CLOCK_REALTIME)

// mask for each futex in futex_waitv list
#define FUTEXV_WAITER_MASK (FUTEX_SIZE_MASK | FUTEX_SHARED_FLAG)

int futex2_hashsize;

/*
 * Reflects a new waiter being added to the waitqueue.
 */
static inline void bucket_inc_waiters(struct futex_bucket *bucket)
{
#ifdef CONFIG_SMP
	atomic_inc(&bucket->waiters);
	/*
	 * Full barrier (A), see the ordering comment above.
	 */
	smp_mb__after_atomic();
#endif
}

/*
 * Reflects a waiter being removed from the waitqueue by wakeup
 * paths.
 */
static inline void bucket_dec_waiters(struct futex_bucket *bucket)
{
#ifdef CONFIG_SMP
	atomic_dec(&bucket->waiters);
#endif
}

/*
 * Get the number of waiters in a bucket
 */
static inline int bucket_get_waiters(struct futex_bucket *bucket)
{
#ifdef CONFIG_SMP
	/*
	 * Full barrier (B), see the ordering comment above.
	 */
	smp_mb();
	return atomic_read(&bucket->waiters);
#else
	return 1;
#endif
}

/**
 * futex_get_bucket - Check if the user address is valid, prepare internal
 *                    data and calculate the hash
 * @uaddr:   futex user address
 * @key:     data that uniquely identifies a futex
 *
 * Return: address of bucket on success, error code otherwise
 */
static struct futex_bucket *futex_get_bucket(void __user *uaddr,
					     struct futex_key *key)
{
	uintptr_t address = (uintptr_t) uaddr;
	u32 hash_key;

	/* Checking if uaddr is valid and accessible */
	if (unlikely(!IS_ALIGNED(address, sizeof(u32))))
		return ERR_PTR(-EINVAL);
	if (unlikely(!access_ok(address, sizeof(u32))))
		return ERR_PTR(-EFAULT);

	key->address = address;
	key->mm = current->mm;

	/* Generate hash key for this futex using uaddr and current->mm */
	hash_key = jhash2((u32 *) key, sizeof(*key) / sizeof(u32), 0);

	/* Since HASH_SIZE is 2^n, subtracting 1 makes a perfect bit mask */
	return &futex_table[hash_key & (futex2_hashsize - 1)];
}

/**
 * futex_get_user - Get the userspace value on this address
 * @uval:  variable to store the value
 * @uaddr: userspace address
 *
 * Check the comment at futex_get_user_val for more information.
 */
static int futex_get_user(u32 *uval, u32 *uaddr)
{
	int ret;

	pagefault_disable();
	ret = __get_user(*uval, uaddr);
	pagefault_enable();

	return ret;
}

/**
 * futex_setup_time - Prepare the timeout mechanism, without starting it.
 * @timo:    Timeout value from userspace
 * @timeout: Pointer to hrtimer handler
 * @flags: Flags from userspace, to decide which clockid to use
 *
 * Return: 0 on success, error code otherwise
 */
static int futex_setup_time(struct __kernel_timespec __user *timo,
			    struct hrtimer_sleeper *timeout,
			    unsigned int flags)
{
	ktime_t time;
	struct timespec64 ts;
	clockid_t clockid = (flags & FUTEX_CLOCK_REALTIME) ?
			    CLOCK_REALTIME : CLOCK_MONOTONIC;

	if (get_timespec64(&ts, timo))
		return -EFAULT;

	if (!timespec64_valid(&ts))
		return -EINVAL;

	time = timespec64_to_ktime(ts);

	hrtimer_init_sleeper(timeout, clockid, HRTIMER_MODE_ABS);

	hrtimer_set_expires(&timeout->timer, time);

	return 0;
}


/**
 * futex_get_user_value - Get the value from the userspace address and compares
 *			  with the expected one. In success, leaves the function
 *			  holding the bucket lock. Else, hold no lock.
 * @bucket: hash bucket of this address
 * @uaddr:  futex's userspace address
 * @val:    expected value
 * @multiple: is this call in the wait on multiple path
 *
 * Return: 0 on success, error code otherwise
 */
static int futex_get_user_value(struct futex_bucket *bucket, u32 __user *uaddr,
				unsigned int val, bool multiple)
{
	u32 uval;
	int ret;

	/*
	 * Get the value from user futex address.
	 *
	 * Since we are in a hurry, we use a spin lock and we can't sleep.
	 * Try to get the value with page fault disabled (when enable, we might
	 * sleep).
	 *
	 * If we fail, we aren't sure if the address is invalid or is just a
	 * page fault. Then, release the lock (so we can sleep) and try to get
	 * the value with page fault enabled. In order to trigger a page fault
	 * handling, we just call __get_user() again.
	 *
	 * If get_user succeeds, this mean that the address is valid and we do
	 * the loop again. Since we just handled the page fault, the page is
	 * likely pinned in memory and we should be luckier this time and be
	 * able to get the value. If we fail anyway, we will try again.
	 *
	 * If even with page faults enabled we get and error, this means that
	 * the address is not valid and we return from the syscall.
	 */
	do {
		spin_lock(&bucket->lock);

		ret = futex_get_user(&uval, uaddr);

		if (ret) {
			spin_unlock(&bucket->lock);
			if (multiple || __get_user(uval, uaddr))
				return -EFAULT;

		}
	} while (ret);

	if (uval != val) {
		spin_unlock(&bucket->lock);
		return -EWOULDBLOCK;
	}

	return 0;
}

/**
 * futex_dequeue - Remove a futex from a queue
 * @bucket: current bucket holding the futex
 * @waiter:   futex to be removed
 *
 * Return: True if futex was removed by this function, false if another wake
 *         thread removed this futex.
 *
 * This function should be used after we found that this futex was in a queue.
 * Thus, it needs to be removed before the next step. However, someone could
 * wake it between the time of the first check and the time to get the lock for
 * the bucket. Check one more time if the futex is there with the bucket locked.
 * If it's there, just remove it and return true. Else, mark the removal as
 * false and do nothing.
 */
static bool futex_dequeue(struct futex_bucket *bucket, struct futex_waiter *waiter)
{
	bool removed = true;

	spin_lock(&bucket->lock);
	if (list_empty(&waiter->list))
		removed = false;
	else
		list_del(&waiter->list);
	spin_unlock(&bucket->lock);

	if (removed)
		bucket_dec_waiters(bucket);

	return removed;
}

/**
 * sys_futex_wait - Wait on a futex address if (*uaddr) == val
 * @uaddr: User address of futex
 * @val:   Expected value of futex
 * @flags: Specify the size of futex and the clockid
 * @timo:  Optional absolute timeout. Supports only 64bit time.
 */
SYSCALL_DEFINE4(futex_wait, void __user *, uaddr, unsigned int, val,
		unsigned int, flags, struct __kernel_timespec __user *, timo)
{
	unsigned int size = flags & FUTEX_SIZE_MASK;
	struct hrtimer_sleeper timeout;
	struct futex_bucket *bucket;
	struct futex_single_waiter wait_single;
	struct futex_waiter *waiter;
	int ret;

	wait_single.parent.task = current;
	wait_single.parent.hint = 0;
	waiter = &wait_single.waiter;
	waiter->index = 0;

	if (flags & ~FUTEX2_MASK)
		return -EINVAL;

	if (size != FUTEX_32)
		return -EINVAL;

	if (timo) {
		ret = futex_setup_time(timo, &timeout, flags);
		if (ret)
			return ret;
	}

	/* Get an unlocked hash bucket */
	bucket = futex_get_bucket(uaddr, &waiter->key);
	if (IS_ERR(bucket))
		return PTR_ERR(bucket);

	if (timo)
		hrtimer_sleeper_start_expires(&timeout, HRTIMER_MODE_ABS);

retry:
	bucket_inc_waiters(bucket);

	/* Compare the expected and current value, get the bucket lock */
	ret = futex_get_user_value(bucket, uaddr, val, false);
	if (ret) {
		bucket_dec_waiters(bucket);
		goto out;
	}

	/* Add the waiter to the hash table and sleep */
	set_current_state(TASK_INTERRUPTIBLE);
	list_add_tail(&waiter->list, &bucket->list);
	spin_unlock(&bucket->lock);

	/* Do not sleep if someone woke this futex or if it was timeouted */
	if (!list_empty_careful(&waiter->list) && (!timo || timeout.task))
		freezable_schedule();

	__set_current_state(TASK_RUNNING);

	/*
	 * One of those things triggered this wake:
	 *
	 * * We have been removed from the bucket. futex_wake() woke us. We just
	 *   need to return 0 to userspace.
	 *
	 * However, if we find ourselves in the bucket we must remove ourselves
	 * from the bucket and ...
	 *
	 * * If the there's a timeout and it has expired, return -ETIMEDOUT.
	 *
	 * * If there is a signal pending, something wants to kill our thread.
	 *   Return -ERESTARTSYS.
	 *
	 * * If there's no signal pending, it was a spurious wake (scheduler
	 *   gave us a change to do some work, even if we don't want to). We
	 *   need to remove ourselves from the bucket and add again, to prevent
	 *   losing wakeups in the meantime.
	 */

	/* Normal wake */
	if (list_empty_careful(&waiter->list))
		goto out;

	if (!futex_dequeue(bucket, waiter))
		goto out;

	/* Timeout */
	if (timo && !timeout.task)
		return -ETIMEDOUT;

	/* Spurious wakeup */
	if (!signal_pending(current))
		goto retry;

	/* Some signal is pending */
	ret = -ERESTARTSYS;
out:
	if (timo)
		hrtimer_cancel(&timeout.timer);

	return ret;
}

static struct futexv *futex_get_parent(uintptr_t waiter, u8 index)
{
	uintptr_t parent = waiter - sizeof(struct futexv)
			   - (uintptr_t) (index * sizeof(struct futex_waiter));

	return (struct futexv *) parent;
}

/**
 * sys_futex_wake - Wake a number of futexes waiting on an address
 * @uaddr:   Address of futex to be woken up
 * @nr_wake: Number of futexes to be woken up
 * @flags:   TODO
 */
SYSCALL_DEFINE3(futex_wake, void __user *, uaddr, unsigned int, nr_wake,
		unsigned int, flags)
{
	unsigned int size = flags & FUTEX_SIZE_MASK;
	struct futex_waiter waiter, *aux, *tmp;
	struct futex_bucket *bucket;
	struct task_struct *task;
	DEFINE_WAKE_Q(wake_q);
	int ret = 0;

	if (flags & ~FUTEX2_MASK)
		return -EINVAL;

	if (size != FUTEX_32)
		return -EINVAL;

	bucket = futex_get_bucket(uaddr, &waiter.key);
	if (IS_ERR(bucket))
		return PTR_ERR(bucket);

	if (!bucket_get_waiters(bucket))
		return 0;

	spin_lock(&bucket->lock);
	list_for_each_entry_safe(aux, tmp, &bucket->list, list) {
		if (ret >= nr_wake)
			break;

		if (waiter.key.address == aux->key.address &&
		    waiter.key.mm == aux->key.mm) {
			struct futexv *parent =
				futex_get_parent((uintptr_t) aux, aux->index);

			parent->hint = 1;
			task = parent->task;
			get_task_struct(task);
			list_del_init_careful(&aux->list);
			wake_q_add_safe(&wake_q, task);
			ret++;
			bucket_dec_waiters(bucket);
		}
	}
	spin_unlock(&bucket->lock);

	wake_up_q(&wake_q);

	return ret;
}

static int __init futex2_init(void)
{
	int i;
	unsigned int futex_shift;

#if CONFIG_BASE_SMALL
	futex2_hashsize = 16;
#else
	futex2_hashsize = roundup_pow_of_two(256 * num_possible_cpus());
#endif

	futex_table = alloc_large_system_hash("futex2", sizeof(struct futex_bucket),
					      futex2_hashsize, 0,
					      futex2_hashsize < 256 ? HASH_SMALL : 0,
					      &futex_shift, NULL,
					      futex2_hashsize, futex2_hashsize);
	futex2_hashsize = 1UL << futex_shift;

	for (i = 0; i < futex2_hashsize; i++) {
		INIT_LIST_HEAD(&futex_table[i].list);
		spin_lock_init(&futex_table[i].lock);
		atomic_set(&futex_table[i].waiters, 0);
	}

	return 0;
}
core_initcall(futex2_init);
