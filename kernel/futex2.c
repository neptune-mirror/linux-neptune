// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * futex2 system call interface by André Almeida <andrealmeid@collabora.com>
 *
 * Copyright 2020 Collabora Ltd.
 */

#include <linux/freezer.h>
#include <linux/hugetlb.h>
#include <linux/jhash.h>
#include <linux/pagemap.h>
#include <linux/sched/wake_q.h>
#include <linux/spinlock.h>
#include <linux/syscalls.h>
#include <linux/memblock.h>
#include <uapi/linux/futex.h>

/**
 * struct futex_waiter - List entry for a waiter
 * @uaddr:        Memory address of userspace futex
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
	uintptr_t uaddr;
	struct futex_key {
		uintptr_t address;
		struct mm_struct *mm;
		unsigned long int offset;
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

/**
 * struct futexv - List of futexes to be waited
 * @task:    Task to be awaken
 * @hint:    Was someone on this list awaken?
 * @objects: List of futexes
 */
struct futexv {
	struct task_struct *task;
	bool hint;
	struct futex_waiter objects[0];
};

/**
 * struct futex_single_waiter - Wrapper for a futexv of one element
 * @futexv: TODO
 * @waiter: TODO
 */
struct futex_single_waiter {
	struct futexv futexv;
	struct futex_waiter waiter;
} __packed;

struct futex_bucket *futex_table;

/* mask for futex2 flag operations */
#define FUTEX2_MASK (FUTEX_SIZE_MASK | FUTEX_SHARED_FLAG | \
		     FUTEX_CLOCK_REALTIME)

/* mask for sys_futex_waitv flag */
#define FUTEXV_MASK (FUTEX_CLOCK_REALTIME)

/* mask for each futex in futex_waitv list */
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

static u64 get_inode_sequence_number(struct inode *inode)
{
	static atomic64_t i_seq;
	u64 old;

	/* Does the inode already have a sequence number? */
	old = atomic64_read(&inode->i_sequence);
	if (likely(old))
		return old;

	for (;;) {
		u64 new = atomic64_add_return(1, &i_seq);
		if (WARN_ON_ONCE(!new))
			continue;

		old = atomic64_cmpxchg_relaxed(&inode->i_sequence, 0, new);
		if (old)
			return old;
		return new;
	}
}

#define FUT_OFF_INODE    1 /* We set bit 0 if key has a reference on inode */
#define FUT_OFF_MMSHARED 2 /* We set bit 1 if key has a reference on mm */

static int futex_get_shared_key(uintptr_t address, struct mm_struct *mm,
				struct futex_key *key)
{
	int err;
	struct page *page, *tail;
	struct address_space *mapping;

again:
	err = get_user_pages_fast(address, 1, 0, &page);

	if (err < 0)
		return err;
	else
		err = 0;


	tail = page;
	page = compound_head(page);
	mapping = READ_ONCE(page->mapping);


	if (unlikely(!mapping)) {
		int shmem_swizzled;

		lock_page(page);
		shmem_swizzled = PageSwapCache(page) || page->mapping;
		unlock_page(page);
		put_page(page);

		if (shmem_swizzled)
			goto again;

		return -EFAULT;
	}

	if (PageAnon(page)) {

		key->mm = mm;
		key->address = address;

		key->offset |= FUT_OFF_MMSHARED;

	} else {
		struct inode *inode;

		rcu_read_lock();

		if (READ_ONCE(page->mapping) != mapping) {
			rcu_read_unlock();
			put_page(page);

			goto again;
		}

		inode = READ_ONCE(mapping->host);
		if (!inode) {
			rcu_read_unlock();
			put_page(page);

			goto again;
		}

		key->address = get_inode_sequence_number(inode);
		key->mm = (struct mm_struct *) basepage_index(tail);
		key->offset |= FUT_OFF_INODE;

		rcu_read_unlock();
	}

	put_page(page);
	return err;
}

/**
 * futex_get_bucket - Check if the user address is valid, prepare internal
 *                    data and calculate the hash
 * @uaddr:   futex user address
 * @key:     data that uniquely identifies a futex
 * @shared:  is this a shared futex?
 *
 * Return: address of bucket on success, error code otherwise
 */
static struct futex_bucket *futex_get_bucket(void __user *uaddr,
					     struct futex_key *key,
					     bool shared)
{
	uintptr_t address = (uintptr_t) uaddr;
	u32 hash_key;

	/* Checking if uaddr is valid and accessible */
	if (unlikely(!IS_ALIGNED(address, sizeof(u32))))
		return ERR_PTR(-EINVAL);
	if (unlikely(!access_ok(address, sizeof(u32))))
		return ERR_PTR(-EFAULT);

	key->offset = address % PAGE_SIZE;
	address -= key->offset;

	if (!shared) {
		key->address = address;
		key->mm = current->mm;
	} else {
		futex_get_shared_key(address, current->mm, key);
	}

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
static int futex_get_user(u32 *uval, u32 __user *uaddr)
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
 * futex_dequeue_multiple - Remove multiple futexes from hash table
 * @futexv: list of waiters
 * @nr:     number of futexes to be removed
 *
 * This function should be used after we found that this futex was in a queue.
 * Thus, it needs to be removed before the next step. However, someone could
 * wake it between the time of the first check and the time to get the lock for
 * the bucket. Check one more time if the futex is there with the bucket locked.
 * If it's there, just remove it and return true. Else, mark the removal as
 * false and do nothing.
 *
 * Return:
 *  * -1 if no futex was woken during the removal
 *  * =< 0 at least one futex was found woken, index of the last one
 */
static int futex_dequeue_multiple(struct futexv *futexv, unsigned int nr)
{
	int i, ret = -1;

	for (i = 0; i < nr; i++) {
		spin_lock(&futexv->objects[i].bucket->lock);
		if (!list_empty_careful(&futexv->objects[i].list)) {
			list_del_init_careful(&futexv->objects[i].list);
			bucket_dec_waiters(futexv->objects[i].bucket);
		} else {
			ret = i;
		}
		spin_unlock(&futexv->objects[i].bucket->lock);
	}

	return ret;
}

/**
 * futex_enqueue - Check the value and enqueue a futex on a wait list
 *
 * @futexv:     List of futexes
 * @nr_futexes: Number of futexes in the list
 * @awaken:	If a futex was awaken during enqueueing, store the index here
 *
 * Get the value from the userspace address and compares with the expected one.
 * In success, enqueue the futex in the correct bucket
 *
 * Get the value from user futex address.
 *
 * Since we are in a hurry, we use a spin lock and we can't sleep.
 * Try to get the value with page fault disabled (when enable, we might
 * sleep).
 *
 * If we fail, we aren't sure if the address is invalid or is just a
 * page fault. Then, release the lock (so we can sleep) and try to get
 * the value with page fault enabled. In order to trigger a page fault
 * handling, we just call __get_user() again. If we sleep with enqueued
 * futexes, we might miss a wake, so dequeue everything before sleeping.
 *
 * If get_user succeeds, this mean that the address is valid and we do
 * the work again. Since we just handled the page fault, the page is
 * likely pinned in memory and we should be luckier this time and be
 * able to get the value. If we fail anyway, we will try again.
 *
 * If even with page faults enabled we get and error, this means that
 * the address is not valid and we return from the syscall.
 *
 * If we got an unexpected value or need to treat a page fault and realized that
 * a futex was awaken, we can priority this and return success.
 *
 * Return: 0 on success, error code otherwise
 */
static int futex_enqueue(struct futexv *futexv, unsigned int nr_futexes,
			 int *awaken)
{
	int i, ret;
	bool shared, retry = false;
	u32 uval, *uaddr, val;
	struct futex_bucket *bucket;

retry:
	set_current_state(TASK_INTERRUPTIBLE);

	for (i = 0; i < nr_futexes; i++) {
		uaddr = (u32 * __user) futexv->objects[i].uaddr;
		val = (u32) futexv->objects[i].val;
		shared = (futexv->objects[i].flags & FUTEX_SHARED_FLAG) ? true : false;

		if (shared && retry) {
			futexv->objects[i].bucket =
				futex_get_bucket((void *) uaddr,
						 &futexv->objects[i].key, true);
			if (IS_ERR(futexv->objects[i].bucket))
				return PTR_ERR(futexv->objects[i].bucket);
		}

		bucket = futexv->objects[i].bucket;

		bucket_inc_waiters(bucket);
	        spin_lock(&bucket->lock);

	        ret = futex_get_user(&uval, uaddr);

	        if (unlikely(ret)) {
			spin_unlock(&bucket->lock);

			bucket_dec_waiters(bucket);
			__set_current_state(TASK_RUNNING);
			*awaken = futex_dequeue_multiple(futexv, i);

			if (shared) {
				retry = true;
				goto retry;
			}

			if (__get_user(uval, uaddr))
				return -EFAULT;

			if (*awaken >= 0)
				return 1;

			retry = true;
			goto retry;
	        }

		if (uval != val) {
			spin_unlock(&bucket->lock);


			bucket_dec_waiters(bucket);
			__set_current_state(TASK_RUNNING);
			*awaken = futex_dequeue_multiple(futexv, i);

			if (*awaken >= 0) {
				return 1;
			}

			return -EWOULDBLOCK;
		}

		list_add_tail(&futexv->objects[i].list, &bucket->list);
		spin_unlock(&bucket->lock);
	}

	return 0;
}


static int __futex_wait(struct futexv *futexv,
			       unsigned int nr_futexes,
			       struct hrtimer_sleeper *timeout)
{
	int ret;


	while (1) {
		int awaken = -1;

		ret = futex_enqueue(futexv, nr_futexes, &awaken);
		if (ret) {
			if (awaken >= 0)
				return awaken;
			return ret;
		}

		/* Before sleeping, check if someone was woken */
		if (!futexv->hint && (!timeout || timeout->task))
			freezable_schedule();

		__set_current_state(TASK_RUNNING);

		/*
		 * One of those things triggered this wake:
		 *
		 * * We have been removed from the bucket. futex_wake() woke
		 *   us. We just need to dequeue return 0 to userspace.
		 *
		 * However, if no futex was dequeued by a futex_wake():
		 *
		 * * If the there's a timeout and it has expired,
		 *   return -ETIMEDOUT.
		 *
		 * * If there is a signal pending, something wants to kill our
		 *   thread, return -ERESTARTSYS.
		 *
		 * * If there's no signal pending, it was a spurious wake
		 *   (scheduler gave us a change to do some work, even if we
		 *   don't want to). We need to remove ourselves from the
		 *   bucket and add again, to prevent losing wakeups in the
		 *   meantime.
		 */

		ret = futex_dequeue_multiple(futexv, nr_futexes);

		/* Normal wake */
		if (ret >= 0)
			break;

		if (timeout && !timeout->task)
			return -ETIMEDOUT;

		/* signal */
		if (signal_pending(current))
			return -ERESTARTSYS;

		/* spurious wake, do everything again */
	}

	return ret;
}

/**
 * futex_wait - Setup the timer and wait on a list of futexes
 * @futexv:     List of waiters
 * @nr_futexes: Number of waiters
 * @timo:	Timeout
 * @timeout:	Timeout
 * @flags:	Timeout flags
 *
 * Return: error code, or a hint of one of the waiters
 */
static int futex_wait(struct futexv *futexv, unsigned int nr_futexes,
		      struct __kernel_timespec __user *timo,
		      struct hrtimer_sleeper *timeout, unsigned int flags)
{
	int ret;

	if (timo) {
		ret = futex_setup_time(timo, timeout, flags);
		if (ret)
			return ret;

		hrtimer_sleeper_start_expires(timeout, HRTIMER_MODE_ABS);
	}


	ret = __futex_wait(futexv, nr_futexes, timo ? timeout : NULL);


	if (timo)
		hrtimer_cancel(&timeout->timer);

	return ret;
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
	bool shared = (flags & FUTEX_SHARED_FLAG) ? true : false;
	unsigned int size = flags & FUTEX_SIZE_MASK;
	struct futex_single_waiter wait_single;
	struct hrtimer_sleeper timeout;
	struct futex_waiter *waiter;
	struct futexv *futexv;
	int ret;

	futexv = &wait_single.futexv;
	futexv->task = current;
	futexv->hint = false;

	waiter = &wait_single.waiter;
	waiter->index = 0;
	waiter->val = val;
	waiter->uaddr = (uintptr_t) uaddr;

	INIT_LIST_HEAD(&waiter->list);

	if (flags & ~FUTEX2_MASK)
		return -EINVAL;

	if (size != FUTEX_32)
		return -EINVAL;

	/* Get an unlocked hash bucket */
	waiter->bucket = futex_get_bucket(uaddr, &waiter->key, shared);
	if (IS_ERR(waiter->bucket)) {
		return PTR_ERR(waiter->bucket);
	}

	ret = futex_wait(futexv, 1, timo, &timeout, flags);
	if (ret > 0)
		ret = 0;

	return ret;
}

/**
 * futex_parse_waitv - Parse a waitv array from userspace
 * @futexv:	list of waiters
 * @uwaitv:     userspace list
 * @nr_futexes: number of waiters in the list
 *
 * Return: Error code on failure, pointer to a prepared futexv otherwise
 */
static int futex_parse_waitv(struct futexv *futexv,
			     struct futex_waitv __user *uwaitv,
			     unsigned int nr_futexes)
{
	struct futex_waitv waitv;
	unsigned int i;
	struct futex_bucket *bucket;
	bool shared;

	for (i = 0; i < nr_futexes; i++) {

		if (copy_from_user(&waitv, &uwaitv[i], sizeof(waitv)))
			return -EFAULT;

		if ((waitv.flags & ~FUTEXV_WAITER_MASK) ||
		    (waitv.flags & FUTEX_SIZE_MASK) != FUTEX_32)
			return -EINVAL;

		shared = (waitv.flags & FUTEX_SHARED_FLAG) ? true : false;

		bucket = futex_get_bucket(waitv.uaddr,
				       &futexv->objects[i].key, shared);
		if (IS_ERR(bucket))
			return PTR_ERR(bucket);

		futexv->objects[i].bucket = bucket;
		futexv->objects[i].val = waitv.val;
		futexv->objects[i].flags = waitv.flags;
		futexv->objects[i].index = i;
		INIT_LIST_HEAD(&futexv->objects[i].list);
		futexv->objects[i].uaddr = (uintptr_t) waitv.uaddr;
	}

	return 0;
}

/**
 * sys_futex_waitv - function
 * @waiters:    TODO
 * @nr_futexes: TODO
 * @flags:      TODO
 * @timo:	TODO
 */
SYSCALL_DEFINE4(futex_waitv, struct futex_waitv __user *, waiters,
		unsigned int, nr_futexes, unsigned int, flags,
		struct __kernel_timespec __user *, timo)
{
	struct hrtimer_sleeper timeout;
	struct futexv *futexv;
	int ret;

	if (flags & ~FUTEXV_MASK)
		return -EINVAL;

	if (!nr_futexes || nr_futexes > FUTEX_WAITV_MAX || !waiters)
		return -EINVAL;

	futexv = kmalloc(sizeof(struct futexv) +
			 (sizeof(struct futex_waiter) * nr_futexes),
			 GFP_KERNEL);
	if (!futexv)
		return -ENOMEM;

	futexv->hint = false;
	futexv->task = current;

	ret = futex_parse_waitv(futexv, waiters, nr_futexes);
	if (!ret)
		ret = futex_wait(futexv, nr_futexes, timo, &timeout, flags);

	kfree(futexv);

	return ret;
}

/**
 * futex_get_parent - Get parent
 * @waiter: TODO
 * @index: TODO
 *
 * Return: TODO
 */
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
	bool shared = (flags & FUTEX_SHARED_FLAG) ? true : false;
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

	bucket = futex_get_bucket(uaddr, &waiter.key, shared);
	if (IS_ERR(bucket))
		return PTR_ERR(bucket);

	if (!bucket_get_waiters(bucket))
		return 0;

	spin_lock(&bucket->lock);
	list_for_each_entry_safe(aux, tmp, &bucket->list, list) {
		if (ret >= nr_wake)
			break;

		if (waiter.key.address == aux->key.address &&
		    waiter.key.mm == aux->key.mm &&
		    waiter.key.offset == aux->key.offset) {
			struct futexv *parent =
				futex_get_parent((uintptr_t) aux, aux->index);

			parent->hint = true;
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

static ssize_t wait_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%u\n", __NR_futex_wait);

}
static struct kobj_attribute futex2_wait_attr = __ATTR_RO(wait);

static ssize_t wake_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%u\n", __NR_futex_wake);

}
static struct kobj_attribute futex2_wake_attr = __ATTR_RO(wake);

static ssize_t waitv_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%u\n", __NR_futex_waitv);

}
static struct kobj_attribute futex2_waitv_attr = __ATTR_RO(waitv);

static struct attribute *futex2_sysfs_attrs[] = {
	&futex2_wait_attr.attr,
	&futex2_wake_attr.attr,
	&futex2_waitv_attr.attr,
	NULL,
};

static const struct attribute_group futex2_sysfs_attr_group = {
	.attrs = futex2_sysfs_attrs,
	.name = "futex2",
};

static int __init futex2_sysfs_init(void)
{
	return sysfs_create_group(kernel_kobj, &futex2_sysfs_attr_group);
}
subsys_initcall(futex2_sysfs_init);

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
