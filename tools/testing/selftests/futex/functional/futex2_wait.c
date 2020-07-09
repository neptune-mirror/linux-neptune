// SPDX-License-Identifier: GPL-2.0-or-later
/******************************************************************************
 *
 *   Copyright Collabora Ltd., 2020
 *
 * DESCRIPTION
 *	Test wait/wake mechanism of futex2, using 32bit sized futexes.
 *
 * AUTHOR
 *	André Almeida <andrealmeid@collabora.com>
 *
 * HISTORY
 *      2020-Jul-9: Initial version by André <andrealmeid@collabora.com>
 *
 *****************************************************************************/

#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <sys/shm.h>
#include "futex2test.h"
#include "logging.h"

#define TEST_NAME "futex2-wait"
#define timeout_ns  30000000
#define WAKE_WAIT_US 10000
futex_t *f1;

void usage(char *prog)
{
	printf("Usage: %s\n", prog);
	printf("  -c	Use color\n");
	printf("  -h	Display this help message\n");
	printf("  -v L	Verbosity level: %d=QUIET %d=CRITICAL %d=INFO\n",
	       VQUIET, VCRITICAL, VINFO);
}

void *waiterfn(void *arg)
{
	struct timespec64 to64;
	unsigned int flags = 0;
	if (arg)
		flags = *((unsigned int *) arg);

	/* setting absolute timeout for futex2 */
	if (gettime64(CLOCK_MONOTONIC, &to64))
		error("gettime64 failed\n", errno);

	to64.tv_nsec += timeout_ns;

	if (to64.tv_nsec >= 1000000000) {
		to64.tv_sec++;
		to64.tv_nsec -= 1000000000;
	}

	if (futex2_wait(f1, *f1, FUTEX_32 | flags, &to64))
		printf("waiter failed errno %d\n", errno);

	return NULL;
}

int main(int argc, char *argv[])
{
	pthread_t waiter;
	unsigned int flags = FUTEX_SHARED_FLAG;
	int res, ret = RET_PASS;
	int c;
	futex_t f_private = 0;
	f1 = &f_private;

	while ((c = getopt(argc, argv, "cht:v:")) != -1) {
		switch (c) {
		case 'c':
			log_color(1);
			break;
		case 'h':
			usage(basename(argv[0]));
			exit(0);
		case 'v':
			log_verbosity(atoi(optarg));
			break;
		default:
			usage(basename(argv[0]));
			exit(1);
		}
	}

	ksft_print_header();
	ksft_set_plan(2);
	ksft_print_msg("%s: Test FUTEX2_WAIT\n",
		       basename(argv[0]));

	info("Calling private futex2_wait on f1: %u @ %p with val=%u\n", *f1, f1, *f1);

	if (pthread_create(&waiter, NULL, waiterfn, NULL))
		error("pthread_create failed\n", errno);

	usleep(WAKE_WAIT_US);

	info("Calling private futex2_wake on f1: %u @ %p with val=%u\n", *f1, f1, *f1);
	res = futex2_wake(f1, 1, FUTEX_32);
	if (res != 1) {
		ksft_test_result_fail("futex2_wake private returned: %d %s\n",
				      res ? errno : res,
				      res ? strerror(errno) : "");
		ret = RET_FAIL;
	} else {
		ksft_test_result_pass("futex2_wake private succeeds\n");
	}

	int shm_id = shmget(IPC_PRIVATE, 4096, IPC_CREAT | 0666);
	if (shm_id < 0) {
		perror("shmget");
		exit(1);
	}

	unsigned int *shared_data = shmat(shm_id, NULL, 0);
	*shared_data = 0;
	f1 = shared_data;

	info("Calling shared futex2_wait on f1: %u @ %p with val=%u\n", *f1, f1, *f1);

	if (pthread_create(&waiter, NULL, waiterfn, &flags))
		error("pthread_create failed\n", errno);

	usleep(WAKE_WAIT_US);

	info("Calling shared futex2_wake on f1: %u @ %p with val=%u\n", *f1, f1, *f1);
	res = futex2_wake(f1, 1, FUTEX_32 | FUTEX_SHARED_FLAG);
	if (res != 1) {
		ksft_test_result_fail("futex2_wake shared returned: %d %s\n",
				      res ? errno : res,
				      res ? strerror(errno) : "");
		ret = RET_FAIL;
	} else {
		ksft_test_result_pass("futex2_wake shared succeeds\n");
	}

	shmdt(shared_data);

	ksft_print_cnts();
	return ret;
}
