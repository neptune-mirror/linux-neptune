#!/bin/python3
# run perf tests, kill them if they run for too long and log error in a file

import subprocess as sub

def run_test(test, shared):
    perf_comm = "/mnt/tools/perf/perf bench " + str(test) + " -s"
    if shared:
        perf_comm += " -S"

    proc = sub.Popen(perf_comm, shell=True)
    try:
        # no test should run for more that two minutes
        proc.wait(timeout=120)
    except sub.TimeoutExpired as err:
        proc.kill()
        with open("/mnt/perf_error", "w") as f:
            f.write(str(err))
        exit(1)

def run_perf(tests):
    for test in tests:
        run_test(test, False)
        run_test(test, True)

run_perf(["futex2 hash",
        "-r 50 futex2 wake",
        "-r 50 futex2 wake-parallel",
        "-r 50 futex2 wake -t 100",
        "-r 50 futex2 wake-parallel -t 100",
        ])
