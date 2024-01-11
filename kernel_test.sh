#!/bin/bash

FAIL_FILE=/mnt/fail.txt

fail() {
    touch ${FAIL_FILE}
}

start_test() {
    echo -e "\t\t\t\t Preparing testing environment"
    # Install kernel headers
    cd /mnt/ && make headers || {
        fail
        return
    }
    # Build required tests
    make -C tools/testing/selftests TARGETS="futex syscall_user_dispatch" || {
        fail
        return
    }

    echo -e "\t\t\t\t Starting futex Tests"
    make -C tools/testing/selftests TARGETS=futex run_tests
    echo -e "\t\t\t\t Completed futex Tests"

    echo -e "\t\t\t\t Starting syscall_user_dispatch Tests"
    cd /mnt/tools/testing/selftests/syscall_user_dispatch || {
        fail
        return
    }
    ./sud_test
    ./sud_benchmark
    echo -e "\t\t\t\t Completed syscall_user_dispatch Tests"
}

start_test 2>&1 | tee -a /mnt/kernel_results.log

[ -e $FAIL_FILE ] || {
    # Check Interception overhead
    MAX_OVERHEAD="10.00"
    found_text=$(grep "Interception" /mnt/kernel_results.log)
    result=$(echo $found_text | grep -Eo '[0-9]+([.][0-9]+)')
    min=$(echo $result $MAX_OVERHEAD | awk '{if ($1 < $2) print "ok"; else print "not_ok"}')
    [ $min == "not_ok" ] &&
        echo "Interception overhead greater than 10%" >${FAIL_FILE}
}

[ -e $FAIL_FILE ] || {
    # Parse result file for fail value
    grep -q "fail:[1-9]" /mnt/kernel_results.log && fail || touch /mnt/pass.txt
}

sync
poweroff
