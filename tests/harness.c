#include "harness.h"

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define NS_PER_S 1000000000

static struct {
    const char *test_name;

    uint64_t harness_start_ts;
    uint64_t harness_end_ts;
    uint64_t suite_start_ts;
    uint64_t suite_end_ts;
    uint64_t test_start_ts;
    uint64_t test_end_ts;

    int suite_tests_failed;
    int suite_tests_passed;
    int total_tests_failed;
    int total_tests_passed;

    bool harness_active;
    bool suite_active;
    bool test_active;
} state;

static uint64_t now(void) {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return (uint64_t)(ts.tv_sec * NS_PER_S) + (uint64_t)ts.tv_nsec;
}

static double test_elapsed_time(void) {
    return (double)(state.test_end_ts - state.test_start_ts) / NS_PER_S;
}

static double suite_elapsed_time(void) {
    return (double)(state.suite_end_ts - state.suite_start_ts) / NS_PER_S;
}

static double harness_elapsed_time(void) {
    return (double)(state.harness_end_ts - state.harness_start_ts) / NS_PER_S;
}

void harness_begin(void) {
    assert(!state.harness_active);
    state.harness_start_ts = now();
    state.harness_active = true;
}

void harness_end(void) {
    assert(state.harness_active);
    state.harness_end_ts = now();
    state.harness_active = false;

    printf("All tests have finished running.\n");
    printf("Summary: %d passed and %d failed after %.4lfs\n",
        state.total_tests_passed,
        state.total_tests_failed,
        harness_elapsed_time()
    );

    state.total_tests_passed = 0;
    state.total_tests_failed = 0;
}

int harness_status(void) {
    return state.total_tests_failed;
}

void suite_begin(const char *name) {
    assert(name != NULL);
    assert(!state.suite_active);
    state.suite_start_ts = now();
    state.suite_active = true;
    printf("[Suite: '%s']\n", name);
}

void suite_end(void) {
    assert(state.suite_active);
    state.suite_end_ts = now();
    state.suite_active = false;

    printf("> Summary: %d passed and %d failed after %.4lfs\n\n",
        state.suite_tests_passed,
        state.suite_tests_failed,
        suite_elapsed_time()
    );

    state.suite_tests_passed = 0;
    state.suite_tests_failed = 0;
}

void test_begin(const char *name) {
    assert(name != NULL);
    assert(!state.test_active);
    state.test_name = name;
    state.test_start_ts = now();
    state.test_active = true;
}

void test_end(void) {
    assert(state.test_active);
    state.test_end_ts = now();
    state.test_active = false;
}

void test_pass(void) {
    test_end();
    state.total_tests_passed += 1;
    state.suite_tests_passed += 1;
    printf("+ Pass (%.4lfs): '%s'\n", test_elapsed_time(), state.test_name);
}

void test_fail(const char *format, ...) {
    test_end();
    state.total_tests_failed += 1;
    state.suite_tests_failed += 1;

    printf("- Fail (%.4lfs): '%s'\n", test_elapsed_time(), state.test_name);
    printf("  | ");

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    putchar('\n');
}
