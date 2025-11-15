#ifndef MOS6502_HARNESS_H
#define MOS6502_HARNESS_H

void harness_begin(void);
void harness_end(void);

int harness_status(void);

void suite_begin(const char *name);
void suite_end(void);

void test_begin(const char *name);
void test_end(void);

void test_pass(void);
void test_fail(const char *format, ...);

#endif
