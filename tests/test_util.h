/*
 * test_util.h -- minimal assertion helpers for the off-target tests. Each
 * test program returns nonzero if any check failed.
 */
#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <stdio.h>

static int g_checks;
static int g_failures;

#define CHECK(cond) do { g_checks++; if (!(cond)) { g_failures++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

#define CHECK_STR(a, b) do { g_checks++; if (strcmp((a), (b)) != 0) { g_failures++; \
    printf("  FAIL %s:%d: \"%s\" != \"%s\"\n", __FILE__, __LINE__, (a), (b)); } } while (0)

static int test_report(const char *name)
{
    printf("%s: %d checks, %d failed\n", name, g_checks, g_failures);
    return g_failures != 0;
}

#endif /* TEST_UTIL_H */
