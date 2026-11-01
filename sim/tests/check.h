#pragma once

#include <cstdio>

// A small test helper, so the project needs no test framework.
inline int& failureCount() {
    static int count = 0;
    return count;
}

#define CHECK(condition)                                                     \
    do {                                                                     \
        if (!(condition)) {                                                  \
            std::printf("  FAILED %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            ++failureCount();                                                \
        }                                                                    \
    } while (false)
