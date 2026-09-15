/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Hugh Frater
 *
 * This file is part of rapidgen. rapidgen is free software: you can
 * redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version. It is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License in LICENSE for details.
 */
/* rg_test.h — the whole test framework. */
#ifndef RG_TEST_H
#define RG_TEST_H

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define RG_TEST_PI 3.14159265358979323846

static int rg_test_fails;
static int rg_test_count;

#define CHECK(cond)                                                          \
    do {                                                                     \
        rg_test_count++;                                                     \
        if (!(cond)) {                                                       \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);         \
            rg_test_fails++;                                                 \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(got, want, tol)                                           \
    do {                                                                     \
        rg_test_count++;                                                     \
        double g_ = (got), w_ = (want);                                      \
        if (!(fabs(g_ - w_) <= (tol))) {                                     \
            printf("  FAIL %s:%d  %s = %.6f, want %.6f (+/- %g)\n",          \
                   __FILE__, __LINE__, #got, g_, w_, (double)(tol));         \
            rg_test_fails++;                                                 \
        }                                                                    \
    } while (0)

#define CHECK_STR(got, want)                                                 \
    do {                                                                     \
        rg_test_count++;                                                     \
        const char *g_ = (got), *w_ = (want);                                \
        if (!g_ || strcmp(g_, w_) != 0) {                                    \
            printf("  FAIL %s:%d  %s = \"%s\", want \"%s\"\n",               \
                   __FILE__, __LINE__, #got, g_ ? g_ : "(null)", w_);        \
            rg_test_fails++;                                                 \
        }                                                                    \
    } while (0)

#define TEST_MAIN(name, ...)                                                 \
    int main(void) {                                                         \
        printf("%s\n", name);                                                \
        __VA_ARGS__                                                          \
        printf("  %d checks, %d failed\n", rg_test_count, rg_test_fails);    \
        return rg_test_fails ? 1 : 0;                                        \
    }

#endif /* RG_TEST_H */
