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
/*
 * rg_text.h — a growing text buffer, number formatting and whole-file I/O.
 */
#ifndef RG_TEXT_H
#define RG_TEXT_H

#include <stdbool.h>
#include <stddef.h>

#if defined(__GNUC__)
#define RG_PRINTF(f, a) __attribute__((format(printf, f, a)))
#else
#define RG_PRINTF(f, a)
#endif

/* Text that grows as it is written. An allocation failure sets `failed` and
 * later writes are ignored, so a writer checks once at the end. */
typedef struct {
    char  *s;
    size_t len, cap;
    bool   failed;
} RgBuf;

void rg_buf_init(RgBuf *b);
void rg_buf_free(RgBuf *b);
void rg_buf_puts(RgBuf *b, const char *s);
void rg_buf_printf(RgBuf *b, const char *fmt, ...) RG_PRINTF(2, 3);

/* Copy with truncation; dst is always terminated. */
void rg_copy(char *dst, size_t cap, const char *src);

/* ASCII case-insensitive equality. */
bool rg_streqi(const char *a, const char *b);

/*
 * A number as RAPID and a person both read it: at most `decimals` places,
 * trailing zeros dropped, never "-0". Returns buf.
 */
const char *rg_fmt(char *buf, size_t cap, double v, int decimals);

/* Whole file in, NUL-terminated, malloc'd; NULL with a reason on failure. */
char *rg_read_file(const char *path, size_t *len, char *err, size_t errcap);

bool rg_write_file(const char *path, const char *data, size_t len, char *err, size_t errcap);

#endif /* RG_TEXT_H */
