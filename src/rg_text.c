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
#include "rg_text.h"
#include "plat.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void rg_buf_init(RgBuf *b)
{
    memset(b, 0, sizeof *b);
}

void rg_buf_free(RgBuf *b)
{
    free(b->s);
    memset(b, 0, sizeof *b);
}

static bool reserve(RgBuf *b, size_t extra)
{
    if (b->failed)
        return false;
    if (b->len + extra + 1 <= b->cap)
        return true;
    size_t cap = b->cap ? b->cap : 1024;
    while (cap < b->len + extra + 1)
        cap *= 2;
    char *s = realloc(b->s, cap);
    if (!s) {
        b->failed = true;
        return false;
    }
    b->s = s;
    b->cap = cap;
    return true;
}

void rg_buf_puts(RgBuf *b, const char *s)
{
    size_t n = strlen(s);
    if (!reserve(b, n))
        return;
    memcpy(b->s + b->len, s, n + 1);
    b->len += n;
}

void rg_buf_printf(RgBuf *b, const char *fmt, ...)
{
    va_list ap, ap2;
    va_start(ap, fmt);
    va_copy(ap2, ap);
    int n = vsnprintf(NULL, 0, fmt, ap);
    va_end(ap);
    if (n < 0 || !reserve(b, (size_t)n)) {
        if (n < 0)
            b->failed = true;
        va_end(ap2);
        return;
    }
    vsnprintf(b->s + b->len, (size_t)n + 1, fmt, ap2);
    va_end(ap2);
    b->len += (size_t)n;
}

void rg_copy(char *dst, size_t cap, const char *src)
{
    if (!cap)
        return;
    size_t n = strlen(src);
    if (n >= cap)
        n = cap - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

bool rg_streqi(const char *a, const char *b)
{
    for (;; a++, b++) {
        char ca = *a, cb = *b;
        if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
        if (ca != cb)
            return false;
        if (!ca)
            return true;
    }
}

const char *rg_fmt(char *buf, size_t cap, double v, int decimals)
{
    snprintf(buf, cap, "%.*f", decimals, v);
    if (strchr(buf, '.')) {
        char *e = buf + strlen(buf) - 1;
        while (*e == '0')
            *e-- = '\0';
        if (*e == '.')
            *e = '\0';
    }
    if (strcmp(buf, "-0") == 0)
        rg_copy(buf, cap, "0");
    return buf;
}

char *rg_read_file(const char *path, size_t *len, char *err, size_t errcap)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        snprintf(err, errcap, "cannot open %s: %s", path, strerror(errno));
        return NULL;
    }
    RgBuf b;
    rg_buf_init(&b);
    char chunk[8192];
    size_t n;
    while ((n = fread(chunk, 1, sizeof chunk, f)) > 0) {
        if (!reserve(&b, n))
            break;
        memcpy(b.s + b.len, chunk, n);
        b.len += n;
        b.s[b.len] = '\0';
    }
    bool bad = ferror(f) != 0;
    fclose(f);
    if (bad || b.failed) {
        snprintf(err, errcap, "cannot read %s%s", path, b.failed ? ": out of memory" : "");
        rg_buf_free(&b);
        return NULL;
    }
    if (!b.s && !reserve(&b, 0)) {
        snprintf(err, errcap, "cannot read %s: out of memory", path);
        return NULL;
    }
    b.s[b.len] = '\0';
    if (len)
        *len = b.len;
    return b.s;
}

bool rg_write_file(const char *path, const char *data, size_t len, char *err, size_t errcap)
{
    FILE *f = fopen(path, "wb");
    if (!f) {
        plat_write_error(err, errcap, path, errno);
        return false;
    }
    bool ok = fwrite(data, 1, len, f) == len;
    ok = (fclose(f) == 0) && ok;
    if (!ok)
        snprintf(err, errcap, "cannot write %s: %s", path, strerror(errno));
    return ok;
}
