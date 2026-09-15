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
 * plat_posix.c — Linux and macOS implementation of plat.h.
 *
 * Windows lives in plat_win32.c; the two are never compiled together.
 */
#if !defined(_WIN32)

#include "plat.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

/* ---- time ---------------------------------------------------------- */

uint64_t plat_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000);
}

void plat_date_now(PlatDate *out)
{
    if (!out)
        return;
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    out->year   = tm.tm_year + 1900;
    out->month  = tm.tm_mon + 1;
    out->day    = tm.tm_mday;
    out->hour   = tm.tm_hour;
    out->minute = tm.tm_min;
    out->second = tm.tm_sec;
}

/* ---- filesystem ---------------------------------------------------- */

bool plat_config_dir(char *buf, size_t cap)
{
    const char *home = getenv("HOME");
    if (!buf || cap == 0 || !home || !*home)
        return false;

    if ((size_t)snprintf(buf, cap, "%s/.config", home) >= cap)
        return false;
    mkdir(buf, 0755);

    if ((size_t)snprintf(buf, cap, "%s/.config/rapidgen", home) >= cap)
        return false;
    if (mkdir(buf, 0755) != 0 && errno != EEXIST)
        return false;
    return true;
}

void plat_write_error(char *buf, size_t cap, const char *path, int err)
{
    snprintf(buf, cap, "Cannot write %s: %s.", path, strerror(err));
}

bool plat_documents_dir(char *buf, size_t cap)
{
    const char *home = getenv("HOME");
    if (!home || !*home)
        return false;

    char docs[PLAT_PATH_MAX];
    if ((size_t)snprintf(docs, sizeof docs, "%s/Documents", home) < sizeof docs
        && plat_is_dir(docs))
        return (size_t)snprintf(buf, cap, "%s", docs) < cap;
    return (size_t)snprintf(buf, cap, "%s", home) < cap;
}

bool plat_path_join(char *buf, size_t cap, const char *dir, const char *leaf)
{
    size_t n = strlen(dir);
    if (n > 0 && dir[n - 1] == '/')
        return (size_t)snprintf(buf, cap, "%s%s", dir, leaf) < cap;
    return (size_t)snprintf(buf, cap, "%s/%s", dir, leaf) < cap;
}

bool plat_mkdir(const char *path)
{
    if (mkdir(path, 0755) == 0)
        return true;
    return errno == EEXIST && plat_is_dir(path);
}

bool plat_is_dir(const char *path)
{
    struct stat st;
    return stat(path, &st) == 0 && S_ISDIR(st.st_mode);
}

bool plat_file_exists(const char *path)
{
    struct stat st;
    return stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

bool plat_path_parent(char *path)
{
    size_t n = strlen(path);
    while (n > 1 && path[n - 1] == '/')
        path[--n] = '\0';
    if (n <= 1)
        return false;

    char *slash = strrchr(path, '/');
    if (!slash)
        return false;
    if (slash == path)
        slash[1] = '\0';               /* keep the root itself */
    else
        *slash = '\0';
    return true;
}

const char *plat_path_leaf(const char *path)
{
    const char *s = strrchr(path, '/');
    return s ? s + 1 : path;
}

static int entry_cmp(const void *a, const void *b)
{
    const PlatDirEntry *x = a, *y = b;
    if (x->is_dir != y->is_dir)
        return x->is_dir ? -1 : 1;
    return strcasecmp(x->name, y->name);
}

int plat_dir_list(const char *dir, PlatDirEntry *out, int max)
{
    DIR *d = opendir(dir);
    if (!d)
        return -1;

    int n = 0;
    struct dirent *e;
    while ((e = readdir(d)) && n < max) {
        if (e->d_name[0] == '.')
            continue;                  /* dot-files and . / .. alike */

        char full[PLAT_PATH_MAX];
        if (!plat_path_join(full, sizeof full, dir, e->d_name))
            continue;
        snprintf(out[n].name, sizeof out[n].name, "%s", e->d_name);
        out[n].is_dir = plat_is_dir(full);
        n++;
    }
    closedir(d);

    qsort(out, (size_t)n, sizeof out[0], entry_cmp);
    return n;
}

#endif /* !_WIN32 */
