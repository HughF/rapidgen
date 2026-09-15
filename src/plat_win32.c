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
 * plat_win32.c — Windows implementation of plat.h.
 *
 * The POSIX file covers Linux and macOS; the two are never compiled together.
 * ANSI entry points throughout, as in svpview and magview: a path outside the
 * active code page will not open.
 */
#if defined(_WIN32)

#include "plat.h"

#include <windows.h>
#include <shlobj.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- time ---------------------------------------------------------- */

uint64_t plat_now_ms(void)
{
    LARGE_INTEGER freq, now;
    if (!QueryPerformanceFrequency(&freq) || freq.QuadPart == 0)
        return (uint64_t)GetTickCount64();
    QueryPerformanceCounter(&now);
    return (uint64_t)((now.QuadPart * 1000) / freq.QuadPart);
}

void plat_date_now(PlatDate *out)
{
    if (!out)
        return;
    SYSTEMTIME st;
    GetLocalTime(&st);
    out->year   = st.wYear;
    out->month  = st.wMonth;
    out->day    = st.wDay;
    out->hour   = st.wHour;
    out->minute = st.wMinute;
    out->second = st.wSecond;
}

/* ---- filesystem ---------------------------------------------------- */

static bool is_sep(char c) { return c == '\\' || c == '/'; }

bool plat_config_dir(char *buf, size_t cap)
{
    char base[MAX_PATH];
    if (SHGetFolderPathA(NULL, CSIDL_APPDATA, NULL, 0, base) != S_OK)
        return false;
    if ((size_t)snprintf(buf, cap, "%s\\rapidgen", base) >= cap)
        return false;
    CreateDirectoryA(buf, NULL);
    return plat_is_dir(buf);
}

void plat_write_error(char *buf, size_t cap, const char *path, int err)
{
    /* Controlled folder access refuses unrecognised programs in Documents,
     * Desktop and Pictures with a plain access-denied, even though the user
     * can write there from Explorer. Nothing here can or should get round it. */
    /* The advice leads: the one-line status bar cuts the end off, and the
     * path is on screen elsewhere. */
    if (err == EACCES)
        snprintf(buf, cap, "Access denied. Controlled folder access (Windows "
                 "Security) may be blocking rapidgen.exe: allow it, or save "
                 "elsewhere. (%s)", path);
    else
        snprintf(buf, cap, "Cannot write %s: %s.", path, strerror(err));
}

bool plat_documents_dir(char *buf, size_t cap)
{
    char base[MAX_PATH];
    if (SHGetFolderPathA(NULL, CSIDL_PERSONAL, NULL, 0, base) == S_OK)
        return (size_t)snprintf(buf, cap, "%s", base) < cap;

    const char *up = getenv("USERPROFILE");
    if (up && *up)
        return (size_t)snprintf(buf, cap, "%s", up) < cap;
    return false;
}

bool plat_path_join(char *buf, size_t cap, const char *dir, const char *leaf)
{
    size_t n = strlen(dir);
    if (n > 0 && is_sep(dir[n - 1]))
        return (size_t)snprintf(buf, cap, "%s%s", dir, leaf) < cap;
    return (size_t)snprintf(buf, cap, "%s\\%s", dir, leaf) < cap;
}

bool plat_mkdir(const char *path)
{
    if (CreateDirectoryA(path, NULL))
        return true;
    return GetLastError() == ERROR_ALREADY_EXISTS && plat_is_dir(path);
}

bool plat_is_dir(const char *path)
{
    DWORD a = GetFileAttributesA(path);
    return a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_DIRECTORY);
}

bool plat_file_exists(const char *path)
{
    DWORD a = GetFileAttributesA(path);
    return a != INVALID_FILE_ATTRIBUTES && !(a & FILE_ATTRIBUTE_DIRECTORY);
}

bool plat_path_parent(char *path)
{
    size_t n = strlen(path);
    /* "C:\" is a root; so is "\\server\share", which is left alone. */
    while (n > 3 && is_sep(path[n - 1]))
        path[--n] = '\0';
    if (n <= 3 && n >= 2 && path[1] == ':')
        return false;

    char *cut = NULL;
    for (char *p = path; *p; p++)
        if (is_sep(*p))
            cut = p;
    if (!cut)
        return false;
    if (cut == path + 2 && path[1] == ':')
        cut[1] = '\0';                 /* "C:\dir" -> "C:\" */
    else
        *cut = '\0';
    return true;
}

const char *plat_path_leaf(const char *path)
{
    const char *leaf = path;
    for (const char *p = path; *p; p++)
        if (is_sep(*p) || *p == ':')
            leaf = p + 1;
    return leaf;
}

static int entry_cmp(const void *a, const void *b)
{
    const PlatDirEntry *x = a, *y = b;
    if (x->is_dir != y->is_dir)
        return x->is_dir ? -1 : 1;
    return _stricmp(x->name, y->name);
}

int plat_dir_list(const char *dir, PlatDirEntry *out, int max)
{
    char pattern[PLAT_PATH_MAX];
    if (!plat_path_join(pattern, sizeof pattern, dir, "*"))
        return -1;

    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA(pattern, &fd);
    if (h == INVALID_HANDLE_VALUE)
        return -1;

    int n = 0;
    do {
        if (fd.cFileName[0] == '.')
            continue;
        if (fd.dwFileAttributes & (FILE_ATTRIBUTE_HIDDEN | FILE_ATTRIBUTE_SYSTEM))
            continue;
        snprintf(out[n].name, sizeof out[n].name, "%.255s", fd.cFileName);
        out[n].is_dir = (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
        n++;
    } while (n < max && FindNextFileA(h, &fd));
    FindClose(h);

    qsort(out, (size_t)n, sizeof out[0], entry_cmp);
    return n;
}

#endif /* _WIN32 */
