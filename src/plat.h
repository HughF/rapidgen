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
 * plat.h — everything OS-specific, behind one header.
 *
 * No file above this line calls an OS API directly. plat_posix.c covers Linux
 * and macOS; plat_win32.c covers Windows. The two are never compiled together.
 * rapidgen talks to no hardware, so this is time and the filesystem only.
 */
#ifndef PLAT_H
#define PLAT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- time ---------------------------------------------------------- */

uint64_t plat_now_ms(void);            /* monotonic, arbitrary origin */

/* Local wall-clock time, for the date on a drawing. */
typedef struct {
    int year, month, day, hour, minute, second;
} PlatDate;

void plat_date_now(PlatDate *out);

/* ---- filesystem ---------------------------------------------------- */

#define PLAT_PATH_MAX 512

/* Per-user config directory, created if needed. False if it cannot be made. */
bool plat_config_dir(char *buf, size_t cap);

/* Documents (or home) directory — a sensible default place for projects. */
bool plat_documents_dir(char *buf, size_t cap);

/* Join with the platform separator. False if it would not fit. */
bool plat_path_join(char *buf, size_t cap, const char *dir, const char *leaf);

/* Create a directory (its parent is assumed to exist). Idempotent: true if
 * the directory now exists. */
bool plat_mkdir(const char *path);

bool plat_is_dir(const char *path);
bool plat_file_exists(const char *path);

/* Cut `path` back to its parent directory, in place. False when it is
 * already a filesystem root and cannot go higher. */
bool plat_path_parent(char *path);

/* The last component of a path (points into `path`). */
const char *plat_path_leaf(const char *path);

typedef struct {
    char name[256];
    bool is_dir;
} PlatDirEntry;

/* List a directory: no "." or "..", directories first, then by name,
 * case-insensitively. Returns the count (at most max), or -1 if the directory
 * cannot be read. */
int plat_dir_list(const char *dir, PlatDirEntry *out, int max);

/* Why a file could not be opened for writing, for the user: `err` is the
 * errno left by fopen. On Windows a refusal also names the likely cause. */
void plat_write_error(char *buf, size_t cap, const char *path, int err);

#endif /* PLAT_H */
