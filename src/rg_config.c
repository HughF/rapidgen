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
#include "rg_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONFIG_LEAF "settings.conf"

void rg_settings_default(RgSettings *s)
{
    memset(s, 0, sizeof *s);
    s->dark = true;
    char docs[PLAT_PATH_MAX];
    if (!plat_documents_dir(docs, sizeof docs))
        snprintf(docs, sizeof docs, ".");
    snprintf(s->job_dir, sizeof s->job_dir, "%s", docs);
    snprintf(s->dxf_dir, sizeof s->dxf_dir, "%s", docs);
}

static bool config_path(char *buf, size_t cap)
{
    char dir[PLAT_PATH_MAX];
    if (!plat_config_dir(dir, sizeof dir))
        return false;
    return plat_path_join(buf, cap, dir, CONFIG_LEAF);
}

bool rg_settings_load(RgSettings *s)
{
    rg_settings_default(s);

    char path[PLAT_PATH_MAX];
    if (!config_path(path, sizeof path))
        return false;
    FILE *f = fopen(path, "r");
    if (!f)
        return false;

    char line[PLAT_PATH_MAX + 64];
    while (fgets(line, sizeof line, f)) {
        size_t n = strlen(line);
        while (n && (line[n - 1] == '\n' || line[n - 1] == '\r'))
            line[--n] = '\0';
        char *eq = strchr(line, '=');
        if (!eq)
            continue;
        *eq = '\0';
        const char *k = line, *v = eq + 1;

        if (strcmp(k, "dark") == 0)
            s->dark = atoi(v) != 0;
        else if (strcmp(k, "last_job") == 0)
            snprintf(s->last_job, sizeof s->last_job, "%s", v);
        else if (strcmp(k, "job_dir") == 0 && *v)
            snprintf(s->job_dir, sizeof s->job_dir, "%s", v);
        else if (strcmp(k, "dxf_dir") == 0 && *v)
            snprintf(s->dxf_dir, sizeof s->dxf_dir, "%s", v);
        else if (strncmp(k, "recent", 6) == 0) {
            int i = atoi(k + 6);
            if (i >= 0 && i < RG_RECENT)
                snprintf(s->recent[i], sizeof s->recent[i], "%s", v);
        }
    }
    fclose(f);
    return true;
}

bool rg_settings_save(const RgSettings *s)
{
    char path[PLAT_PATH_MAX];
    if (!config_path(path, sizeof path))
        return false;
    FILE *f = fopen(path, "w");
    if (!f)
        return false;
    fprintf(f, "dark=%d\n", s->dark ? 1 : 0);
    fprintf(f, "last_job=%s\n", s->last_job);
    fprintf(f, "job_dir=%s\n", s->job_dir);
    fprintf(f, "dxf_dir=%s\n", s->dxf_dir);
    for (int i = 0; i < RG_RECENT; i++)
        if (s->recent[i][0])
            fprintf(f, "recent%d=%s\n", i, s->recent[i]);
    return fclose(f) == 0;
}

void rg_settings_add_recent(RgSettings *s, const char *path)
{
    if (!path || !path[0])
        return;
    char keep[RG_RECENT][PLAT_PATH_MAX];
    int n = 0;
    snprintf(keep[n++], PLAT_PATH_MAX, "%s", path);
    for (int i = 0; i < RG_RECENT && n < RG_RECENT; i++)
        if (s->recent[i][0] && strcmp(s->recent[i], path) != 0)
            snprintf(keep[n++], PLAT_PATH_MAX, "%s", s->recent[i]);
    for (int i = 0; i < RG_RECENT; i++) {
        if (i < n)
            memcpy(s->recent[i], keep[i], PLAT_PATH_MAX);
        else
            s->recent[i][0] = '\0';
    }
}
