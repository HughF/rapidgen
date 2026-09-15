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
 * rg_config.h — per-user settings, remembered between runs
 *
 * Not the job: that lives in the job file. This is where the program was
 * left — theme, folders, the recent jobs.
 */
#ifndef RG_CONFIG_H
#define RG_CONFIG_H

#include <stdbool.h>
#include "plat.h"

#define RG_RECENT 6

typedef struct {
    bool dark;
    char last_job[PLAT_PATH_MAX];
    char job_dir[PLAT_PATH_MAX];
    char dxf_dir[PLAT_PATH_MAX];
    char recent[RG_RECENT][PLAT_PATH_MAX];
} RgSettings;

void rg_settings_default(RgSettings *s);
bool rg_settings_load(RgSettings *s);
bool rg_settings_save(const RgSettings *s);

/* Move `path` to the top of the recent list. */
void rg_settings_add_recent(RgSettings *s, const char *path);

#endif /* RG_CONFIG_H */
