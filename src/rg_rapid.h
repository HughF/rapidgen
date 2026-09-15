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
 * rg_rapid.h — writing a plan as a RAPID program for an S4-family controller.
 *
 * One module, one main procedure, written in the subset of RAPID the S4
 * controllers share with everything after them: named CONST robtargets,
 * MoveJ/MoveL, ConfJ/ConfL, SetDO and TPReadFK. Plain ASCII, identifiers of
 * 16 characters or fewer, 8.3 file names where the controller reads floppies.
 *
 * NOT YET VERIFIED against a controller. Each dialect below records what is
 * assumed of it; they are to be matched against programs saved from the real
 * S4, S4C and S4C+ controllers before a generated program is loaded on one.
 */
#ifndef RG_RAPID_H
#define RG_RAPID_H

#include <stdbool.h>
#include <stddef.h>

#include "rg_job.h"
#include "rg_text.h"

struct RgPlan;

typedef struct {
    const char *id;           /* job-file name          */
    const char *name;         /* as ABB writes it       */
    const char *ext;          /* program file extension */
    bool        dos_names;    /* 8.3 file names (floppy) */
    bool        moveabsj;     /* has MoveAbsJ           */
} RgDialect;

int              rg_dialect_count(void);
const RgDialect *rg_dialect_at(int i);
const RgDialect *rg_dialect_find(const char *id);

/* NAME.EXT for the job's controller. */
void rg_rapid_filename(const RgJob *j, char *buf, size_t cap);

/*
 * The program text. Refuses a plan that was refused. `source` names what the
 * program came from and `stamp` when, both for the header comment.
 */
bool rg_rapid_write(const RgJob *j, const struct RgPlan *plan, const char *source,
                    const char *stamp, RgBuf *out, char *err, size_t errcap);

#endif /* RG_RAPID_H */
