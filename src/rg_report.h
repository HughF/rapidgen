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
 * rg_report.h — the plain-text report written beside every program, and in
 * place of one that was refused: what was worked out, what was checked and
 * how close it came, what was refused and why, and what was not checked.
 */
#ifndef RG_REPORT_H
#define RG_REPORT_H

#include "rg_job.h"
#include "rg_plan.h"
#include "rg_text.h"

void rg_report_write(const RgJob *j, const RgPlan *pl, const char *program_file,
                     const char *source, RgBuf *out);

#endif /* RG_REPORT_H */
