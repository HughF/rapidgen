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
 * rg_shape.h — closed outlines from a drawing, and how much of each
 * horizontal line they cover.
 *
 * Open pieces (separate LINEs and ARCs) are joined end to end into closed
 * outlines. An end that meets nothing within the joining tolerance is an
 * error that says where it is and how big the gap is.
 *
 * Each outline is a region. Overlapping outlines are merged (their union is
 * what gets sprayed). An outline wholly inside another is a *hole*, which
 * the caller can find and decide about. An outline that crosses itself is
 * an error — it has no clear inside.
 *
 * The editor builds shapes leniently instead: a real part drawing carries
 * centre lines, dimension lines and construction lines that close nothing,
 * and those are skipped, not fatal, when all that is wanted is the outlines
 * to trace over.
 */
#ifndef RG_SHAPE_H
#define RG_SHAPE_H

#include <stdbool.h>
#include <stddef.h>

#include "rg_dxf.h"

typedef struct {
    RgPt *pts;           /* implicitly closed */
    int   n;
    char  layer[64];
    double x0, y0, x1, y1;
} RgLoop;

typedef struct {
    RgLoop *loops;
    int     n;
    double *ybreaks;     /* sorted: every height at which coverage can change slope */
    int     nbreaks;
} RgShape;

bool rg_shape_build(const RgDrawing *d, double join_tol, RgShape *out, char *err, size_t errcap);

/* Every outline that closes; the pieces that do not are counted in *skipped.
 * No coverage breaks are worked out. False only out of memory. */
bool rg_shape_build_lenient(const RgDrawing *d, double join_tol, RgShape *out, int *skipped);

void rg_shape_free(RgShape *s);

void rg_shape_bounds(const RgShape *s, double *x0, double *y0, double *x1, double *y1);

/* Length of the horizontal line at height y inside the union of the loops. */
double rg_shape_span(const RgShape *s, double y);

/*
 * Between consecutive break heights the covered length is linear in height,
 * so the two ends of each run bound it exactly. span0/span1 are measured just
 * inside the run.
 */
typedef struct { double y0, y1, span0, span1; } RgSpanRun;

/* The runs covering the drawing bottom to top; count, or -1 out of memory. */
int rg_shape_runs(const RgShape *s, RgSpanRun **out);

/* A loop lying wholly inside another: true, with a point on the inner one. */
bool rg_shape_find_hole(const RgShape *s, RgPt *at);

bool   rg_loop_contains(const RgLoop *l, RgPt p);
double rg_loop_area(const RgLoop *l);          /* positive counter-clockwise */

/* The point on the loop's outline nearest p, and its distance. */
double rg_loop_nearest(const RgLoop *l, RgPt p, RgPt *on);

/* The innermost loop containing p, or -1. */
int rg_shape_loop_at(const RgShape *s, RgPt p);

/* The loop whose outline passes nearest p, within radius; -1 if none. */
int rg_shape_loop_near(const RgShape *s, RgPt p, double radius, RgPt *on);

#endif /* RG_SHAPE_H */
