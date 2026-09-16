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
 * rg_pattern.h — making strokes over a drawing: the geometry behind the
 * editor's painting tools, kept free of any interface so it can be tested.
 *
 *   simplify   a freehand stroke thinned to the points that matter
 *   snap       the drawing's corner or line nearest the pointer
 *   trace      a stroke round an outline, optionally inset from it
 *   fill       parallel passes across a region, joined end to end where
 *              the region allows, run past its edges so they get a full coat
 *
 * All in drawing millimetres (after the drawing scale).
 */
#ifndef RG_PATTERN_H
#define RG_PATTERN_H

#include <stdbool.h>

#include "rg_job.h"
#include "rg_shape.h"

double rg_stroke_length(const RgPt *pts, int n);

/*
 * Ramer–Douglas–Peucker: keep the first and last points and every point
 * further than `tol` from the line through its kept neighbours. `out` has
 * room for n points; returns how many were kept.
 */
int rg_simplify(const RgPt *in, int n, double tol, RgPt *out);

typedef enum { RG_SNAP_NONE, RG_SNAP_VERTEX, RG_SNAP_EDGE } RgSnap;

/* The drawing vertex nearest p within radius, else the nearest point on any
 * of its lines within radius. */
RgSnap rg_snap_drawing(const RgDrawing *d, RgPt p, double radius, RgPt *out);

/*
 * Round a closed outline once, starting and ending at the point on it
 * nearest `start`. `inset` moves the path inside the outline by that much
 * (negative: outside); corners are mitred, and a mitre is limited to four
 * times the inset. A large inset on a tight concave shape can cross itself:
 * that is not detected here. The points are malloc'd.
 */
bool rg_pattern_trace(const RgLoop *l, RgPt start, double inset, RgPt **out, int *n);

typedef struct {
    double pitch;       /* between passes; the fan width less the overlap   */
    double angle_deg;   /* direction of the passes, from drawing X          */
    double extend;      /* each pass runs this far past the region's own
                           outline; at a hole's edge it stops on the edge  */
} RgFillOpts;

/*
 * Passes across the region inside loop `which`, leaving out any loops that
 * lie inside it. Passes are spaced evenly at no more than `pitch`. Where
 * consecutive passes each cross the region once they are joined into one
 * zig-zag stroke, turning outside the edge; otherwise each piece of a pass
 * is its own stroke. Returns the number of strokes, -1 out of memory.
 */
int rg_pattern_fill(const RgShape *s, int which, const RgFillOpts *o, RgStroke **out);

/*
 * Cover the region inside loop `which` by following its outline inward: a
 * ring at `first` inside the edge, then one every `pitch` further in, each
 * ring joined to the next by a step across, so the whole region is one
 * continuous path the gun can be driven round without leaving the work.
 *
 * This is the shape a torch is actually driven in. The zig-zag `fill` turns
 * through a square corner at the end of every pass; a spiral only ever
 * follows the outline.
 *
 * Rings stop when one collapses: inset far enough and a corner turns itself
 * inside out, so a ring is kept only while it still runs the same way round
 * as the outline, still holds area, and is smaller than the ring outside it.
 * Holes are NOT handled: a region with a hole in it is refused (0 strokes),
 * because a ring crossing a hole would coat what is meant to stay bare.
 *
 * One stroke, malloc'd, closed on itself only when a single ring fits.
 * Returns the number of strokes (0 or 1), -1 out of memory.
 */
int rg_pattern_spiral(const RgShape *s, int which, double first, double pitch, RgStroke **out);

void rg_strokes_free(RgStroke *s, int n);

#endif /* RG_PATTERN_H */
