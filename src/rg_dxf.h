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
 * rg_dxf.h — reading 2D outlines from an ASCII DXF.
 *
 * Reads LINE, ARC, CIRCLE, LWPOLYLINE and 2D POLYLINE (arcs as bulges), from
 * any AutoCAD version. Arcs are flattened to chords no further than the chord
 * tolerance from the true curve. Text, dimensions and points are skipped as
 * annotation.
 *
 * Anything else that carries geometry is REFUSED, not skipped: a SPLINE,
 * ELLIPSE, block INSERT or HATCH quietly left out would leave a region
 * missing from the program or, for a hatch, draw its outline twice. The
 * message says what to do in CAD instead.
 *
 * Coordinates come back in millimetres, scaled by $INSUNITS; a drawing with
 * no units is taken as millimetres and says so.
 */
#ifndef RG_DXF_H
#define RG_DXF_H

#include <stdbool.h>
#include <stddef.h>

typedef struct { double x, y; } RgPt;

typedef struct {
    RgPt *pts;
    int   n, cap;
    bool  closed;        /* the last point joins back to the first */
    char  layer[64];
    char  kind[16];      /* the entity it came from, for messages */
} RgPath;

typedef struct {
    RgPath *paths;
    int     n, cap;
    int     insunits;    /* $INSUNITS as read; -1 when absent */
    double  to_mm;       /* the scale that was applied        */
    int     skipped;     /* annotation entities passed over   */
    int     unsupported; /* lenient: geometry that was left out */
    char    unsupported_kind[16];   /* ...and the first kind of it */
} RgDrawing;

typedef struct {
    double      chord_tol_mm;   /* > 0 */
    const char *layer;          /* NULL or "" reads every layer */
    bool        lenient;        /* count what cannot be read instead of refusing:
                                   for showing a drawing, never for a program */
} RgDxfOptions;

bool rg_dxf_read(const char *text, size_t len, const RgDxfOptions *opt, RgDrawing *out,
                 char *err, size_t errcap);

bool rg_dxf_load(const char *path, const RgDxfOptions *opt, RgDrawing *out,
                 char *err, size_t errcap);

void rg_drawing_free(RgDrawing *d);

/* Every coordinate multiplied by s. */
void rg_drawing_scale(RgDrawing *d, double s);

/* A deep copy; false out of memory, with dst left empty. */
bool rg_drawing_copy(RgDrawing *dst, const RgDrawing *src);

#endif /* RG_DXF_H */
