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
#include "rg_pattern.h"
#include "rg_vec.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static double dist(RgPt a, RgPt b)
{
    return hypot(a.x - b.x, a.y - b.y);
}

static double seg_dist(RgPt p, RgPt a, RgPt b, RgPt *on)
{
    double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
    double t = len2 > 0.0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
    t = fmax(0.0, fmin(1.0, t));
    RgPt q = { a.x + t * dx, a.y + t * dy };
    if (on)
        *on = q;
    return dist(p, q);
}

double rg_stroke_length(const RgPt *pts, int n)
{
    double len = 0.0;
    for (int i = 1; i < n; i++)
        len += dist(pts[i - 1], pts[i]);
    return len;
}

int rg_simplify(const RgPt *in, int n, double tol, RgPt *out)
{
    if (n <= 2) {
        memcpy(out, in, (size_t)(n > 0 ? n : 0) * sizeof *out);
        return n;
    }
    bool *keep = calloc((size_t)n, sizeof *keep);
    int *stack = malloc((size_t)(2 * n + 2) * sizeof *stack);
    if (!keep || !stack) {
        free(keep);
        free(stack);
        memcpy(out, in, (size_t)n * sizeof *out);
        return n;
    }
    keep[0] = keep[n - 1] = true;
    int sp = 0;
    stack[sp++] = 0;
    stack[sp++] = n - 1;
    while (sp) {
        int j = stack[--sp], i = stack[--sp];
        double worst = -1.0;
        int at = -1;
        for (int k = i + 1; k < j; k++) {
            double d = seg_dist(in[k], in[i], in[j], NULL);
            if (d > worst) {
                worst = d;
                at = k;
            }
        }
        if (at >= 0 && worst > tol) {
            keep[at] = true;
            stack[sp++] = i;
            stack[sp++] = at;
            stack[sp++] = at;
            stack[sp++] = j;
        }
    }
    int m = 0;
    for (int k = 0; k < n; k++)
        if (keep[k])
            out[m++] = in[k];
    free(keep);
    free(stack);
    return m;
}

RgSnap rg_snap_drawing(const RgDrawing *d, RgPt p, double radius, RgPt *out)
{
    double best = radius;
    RgSnap kind = RG_SNAP_NONE;
    for (int i = 0; i < d->n; i++)
        for (int k = 0; k < d->paths[i].n; k++) {
            double dd = dist(p, d->paths[i].pts[k]);
            if (dd <= best) {
                best = dd;
                *out = d->paths[i].pts[k];
                kind = RG_SNAP_VERTEX;
            }
        }
    if (kind != RG_SNAP_NONE)
        return kind;

    for (int i = 0; i < d->n; i++) {
        const RgPath *path = &d->paths[i];
        int segs = path->closed ? path->n : path->n - 1;
        for (int k = 0; k < segs; k++) {
            RgPt q;
            double dd = seg_dist(p, path->pts[k], path->pts[(k + 1) % path->n], &q);
            if (dd <= best) {
                best = dd;
                *out = q;
                kind = RG_SNAP_EDGE;
            }
        }
    }
    return kind;
}

/* ---- trace ---------------------------------------------------------- */

static void offset_loop(const RgPt *src, int n, double inset, bool ccw, RgPt *dst)
{
    double sgn = ccw ? 1.0 : -1.0;
    for (int i = 0; i < n; i++) {
        RgPt prev = src[(i + n - 1) % n], cur = src[i], next = src[(i + 1) % n];
        double l0 = dist(prev, cur), l1 = dist(cur, next);
        double d0x = l0 > 0 ? (cur.x - prev.x) / l0 : 0, d0y = l0 > 0 ? (cur.y - prev.y) / l0 : 0;
        double d1x = l1 > 0 ? (next.x - cur.x) / l1 : 0, d1y = l1 > 0 ? (next.y - cur.y) / l1 : 0;
        /* Inward normals: to the left of travel round a counter-clockwise loop. */
        double n0x = -sgn * d0y, n0y = sgn * d0x;
        double n1x = -sgn * d1y, n1y = sgn * d1x;
        double mx = n0x + n1x, my = n0y + n1y, ml = hypot(mx, my);
        double c = 1.0;
        if (ml < 1e-9) {
            mx = n0x;
            my = n0y;
        } else {
            mx /= ml;
            my /= ml;
            c = mx * n0x + my * n0y;
        }
        if (c < 0.25)
            c = 0.25;
        dst[i].x = cur.x + mx * inset / c;
        dst[i].y = cur.y + my * inset / c;
    }
}

bool rg_pattern_trace(const RgLoop *l, RgPt start, double inset, RgPt **out, int *nout)
{
    *out = NULL;
    *nout = 0;
    int n = l->n;
    if (n < 3)
        return false;

    RgPt *off = NULL;
    const RgPt *src = l->pts;
    if (fabs(inset) > 1e-9) {
        off = malloc((size_t)n * sizeof *off);
        if (!off)
            return false;
        offset_loop(l->pts, n, inset, rg_loop_area(l) >= 0.0, off);
        src = off;
    }

    int edge = 0;
    RgPt q = src[0];
    double best = DBL_MAX;
    for (int k = 0; k < n; k++) {
        RgPt on;
        double d = seg_dist(start, src[k], src[(k + 1) % n], &on);
        if (d < best) {
            best = d;
            edge = k;
            q = on;
        }
    }

    RgPt *r = malloc((size_t)(n + 2) * sizeof *r);
    if (!r) {
        free(off);
        return false;
    }
    int m = 0;
    r[m++] = q;
    for (int i = 1; i <= n; i++) {
        RgPt v = src[(edge + i) % n];
        if (dist(v, r[m - 1]) > 1e-9)
            r[m++] = v;
    }
    if (dist(q, r[m - 1]) > 1e-9)
        r[m++] = q;

    free(off);
    *out = r;
    *nout = m;
    return true;
}

/* ---- fill ----------------------------------------------------------- */

typedef struct {
    RgStroke *s;
    int n, cap;
} Strokes;

typedef struct {
    RgPt *p;
    int n, cap;
} Pts;

static bool pts_add(Pts *v, double x, double y)
{
    if (v->n == v->cap) {
        int cap = v->cap ? v->cap * 2 : 32;
        RgPt *q = realloc(v->p, (size_t)cap * sizeof *q);
        if (!q)
            return false;
        v->p = q;
        v->cap = cap;
    }
    v->p[v->n].x = x;
    v->p[v->n].y = y;
    v->n++;
    return true;
}

/* Hands the points over to the list (and resets v), rotated back to the
 * drawing's axes. */
static bool strokes_take(Strokes *list, Pts *v, double ca, double sa)
{
    if (v->n < 2) {
        v->n = 0;
        return true;
    }
    if (list->n == list->cap) {
        int cap = list->cap ? list->cap * 2 : 16;
        RgStroke *q = realloc(list->s, (size_t)cap * sizeof *q);
        if (!q)
            return false;
        list->s = q;
        list->cap = cap;
    }
    for (int i = 0; i < v->n; i++) {
        double x = v->p[i].x, y = v->p[i].y;
        v->p[i].x = x * ca - y * sa;
        v->p[i].y = x * sa + y * ca;
    }
    RgPt *owned = realloc(v->p, (size_t)v->n * sizeof *owned);
    list->s[list->n].pts = owned ? owned : v->p;
    list->s[list->n].n = v->n;
    list->n++;
    v->p = NULL;
    v->n = v->cap = 0;
    return true;
}

/* Where a pass crosses an outline, and whether it is the region's own
 * outline (passes run past it) or a hole's (passes stop on it, so nothing
 * meant to stay bare is sprayed). */
typedef struct {
    double x;
    bool   outer;
} Cross;

static int cross_cmp(const void *pa, const void *pb)
{
    const Cross *a = pa, *b = pb;
    return (a->x > b->x) - (a->x < b->x);
}

void rg_strokes_free(RgStroke *s, int n)
{
    for (int i = 0; i < n; i++)
        free(s[i].pts);
    free(s);
}

/* ---- spiral --------------------------------------------------------- */

/* The signed area of a bare ring, so a collapsed one can be told from a
 * good one without building a whole RgLoop for it. */
static double ring_area(const RgPt *p, int n)
{
    double a = 0.0;
    for (int i = 0; i < n; i++) {
        RgPt u = p[i], v = p[(i + 1) % n];
        a += u.x * v.y - v.x * u.y;
    }
    return 0.5 * a;
}

/* Start the ring at the point nearest `from`, so the step across from the
 * ring outside it is as short as it can be, and go round once. */
static bool ring_from(Pts *cur, const RgPt *ring, int n, RgPt from)
{
    int at = 0;
    double best = DBL_MAX;
    for (int i = 0; i < n; i++) {
        double d = dist(from, ring[i]);
        if (d < best) {
            best = d;
            at = i;
        }
    }
    for (int i = 0; i <= n; i++) {
        RgPt v = ring[(at + i) % n];
        if (cur->n && dist(v, cur->p[cur->n - 1]) < 1e-9)
            continue;
        if (!pts_add(cur, v.x, v.y))
            return false;
    }
    return true;
}

int rg_pattern_spiral(const RgShape *s, int which, double first, double pitch, RgStroke **out)
{
    *out = NULL;
    if (which < 0 || which >= s->n || !(pitch > 0.0))
        return 0;

    const RgLoop *outer = &s->loops[which];
    int n = outer->n;
    if (n < 3)
        return 0;

    /* A ring that ran through a hole would coat what is meant to stay bare,
     * and an inset ring cannot be trusted to keep clear of one. */
    double outer_area = fabs(rg_loop_area(outer));
    for (int i = 0; i < s->n; i++)
        if (i != which && rg_loop_contains(outer, s->loops[i].pts[0]) &&
            fabs(rg_loop_area(&s->loops[i])) < outer_area)
            return 0;

    bool ccw = rg_loop_area(outer) >= 0.0;
    double sign = ccw ? 1.0 : -1.0;

    RgPt *ring = malloc((size_t)n * sizeof *ring);
    if (!ring)
        return -1;

    Pts cur = { 0 };
    RgPt from = outer->pts[0];
    double prev_area = fabs(rg_loop_area(outer));
    bool ok = true, any = false;

    for (int r = 0; ok; r++) {
        double inset = first + r * pitch;
        offset_loop(outer->pts, n, inset, ccw, ring);

        /*
         * Inset far enough and the ring folds through the middle and comes
         * out the other side: on a 100 mm square an inset of 60 gives a tidy
         * 20 mm square, the right way round and smaller than its parent, so
         * area alone lets it through. The test that catches it is the
         * definition of an offset — every point of a ring inset by d stands
         * d clear of the outline — and a folded ring stands nearer than it
         * claims.
         */
        double area = ring_area(ring, n);
        if (area * sign <= 0.0 || fabs(area) < 1e-6 || fabs(area) >= prev_area)
            break;
        bool sound = true;
        for (int i = 0; i < n && sound; i++) {
            RgPt on;
            sound = rg_loop_contains(outer, ring[i]) &&
                    rg_loop_nearest(outer, ring[i], &on) + 1e-6 >= inset;
        }
        if (!sound)
            break;
        prev_area = fabs(area);

        ok = ring_from(&cur, ring, n, from);
        if (!ok)
            break;
        from = cur.n ? cur.p[cur.n - 1] : from;
        any = true;
    }
    free(ring);

    if (!ok || !any) {
        free(cur.p);
        return ok ? 0 : -1;
    }

    Strokes list = { 0 };
    if (!strokes_take(&list, &cur, 1.0, 0.0)) {     /* already in drawing axes */
        free(cur.p);
        rg_strokes_free(list.s, list.n);
        return -1;
    }
    free(cur.p);
    *out = list.s;
    return list.n;
}

int rg_pattern_fill(const RgShape *s, int which, const RgFillOpts *o, RgStroke **out)
{
    *out = NULL;
    if (which < 0 || which >= s->n || !(o->pitch > 0.0))
        return 0;

    const RgLoop *outer = &s->loops[which];
    double outer_area = fabs(rg_loop_area(outer));
    double a = o->angle_deg * RG_DEG, ca = cos(a), sa = sin(a);

    /* The outline and the holes in it, turned so the passes run along X. */
    int nl = 0, total = 0;
    const RgLoop **use = malloc((size_t)s->n * sizeof *use);
    if (!use)
        return -1;
    for (int i = 0; i < s->n; i++) {
        const RgLoop *l = &s->loops[i];
        bool hole = i != which && rg_loop_contains(outer, l->pts[0]) &&
                    fabs(rg_loop_area(l)) < outer_area;
        if (i == which || hole) {
            use[nl++] = l;
            total += l->n;
        }
    }
    RgPt *rot = malloc((size_t)total * sizeof *rot);
    Cross *xs = malloc((size_t)total * sizeof *xs);
    if (!rot || !xs) {
        free(use);
        free(rot);
        free(xs);
        return -1;
    }
    double y0 = DBL_MAX, y1 = -DBL_MAX;
    for (int i = 0, k = 0; i < nl; i++)
        for (int p = 0; p < use[i]->n; p++, k++) {
            RgPt v = use[i]->pts[p];
            rot[k].x = v.x * ca + v.y * sa;
            rot[k].y = -v.x * sa + v.y * ca;
            if (i == 0) {
                y0 = fmin(y0, rot[k].y);
                y1 = fmax(y1, rot[k].y);
            }
        }

    int rows = (int)ceil((y1 - y0) / o->pitch - 1e-9);
    if (rows < 1)
        rows = 1;
    double step = (y1 - y0) / rows, e = o->extend;

    Strokes list = { 0 };
    Pts cur = { 0 };
    bool ok = true, zig_open = false;
    int prev_count = 0, dir = 1;

    for (int r = 0; r < rows && ok; r++) {
        double y = y0 + (r + 0.5) * step;
        int nx = 0;
        for (int i = 0, k = 0; i < nl; i++) {
            for (int p = 0; p < use[i]->n; p++) {
                RgPt pa = rot[k + p], pb = rot[k + (p + 1) % use[i]->n];
                if ((pa.y > y) != (pb.y > y)) {
                    xs[nx].x = pa.x + (y - pa.y) * (pb.x - pa.x) / (pb.y - pa.y);
                    xs[nx].outer = i == 0;
                    nx++;
                }
            }
            k += use[i]->n;
        }
        qsort(xs, (size_t)nx, sizeof *xs, cross_cmp);
        int count = nx / 2;

        if (count == 1 && zig_open && prev_count == 1) {
            dir = -dir;
        } else {
            ok = strokes_take(&list, &cur, ca, sa);
            zig_open = false;
            dir = (r % 2 == 0) ? 1 : -1;
        }
        for (int k = 0; ok && k < count; k++) {
            int idx = dir > 0 ? k : count - 1 - k;
            double xa = xs[2 * idx].x - (xs[2 * idx].outer ? e : 0.0);
            double xb = xs[2 * idx + 1].x + (xs[2 * idx + 1].outer ? e : 0.0);
            if (dir > 0)
                ok = pts_add(&cur, xa, y) && pts_add(&cur, xb, y);
            else
                ok = pts_add(&cur, xb, y) && pts_add(&cur, xa, y);
            if (ok && count > 1)
                ok = strokes_take(&list, &cur, ca, sa);
        }
        zig_open = ok && count == 1;
        prev_count = count;
    }
    if (ok)
        ok = strokes_take(&list, &cur, ca, sa);

    free(cur.p);
    free(use);
    free(rot);
    free(xs);
    if (!ok) {
        rg_strokes_free(list.s, list.n);
        return -1;
    }
    *out = list.s;
    return list.n;
}
