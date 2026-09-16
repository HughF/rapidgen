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
#include <stdio.h>
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
    RgPt          *p;
    unsigned char *off;
    int            n, cap;
} Pts;

static bool pts_add_off(Pts *v, double x, double y, bool off)
{
    if (v->n == v->cap) {
        int cap = v->cap ? v->cap * 2 : 32;
        RgPt *q = realloc(v->p, (size_t)cap * sizeof *q);
        if (!q)
            return false;
        v->p = q;
        unsigned char *f = realloc(v->off, (size_t)cap);
        if (!f)
            return false;
        v->off = f;
        v->cap = cap;
    }
    v->p[v->n].x = x;
    v->p[v->n].y = y;
    v->off[v->n] = off;
    v->n++;
    return true;
}

static bool pts_add(Pts *v, double x, double y)
{
    return pts_add_off(v, x, y, false);
}

static void pts_free(Pts *v)
{
    free(v->p);
    free(v->off);
    v->p = NULL;
    v->off = NULL;
    v->n = v->cap = 0;
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
    /* The flags go with the points only when they say something; the list
     * is realloc'd, so the field must be set either way. */
    int noff = 0;
    for (int i = 0; i < v->n; i++)
        noff += v->off[i] != 0;
    if (noff > 0 && noff < v->n) {
        list->s[list->n].off = v->off;
    } else {
        list->s[list->n].off = NULL;
        free(v->off);
    }
    list->n++;
    v->p = NULL;
    v->off = NULL;
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
    for (int i = 0; i < n; i++) {
        free(s[i].pts);
        free(s[i].off);
    }
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
        pts_free(&cur);
        return ok ? 0 : -1;
    }

    Strokes list = { 0 };
    if (!strokes_take(&list, &cur, 1.0, 0.0)) {     /* already in drawing axes */
        pts_free(&cur);
        rg_strokes_free(list.s, list.n);
        return -1;
    }
    pts_free(&cur);
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
    double runout = o->runout > 0.0 ? o->runout : 0.0;   /* not `r`: the rows are */

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
            bool outer_a = xs[2 * idx].outer, outer_b = xs[2 * idx + 1].outer;
            double xa = xs[2 * idx].x - (outer_a ? e : 0.0);
            double xb = xs[2 * idx + 1].x + (outer_b ? e : 0.0);
            /* Past the region's own outline the gun carries on off the work,
             * far enough to turn round or leave without slowing over it. Not
             * at a hole's edge: that would coat what is meant to stay bare. */
            double from = dir > 0 ? xa : xb, to = dir > 0 ? xb : xa, sgn = dir > 0 ? 1.0 : -1.0;
            double r_from = (dir > 0 ? outer_a : outer_b) ? runout : 0.0;
            double r_to = (dir > 0 ? outer_b : outer_a) ? runout : 0.0;
            if (r_from > 0.0)
                ok = pts_add_off(&cur, from - sgn * r_from, y, true);
            ok = ok && pts_add(&cur, from, y) && pts_add(&cur, to, y);
            if (ok && r_to > 0.0)
                ok = pts_add_off(&cur, to + sgn * r_to, y, true);
            if (ok && count > 1)
                ok = strokes_take(&list, &cur, ca, sa);
        }
        zig_open = ok && count == 1;
        prev_count = count;
    }
    if (ok)
        ok = strokes_take(&list, &cur, ca, sa);

    pts_free(&cur);
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

/* ---- track rings ---------------------------------------------------- */

/* Where two segments cross, not at their ends: the parameters along each. */
static bool seg_cross(RgPt a, RgPt b, RgPt c, RgPt d, double *t, double *u)
{
    double rx = b.x - a.x, ry = b.y - a.y, sx = d.x - c.x, sy = d.y - c.y;
    double den = rx * sy - ry * sx;
    if (fabs(den) < 1e-12)
        return false;
    double qx = c.x - a.x, qy = c.y - a.y;
    *t = (qx * sy - qy * sx) / den;
    *u = (qx * ry - qy * rx) / den;
    return *t > 1e-9 && *t < 1.0 - 1e-9 && *u > 1e-9 && *u < 1.0 - 1e-9;
}

/*
 * Cut out the loops an offset ring makes where it crosses itself - offsetting
 * a concave corner by more than its radius folds it over - keeping whichever
 * side encloses more. In place; returns the new count.
 */
static int untangle(RgPt *p, int n)
{
    for (int pass = 0; pass < 4 * n && n > 3; pass++) {
        int ci = -1, cj = -1;
        RgPt x = { 0.0, 0.0 };
        for (int i = 0; i < n && ci < 0; i++)
            for (int j = i + 2; j < n && ci < 0; j++) {
                double t, u;
                if (i == 0 && j == n - 1)
                    continue;
                if (seg_cross(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n], &t, &u)) {
                    ci = i;
                    cj = j;
                    x.x = p[i].x + t * (p[i + 1].x - p[i].x);
                    x.y = p[i].y + t * (p[i + 1].y - p[i].y);
                }
            }
        if (ci < 0)
            break;
        /* Split at the crossing: x, p[ci+1..cj] is one loop, the rest the other.
         * Their signed areas add up to the whole. */
        double a_in = 0.0;
        int m = cj - ci + 1;
        for (int k = 0; k < m; k++) {
            RgPt u = k == 0 ? x : p[ci + k];
            RgPt v = k + 1 == m ? x : p[ci + k + 1];
            a_in += u.x * v.y - v.x * u.y;
        }
        double a_out = 2.0 * ring_area(p, n) - a_in;
        if (fabs(a_in) > fabs(a_out)) {
            memmove(&p[1], &p[ci + 1], (size_t)(cj - ci) * sizeof *p);
            p[0] = x;
            n = cj - ci + 1;
        } else {
            p[ci + 1] = x;
            memmove(&p[ci + 2], &p[cj + 1], (size_t)(n - cj - 1) * sizeof *p);
            n = n - (cj - ci) + 1;
        }
    }
    return n;
}

/*
 * Not every fold crosses. Offset a slot narrower than twice the offset and its
 * two walls pass each other without meeting, leaving an inverted pocket whose
 * edge runs back over itself. A point of a true offset stands the full offset
 * from its edge, so any that stands nearer is part of such a fold and goes,
 * and so does any spike where the path then doubles straight back.
 */
static int drop_folds(RgPt *p, int n, const RgLoop *src, double inset)
{
    double want = fabs(inset) - 0.02 * fabs(inset) - 0.05;
    int m = 0;
    for (int i = 0; i < n; i++)
        if (fabs(inset) < 1e-6 || rg_loop_nearest(src, p[i], NULL) >= want)
            p[m++] = p[i];
    for (bool changed = true; changed && m > 3;) {
        changed = false;
        for (int i = 0; i < m && m > 3; i++) {
            RgPt a = p[(i + m - 1) % m], b = p[i], c = p[(i + 1) % m];
            double ux = b.x - a.x, uy = b.y - a.y, vx = c.x - b.x, vy = c.y - b.y;
            double lu = hypot(ux, uy), lv = hypot(vx, vy);
            if (lu < 1e-9 || lv < 1e-9 || (ux * vx + uy * vy) / (lu * lv) < -0.99) {
                memmove(&p[i], &p[i + 1], (size_t)(m - i - 1) * sizeof *p);
                m--;
                changed = true;
            }
        }
    }
    return m;
}

/* Rotate a closed ring to start at the point on it nearest p, which becomes a
 * vertex. `out` needs room for n + 1 points; returns the count. */
static int ring_start_at(const RgPt *r, int n, RgPt p, RgPt *out)
{
    int seg = 0;
    RgPt q = r[0];
    double best = DBL_MAX;
    for (int k = 0; k < n; k++) {
        RgPt on;
        double d = seg_dist(p, r[k], r[(k + 1) % n], &on);
        if (d < best) {
            best = d;
            seg = k;
            q = on;
        }
    }
    int m = 0;
    out[m++] = q;
    for (int i = 1; i <= n; i++) {
        RgPt v = r[(seg + i) % n];
        if (dist(v, out[m - 1]) > 1e-9 && dist(v, q) > 1e-9)
            out[m++] = v;
    }
    return m;
}

static double ring_len(const RgPt *r, int n)
{
    double len = 0.0;
    for (int k = 0; k < n; k++)
        len += dist(r[k], r[(k + 1) % n]);
    return len;
}

/* The point `a` along a closed ring from its first point. */
static RgPt ring_at(const RgPt *r, int n, double a)
{
    for (int k = 0; k < n; k++) {
        RgPt u = r[k], v = r[(k + 1) % n];
        double len = dist(u, v);
        if (a <= len || k == n - 1) {
            double t = len > 0.0 ? fmax(0.0, fmin(1.0, a / len)) : 0.0;
            RgPt p = { u.x + t * (v.x - u.x), u.y + t * (v.y - u.y) };
            return p;
        }
        a -= len;
    }
    return r[0];
}

static int dbl_cmp(const void *pa, const void *pb)
{
    double a = *(const double *)pa, b = *(const double *)pb;
    return (a > b) - (a < b);
}

/* How far apart a track's two edges are, sampled round the outer edge: the
 * typical width, and the narrowest and widest. */
static bool measure_track(const RgLoop *outer, const RgLoop *inner, RgRingInfo *info)
{
    int n = 2 * outer->n;
    double *d = malloc((size_t)n * sizeof *d);
    if (!d)
        return false;
    for (int k = 0; k < outer->n; k++) {
        RgPt a = outer->pts[k], b = outer->pts[(k + 1) % outer->n];
        RgPt mid = { 0.5 * (a.x + b.x), 0.5 * (a.y + b.y) };
        d[2 * k] = rg_loop_nearest(inner, a, NULL);
        d[2 * k + 1] = rg_loop_nearest(inner, mid, NULL);
    }
    qsort(d, (size_t)n, sizeof *d, dbl_cmp);
    info->width_min = d[0];
    info->width_max = d[n - 1];
    info->width = d[n / 2];
    free(d);
    return true;
}

static void add_ring_arc(Pts *cur, const RgPt *r, int n, double from, double to, bool *ok)
{
    double acc = 0.0;
    for (int i = 1; i <= n && *ok; i++) {
        acc += dist(r[i - 1], r[i % n]);
        if (acc > from + 1e-9 && acc < to - 1e-9)
            *ok = pts_add(cur, r[i % n].x, r[i % n].y);
    }
    if (*ok) {
        RgPt e = ring_at(r, n, to);
        *ok = pts_add(cur, e.x, e.y);
    }
}

int rg_pattern_rings(const RgShape *s, const RgRingOpts *o, RgStroke **out, RgRingInfo *info)
{
    *out = NULL;
    memset(info, 0, sizeof *info);
    if (o->outer < 0 || o->outer >= s->n || o->inner >= s->n || o->inner == o->outer) {
        snprintf(info->why, sizeof info->why, "pick the track's outer edge");
        return 0;
    }
    if (!(o->step > 0.0)) {
        snprintf(info->why, sizeof info->why, "set the step-over first: it spaces the passes");
        return 0;
    }
    const RgLoop *outer = &s->loops[o->outer];
    const RgLoop *inner = o->inner >= 0 ? &s->loops[o->inner] : NULL;
    if (outer->n < 3 || (inner && inner->n < 3))
        return 0;

    double width = o->width;
    if (inner) {
        if (!rg_loop_contains(outer, inner->pts[0]) ||
            fabs(rg_loop_area(inner)) >= fabs(rg_loop_area(outer))) {
            snprintf(info->why, sizeof info->why, "the inner edge picked is not inside the outer "
                                                  "edge");
            return 0;
        }
        if (!measure_track(outer, inner, info))
            return -1;
        width = info->width;
    } else {
        info->width = info->width_min = info->width_max = width;
    }
    if (!(width > 0.0)) {
        snprintf(info->why, sizeof info->why, "pick the inner edge, or give the track's width");
        return 0;
    }

    double first_out = fmax(0.0, o->first_out), last_past = fmax(0.0, o->last_past);
    double span = first_out + width + last_past;
    int nrings = (int)ceil(span / o->step - 1e-9) + 1;
    if (nrings < 2)
        nrings = 2;
    if (nrings > 2000) {
        snprintf(info->why, sizeof info->why, "that would be %d passes: the step-over is too "
                                              "small for the track", nrings);
        return 0;
    }
    double spacing = span / (nrings - 1);
    info->rings = nrings;
    info->spacing = spacing;

    int cap = outer->n + (inner ? inner->n : 0) + 2;
    RgPt *tmp = malloc((size_t)cap * sizeof *tmp);
    RgPt **ring = calloc((size_t)nrings, sizeof *ring);
    int *rn = calloc((size_t)nrings, sizeof *rn);
    int result = -1;
    Pts cur = { 0 };
    if (!tmp || !ring || !rn)
        goto done;

    for (int k = 0; k < nrings; k++) {
        double offset = -first_out + k * spacing;   /* inward from the outer edge */
        const RgLoop *src = outer;
        double inset = offset;
        if (inner && offset > 0.5 * width) {
            src = inner;
            inset = offset - width;                 /* from the inner edge, into the middle */
        }
        bool ccw = rg_loop_area(src) >= 0.0;
        offset_loop(src->pts, src->n, inset, ccw, tmp);
        int m = untangle(tmp, drop_folds(tmp, src->n, src, inset));
        double area = ring_area(tmp, m);
        /* A ring that has turned inside out, or stands nearer its edge than it
         * should, is not an offset of it: the track has no room for it. */
        bool sound = m >= 3 && fabs(area) > 1e-6 && (area > 0.0) == ccw;
        for (int i = 0; i < m && sound; i++) {
            bool inside = rg_loop_contains(src, tmp[i]);
            double d = rg_loop_nearest(src, tmp[i], NULL);
            sound = d + 0.02 * fabs(inset) + 0.05 >= fabs(inset) &&
                    (fabs(inset) < 1e-6 || inside == (inset > 0.0));
        }
        if (!sound) {
            snprintf(info->why, sizeof info->why, "pass %d, %.1f mm %s the %s edge, does not fit: "
                     "the %s is too small for it", k + 1, fabs(inset),
                     inset >= 0.0 ? "inside" : "outside", src == outer ? "outer" : "inner",
                     inset > 0.0 && src == inner ? "middle of the track" : "track");
            result = 0;
            goto done;
        }
        if (area < 0.0)
            for (int a = 0, b = m - 1; a < b; a++, b--) {
                RgPt t = tmp[a];
                tmp[a] = tmp[b];
                tmp[b] = t;
            }
        ring[k] = malloc((size_t)(m + 1) * sizeof *ring[k]);
        if (!ring[k])
            goto done;
        rn[k] = ring_start_at(tmp, m, k == 0 ? o->seam : ring[k - 1][0], ring[k]);
    }

    bool ok = true;
    /* On along a tangent to the first ring, off the work. */
    RgPt s0 = ring[0][0], s1 = ring[0][1];
    double t0 = dist(s0, s1);
    if (o->lead > 0.0 && t0 > 1e-9)
        ok = pts_add_off(&cur, s0.x - (s1.x - s0.x) / t0 * o->lead,
                         s0.y - (s1.y - s0.y) / t0 * o->lead, true);
    /* The first ring all the way round. */
    for (int i = 0; ok && i < rn[0]; i++)
        ok = pts_add(&cur, ring[0][i].x, ring[0][i].y);
    ok = ok && pts_add(&cur, s0.x, s0.y);

    double last_drift = 0.0;
    for (int k = 0; ok && k + 1 < nrings; k++) {
        double l0 = ring_len(ring[k], rn[k]), l1 = ring_len(ring[k + 1], rn[k + 1]);
        double d = fmax(0.0, fmin(o->drift, 0.5 * fmin(l0, l1)));
        /*
         * Drift across: blend from this ring to the next over d of path. The
         * stretch is left again the same way on the next drift, (1 - t) out
         * and t in, so in the middle of the track it is covered once. Each
         * ring is taken at the same fraction of its own length, not the same
         * distance: rings of different sizes turn through different angles in
         * the same distance, and blending those twists the drift across the
         * inner ring.
         */
        double d1 = d * l1 / l0;
        int steps = d > 0.0 ? (int)ceil(d / 2.0) : 0;
        for (int j = 1; ok && j <= steps; j++) {
            double t = (double)j / steps;
            RgPt a = ring_at(ring[k], rn[k], t * d), b = ring_at(ring[k + 1], rn[k + 1], t * d1);
            ok = pts_add(&cur, a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
        }
        if (ok && steps == 0)
            ok = pts_add(&cur, ring[k + 1][0].x, ring[k + 1][0].y);
        /* The rest of the next ring, back to where it was joined. */
        if (ok)
            add_ring_arc(&cur, ring[k + 1], rn[k + 1], steps ? d1 : 0.0, l1, &ok);
        last_drift = steps ? d1 : 0.0;
    }
    /* The last ring was only drifted onto over its first stretch: go over it
     * once more, so it is not left thin. */
    int lr = nrings - 1;
    if (ok && last_drift > 0.0)
        add_ring_arc(&cur, ring[lr], rn[lr], 0.0, last_drift, &ok);
    /* Off into the middle, square to the last ring. */
    if (ok && o->lead > 0.0 && cur.n >= 2) {
        RgPt e = cur.p[cur.n - 1], pe = cur.p[cur.n - 2];
        double tl = dist(e, pe);
        if (tl > 1e-9)
            ok = pts_add_off(&cur, e.x - (e.y - pe.y) / tl * o->lead,
                             e.y + (e.x - pe.x) / tl * o->lead, true);
    }
    if (!ok)
        goto done;

    Strokes list = { 0 };
    if (!strokes_take(&list, &cur, 1.0, 0.0)) {
        rg_strokes_free(list.s, list.n);
        goto done;
    }
    *out = list.s;
    result = list.n;

done:
    pts_free(&cur);
    for (int k = 0; ring && k < nrings; k++)
        free(ring[k]);
    free(ring);
    free(rn);
    free(tmp);
    return result;
}
