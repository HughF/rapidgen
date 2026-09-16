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
#include "rg_shape.h"
#include "rg_text.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Pairwise edge tests are quadratic in the worst case; beyond this a drawing
 * is refused with advice rather than left to grind. */
#define MAX_EDGES 50000

static double dist(RgPt a, RgPt b)
{
    return hypot(a.x - b.x, a.y - b.y);
}

/* ---- building ------------------------------------------------------- */

typedef struct {
    RgPt *pts;
    int   n, cap;
} Chain;

static bool chain_add(Chain *c, RgPt p)
{
    if (c->n == c->cap) {
        int cap = c->cap ? c->cap * 2 : 64;
        RgPt *q = realloc(c->pts, (size_t)cap * sizeof *q);
        if (!q)
            return false;
        c->pts = q;
        c->cap = cap;
    }
    c->pts[c->n++] = p;
    return true;
}

typedef struct {
    const RgDrawing *d;
    double   tol;
    bool     lenient;
    int      skipped;
    RgShape *s;
    int      cap;
    char    *err;
    size_t   errcap;
} Build;

static bool oom(Build *b)
{
    snprintf(b->err, b->errcap, "out of memory");
    return false;
}

/* Takes ownership of pts. */
static bool take_loop(Build *b, RgPt *pts, int n, const char *layer)
{
    /* Drop a closing point that repeats the first, and repeated points. */
    int m = 0;
    for (int i = 0; i < n; i++)
        if (m == 0 || dist(pts[i], pts[m - 1]) > 1e-9)
            pts[m++] = pts[i];
    while (m > 1 && dist(pts[m - 1], pts[0]) <= b->tol)
        m--;
    if (m < 3) {
        RgPt at = pts[0];
        free(pts);
        if (b->lenient) {
            b->skipped++;
            return true;
        }
        snprintf(b->err, b->errcap, "an outline on layer %s near (%.1f, %.1f) has no area",
                 layer, at.x, at.y);
        return false;
    }
    RgShape *s = b->s;
    if (s->n == b->cap) {
        int nc = b->cap ? b->cap * 2 : 8;
        RgLoop *l = realloc(s->loops, (size_t)nc * sizeof *l);
        if (!l) {
            free(pts);
            return oom(b);
        }
        s->loops = l;
        b->cap = nc;
    }
    RgLoop *l = &s->loops[s->n++];
    memset(l, 0, sizeof *l);
    l->pts = pts;
    l->n = m;
    rg_copy(l->layer, sizeof l->layer, layer);
    l->x0 = l->y0 = DBL_MAX;
    l->x1 = l->y1 = -DBL_MAX;
    for (int i = 0; i < m; i++) {
        l->x0 = fmin(l->x0, pts[i].x);
        l->x1 = fmax(l->x1, pts[i].x);
        l->y0 = fmin(l->y0, pts[i].y);
        l->y1 = fmax(l->y1, pts[i].y);
    }
    return true;
}

static RgPt path_end(const RgPath *p, bool last)
{
    return last ? p->pts[p->n - 1] : p->pts[0];
}

/*
 * Whether path j repeats path i: the same points, either way round, within
 * the joining tolerance. CAD drawings often carry an entity drawn twice over,
 * and a copy at a junction gives the greedy joiner two ends at no distance:
 * it follows the copy straight back, closes a sliver on itself, and the real
 * outline is left without the piece it needed.
 */
static bool same_path(const RgPath *a, const RgPath *b, double tol)
{
    if (a->n != b->n || a->closed != b->closed || a->n < 2)
        return false;
    bool fwd = true, rev = true;
    for (int k = 0; k < a->n && (fwd || rev); k++) {
        fwd = fwd && dist(a->pts[k], b->pts[k]) <= tol;
        rev = rev && dist(a->pts[k], b->pts[a->n - 1 - k]) <= tol;
    }
    return fwd || rev;
}

static bool join_open(Build *b, bool *used)
{
    const RgDrawing *d = b->d;
    bool ok = true;

    for (int i = 0; i < d->n && ok; i++) {
        const RgPath *first = &d->paths[i];
        if (used[i] || first->closed || first->n < 2)
            continue;
        used[i] = true;

        Chain c = { 0 };
        for (int k = 0; k < first->n && ok; k++)
            if (!chain_add(&c, first->pts[k]))
                ok = oom(b);

        while (ok) {
            RgPt end = c.pts[c.n - 1];
            double closing = dist(end, c.pts[0]);
            double best = DBL_MAX;
            int best_j = -1;
            bool best_rev = false;
            for (int j = 0; j < d->n; j++) {
                const RgPath *p = &d->paths[j];
                if (used[j] || p->closed || p->n < 2)
                    continue;
                double ds = dist(end, path_end(p, false));
                double de = dist(end, path_end(p, true));
                if (ds < best) { best = ds; best_j = j; best_rev = false; }
                if (de < best) { best = de; best_j = j; best_rev = true; }
            }
            if (c.n >= 3 && closing <= b->tol && closing <= best) {
                ok = take_loop(b, c.pts, c.n, first->layer);
                c.pts = NULL;
                break;
            }
            if (best_j >= 0 && best <= b->tol) {
                const RgPath *p = &d->paths[best_j];
                used[best_j] = true;
                for (int k = 1; k < p->n && ok; k++)
                    if (!chain_add(&c, p->pts[best_rev ? p->n - 1 - k : k]))
                        ok = oom(b);
                continue;
            }
            if (b->lenient) {
                b->skipped++;
                break;
            }
            double gap = fmin(best, c.n >= 3 ? closing : DBL_MAX);
            if (gap < DBL_MAX)
                snprintf(b->err, b->errcap, "an outline on layer %s is not closed: its end at "
                         "(%.2f, %.2f) is %.2f mm from the nearest other end (joining "
                         "tolerance %.2f mm)", first->layer, end.x, end.y, gap, b->tol);
            else
                snprintf(b->err, b->errcap, "a %s on layer %s from (%.2f, %.2f) to (%.2f, %.2f) "
                         "is not part of a closed outline", first->kind, first->layer,
                         c.pts[0].x, c.pts[0].y, end.x, end.y);
            ok = false;
        }
        free(c.pts);
    }
    return ok;
}

/* ---- crossings ------------------------------------------------------ */

typedef struct {
    RgPt a, b;
    int  loop, idx;
    double ymin, ymax;
} Edge;

static int edge_cmp(const void *pa, const void *pb)
{
    const Edge *a = pa, *b = pb;
    return (a->ymin > b->ymin) - (a->ymin < b->ymin);
}

static int dbl_cmp(const void *pa, const void *pb)
{
    double a = *(const double *)pa, b = *(const double *)pb;
    return (a > b) - (a < b);
}

/* Parameters along both segments where they meet; false for parallel. */
static bool seg_meet(RgPt p, RgPt p2, RgPt q, RgPt q2, double *t, double *u)
{
    double rx = p2.x - p.x, ry = p2.y - p.y;
    double sx = q2.x - q.x, sy = q2.y - q.y;
    double den = rx * sy - ry * sx;
    if (fabs(den) <= 1e-12 * hypot(rx, ry) * hypot(sx, sy))
        return false;
    double qx = q.x - p.x, qy = q.y - p.y;
    *t = (qx * sy - qy * sx) / den;
    *u = (qx * ry - qy * rx) / den;
    return *t >= 0.0 && *t <= 1.0 && *u >= 0.0 && *u <= 1.0;
}

static bool adjacent(const Edge *a, const Edge *b, int loop_n)
{
    if (a->loop != b->loop)
        return false;
    int d = abs(a->idx - b->idx);
    return d == 1 || d == loop_n - 1;
}

static bool push_break(RgShape *s, int *cap, double y)
{
    if (s->nbreaks == *cap) {
        int nc = *cap ? *cap * 2 : 64;
        double *b = realloc(s->ybreaks, (size_t)nc * sizeof *b);
        if (!b)
            return false;
        s->ybreaks = b;
        *cap = nc;
    }
    s->ybreaks[s->nbreaks++] = y;
    return true;
}

static bool analyse(RgShape *s, char *err, size_t errcap)
{
    int total = 0;
    for (int i = 0; i < s->n; i++)
        total += s->loops[i].n;
    if (total > MAX_EDGES) {
        snprintf(err, errcap, "the outlines have %d segments, more than the %d rapidgen "
                 "checks: raise chord_tolerance or simplify the drawing", total, MAX_EDGES);
        return false;
    }

    Edge *e = malloc((size_t)(total ? total : 1) * sizeof *e);
    if (!e) {
        snprintf(err, errcap, "out of memory");
        return false;
    }
    int n = 0, bcap = 0;
    bool ok = true;
    for (int i = 0; i < s->n && ok; i++) {
        const RgLoop *l = &s->loops[i];
        for (int k = 0; k < l->n && ok; k++) {
            Edge *g = &e[n++];
            g->a = l->pts[k];
            g->b = l->pts[(k + 1) % l->n];
            g->loop = i;
            g->idx = k;
            g->ymin = fmin(g->a.y, g->b.y);
            g->ymax = fmax(g->a.y, g->b.y);
            ok = push_break(s, &bcap, l->pts[k].y);
        }
    }
    if (!ok)
        snprintf(err, errcap, "out of memory");
    qsort(e, (size_t)n, sizeof *e, edge_cmp);

    for (int i = 0; i < n && ok; i++) {
        for (int j = i + 1; j < n && e[j].ymin <= e[i].ymax && ok; j++) {
            if (fmax(e[i].a.x, e[i].b.x) < fmin(e[j].a.x, e[j].b.x) ||
                fmax(e[j].a.x, e[j].b.x) < fmin(e[i].a.x, e[i].b.x))
                continue;
            if (adjacent(&e[i], &e[j], s->loops[e[i].loop].n))
                continue;
            double t, u;
            if (!seg_meet(e[i].a, e[i].b, e[j].a, e[j].b, &t, &u))
                continue;
            double y = e[i].a.y + t * (e[i].b.y - e[i].a.y);
            if (e[i].loop == e[j].loop) {
                const double in = 1e-9;
                if (t > in && t < 1.0 - in && u > in && u < 1.0 - in) {
                    double x = e[i].a.x + t * (e[i].b.x - e[i].a.x);
                    snprintf(err, errcap, "an outline on layer %s crosses itself at "
                             "(%.2f, %.2f)", s->loops[e[i].loop].layer, x, y);
                    ok = false;
                }
            } else {
                ok = push_break(s, &bcap, y);
                if (!ok)
                    snprintf(err, errcap, "out of memory");
            }
        }
    }
    free(e);
    if (!ok)
        return false;

    qsort(s->ybreaks, (size_t)s->nbreaks, sizeof *s->ybreaks, dbl_cmp);
    int m = 0;
    for (int i = 0; i < s->nbreaks; i++)
        if (m == 0 || s->ybreaks[i] - s->ybreaks[m - 1] > 1e-9)
            s->ybreaks[m++] = s->ybreaks[i];
    s->nbreaks = m;
    return true;
}

static bool build(const RgDrawing *d, double join_tol, bool lenient, RgShape *out,
                  int *skipped, char *err, size_t errcap)
{
    memset(out, 0, sizeof *out);
    Build b = { d, join_tol, lenient, 0, out, 0, err, errcap };
    bool *used = calloc((size_t)(d->n ? d->n : 1), sizeof *used);
    if (!used)
        return oom(&b);
    bool ok = true;

    /* Copies are marked used before anything is joined, so the joiner never
     * sees two ends where the drawing means one. */
    for (int i = 0; i < d->n; i++)
        for (int j = 0; j < i && !used[i]; j++)
            if (!used[j] && same_path(&d->paths[j], &d->paths[i], join_tol)) {
                used[i] = true;
                out->duplicates++;
            }

    for (int i = 0; i < d->n && ok; i++) {
        const RgPath *p = &d->paths[i];
        if (!p->closed || used[i])
            continue;
        RgPt *pts = malloc((size_t)(p->n ? p->n : 1) * sizeof *pts);
        if (!pts) {
            ok = oom(&b);
            break;
        }
        memcpy(pts, p->pts, (size_t)p->n * sizeof *pts);
        ok = take_loop(&b, pts, p->n, p->layer);
    }
    if (ok)
        ok = join_open(&b, used);
    free(used);
    if (ok && out->n == 0 && !lenient) {
        snprintf(err, errcap, "the drawing has no closed outlines");
        ok = false;
    }
    if (ok && !lenient)
        ok = analyse(out, err, errcap);
    if (skipped)
        *skipped = b.skipped;
    if (!ok)
        rg_shape_free(out);
    return ok;
}

bool rg_shape_build(const RgDrawing *d, double join_tol, RgShape *out, char *err, size_t errcap)
{
    return build(d, join_tol, false, out, NULL, err, errcap);
}

bool rg_shape_build_lenient(const RgDrawing *d, double join_tol, RgShape *out, int *skipped)
{
    char err[256];
    return build(d, join_tol, true, out, skipped, err, sizeof err);
}

void rg_shape_free(RgShape *s)
{
    for (int i = 0; i < s->n; i++)
        free(s->loops[i].pts);
    free(s->loops);
    free(s->ybreaks);
    memset(s, 0, sizeof *s);
}

void rg_shape_bounds(const RgShape *s, double *x0, double *y0, double *x1, double *y1)
{
    double a = DBL_MAX, b = DBL_MAX, c = -DBL_MAX, d = -DBL_MAX;
    for (int i = 0; i < s->n; i++) {
        a = fmin(a, s->loops[i].x0);
        b = fmin(b, s->loops[i].y0);
        c = fmax(c, s->loops[i].x1);
        d = fmax(d, s->loops[i].y1);
    }
    *x0 = a; *y0 = b; *x1 = c; *y1 = d;
}

/* ---- coverage ------------------------------------------------------- */

typedef struct { double a, b; } Iv;

static int iv_cmp(const void *pa, const void *pb)
{
    const Iv *a = pa, *b = pb;
    return (a->a > b->a) - (a->a < b->a);
}

/* Crossings of one loop with the line at y, sorted: inside between pairs. */
static int loop_crossings(const RgLoop *l, double y, double *xs, int cap)
{
    int n = 0;
    for (int k = 0; k < l->n && n < cap; k++) {
        RgPt a = l->pts[k], b = l->pts[(k + 1) % l->n];
        if ((a.y > y) != (b.y > y))
            xs[n++] = a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y);
    }
    qsort(xs, (size_t)n, sizeof *xs, dbl_cmp);
    return n;
}

double rg_shape_span(const RgShape *s, double y)
{
    int total = 0;
    for (int i = 0; i < s->n; i++)
        if (y > s->loops[i].y0 && y < s->loops[i].y1)
            total += s->loops[i].n;
    if (total == 0)
        return 0.0;

    double *xs = malloc((size_t)total * sizeof *xs);
    Iv *iv = malloc((size_t)total * sizeof *iv);
    double len = 0.0;
    if (!xs || !iv)
        goto done;

    int niv = 0;
    for (int i = 0; i < s->n; i++) {
        const RgLoop *l = &s->loops[i];
        if (!(y > l->y0 && y < l->y1))
            continue;
        int n = loop_crossings(l, y, xs, total);
        for (int k = 0; k + 1 < n; k += 2) {
            iv[niv].a = xs[k];
            iv[niv].b = xs[k + 1];
            niv++;
        }
    }
    qsort(iv, (size_t)niv, sizeof *iv, iv_cmp);
    double a = 0.0, b = 0.0;
    bool open = false;
    for (int k = 0; k < niv; k++) {
        if (open && iv[k].a <= b) {
            b = fmax(b, iv[k].b);
            continue;
        }
        if (open)
            len += b - a;
        a = iv[k].a;
        b = iv[k].b;
        open = true;
    }
    if (open)
        len += b - a;
done:
    free(xs);
    free(iv);
    return len;
}

int rg_shape_runs(const RgShape *s, RgSpanRun **out)
{
    *out = NULL;
    if (s->nbreaks < 2)
        return 0;
    RgSpanRun *r = malloc((size_t)(s->nbreaks - 1) * sizeof *r);
    if (!r)
        return -1;
    int n = 0;
    for (int i = 0; i + 1 < s->nbreaks; i++) {
        double y0 = s->ybreaks[i], y1 = s->ybreaks[i + 1];
        double eps = (y1 - y0) * 1e-7;
        r[n].y0 = y0;
        r[n].y1 = y1;
        r[n].span0 = rg_shape_span(s, y0 + eps);
        r[n].span1 = rg_shape_span(s, y1 - eps);
        n++;
    }
    *out = r;
    return n;
}

/* ---- loops ---------------------------------------------------------- */

bool rg_loop_contains(const RgLoop *l, RgPt p)
{
    if (p.x < l->x0 || p.x > l->x1 || p.y < l->y0 || p.y > l->y1)
        return false;
    bool in = false;
    for (int k = 0, j = l->n - 1; k < l->n; j = k++) {
        RgPt a = l->pts[k], b = l->pts[j];
        if ((a.y > p.y) != (b.y > p.y) &&
            p.x < a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y))
            in = !in;
    }
    return in;
}

double rg_loop_area(const RgLoop *l)
{
    double a = 0.0;
    for (int k = 0; k < l->n; k++) {
        RgPt p = l->pts[k], q = l->pts[(k + 1) % l->n];
        a += p.x * q.y - q.x * p.y;
    }
    return 0.5 * a;
}

double rg_loop_nearest(const RgLoop *l, RgPt p, RgPt *on)
{
    double best = DBL_MAX;
    for (int k = 0; k < l->n; k++) {
        RgPt a = l->pts[k], b = l->pts[(k + 1) % l->n];
        double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
        double t = len2 > 0.0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
        t = fmax(0.0, fmin(1.0, t));
        RgPt q = { a.x + t * dx, a.y + t * dy };
        double d = dist(p, q);
        if (d < best) {
            best = d;
            if (on)
                *on = q;
        }
    }
    return best;
}

int rg_shape_loop_at(const RgShape *s, RgPt p)
{
    int best = -1;
    double best_area = DBL_MAX;
    for (int i = 0; i < s->n; i++) {
        if (!rg_loop_contains(&s->loops[i], p))
            continue;
        double a = fabs(rg_loop_area(&s->loops[i]));
        if (a < best_area) {
            best_area = a;
            best = i;
        }
    }
    return best;
}

int rg_shape_loop_near(const RgShape *s, RgPt p, double radius, RgPt *on)
{
    int best = -1;
    double best_d = radius;
    for (int i = 0; i < s->n; i++) {
        const RgLoop *l = &s->loops[i];
        if (p.x < l->x0 - radius || p.x > l->x1 + radius ||
            p.y < l->y0 - radius || p.y > l->y1 + radius)
            continue;
        RgPt q;
        double d = rg_loop_nearest(l, p, &q);
        if (d <= best_d) {
            best_d = d;
            best = i;
            if (on)
                *on = q;
        }
    }
    return best;
}

static bool loops_meet(const RgLoop *a, const RgLoop *b)
{
    for (int i = 0; i < a->n; i++) {
        RgPt p = a->pts[i], p2 = a->pts[(i + 1) % a->n];
        for (int j = 0; j < b->n; j++) {
            double t, u;
            if (seg_meet(p, p2, b->pts[j], b->pts[(j + 1) % b->n], &t, &u))
                return true;
        }
    }
    return false;
}

bool rg_shape_find_hole(const RgShape *s, RgPt *at)
{
    for (int i = 0; i < s->n; i++) {
        const RgLoop *in = &s->loops[i];
        RgPt probe = { 0.5 * (in->pts[0].x + in->pts[1].x),
                       0.5 * (in->pts[0].y + in->pts[1].y) };
        for (int j = 0; j < s->n; j++) {
            const RgLoop *outer = &s->loops[j];
            if (i == j || in->x0 < outer->x0 || in->x1 > outer->x1 ||
                in->y0 < outer->y0 || in->y1 > outer->y1)
                continue;
            if (rg_loop_contains(outer, probe) && !loops_meet(in, outer)) {
                *at = probe;
                return true;
            }
        }
    }
    return false;
}
