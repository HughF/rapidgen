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
/* test_pattern.c — the painting tools' geometry. */
#include "rg_test.h"
#include "rg_pattern.h"

static char err[512];

static bool drawing_of(const char *entities, RgDrawing *d)
{
    char text[8192];
    snprintf(text, sizeof text, "0\nSECTION\n2\nENTITIES\n%s0\nENDSEC\n0\nEOF\n", entities);
    RgDxfOptions opt = { 0.2, NULL, false };
    return rg_dxf_read(text, strlen(text), &opt, d, err, sizeof err);
}

static const char *rect(char *buf, size_t cap, double x0, double y0, double x1, double y1)
{
    snprintf(buf, cap, "0\nLWPOLYLINE\n8\n0\n70\n1\n10\n%g\n20\n%g\n10\n%g\n20\n%g\n"
             "10\n%g\n20\n%g\n10\n%g\n20\n%g\n", x0, y0, x1, y0, x1, y1, x0, y1);
    return buf;
}

static void test_simplify(void)
{
    RgPt line[100], out[100];
    for (int i = 0; i < 100; i++) {
        line[i].x = i;
        line[i].y = 0.3 * sin(i * 0.7);      /* wobble inside the tolerance */
    }
    CHECK(rg_simplify(line, 100, 0.5, out) == 2);
    CHECK_NEAR(out[1].x, 99.0, 0);

    /* An L drawn by hand: the corner survives. */
    RgPt ell[40];
    for (int i = 0; i < 20; i++) {
        ell[i].x = i * 5.0;
        ell[i].y = 0.1 * (i % 2);
        ell[20 + i].x = 95.0 + 0.1 * (i % 2);
        ell[20 + i].y = (i + 1) * 5.0;
    }
    int n = rg_simplify(ell, 40, 0.5, out);
    CHECK(n == 3);
    CHECK_NEAR(out[1].x, 95.0, 0.2);
    CHECK_NEAR(out[1].y, 0.0, 0.2);

    CHECK(rg_simplify(ell, 2, 0.5, out) == 2);
    CHECK_NEAR(rg_stroke_length(ell, 2), 5.0, 0.01);
}

static void test_snap(void)
{
    char r[512];
    RgDrawing d;
    CHECK(drawing_of(rect(r, sizeof r, 0, 0, 100, 50), &d));
    RgPt q;
    RgPt near_corner = { 101, 49 };
    CHECK(rg_snap_drawing(&d, near_corner, 3, &q) == RG_SNAP_VERTEX);
    CHECK_NEAR(q.x, 100, 0);
    CHECK_NEAR(q.y, 50, 0);
    RgPt near_edge = { 40, 1.5 };
    CHECK(rg_snap_drawing(&d, near_edge, 3, &q) == RG_SNAP_EDGE);
    CHECK_NEAR(q.x, 40, 1e-9);
    CHECK_NEAR(q.y, 0, 1e-9);
    RgPt closing = { -1, 25 };                  /* the edge back to the start */
    CHECK(rg_snap_drawing(&d, closing, 3, &q) == RG_SNAP_EDGE);
    CHECK_NEAR(q.x, 0, 1e-9);
    RgPt far = { 50, 25 };
    CHECK(rg_snap_drawing(&d, far, 3, &q) == RG_SNAP_NONE);
    rg_drawing_free(&d);
}

static void test_trace(void)
{
    char r[512];
    RgDrawing d;
    RgShape s;
    int skipped = -1;
    CHECK(drawing_of(rect(r, sizeof r, 0, 0, 100, 100), &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));
    CHECK(s.n == 1 && skipped == 0);

    RgPt *pts;
    int n;
    RgPt start = { 50, -5 };
    CHECK(rg_pattern_trace(&s.loops[0], start, 0, &pts, &n));
    CHECK(n == 6);
    CHECK_NEAR(pts[0].x, 50, 1e-9);
    CHECK_NEAR(pts[0].y, 0, 1e-9);
    CHECK_NEAR(pts[1].x, 100, 1e-9);            /* counter-clockwise, as drawn */
    CHECK_NEAR(pts[n - 1].x, 50, 1e-9);
    CHECK_NEAR(rg_stroke_length(pts, n), 400, 1e-9);
    free(pts);

    CHECK(rg_pattern_trace(&s.loops[0], start, 10, &pts, &n));
    CHECK_NEAR(pts[0].y, 10, 1e-9);
    CHECK_NEAR(rg_stroke_length(pts, n), 320, 1e-9);
    free(pts);

    CHECK(rg_pattern_trace(&s.loops[0], start, -10, &pts, &n));
    CHECK_NEAR(rg_stroke_length(pts, n), 480, 1e-9);
    free(pts);
    rg_shape_free(&s);
    rg_drawing_free(&d);

    /* Drawn clockwise, inset still goes inward. */
    CHECK(drawing_of("0\nLWPOLYLINE\n8\n0\n70\n1\n10\n0\n20\n0\n10\n0\n20\n100\n"
                     "10\n100\n20\n100\n10\n100\n20\n0\n", &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));
    CHECK(rg_loop_area(&s.loops[0]) < 0);
    CHECK(rg_pattern_trace(&s.loops[0], start, 10, &pts, &n));
    CHECK_NEAR(rg_stroke_length(pts, n), 320, 1e-9);
    free(pts);
    rg_shape_free(&s);
    rg_drawing_free(&d);
}

/* A real part drawing has lines that close nothing; lenient building keeps
 * the outlines and counts the rest. */
static void test_lenient(void)
{
    char r[512], both[1024];
    snprintf(both, sizeof both, "%s0\nLINE\n8\nCENTRE\n10\n-20\n20\n50\n11\n120\n21\n50\n",
             rect(r, sizeof r, 0, 0, 100, 100));
    RgDrawing d;
    RgShape s;
    int skipped = 0;
    CHECK(drawing_of(both, &d));
    CHECK(!rg_shape_build(&d, 0.1, &s, err, sizeof err));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));
    CHECK(s.n == 1);
    CHECK(skipped == 1);

    RgPt inside = { 50, 50 }, outside = { 150, 50 }, by_edge = { 102, 30 };
    CHECK(rg_shape_loop_at(&s, inside) == 0);
    CHECK(rg_shape_loop_at(&s, outside) == -1);
    RgPt on;
    CHECK(rg_shape_loop_near(&s, by_edge, 5, &on) == 0);
    CHECK_NEAR(on.x, 100, 1e-9);
    CHECK(rg_shape_loop_near(&s, outside, 5, &on) == -1);
    rg_shape_free(&s);
    rg_drawing_free(&d);
}

/*
 * A spiral follows the outline inward: one continuous path, so the gun is
 * driven round the work rather than turned through a square corner at the
 * end of every pass.
 */
static void test_spiral(void)
{
    char r[512];
    RgDrawing d;
    RgShape s;
    int skipped = -1;
    CHECK(drawing_of(rect(r, sizeof r, 0, 0, 100, 100), &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));

    /* First ring 6 mm in, then every 10 mm: 6, 16, 26, 36, 46 — and 46 is the
     * last that still holds area in a 100 mm square (the centre is at 50). */
    RgStroke *st;
    int n = rg_pattern_spiral(&s, 0, 6, 10, &st);
    CHECK(n == 1);
    if (n == 1) {
        /* Five rings of five points each: four corners and the point that
         * closes the ring, less the joins that land on each other. */
        CHECK(st[0].n >= 25);
        /* It starts on the outermost ring, 6 mm in from the corner it began at. */
        CHECK_NEAR(st[0].pts[0].x, 6, 1e-9);
        CHECK_NEAR(st[0].pts[0].y, 6, 1e-9);
        /* Every point stays inside the outline, and no ring escapes it. */
        bool inside = true;
        for (int i = 0; i < st[0].n; i++)
            inside = inside && rg_loop_contains(&s.loops[0], st[0].pts[i]);
        CHECK(inside);
        /* The path works inward: the last point is nearer the centre than the
         * first, and the whole thing is one stroke, not five. */
        RgPt a = st[0].pts[0], b = st[0].pts[st[0].n - 1];
        CHECK(hypot(b.x - 50, b.y - 50) < hypot(a.x - 50, a.y - 50));
        rg_strokes_free(st, n);
    }

    /* Too coarse to fit even one ring inside: nothing, rather than a ring
     * folded through the middle. An inset of 60 on a 100 mm square offsets
     * to a tidy 20 mm square that is the right way round and smaller than
     * its parent — it is only caught by standing every point off the
     * outline. */
    int coarse = rg_pattern_spiral(&s, 0, 60, 10, &st);
    CHECK(coarse == 0);
    if (coarse > 0)
        rg_strokes_free(st, coarse);
    rg_shape_free(&s);
    rg_drawing_free(&d);

    /* A region with a hole is refused: a ring would run straight over it. */
    char inner[512], both[1024];
    snprintf(both, sizeof both, "%s%s", rect(r, sizeof r, 0, 0, 100, 100),
             rect(inner, sizeof inner, 40, 40, 60, 60));
    CHECK(drawing_of(both, &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));
    CHECK(s.n == 2);
    CHECK(rg_pattern_spiral(&s, 0, 6, 10, &st) == 0);
    rg_shape_free(&s);
    rg_drawing_free(&d);
}

/*
 * A weave with run-outs: each pass carries on past the outline, off the work,
 * so the gun comes on at speed, turns round clear of the part, and leaves.
 */
static void test_fill_runout(void)
{
    char r[512], hole[512], both[1024];
    RgDrawing d;
    RgShape s;
    int skipped;
    CHECK(drawing_of(rect(r, sizeof r, 0, 0, 100, 60), &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));

    RgFillOpts o = { 10, 0, 0, 30 };
    RgStroke *st;
    int n = rg_pattern_fill(&s, 0, &o, &st);
    CHECK(n == 1);
    if (n == 1) {
        /* six passes, each: off, on, on, off */
        CHECK(st[0].n == 24);
        CHECK(st[0].off != NULL);
        int noff = 0;
        bool shape = true;
        for (int k = 0; k < st[0].n && st[0].off; k++) {
            noff += st[0].off[k];
            shape = shape && st[0].off[k] == (k % 4 == 0 || k % 4 == 3);
            /* off the work is outside the outline, on the work inside it */
            if (st[0].off[k])
                shape = shape && (st[0].pts[k].x < -29.9 || st[0].pts[k].x > 129.9);
            else
                shape = shape && st[0].pts[k].x > -1e-9 && st[0].pts[k].x < 100 + 1e-9;
        }
        CHECK(noff == 12);
        CHECK(shape);
        CHECK_NEAR(st[0].pts[0].x, -30, 1e-9);            /* comes on from outside */
        CHECK_NEAR(st[0].pts[4].x, 130, 1e-9);            /* second pass turned beyond */
        rg_strokes_free(st, n);
    }
    rg_shape_free(&s);
    rg_drawing_free(&d);

    /* With a hole, a pass runs out past the outline but never into the hole. */
    snprintf(both, sizeof both, "%s%s", rect(r, sizeof r, 0, 0, 100, 60),
             rect(hole, sizeof hole, 40, 20, 60, 40));
    CHECK(drawing_of(both, &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));
    n = rg_pattern_fill(&s, 0, &o, &st);
    CHECK(n > 1);
    bool clear = true;
    for (int i = 0; i < n; i++)
        for (int k = 0; k < st[i].n; k++)
            if (rg_stroke_off(&st[i], k))
                clear = clear && !rg_loop_contains(&s.loops[0], st[i].pts[k]);
    CHECK(clear);
    rg_strokes_free(st, n);
    rg_shape_free(&s);
    rg_drawing_free(&d);
}

/* A stadium: straights 200 long, semicircles of radius r at x = +-100. */
static const char *stadium(char *buf, size_t cap, double r)
{
    snprintf(buf, cap,
             "0\nLINE\n8\n0\n10\n-100\n20\n%g\n11\n100\n21\n%g\n"
             "0\nARC\n8\n0\n10\n100\n20\n0\n40\n%g\n50\n-90\n51\n90\n"
             "0\nLINE\n8\n0\n10\n100\n20\n%g\n11\n-100\n21\n%g\n"
             "0\nARC\n8\n0\n10\n-100\n20\n0\n40\n%g\n50\n90\n51\n270\n",
             -r, -r, r, r, r, r);
    return buf;
}

/* The track's edge a point is nearest, signed: + inside the outer edge. */
static double from_outer(const RgLoop *outer, RgPt p)
{
    double d = rg_loop_nearest(outer, p, NULL);
    return rg_loop_contains(outer, p) ? d : -d;
}

static bool stroke_crosses_itself(const RgStroke *st)
{
    for (int i = 0; i + 1 < st->n; i++)
        for (int j = i + 2; j + 1 < st->n; j++) {
            RgPt a = st->pts[i], b = st->pts[i + 1], c = st->pts[j], d = st->pts[j + 1];
            double rx = b.x - a.x, ry = b.y - a.y, sx = d.x - c.x, sy = d.y - c.y;
            double den = rx * sy - ry * sx;
            if (fabs(den) < 1e-12)
                continue;
            double t = ((c.x - a.x) * sy - (c.y - a.y) * sx) / den;
            double u = ((c.x - a.x) * ry - (c.y - a.y) * rx) / den;
            if (t > 1e-6 && t < 1 - 1e-6 && u > 1e-6 && u < 1 - 1e-6)
                return true;
        }
    return false;
}

/*
 * A closed track covered by rings from the outside in: the first pass outside
 * the outer edge, the last past the inner edge, the gun drifting from ring to
 * ring, on along a lead-in and off into the middle.
 */
static void test_rings(void)
{
    char out_e[1024], in_e[1024], both[2048];
    snprintf(both, sizeof both, "%s%s", stadium(out_e, sizeof out_e, 100),
             stadium(in_e, sizeof in_e, 80));
    RgDrawing d;
    RgShape s;
    int skipped;
    CHECK(drawing_of(both, &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));
    CHECK(s.n == 2);
    int o = fabs(rg_loop_area(&s.loops[0])) > fabs(rg_loop_area(&s.loops[1])) ? 0 : 1;
    int in = 1 - o;

    RgRingOpts opt = { o, in, 0, 6, 6, 6, 30, 60, { 0, 200 } };
    RgStroke *st;
    RgRingInfo info;
    int n = rg_pattern_rings(&s, &opt, &st, &info);
    CHECK(n == 1);
    /* 6 + 20 + 6 = 32 mm across, no more than 6 apart: 7 passes, 5.33 apart */
    CHECK(info.rings == 7);
    CHECK_NEAR(info.spacing, 32.0 / 6.0, 1e-9);
    CHECK_NEAR(info.width, 20.0, 0.3);
    CHECK(info.width_max - info.width_min < 0.5);
    if (n == 1) {
        const RgStroke *r = &st[0];
        CHECK(r->off && r->off[0] && r->off[r->n - 1]);
        bool bounds = true, work = true;
        for (int k = 1; k + 1 < r->n; k++) {
            double sd = from_outer(&s.loops[o], r->pts[k]);
            bounds = bounds && sd > -6.5 && sd < 26.5;
            work = work && r->off && !r->off[k];
        }
        CHECK(bounds);
        CHECK(work);
        /* the first pass 6 outside the outer edge, the last 6 past the inner */
        CHECK_NEAR(from_outer(&s.loops[o], r->pts[1]), -6.0, 0.3);
        CHECK_NEAR(rg_loop_nearest(&s.loops[in], r->pts[r->n - 2], NULL), 6.0, 0.3);
        CHECK(rg_loop_contains(&s.loops[in], r->pts[r->n - 2]));
        /* on from outside the track, off into its middle */
        CHECK(!rg_loop_contains(&s.loops[o], r->pts[0]));
        CHECK(rg_loop_contains(&s.loops[in], r->pts[r->n - 1]));
        CHECK(!stroke_crosses_itself(r));
        rg_strokes_free(st, n);
    }

    /* Only the outer edge, and the width typed in: the same passes. */
    RgRingOpts by_width = opt;
    by_width.inner = -1;
    by_width.width = 20;
    n = rg_pattern_rings(&s, &by_width, &st, &info);
    CHECK(n == 1 && info.rings == 7);
    if (n > 0)
        rg_strokes_free(st, n);

    /* A last pass further past the inner edge than the middle is wide. */
    RgRingOpts too_far = opt;
    too_far.last_past = 200;
    CHECK(rg_pattern_rings(&s, &too_far, &st, &info) == 0);
    CHECK(strstr(info.why, "does not fit") != NULL);

    /* The edges picked the wrong way round. */
    RgRingOpts swapped = opt;
    swapped.outer = in;
    swapped.inner = o;
    CHECK(rg_pattern_rings(&s, &swapped, &st, &info) == 0);
    CHECK(strstr(info.why, "not inside") != NULL);
    rg_shape_free(&s);
    rg_drawing_free(&d);

    /*
     * An outline with a slot 10 mm wide cut into it. A pass 8 mm outside it
     * folds across the slot - its walls pass each other - and the fold is cut
     * out. (The width is small: legs 45 mm wide cannot hold passes deeper.)
     */
    CHECK(drawing_of("0\nLWPOLYLINE\n8\n0\n70\n1\n10\n0\n20\n0\n10\n100\n20\n0\n"
                     "10\n100\n20\n100\n10\n55\n20\n100\n10\n55\n20\n50\n10\n45\n20\n50\n"
                     "10\n45\n20\n100\n10\n0\n20\n100\n", &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));
    RgRingOpts slot = { 0, -1, 4, 8, 0, 4, 20, 40, { 0, -50 } };
    n = rg_pattern_rings(&s, &slot, &st, &info);
    CHECK(n == 1);
    if (n == 1) {
        CHECK(!stroke_crosses_itself(&st[0]));
        /* Nothing deeper than the innermost pass, 4 mm in - or its mitred
         * corners, 4 x sqrt 2 from the corner they turn round: nothing has
         * folded back through the part. */
        bool clear = true;
        for (int k = 1; k + 1 < st[0].n; k++)
            clear = clear && from_outer(&s.loops[0], st[0].pts[k]) < 4.0 * sqrt(2.0) + 0.05;
        CHECK(clear);
        CHECK(info.rings == 4);
        rg_strokes_free(st, n);
    } else {
        printf("  slot: %s\n", info.why);
    }
    rg_shape_free(&s);
    rg_drawing_free(&d);
}

/* How far round a closed loop, from its first point, the point on it nearest p lies. */
static double along_loop(const RgLoop *l, RgPt p)
{
    double best = 1e300, at = 0.0, run = 0.0;
    for (int k = 0; k < l->n; k++) {
        RgPt a = l->pts[k], b = l->pts[(k + 1) % l->n];
        double dx = b.x - a.x, dy = b.y - a.y, len = hypot(dx, dy);
        double t = len > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / (len * len) : 0;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        double d = hypot(a.x + t * dx - p.x, a.y + t * dy - p.y);
        if (d < best) {
            best = d;
            at = run + t * len;
        }
        run += len;
    }
    return at;
}

/*
 * A seam that moves from cycle to cycle: one version of the rings each cycle,
 * each stepping somewhere else, never two in the same stretch of the track.
 */
static void test_ring_seams(void)
{
    char out_e[1024], in_e[1024], both[2048];
    snprintf(both, sizeof both, "%s%s", stadium(out_e, sizeof out_e, 100),
             stadium(in_e, sizeof in_e, 80));
    RgDrawing d;
    RgShape s;
    int skipped;
    CHECK(drawing_of(both, &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));
    int o = fabs(rg_loop_area(&s.loops[0])) > fabs(rg_loop_area(&s.loops[1])) ? 0 : 1;
    const RgLoop *edge = &s.loops[o];
    double perim = 0.0;
    for (int k = 0; k < edge->n; k++)
        perim += hypot(edge->pts[(k + 1) % edge->n].x - edge->pts[k].x,
                       edge->pts[(k + 1) % edge->n].y - edge->pts[k].y);

    RgRingOpts opt = { o, 1 - o, 0, 6, 6, 6, 30, 60, { 0, 0 } };
    RgStroke *st, *again;
    RgRingInfo info;
    enum { V = 6 };
    int n = rg_pattern_ring_seams(&s, &opt, V, 42, &st, &info);
    CHECK(n == V);
    CHECK(info.rings == 7);
    if (n == V) {
        bool bins[V] = { false }, numbered = true, clean = true;
        for (int k = 0; k < V; k++) {
            numbered = numbered && st[k].cycle == k + 1;
            clean = clean && !stroke_crosses_itself(&st[k]) && !rg_stroke_off_crosses_work(&st[k]);
            int bin = (int)(along_loop(edge, st[k].pts[1]) / perim * V);
            if (bin >= 0 && bin < V)
                bins[bin] = true;
        }
        CHECK(numbered);
        CHECK(clean);
        bool spread = true;
        for (int k = 0; k < V; k++)
            spread = spread && bins[k];
        CHECK(spread);                            /* one seam in every stretch */

        /* the same seed, the same seams; another seed, others */
        CHECK(rg_pattern_ring_seams(&s, &opt, V, 42, &again, &info) == V);
        CHECK(again[3].n == st[3].n &&
              memcmp(again[3].pts, st[3].pts, (size_t)st[3].n * sizeof *st[3].pts) == 0);
        rg_strokes_free(again, V);
        CHECK(rg_pattern_ring_seams(&s, &opt, V, 7, &again, &info) == V);
        CHECK(hypot(again[0].pts[1].x - st[0].pts[1].x, again[0].pts[1].y - st[0].pts[1].y) > 1.0);
        rg_strokes_free(again, V);
        rg_strokes_free(st, V);
    }
    CHECK(rg_pattern_ring_seams(&s, &opt, 0, 1, &st, &info) == 0);
    rg_shape_free(&s);
    rg_drawing_free(&d);

    /* A run-out that cuts back across the pass, and one that stays clear. */
    RgPt p[] = { { 50, 10 }, { 50, -10 }, { 0, -10 }, { 0, 0 }, { 100, 0 } };
    unsigned char off[] = { 1, 0, 0, 0, 0 };
    RgStroke cut = { p, 5, off, 0 };
    CHECK(rg_stroke_off_crosses_work(&cut));
    p[0].y = -20;
    CHECK(!rg_stroke_off_crosses_work(&cut));
    off[4] = 1;                                   /* off crossing off coats nothing twice */
    p[0].y = 10;
    CHECK(!rg_stroke_off_crosses_work(&cut));
}

static void test_fill(void)
{
    char a[512], b[512], both[1024];
    RgDrawing d;
    RgShape s;
    int skipped;

    CHECK(drawing_of(rect(a, sizeof a, 0, 0, 100, 40), &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));
    RgFillOpts o = { 10, 0, 5, 0 };
    RgStroke *st;
    int n = rg_pattern_fill(&s, 0, &o, &st);
    CHECK(n == 1);                              /* one zig-zag */
    CHECK(n == 1 && st[0].n == 8);
    if (n == 1 && st[0].n == 8) {
        CHECK_NEAR(st[0].pts[0].x, -5, 1e-9);
        CHECK_NEAR(st[0].pts[0].y, 5, 1e-9);
        CHECK_NEAR(st[0].pts[1].x, 105, 1e-9);
        CHECK_NEAR(st[0].pts[2].x, 105, 1e-9);
        CHECK_NEAR(st[0].pts[2].y, 15, 1e-9);
        CHECK_NEAR(st[0].pts[7].y, 35, 1e-9);
    }
    rg_strokes_free(st, n);

    /* Along Y instead: ten passes, still one stroke, inside the part's width. */
    o.angle_deg = 90;
    n = rg_pattern_fill(&s, 0, &o, &st);
    CHECK(n == 1 && st[0].n == 20);
    double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
    for (int i = 0; n == 1 && i < st[0].n; i++) {
        xmin = fmin(xmin, st[0].pts[i].x); xmax = fmax(xmax, st[0].pts[i].x);
        ymin = fmin(ymin, st[0].pts[i].y); ymax = fmax(ymax, st[0].pts[i].y);
    }
    CHECK_NEAR(xmin, 5, 1e-9);
    CHECK_NEAR(xmax, 95, 1e-9);
    CHECK_NEAR(ymin, -5, 1e-9);
    CHECK_NEAR(ymax, 45, 1e-9);
    rg_strokes_free(st, n);

    /* A pitch that does not divide the height is tightened, never widened. */
    o.angle_deg = 0;
    o.pitch = 15;
    n = rg_pattern_fill(&s, 0, &o, &st);
    CHECK(n == 1 && st[0].n == 6);              /* three passes of 13.3 mm */
    rg_strokes_free(st, n);
    rg_shape_free(&s);
    rg_drawing_free(&d);

    /* A hole splits the middle passes into pieces of their own. */
    snprintf(both, sizeof both, "%s%s", rect(a, sizeof a, 0, 0, 100, 40),
             rect(b, sizeof b, 40, 10, 60, 30));
    CHECK(drawing_of(both, &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));
    int outer = fabs(rg_loop_area(&s.loops[0])) > fabs(rg_loop_area(&s.loops[1])) ? 0 : 1;
    o.pitch = 10;
    n = rg_pattern_fill(&s, outer, &o, &st);
    CHECK(n == 6);
    for (int i = 0; i < n; i++)
        for (int k = 0; k < st[i].n; k++) {
            RgPt p = st[i].pts[k];
            CHECK(!(p.x > 40 + 1e-9 && p.x < 60 - 1e-9 && p.y > 10 && p.y < 30));
        }
    rg_strokes_free(st, n);

    /* Filling the hole itself is allowed too. */
    n = rg_pattern_fill(&s, 1 - outer, &o, &st);
    CHECK(n == 1 && st[0].n == 4);
    rg_strokes_free(st, n);
    rg_shape_free(&s);
    rg_drawing_free(&d);
}

TEST_MAIN("test_pattern",
    test_simplify();
    test_snap();
    test_trace();
    test_lenient();
    test_fill();
    test_fill_runout();
    test_rings();
    test_ring_seams();
    test_spiral();
)
