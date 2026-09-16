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

static void test_fill(void)
{
    char a[512], b[512], both[1024];
    RgDrawing d;
    RgShape s;
    int skipped;

    CHECK(drawing_of(rect(a, sizeof a, 0, 0, 100, 40), &d));
    CHECK(rg_shape_build_lenient(&d, 0.1, &s, &skipped));
    RgFillOpts o = { 10, 0, 5 };
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
    test_spiral();
)
