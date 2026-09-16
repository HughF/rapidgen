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
/* test_shape.c — joining outlines and measuring their coverage. */
#include "rg_test.h"
#include "rg_shape.h"

static char err[512];

static bool shape_of(const char *entities, RgShape *s)
{
    char text[8192];
    snprintf(text, sizeof text, "0\nSECTION\n2\nENTITIES\n%s0\nENDSEC\n0\nEOF\n", entities);
    RgDxfOptions opt = { 0.2, NULL, false };
    RgDrawing d;
    err[0] = '\0';
    if (!rg_dxf_read(text, strlen(text), &opt, &d, err, sizeof err))
        return false;
    bool ok = rg_shape_build(&d, 0.1, s, err, sizeof err);
    rg_drawing_free(&d);
    return ok;
}

static const char *rect(char *buf, size_t cap, double x0, double y0, double x1, double y1)
{
    snprintf(buf, cap, "0\nLWPOLYLINE\n8\n0\n70\n1\n10\n%g\n20\n%g\n10\n%g\n20\n%g\n"
             "10\n%g\n20\n%g\n10\n%g\n20\n%g\n", x0, y0, x1, y0, x1, y1, x0, y1);
    return buf;
}

static void test_rectangle(void)
{
    char r[512];
    RgShape s;
    CHECK(shape_of(rect(r, sizeof r, 0, 100, 1885, 900), &s));
    CHECK(s.n == 1);
    CHECK_NEAR(rg_shape_span(&s, 500), 1885.0, 1e-9);
    CHECK_NEAR(rg_shape_span(&s, 50), 0.0, 1e-12);
    RgSpanRun *runs;
    int n = rg_shape_runs(&s, &runs);
    CHECK(n == 1);
    CHECK_NEAR(runs[0].y0, 100.0, 1e-12);
    CHECK_NEAR(runs[0].y1, 900.0, 1e-12);
    CHECK_NEAR(runs[0].span0, 1885.0, 1e-6);
    free(runs);
    rg_shape_free(&s);
}

/* Four separate LINEs, one of them drawn backwards, joined into one loop. */
static void test_joined_lines(void)
{
    RgShape s;
    CHECK(shape_of(
        "0\nLINE\n8\n0\n10\n0\n20\n0\n11\n100\n21\n0\n"
        "0\nLINE\n8\n0\n10\n100\n20\n50\n11\n100\n21\n0.05\n"
        "0\nLINE\n8\n0\n10\n100\n20\n50\n11\n0\n21\n50\n"
        "0\nLINE\n8\n0\n10\n0\n20\n50\n11\n0\n21\n0\n", &s));
    CHECK(s.n == 1);
    CHECK(s.n == 1 && s.loops[0].n == 4);
    CHECK_NEAR(rg_shape_span(&s, 25), 100.0, 1e-9);
    rg_shape_free(&s);
}

static void test_gap_reported(void)
{
    RgShape s;
    CHECK(!shape_of(
        "0\nLINE\n8\nA\n10\n0\n20\n0\n11\n100\n21\n0\n"
        "0\nLINE\n8\nA\n10\n100\n20\n0\n11\n100\n21\n50\n"
        "0\nLINE\n8\nA\n10\n100\n20\n50\n11\n0\n21\n50\n"
        "0\nLINE\n8\nA\n10\n0\n20\n50\n11\n0\n21\n3.2\n", &s));
    CHECK(strstr(err, "not closed") != NULL);
    CHECK(strstr(err, "3.20 mm") != NULL);

    CHECK(!shape_of("0\nLINE\n8\nA\n10\n0\n20\n0\n11\n100\n21\n0\n", &s));
    CHECK(strstr(err, "not part of a closed outline") != NULL);
}

static void test_self_crossing(void)
{
    RgShape s;
    CHECK(!shape_of("0\nLWPOLYLINE\n8\n0\n70\n1\n10\n0\n20\n0\n10\n100\n20\n100\n"
                    "10\n100\n20\n0\n10\n0\n20\n100\n", &s));
    CHECK(strstr(err, "crosses itself at (50.00, 50.00)") != NULL);
}

static void test_union_and_holes(void)
{
    char a[512], b[512], both[1024];
    RgShape s;

    snprintf(both, sizeof both, "%s%s", rect(a, sizeof a, 0, 0, 100, 50),
             rect(b, sizeof b, 50, 25, 150, 75));
    CHECK(shape_of(both, &s));
    CHECK_NEAR(rg_shape_span(&s, 30), 150.0, 1e-9);      /* union, not even-odd */
    CHECK_NEAR(rg_shape_span(&s, 60), 100.0, 1e-9);
    RgPt at;
    CHECK(!rg_shape_find_hole(&s, &at));
    rg_shape_free(&s);

    snprintf(both, sizeof both, "%s%s", rect(a, sizeof a, 0, 0, 100, 100),
             rect(b, sizeof b, 20, 20, 80, 80));
    CHECK(shape_of(both, &s));
    CHECK(rg_shape_find_hole(&s, &at));
    CHECK_NEAR(at.y, 20.0, 1e-9);
    rg_shape_free(&s);

    /* Stacked bands sharing an edge cover the full width on both sides of it. */
    snprintf(both, sizeof both, "%s%s", rect(a, sizeof a, 0, 0, 1000, 500),
             rect(b, sizeof b, 0, 500, 1000, 900));
    CHECK(shape_of(both, &s));
    CHECK_NEAR(rg_shape_span(&s, 499.9), 1000.0, 1e-9);
    CHECK_NEAR(rg_shape_span(&s, 500.1), 1000.0, 1e-9);
    CHECK(!rg_shape_find_hole(&s, &at));
    rg_shape_free(&s);
}

/* Coverage changing slope where two outlines cross inside a run must be
 * caught by a break there. */
static void test_crossing_breaks(void)
{
    RgShape s;
    CHECK(shape_of(
        "0\nLWPOLYLINE\n8\n0\n70\n1\n10\n0\n20\n0\n10\n60\n20\n0\n10\n0\n20\n100\n"
        "0\nLWPOLYLINE\n8\n0\n70\n1\n10\n100\n20\n0\n10\n40\n20\n0\n10\n100\n20\n100\n", &s));
    /* (60,0)-(0,100) meets (40,0)-(100,100) a sixth of the way up. */
    bool has_crossing = false;
    for (int i = 0; i < s.nbreaks; i++)
        has_crossing = has_crossing || fabs(s.ybreaks[i] - 100.0 / 6.0) < 1e-9;
    CHECK(has_crossing);
    /* Below the crossing the two overlap and cover everything; above it a
     * gap opens between them. */
    CHECK_NEAR(rg_shape_span(&s, 10.0), 100.0, 1e-9);
    CHECK_NEAR(rg_shape_span(&s, 90.0), 12.0, 1e-9);
    rg_shape_free(&s);
}

static const char *line(char *buf, size_t cap, double x0, double y0, double x1, double y1)
{
    snprintf(buf, cap, "0\nLINE\n8\n0\n10\n%g\n20\n%g\n11\n%g\n21\n%g\n", x0, y0, x1, y1);
    return buf;
}

/*
 * A real CAD drawing often has an entity drawn twice over. At a junction the
 * copy gives the joiner two ends at no distance: it used to follow the copy
 * straight back, close a zero-area sliver, and leave the real outline short
 * of the piece it needed - so nothing closed.
 */
static void test_duplicates(void)
{
    char a[160], b[160], c[160], d[160], a2[160], c2[160], ents[2048];
    snprintf(ents, sizeof ents, "%s%s%s%s%s%s",
             line(a, sizeof a, 0, 0, 100, 0),
             line(a2, sizeof a2, 0, 0, 100, 0),            /* the same line again */
             line(b, sizeof b, 100, 0, 100, 100),
             line(c, sizeof c, 100, 100, 0, 100),
             line(c2, sizeof c2, 0, 100, 100, 100),        /* again, drawn backwards */
             line(d, sizeof d, 0, 100, 0, 0));
    RgShape s;
    CHECK(shape_of(ents, &s));
    CHECK(s.n == 1);
    CHECK(s.duplicates == 2);
    if (s.n == 1)
        CHECK_NEAR(fabs(rg_loop_area(&s.loops[0])), 10000.0, 1e-6);
    rg_shape_free(&s);

    /* Two identical closed outlines are one outline. */
    char r1[512], r2[512], both[1100];
    snprintf(both, sizeof both, "%s%s", rect(r1, sizeof r1, 0, 0, 50, 50),
             rect(r2, sizeof r2, 0, 0, 50, 50));
    CHECK(shape_of(both, &s));
    CHECK(s.n == 1 && s.duplicates == 1);
    rg_shape_free(&s);

    /* A line and its copy on their own enclose nothing: not an outline. */
    snprintf(ents, sizeof ents, "%s%s%s", rect(r1, sizeof r1, 0, 0, 50, 50),
             line(a, sizeof a, 200, 0, 300, 0), line(b, sizeof b, 300, 0, 200, 0.05));
    RgDrawing dr;
    char text[4096];
    snprintf(text, sizeof text, "0\nSECTION\n2\nENTITIES\n%s0\nENDSEC\n0\nEOF\n", ents);
    RgDxfOptions opt = { 0.2, NULL, false };
    CHECK(rg_dxf_read(text, strlen(text), &opt, &dr, err, sizeof err));
    int skipped = 0;
    CHECK(rg_shape_build_lenient(&dr, 0.1, &s, &skipped));
    CHECK(s.n == 1);                     /* the rectangle, and no sliver */
    CHECK(skipped >= 1);
    rg_shape_free(&s);
    rg_drawing_free(&dr);
}

TEST_MAIN("test_shape",
    test_duplicates();
    test_rectangle();
    test_joined_lines();
    test_gap_reported();
    test_self_crossing();
    test_union_and_holes();
    test_crossing_breaks();
)
