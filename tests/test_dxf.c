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
/* test_dxf.c — reading outlines from DXF text. */
#include "rg_test.h"
#include "rg_dxf.h"

static char err[512];

static bool read_dxf(const char *text, const char *layer, RgDrawing *d)
{
    RgDxfOptions opt = { 0.2, layer, false };
    err[0] = '\0';
    return rg_dxf_read(text, strlen(text), &opt, d, err, sizeof err);
}

#define ENTITIES(body) "0\nSECTION\n2\nENTITIES\n" body "0\nENDSEC\n0\nEOF\n"

static void test_lwpolyline(void)
{
    RgDrawing d;
    CHECK(read_dxf(ENTITIES(
        "0\nLWPOLYLINE\n8\nSPRAY\n90\n4\n70\n1\n"
        "10\n0\n20\n100\n10\n1885\n20\n100\n10\n1885\n20\n900\n10\n0\n20\n900\n"), NULL, &d));
    CHECK(d.n == 1);
    CHECK(d.paths[0].closed);
    CHECK(d.paths[0].n == 4);
    CHECK_STR(d.paths[0].layer, "SPRAY");
    CHECK_NEAR(d.paths[0].pts[2].x, 1885.0, 1e-12);
    CHECK_NEAR(d.paths[0].pts[2].y, 900.0, 1e-12);
    CHECK(d.insunits == -1);
    rg_drawing_free(&d);
}

/* R12 POLYLINE, two half-circle bulges: a circle of radius 5. */
static void test_polyline_bulge(void)
{
    RgDrawing d;
    CHECK(read_dxf(ENTITIES(
        "0\nPOLYLINE\n8\n0\n66\n1\n70\n1\n"
        "0\nVERTEX\n8\n0\n10\n0\n20\n0\n42\n1\n"
        "0\nVERTEX\n8\n0\n10\n10\n20\n0\n42\n1\n"
        "0\nSEQEND\n8\n0\n"), NULL, &d));
    CHECK(d.n == 1);
    double area = 0.0, ymin = 1e9, rmax = 0.0, rmin = 1e9;
    const RgPath *p = &d.paths[0];
    for (int i = 0; i < p->n; i++) {
        RgPt a = p->pts[i], b = p->pts[(i + 1) % p->n];
        area += a.x * b.y - b.x * a.y;
        ymin = fmin(ymin, a.y);
        double r = hypot(a.x - 5.0, a.y);
        rmax = fmax(rmax, r);
        rmin = fmin(rmin, r);
    }
    /* Counter-clockwise from (0,0) to (10,0) passes below the chord. */
    CHECK_NEAR(ymin, -5.0, 1e-9);
    CHECK_NEAR(rmin, 5.0, 1e-9);
    CHECK_NEAR(rmax, 5.0, 1e-9);
    CHECK(area > 0.0);
    CHECK_NEAR(0.5 * area, RG_TEST_PI * 25.0, 0.2 * 2 * RG_TEST_PI * 5.0);
    rg_drawing_free(&d);
}

static void test_circle_chords(void)
{
    RgDrawing d;
    CHECK(read_dxf(ENTITIES("0\nCIRCLE\n8\n0\n10\n0\n20\n0\n40\n100\n"), NULL, &d));
    CHECK(d.n == 1 && d.paths[0].closed);
    /* 2·acos(1 − 0.2/100) per chord → 50 chords round. */
    CHECK(d.paths[0].n == 50);
    rg_drawing_free(&d);
}

static void test_mirrored_arc(void)
{
    RgDrawing d;
    CHECK(read_dxf(ENTITIES(
        "0\nARC\n8\n0\n10\n10\n20\n0\n40\n5\n50\n0\n51\n90\n230\n-1\n"), NULL, &d));
    CHECK(d.n == 1 && !d.paths[0].closed);
    const RgPath *p = &d.paths[0];
    CHECK_NEAR(p->pts[0].x, -10.0, 1e-9);
    CHECK_NEAR(p->pts[0].y, 5.0, 1e-9);
    CHECK_NEAR(p->pts[p->n - 1].x, -15.0, 1e-9);
    CHECK_NEAR(p->pts[p->n - 1].y, 0.0, 1e-9);
    rg_drawing_free(&d);
}

static void test_units(void)
{
    RgDrawing d;
    CHECK(read_dxf(
        "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n1\n0\nENDSEC\n"
        "0\nSECTION\n2\nENTITIES\n0\nLINE\n8\n0\n10\n0\n20\n0\n11\n2\n21\n1\n0\nENDSEC\n0\nEOF\n",
        NULL, &d));
    CHECK(d.insunits == 1);
    CHECK_NEAR(d.paths[0].pts[1].x, 50.8, 1e-9);
    CHECK_NEAR(d.paths[0].pts[1].y, 25.4, 1e-9);
    rg_drawing_free(&d);

    CHECK(!read_dxf("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n14\n0\nENDSEC\n"
                    "0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n0\n20\n0\n11\n1\n21\n1\n0\nENDSEC\n",
                    NULL, &d));
    CHECK(strstr(err, "$INSUNITS = 14") != NULL);
}

static void test_refusals(void)
{
    RgDrawing d;
    CHECK(!read_dxf(ENTITIES("0\nSPLINE\n8\nOUTLINE\n70\n8\n"), NULL, &d));
    CHECK(strstr(err, "SPLINE") && strstr(err, "OUTLINE"));
    CHECK(!read_dxf(ENTITIES("0\nHATCH\n8\n0\n"), NULL, &d));
    CHECK(strstr(err, "delete the hatch") != NULL);
    CHECK(!read_dxf(ENTITIES("0\nINSERT\n8\n0\n2\nBLOCK1\n"), NULL, &d));
    CHECK(strstr(err, "explode") != NULL);
    CHECK(!read_dxf("AutoCAD Binary DXF\r\n\032\0", NULL, &d));
    CHECK(strstr(err, "binary") != NULL);
    CHECK(!read_dxf(ENTITIES("0\nTEXT\n8\n0\n1\nhello\n"), NULL, &d));
    CHECK(strstr(err, "no lines") != NULL);
    CHECK(!read_dxf(ENTITIES("0\nLINE\n8\n0\n10\nabc\n"), NULL, &d));
    CHECK(strstr(err, "expected a number") != NULL);
    CHECK(!read_dxf(ENTITIES("0\nARC\n8\n0\n10\n0\n20\n0\n40\n5\n220\n1\n230\n0\n"), NULL, &d));
    CHECK(strstr(err, "XY plane") != NULL);
}

static void test_layers(void)
{
    const char *text = ENTITIES(
        "0\nLINE\n8\nspray\n10\n0\n20\n0\n11\n10\n21\n0\n"
        "0\nSPLINE\n8\nNOTES\n"
        "0\nTEXT\n8\nspray\n1\nlabel\n");
    RgDrawing d;
    CHECK(read_dxf(text, "SPRAY", &d));        /* layer names ignore case */
    CHECK(d.n == 1);
    CHECK(d.skipped == 1);
    rg_drawing_free(&d);
    CHECK(!read_dxf(text, NULL, &d));
    CHECK(!read_dxf(text, "MISSING", &d));
    CHECK(strstr(err, "on layer MISSING") != NULL);
}

TEST_MAIN("test_dxf",
    test_lwpolyline();
    test_polyline_bulge();
    test_circle_chords();
    test_mirrored_arc();
    test_units();
    test_refusals();
    test_layers();
)
