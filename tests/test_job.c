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
/* test_job.c — the job file: parsing, rules, frames. */
#include "rg_test.h"
#include "rg_job.h"

static char err[512];

/* The template with some lines appended; later lines override earlier. */
static bool job_with(RgJob *j, const char *extra)
{
    char text[8192];
    snprintf(text, sizeof text, "%s\n%s\n", rg_job_template(), extra);
    rg_job_default(j);
    err[0] = '\0';
    return rg_job_parse(j, text, err, sizeof err) && rg_job_validate(j, err, sizeof err);
}

static void test_template(void)
{
    RgJob j;
    CHECK(job_with(&j, ""));
    CHECK_STR(j.name, "TANK01");
    CHECK_STR(j.controller, "s4c_plus");
    CHECK(j.nbands == 1);
    CHECK_NEAR(j.bands[0].y0, 100.0, 0);
    CHECK_NEAR(j.radius, 300.0, 0);
    CHECK_NEAR(j.axis.z, 300.0, 0);
    CHECK_NEAR(j.tool_rot.q1, 0.866025, 1e-6);
    CHECK(j.start_top);
    CHECK(j.ready_prompt);
    CHECK(!j.tool_define);
    CHECK_NEAR(j.home[4], 30.0, 0);
}

static void test_errors(void)
{
    RgJob j;
    CHECK(!job_with(&j, "fan_widht = 80"));
    CHECK(strstr(err, "unknown setting \"fan_widht\"") != NULL);

    CHECK(!job_with(&j, "rpm = fast"));
    CHECK(strstr(err, "rpm: expected a number") != NULL);

    CHECK(!job_with(&j, "overlap = 95"));
    CHECK(strstr(err, "95 is outside 0 to 90") != NULL);

    CHECK(!job_with(&j, "coats = 1.5"));
    CHECK(strstr(err, "whole number") != NULL);

    CHECK(!job_with(&j, "name = MODULE"));
    CHECK(strstr(err, "not a RAPID name") != NULL);

    CHECK(!job_with(&j, "name = 2TANK"));
    CHECK(!job_with(&j, "tool = tGunWithAVeryLongName"));

    CHECK(!job_with(&j, "controller = s4c\nname = TANKLONG1"));
    CHECK(strstr(err, "8 characters") != NULL);

    CHECK(!job_with(&j, "controller = irc5"));
    CHECK(strstr(err, "not one of") != NULL);

    CHECK(!job_with(&j, "drawing = x.dxf"));
    CHECK(strstr(err, "not both") != NULL);

    CHECK(!job_with(&j, "band = 500 400"));
    CHECK(strstr(err, "bottom then top") != NULL);

    CHECK(!job_with(&j, "tool_define = yes"));
    CHECK(strstr(err, "tool_mass is required") != NULL);

    CHECK(!job_with(&j, "tool_rot = 1 1 0 0"));
    CHECK(strstr(err, "not 1") != NULL);

    CHECK(!job_with(&j, "no equals sign here"));
    CHECK(strstr(err, "expected key = value") != NULL);

    /* The error names the line it is on. */
    rg_job_default(&j);
    CHECK(!rg_job_parse(&j, "# comment\n\nradius = 300\nbogus = 1\n", err, sizeof err));
    CHECK(strstr(err, "line 4:") != NULL);
}

static void test_required(void)
{
    RgJob j;
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, "name = T1\ncontroller = s4c\nband = 0 100\n", err, sizeof err));
    CHECK(!rg_job_validate(&j, err, sizeof err));
    CHECK(strstr(err, "radius is required") != NULL);

    rg_job_default(&j);
    CHECK(rg_job_parse(&j, "name = T1\n", err, sizeof err));
    CHECK(!rg_job_validate(&j, err, sizeof err));
    CHECK(strstr(err, "controller is required") != NULL);
}

static void test_frames(void)
{
    RgJob j;
    CHECK(job_with(&j, ""));
    /* Work object X points from the rotator axis back towards the robot. */
    RgPose w = rg_job_wobj(&j);
    CHECK_NEAR(w.rot.m[0][0], -1.0, 1e-12);
    CHECK_NEAR(w.rot.m[1][0], 0.0, 1e-12);
    CHECK_NEAR(w.rot.m[2][2], 1.0, 1e-12);
    CHECK_NEAR(w.pos.x, 1450.0, 0);

    CHECK(job_with(&j, "axis = 0 1000 0"));
    w = rg_job_wobj(&j);
    CHECK_NEAR(w.rot.m[1][0], -1.0, 1e-12);

    RgPose t = rg_job_tool(&j);
    CHECK_NEAR(t.pos.y, -120.0, 0);
    /* 60 degrees about X: tool Z leans towards flange -Y. */
    CHECK_NEAR(t.rot.m[1][2], -sin(60.0 * RG_DEG), 1e-6);
}

static void test_identifiers(void)
{
    CHECK(rg_rapid_ident_ok("tGun_1", 16));
    CHECK(rg_rapid_ident_ok("abcdefghijklmnop", 16));
    CHECK(!rg_rapid_ident_ok("abcdefghijklmnopq", 16));
    CHECK(!rg_rapid_ident_ok("_gun", 16));
    CHECK(!rg_rapid_ident_ok("gun-1", 16));
    CHECK(!rg_rapid_ident_ok("endproc", 16));
    CHECK(!rg_rapid_ident_ok("", 16));
}

/* Writing a job and reading it back gives the same job, byte for byte. */
static void round_trip(const char *text)
{
    RgJob a, b;
    rg_job_default(&a);
    rg_job_default(&b);
    CHECK(rg_job_parse(&a, text, err, sizeof err));
    RgBuf wa, wb;
    rg_buf_init(&wa);
    rg_buf_init(&wb);
    rg_job_write(&a, &wa);
    CHECK(rg_job_parse(&b, wa.s, err, sizeof err));
    rg_job_write(&b, &wb);
    CHECK(wa.len == wb.len && memcmp(wa.s, wb.s, wa.len) == 0);
    CHECK(rg_job_validate(&b, err, sizeof err));
    CHECK(a.nstrokes == b.nstrokes);
    CHECK(a.nbands == b.nbands);
    rg_buf_free(&wa);
    rg_buf_free(&wb);
    rg_job_free(&a);
    rg_job_free(&b);
}

static void test_flat(void)
{
    RgJob j;
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, rg_job_template_flat(), err, sizeof err));
    CHECK(rg_job_validate(&j, err, sizeof err));
    CHECK(j.part == RG_PART_FLAT);
    CHECK(j.nstrokes == 2);
    CHECK(j.strokes[0].n == 4);
    CHECK_NEAR(j.strokes[0].pts[2].x, 550, 0);
    CHECK_NEAR(j.strokes[0].pts[2].y, 130, 0);
    CHECK_NEAR(j.spray_speed, 300, 0);

    RgPose w = rg_job_wobj(&j);
    CHECK_NEAR(w.pos.x, 800, 0);
    CHECK_NEAR(w.rot.m[0][0], 1, 1e-12);

    RgJob copy;
    CHECK(rg_job_copy(&copy, &j));
    rg_job_delete_stroke(&copy, 0);
    CHECK(copy.nstrokes == 1 && copy.strokes[0].n == 2);
    CHECK(j.nstrokes == 2 && j.strokes[0].n == 4);     /* the original is untouched */
    rg_job_delete_stroke(&copy, 0);
    CHECK(!rg_job_validate(&copy, err, sizeof err));
    CHECK(strstr(err, "no strokes") != NULL);
    rg_job_free(&copy);
    rg_job_free(&j);

    rg_job_default(&j);
    CHECK(!rg_job_parse(&j, "stroke = 1 2 3\n", err, sizeof err));
    CHECK(strstr(err, "x y pairs") != NULL);
    CHECK(!rg_job_parse(&j, "stroke = 1 2\n", err, sizeof err));
    CHECK(!rg_job_parse(&j, "part = sphere\n", err, sizeof err));
    rg_job_free(&j);

    /* Strokes belong to flat parts, bands to cylinders. */
    char text[8192];
    snprintf(text, sizeof text, "%s\nstroke = 0 0 10 10\n", rg_job_template());
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, text, err, sizeof err));
    CHECK(!rg_job_validate(&j, err, sizeof err));
    CHECK(strstr(err, "strokes are for flat parts") != NULL);
    rg_job_free(&j);
    snprintf(text, sizeof text, "%s\nband = 0 10\n", rg_job_template_flat());
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, text, err, sizeof err));
    CHECK(!rg_job_validate(&j, err, sizeof err));
    CHECK(strstr(err, "bands are for cylinders") != NULL);
    rg_job_free(&j);

    /* A long freehand stroke is one long line. */
    RgBuf big;
    rg_buf_init(&big);
    rg_buf_puts(&big, rg_job_template_flat());
    rg_buf_puts(&big, "stroke =");
    for (int i = 0; i < 3000; i++)
        rg_buf_printf(&big, " %d.125 %d.5", i, i % 97);
    rg_buf_puts(&big, "\n");
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, big.s, err, sizeof err));
    CHECK(j.nstrokes == 3 && j.strokes[2].n == 3000);
    CHECK_NEAR(j.strokes[2].pts[2999].x, 2999.125, 1e-9);
    rg_buf_free(&big);
    rg_job_free(&j);

    round_trip(rg_job_template());
    round_trip(rg_job_template_flat());
}

/*
 * Tabs: the lead-in and run-out written as  lead-in | work | run-out.
 * The robot does not switch the torch, so a stroke has to start and finish
 * clear of the part, and the job file has to say where that is.
 */
static void test_tabs(void)
{
    RgJob j;
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, "stroke = -20 50 | 50 50  550 50 | 620 50\n", err, sizeof err));
    CHECK(j.nstrokes == 1);
    if (j.nstrokes == 1) {
        const RgStroke *st = &j.strokes[0];
        CHECK(st->n == 4);
        CHECK(st->tab_in == 1 && st->tab_out == 1);
        /* the sections are concatenated in order, tabs included */
        CHECK_NEAR(st->pts[0].x, -20, 1e-9);
        CHECK_NEAR(st->pts[1].x, 50, 1e-9);
        CHECK_NEAR(st->pts[3].x, 620, 1e-9);
    }
    rg_job_free(&j);

    /* No bars at all: the one section is the WORK, not a giant lead-in. */
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, "stroke = 50 50  550 50\n", err, sizeof err));
    CHECK(j.nstrokes == 1 && j.strokes[0].tab_in == 0 && j.strokes[0].tab_out == 0);
    rg_job_free(&j);

    /* Either end may be empty, so long as both bars are there. */
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, "stroke = | 50 50  550 50 | 620 50\n", err, sizeof err));
    CHECK(j.nstrokes == 1 && j.strokes[0].tab_in == 0 && j.strokes[0].tab_out == 1);
    rg_job_free(&j);

    /* One bar is ambiguous, three is nonsense. */
    rg_job_default(&j);
    CHECK(!rg_job_parse(&j, "stroke = -20 50 | 50 50  550 50\n", err, sizeof err));
    rg_job_free(&j);
    rg_job_default(&j);
    CHECK(!rg_job_parse(&j, "stroke = -20 50 | 50 50 | 550 50 | 620 50\n", err, sizeof err));
    rg_job_free(&j);

    /* Tabs that leave no work are not believed. */
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, "stroke = -20 50  0 50 | | 620 50  700 50\n", err, sizeof err));
    CHECK(j.nstrokes == 1 && j.strokes[0].tab_in == 0 && j.strokes[0].tab_out == 0);
    rg_job_free(&j);

    /* And the bars survive being written back out. round_trip validates what
     * it reads back, so this starts from a job that is actually complete. */
    char text[8192];
    snprintf(text, sizeof text, "%s\nstroke = -20 50 | 50 50  550 50 | 620 50\n",
             rg_job_template_flat());
    round_trip(text);

    rg_job_default(&j);
    RgBuf b;
    rg_buf_init(&b);
    CHECK(rg_job_parse(&j, "name = P1\npart = flat\n"
                           "stroke = -20 50 | 50 50  550 50 | 620 50\n", err, sizeof err));
    rg_job_write(&j, &b);
    CHECK(strstr(b.s, "|") != NULL);
    RgJob back;
    rg_job_default(&back);
    CHECK(rg_job_parse(&back, b.s, err, sizeof err));
    CHECK(back.nstrokes == 1 && back.strokes[0].tab_in == 1 && back.strokes[0].tab_out == 1);
    rg_buf_free(&b);
    rg_job_free(&back);
    rg_job_free(&j);
}

TEST_MAIN("test_job",
    test_tabs();
    test_template();
    test_errors();
    test_required();
    test_frames();
    test_identifiers();
    test_flat();
)
