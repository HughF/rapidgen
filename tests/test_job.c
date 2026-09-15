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

TEST_MAIN("test_job",
    test_template();
    test_errors();
    test_required();
    test_frames();
    test_identifiers();
)
