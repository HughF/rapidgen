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
/* test_rapid.c — the RAPID program text. */
#include "rg_test.h"
#include "rg_plan.h"
#include "rg_rapid.h"

#include <ctype.h>

static char err[512];

static bool program_for(const char *extra, RgBuf *out, RgJob *jout)
{
    char text[8192];
    snprintf(text, sizeof text, "%s\n%s\n", rg_job_template(), extra);
    RgJob j;
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, text, err, sizeof err) && rg_job_validate(&j, err, sizeof err));
    RgPlan pl;
    bool ok = rg_plan_build(&j, NULL, &pl);
    if (!ok) {
        printf("  plan refused: %s\n", pl.nissues ? pl.issues[0].text : "(no reason)");
        CHECK(ok);
    }
    rg_buf_init(out);
    ok = ok && rg_rapid_write(&j, &pl, "test.rgj", "2026-09-15 12:00", out, err, sizeof err);
    if (!ok)
        printf("  no program written: %s\n", err);
    rg_plan_free(&pl);
    if (jout)
        *jout = j;
    return ok;
}

static int count(const char *hay, const char *needle)
{
    int n = 0;
    if (!hay)
        return 0;
    for (const char *p = strstr(hay, needle); p; p = strstr(p + 1, needle))
        n++;
    return n;
}

static void test_structure(void)
{
    RgBuf b;
    CHECK(program_for("", &b, NULL));
    const char *s = b.s;
    CHECK(strncmp(s, "%%%\n  VERSION:1\n  LANGUAGE:ENGLISH\n%%%\n\nMODULE TANK01\n", 52) == 0);
    CHECK(count(s, "MODULE TANK01") == 1);
    CHECK(count(s, "ENDMODULE") == 1);
    CHECK(count(s, "PROC main()") == 1);
    CHECK(count(s, "ENDPROC") == 1);
    CHECK(count(s, "MoveAbsJ jRgHome,vRgTravel,fine,tSprayGun;") == 2);
    CHECK(count(s, "MoveJ pRg") == 1);
    CHECK(count(s, "MoveL pRg") == 4);
    CHECK(count(s, "TPReadFK nKey,") == 1);
    CHECK(count(s, "VAR num nKey;") == 1);
    CHECK(count(s, "SetDO") == 0);
    CHECK(count(s, "PERS tooldata") == 0);
    CHECK(count(s, "CONST speeddata vRgSpray:=[3,500,5000,1000];") == 1);
    CHECK(count(s, "FOR nCycle FROM 1 TO 8 DO") == 1);
    CHECK(count(s, "ENDFOR") == 1);
    CHECK(count(s, "WaitTime 20;") == 1);
    CHECK(count(s, "VAR num nCycle;") == 1);
    CHECK(count(s, "ConfL \\Off;") == 1);
    CHECK(strstr(s, "\\WObj:=wRgCylinder;") != NULL);

    /* Plain ASCII, and every statement line inside the procedure ends in ';'. */
    bool ascii = true, terminated = true, in_proc = false;
    char line[512];
    for (const char *p = s; *p;) {
        const char *nl = strchr(p, '\n');
        size_t n = nl ? (size_t)(nl - p) : strlen(p);
        if (n >= sizeof line)
            n = sizeof line - 1;
        memcpy(line, p, n);
        line[n] = '\0';
        p = nl ? nl + 1 : p + n;
        for (size_t i = 0; i < n; i++)
            ascii = ascii && (unsigned char)line[i] < 128;
        const char *t = line;
        while (*t == ' ')
            t++;
        if (strncmp(t, "PROC ", 5) == 0) { in_proc = true; continue; }
        if (strncmp(t, "ENDPROC", 7) == 0) { in_proc = false; continue; }
        /* FOR ... DO and ENDFOR are the loop, not statements */
        if (strncmp(t, "FOR ", 4) == 0 || strncmp(t, "ENDFOR", 6) == 0)
            continue;
        if (in_proc && *t && *t != '!')
            terminated = terminated && line[n - 1] == ';';
    }
    CHECK(ascii);
    CHECK(terminated);
    rg_buf_free(&b);
}

/* Every robtarget reads back as the pose planned, with a unit quaternion. */
static void test_targets(void)
{
    RgBuf b;
    RgJob j;
    CHECK(program_for("", &b, &j));

    RgPlan pl;
    CHECK(rg_plan_build(&j, NULL, &pl));

    int found = 0;
    double worst_norm = 0.0;
    for (const char *p = strstr(b.s, "CONST robtarget pRg"); p;
         p = strstr(p + 1, "CONST robtarget pRg")) {
        int idx, cf[4];
        double x, y, z, q[4];
        int got = sscanf(p, "CONST robtarget pRg%d:=[[%lf,%lf,%lf],[%lf,%lf,%lf,%lf],[%d,%d,%d,%d],"
                         "[9E9,9E9,9E9,9E9,9E9,9E9]];", &idx, &x, &y, &z,
                         &q[0], &q[1], &q[2], &q[3], &cf[0], &cf[1], &cf[2], &cf[3]);
        CHECK(got == 12);
        worst_norm = fmax(worst_norm, fabs(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1.0));
        bool matched = false;
        for (int i = 0; i < pl.nmoves; i++) {
            const RgPose *t = &pl.moves[i].tcp;
            if (fabs(t->pos.x - x) < 0.006 && fabs(t->pos.y - y) < 0.006 && fabs(t->pos.z - z) < 0.006 &&
                cf[3] == pl.moves[i].cf[3])
                matched = true;
        }
        CHECK(matched);
        found++;
    }
    /* approach, band start, band end: the second coat ends on the start, and
     * the retract is the approach point, so both reuse their names */
    CHECK(found == 3);
    CHECK(worst_norm < 3e-6);
    rg_plan_free(&pl);
    rg_buf_free(&b);
}

static void test_dialects(void)
{
    RgBuf b;
    RgJob j;
    CHECK(program_for("controller = s4\nname = tank1", &b, &j));
    CHECK(strstr(b.s, "MoveAbsJ") == NULL);
    CHECK(count(b.s, "CONST robtarget pRgHome:=") == 1);
    CHECK(count(b.s, "ConfJ \\On;\n    MoveJ pRgHome,vRgTravel,fine,tSprayGun\\WObj:=wRgCylinder;\n"
                     "    ConfJ \\Off;") == 2);
    CHECK(strstr(b.s, "for an ABB S4\n") != NULL);
    char name[64];
    rg_rapid_filename(&j, name, sizeof name);
    CHECK_STR(name, "TANK1.PRG");
    rg_buf_free(&b);

    CHECK(program_for("controller = s4c_plus\nname = Tank_Band_Top", &b, &j));
    rg_rapid_filename(&j, name, sizeof name);
    CHECK_STR(name, "Tank_Band_Top.PRG");
    rg_buf_free(&b);
}

static void test_options(void)
{
    RgBuf b;
    CHECK(program_for("gun = switched\ngun_signal = doGunOn\nready_prompt = no\n"
                      "tool_define = yes\ntool_mass = 2.5\ntool_cog = 0 -40 80", &b, NULL));
    CHECK(count(b.s, "SetDO doGunOn,1;") == 1);
    CHECK(count(b.s, "SetDO doGunOn,0;") == 1);
    CHECK(strstr(b.s, "TPReadFK") == NULL);
    CHECK(strstr(b.s, "nKey") == NULL);
    CHECK(count(b.s, "PERS tooldata tSprayGun:=[TRUE,[[0,-120,180],[0.866025,0.5,0,0]],"
                     "[2.5,[0,-40,80],[1,0,0,0],0,0,0]];") == 1);
    /* on after the prompt-less approach, off after the last coat */
    const char *on = strstr(b.s, "SetDO doGunOn,1;"), *off = strstr(b.s, "SetDO doGunOn,0;");
    CHECK(on && off && on < off);
    CHECK(count(off, "MoveL pRg") == 1);
    rg_buf_free(&b);
}

static void test_refused_and_stable(void)
{
    char text[8192];
    snprintf(text, sizeof text, "%s\naxis = 2600 0 300\n", rg_job_template());
    RgJob j;
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, text, err, sizeof err));
    RgPlan pl;
    CHECK(!rg_plan_build(&j, NULL, &pl));
    RgBuf b;
    rg_buf_init(&b);
    CHECK(!rg_rapid_write(&j, &pl, "x", "y", &b, err, sizeof err));
    CHECK(strstr(err, "refused") != NULL);
    rg_buf_free(&b);
    rg_plan_free(&pl);

    RgBuf b1, b2;
    CHECK(program_for("", &b1, NULL));
    CHECK(program_for("", &b2, NULL));
    CHECK(b1.len == b2.len && memcmp(b1.s, b2.s, b1.len) == 0);
    rg_buf_free(&b1);
    rg_buf_free(&b2);
}

static void test_numbers(void)
{
    char buf[32];
    CHECK_STR(rg_fmt(buf, sizeof buf, -0.0001, 2), "0");
    CHECK_STR(rg_fmt(buf, sizeof buf, 1.50, 2), "1.5");
    CHECK_STR(rg_fmt(buf, sizeof buf, 2.0, 6), "2");
    CHECK_STR(rg_fmt(buf, sizeof buf, -12.345678, 2), "-12.35");
    CHECK_STR(rg_fmt(buf, sizeof buf, 100.0, 0), "100");
}

static void test_flat_program(void)
{
    RgJob j;
    rg_job_default(&j);
    CHECK(rg_job_parse(&j, rg_job_template_flat(), err, sizeof err));
    RgPlan pl;
    CHECK(rg_plan_build(&j, NULL, &pl));
    RgBuf b;
    rg_buf_init(&b);
    CHECK(rg_rapid_write(&j, &pl, "panel.rgj", "2026-09-15 12:00", &b, err, sizeof err));
    const char *s = b.s ? b.s : "";
    CHECK(strstr(s, "! Flat part: 2 strokes, 1580 mm at 300 mm/s\n") != NULL);
    CHECK(count(s, "PERS wobjdata wRgPart:=[FALSE,TRUE,\"\",[[800,-300,200],[1,0,0,0]]") == 1);
    CHECK(strstr(s, "wRgCylinder") == NULL);
    CHECK(count(s, "! Stroke 1: 4 points, 1080 mm") == 1);
    CHECK(count(s, "! Stroke 2: 2 points, 500 mm") == 1);
    CHECK(count(s, ",vRgSpray,z1,tSprayGun\\WObj:=wRgPart;") == 6);
    CHECK(count(s, ",vRgSpray,z10,tSprayGun\\WObj:=wRgPart;") == 2);
    /* the torch is continuous: nothing switches it, and nothing stops on the
     * work — every spray move blends */
    CHECK(count(s, ",vRgSpray,fine,tSprayGun\\WObj:=wRgPart;") == 0);
    CHECK(count(s, "CONST speeddata vRgSpray:=[300,500,5000,1000];") == 1);
    CHECK(count(s, "SetDO") == 0);
    CHECK(count(s, "FOR nCycle FROM 1 TO 6 DO") == 1);
    CHECK(count(s, "ENDFOR") == 1);
    CHECK(count(s, "WaitTime 15;") == 1);
    const char *prompt = strstr(s, "TPReadFK"), *loop = strstr(s, "FOR nCycle");
    CHECK(prompt && loop && loop < prompt);
    rg_buf_free(&b);
    rg_plan_free(&pl);
    rg_job_free(&j);
}

TEST_MAIN("test_rapid",
    test_flat_program();
    test_structure();
    test_targets();
    test_dialects();
    test_options();
    test_refused_and_stable();
    test_numbers();
)
