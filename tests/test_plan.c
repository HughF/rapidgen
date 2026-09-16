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
/* test_plan.c — bands, moves, and every rule that refuses a plan. */
#include "rg_test.h"
#include "rg_plan.h"
#include "rg_text.h"

static char err[512];

static void job_with(RgJob *j, const char *extra)
{
    char text[8192];
    snprintf(text, sizeof text, "%s\n%s\n", rg_job_template(), extra);
    rg_job_default(j);
    bool ok = rg_job_parse(j, text, err, sizeof err) && rg_job_validate(j, err, sizeof err);
    if (!ok)
        printf("  job did not parse: %s\n", err);
    CHECK(ok);
}

static bool has_issue(const RgPlan *pl, RgSeverity sev, const char *needle)
{
    for (int i = 0; i < pl->nissues; i++)
        if (pl->issues[i].sev == sev && strstr(pl->issues[i].text, needle))
            return true;
    return false;
}

static void show(const RgPlan *pl)
{
    for (int i = 0; i < pl->nissues; i++)
        if (pl->issues[i].sev == RG_REFUSE)
            printf("    refused: %s\n", pl->issues[i].text);
}

/* The planned joints put the gun exactly where the move says it is. */
static void check_moves_consistent(const RgJob *j, const RgPlan *pl)
{
    const RgRobot *r = rg_robot_find(j->robot);
    RgPose wobj = rg_job_wobj(j), tool = rg_job_tool(j);
    double worst = 0.0;
    for (int i = 0; i < pl->nmoves; i++) {
        RgPose t0;
        rg_robot_fk(r, pl->moves[i].joints, &t0, NULL);
        RgPose tcp = rg_pose_mul(t0, tool);
        RgPose want = rg_pose_mul(wobj, pl->moves[i].tcp);
        worst = fmax(worst, rg_v3_len(rg_v3_sub(tcp.pos, want.pos)));
        CHECK(rg_robot_within_limits(r, pl->moves[i].joints));
    }
    CHECK(worst < 1e-6);
}

static void test_template_plan(void)
{
    RgJob j;
    job_with(&j, "");
    RgPlan pl;
    bool ok = rg_plan_build(&j, NULL, &pl);
    if (!ok)
        show(&pl);
    CHECK(ok);
    CHECK_NEAR(pl.circumference, 2.0 * RG_TEST_PI * 300.0, 1e-9);
    CHECK_NEAR(pl.pitch, 6.0, 1e-12);                 /* the step-over        */
    CHECK_NEAR(pl.spray_speed, 3.0, 1e-12);           /* 6 mm × 30 rpm / 60   */
    CHECK_NEAR(pl.surface_speed, 2.0 * RG_TEST_PI * 300.0 * 30.0 / 60.0, 1e-9);
    CHECK_NEAR(pl.passes_per_point, 2.0, 1e-12);      /* 12 mm spot / 6 mm    */
    CHECK_NEAR(pl.runup, 9.0 / 4000.0, 1e-12);        /* 3² / (2 × 2000)      */
    CHECK_NEAR(pl.overrun, 6.0 + 9.0 / 4000.0, 1e-12);
    CHECK_NEAR(pl.cycle_time, 2 * (800.0 + 2 * pl.overrun) / 3.0, 1e-9);
    CHECK(pl.cycles == 8);
    CHECK_NEAR(pl.spray_time, pl.cycle_time * 8, 1e-9);
    /* 25 um a pass, covered twice a cycle, eight cycles */
    CHECK_NEAR(pl.thickness_cycle, 50.0, 1e-9);
    CHECK_NEAR(pl.thickness_total, 400.0, 1e-9);
    CHECK(pl.cycles_for_target == 7);                 /* 350 um wanted        */

    /* home, approach, in, two coats, out, home */
    CHECK(pl.nmoves == 7);
    CHECK(pl.moves[0].kind == RG_MV_HOME);
    CHECK(pl.moves[1].kind == RG_MV_JOINT);
    CHECK(pl.moves[0].after == RG_ACT_READY);   /* asked once, before the cycles */
    CHECK(pl.moves[3].speed == RG_SPD_SPRAY && pl.moves[4].speed == RG_SPD_SPRAY);
    CHECK(pl.moves[6].kind == RG_MV_HOME);

    /* Gun square to the surface, facing the robot, starting above the band. */
    CHECK_NEAR(pl.moves[2].tcp.pos.x, 450.0, 1e-9);         /* radius + standoff  */
    CHECK_NEAR(pl.moves[2].tcp.pos.y, 0.0, 1e-9);
    CHECK_NEAR(pl.moves[2].tcp.pos.z, 900.0 + pl.overrun, 1e-9);
    CHECK_NEAR(pl.moves[3].tcp.pos.z, 100.0 - pl.overrun, 1e-9);
    CHECK_NEAR(pl.moves[4].tcp.pos.z, 900.0 + pl.overrun, 1e-9);
    CHECK_NEAR(pl.moves[1].tcp.pos.x, 600.0, 1e-9);         /* ...plus the approach */
    CHECK_NEAR(pl.moves[2].tcp.rot.m[0][2], -1.0, 1e-12);   /* tool Z at the axis */
    CHECK_NEAR(pl.moves[2].tcp.rot.m[2][0], 1.0, 1e-12);    /* tool X up the part */

    check_moves_consistent(&j, &pl);
    CHECK(pl.samples > 300);
    CHECK(pl.min_wrist >= j.min_wrist);
    CHECK(pl.min_margin >= j.min_margin);
    CHECK(pl.min_clearance >= j.clearance);
    /* A torch cannot be switched, so the part is coated from the moment the
     * gun arrives until it leaves. */
    CHECK(has_issue(&pl, RG_WARN, "the gun runs continuously"));
    CHECK(rg_plan_count(&pl, RG_REFUSE) == 0);
    rg_plan_free(&pl);

    job_with(&j, "start = bottom\ncoats = 3");
    CHECK(rg_plan_build(&j, NULL, &pl));
    CHECK(pl.nmoves == 8);
    CHECK_NEAR(pl.moves[2].tcp.pos.z, 100.0 - pl.overrun, 1e-9);
    CHECK_NEAR(pl.moves[5].tcp.pos.z, 900.0 + pl.overrun, 1e-9);  /* odd coats end at the far end */
    rg_plan_free(&pl);
}

static void test_bands(void)
{
    RgJob j;
    RgPlan pl;

    /* Appended band lines add to the template's 100-900 band, so the bands
     * are set directly. */
    /* A torch that cannot be switched coats the gap between bands as it
     * travels: that is a warning, not a refusal — it is how the process works. */
    job_with(&j, "");
    j.nbands = 2;
    j.bands[0].y0 = 300; j.bands[0].y1 = 500;
    j.bands[1].y0 = 750; j.bands[1].y1 = 900;
    CHECK(rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_WARN, "the part is coated between them as the gun travels"));
    rg_plan_free(&pl);

    /* Switched, and with no output to switch it with, the job is incomplete.
     * Built by hand: job_with insists on a job that validates. */
    {
        char text[8192];
        snprintf(text, sizeof text, "%s\ngun = switched\n", rg_job_template());
        RgJob bad;
        rg_job_default(&bad);
        CHECK(rg_job_parse(&bad, text, err, sizeof err));
        CHECK(!rg_job_validate(&bad, err, sizeof err));
        CHECK(strstr(err, "gun = switched needs gun_signal") != NULL);
        rg_job_free(&bad);
    }

    /* Overlapping bands merge, in whatever order they were given. */
    job_with(&j, "approach = 50");
    j.nbands = 2;
    j.bands[0].y0 = 150; j.bands[0].y1 = 500;
    j.bands[1].y0 = 100; j.bands[1].y1 = 300;
    bool ok = rg_plan_build(&j, NULL, &pl);
    if (!ok)
        show(&pl);
    CHECK(ok);
    CHECK(pl.nbands == 1);
    CHECK_NEAR(pl.bands[0].y0, 100.0, 0);
    CHECK_NEAR(pl.bands[0].y1, 500.0, 0);
    rg_plan_free(&pl);

    /* Stand the part further out than the arm can reach and the approach to
     * the band is the first point that fails. */
    job_with(&j, "axis = 2400 0 300");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "at the approach to band 1"));
    rg_plan_free(&pl);

    job_with(&j, "gun = switched\ngun_signal = doGunOn\napproach = 50");
    j.nbands = 2;
    j.bands[0].y0 = 300; j.bands[0].y1 = 500;
    j.bands[1].y0 = 750; j.bands[1].y1 = 900;
    ok = rg_plan_build(&j, NULL, &pl);
    if (!ok)
        show(&pl);
    CHECK(ok);
    int on = 0, off = 0, ready = 0;
    for (int i = 0; i < pl.nmoves; i++) {
        on += pl.moves[i].after == RG_ACT_GUN_ON;
        off += pl.moves[i].after == RG_ACT_GUN_OFF;
        ready += pl.moves[i].after == RG_ACT_READY;
    }
    /* The prompt is asked once at home; each band switches its own gun on. */
    CHECK(ready == 1 && on == 2 && off == 2);
    CHECK(!has_issue(&pl, RG_WARN, "the gun runs continuously"));
    rg_plan_free(&pl);

    /* Switched, but too close together for the gun to be off in between:
     * the spray reaches 12 mm past each edge, so the gap needs 24 mm. */
    j.bands[1].y0 = 510;
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "bands 1 and 2 are 10.0 mm apart, but spray lands"));
    rg_plan_free(&pl);

    job_with(&j, "part_height = 800");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "is not on the part"));
    rg_plan_free(&pl);
}

static bool plan_drawing(const RgJob *j, const char *entities, RgPlan *pl)
{
    char text[4096];
    snprintf(text, sizeof text, "0\nSECTION\n2\nENTITIES\n%s0\nENDSEC\n0\nEOF\n", entities);
    RgDxfOptions opt = { j->chord_tolerance, NULL, false };
    RgDrawing d;
    RgShape s;
    CHECK(rg_dxf_read(text, strlen(text), &opt, &d, err, sizeof err));
    CHECK(rg_shape_build(&d, j->join_tolerance, &s, err, sizeof err));
    bool ok = rg_plan_build(j, &s, pl);
    rg_shape_free(&s);
    rg_drawing_free(&d);
    return ok;
}

static const char *rect(char *buf, size_t cap, double x0, double y0, double x1, double y1)
{
    snprintf(buf, cap, "0\nLWPOLYLINE\n8\n0\n70\n1\n10\n%.4f\n20\n%.4f\n10\n%.4f\n20\n%.4f\n"
             "10\n%.4f\n20\n%.4f\n10\n%.4f\n20\n%.4f\n", x0, y0, x1, y0, x1, y1, x0, y1);
    return buf;
}

static void test_drawings(void)
{
    RgJob j;
    job_with(&j, "");
    j.nbands = 0;
    rg_copy(j.drawing, sizeof j.drawing, "test.dxf");
    double C = 2.0 * RG_TEST_PI * j.radius;
    char a[512], b[512], both[1024];
    RgPlan pl;

    CHECK(plan_drawing(&j, rect(a, sizeof a, 0, 100, C, 900), &pl));
    CHECK(pl.nbands == 1);
    CHECK_NEAR(pl.bands[0].y0, 100.0, 1e-9);
    CHECK_NEAR(pl.bands[0].y1, 900.0, 1e-9);
    rg_plan_free(&pl);

    /* Within the wrap tolerance either way. */
    CHECK(plan_drawing(&j, rect(a, sizeof a, 50, 100, 50 + C - 4.0, 900), &pl));
    rg_plan_free(&pl);

    CHECK(!plan_drawing(&j, rect(a, sizeof a, 0, 100, 1000, 900), &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "covers only 53-53 % of the way round"));
    rg_plan_free(&pl);

    CHECK(!plan_drawing(&j, rect(a, sizeof a, 0, 100, 2000, 900), &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "is 2000.0 mm round"));
    rg_plan_free(&pl);

    /* A full band with a notch out of its top corner: the notch height is refused. */
    snprintf(both, sizeof both,
             "0\nLWPOLYLINE\n8\n0\n70\n1\n10\n0\n20\n100\n10\n%.4f\n20\n100\n10\n%.4f\n20\n700\n"
             "10\n%.4f\n20\n700\n10\n%.4f\n20\n900\n10\n0\n20\n900\n", C, C, C - 300, C - 300);
    CHECK(!plan_drawing(&j, both, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "between heights 700.0 and 900.0 mm"));
    rg_plan_free(&pl);

    snprintf(both, sizeof both, "%s%s", rect(a, sizeof a, 0, 100, C, 900),
             rect(b, sizeof b, 200, 300, 400, 500));
    CHECK(!plan_drawing(&j, both, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "Holes"));
    rg_plan_free(&pl);

    /* Two stacked outlines meeting along an edge are one band. */
    snprintf(both, sizeof both, "%s%s", rect(a, sizeof a, 0, 100, C, 500),
             rect(b, sizeof b, 0, 500, C, 900));
    CHECK(plan_drawing(&j, both, &pl));
    CHECK(pl.nbands == 1);
    rg_plan_free(&pl);
}

static void test_robot_rules(void)
{
    RgJob j;
    RgPlan pl;

    job_with(&j, "axis = 2600 0 300");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "mm outside its reach"));
    rg_plan_free(&pl);

    job_with(&j, "min_wrist = 60");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "axis 5 comes within 30.0 deg of straight"));
    rg_plan_free(&pl);

    job_with(&j, "min_margin = 45");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "of its limit"));
    rg_plan_free(&pl);

    /* The home position puts the gun tip 85 mm from this part. */
    job_with(&j, "clearance = 90");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "the gun tip comes within 85 mm of the part surface at the "
                                    "home position (clearance 90 mm)"));
    CHECK_NEAR(pl.min_clearance, 85.0, 0.5);
    rg_plan_free(&pl);

    job_with(&j, "clearance = 300");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "the gun tip comes within"));
    CHECK(has_issue(&pl, RG_REFUSE, "(clearance 300 mm)"));
    rg_plan_free(&pl);

    /* Radius 300 + standoff 150 + clearance 50 needs 500 mm of room. */
    job_with(&j, "axis = 400 0 300");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "inside the part and gun"));
    rg_plan_free(&pl);

    job_with(&j, "rpm = 300\nspot_diameter = 200\nstep_over = 200\nmax_spray_speed = 500");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "1000.0 mm/s, above max_spray_speed 500"));
    rg_plan_free(&pl);

    job_with(&j, "home = 0 0 70 0 30 0");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "home position"));
    rg_plan_free(&pl);
}

static void flat_with(RgJob *j, const char *extra)
{
    char text[8192];
    snprintf(text, sizeof text, "%s\n%s\n", rg_job_template_flat(), extra);
    rg_job_default(j);
    bool ok = rg_job_parse(j, text, err, sizeof err) && rg_job_validate(j, err, sizeof err);
    if (!ok)
        printf("  job did not parse: %s\n", err);
    CHECK(ok);
}

static void test_flat(void)
{
    RgJob j;
    RgPlan pl;
    flat_with(&j, "");
    bool ok = rg_plan_build(&j, NULL, &pl);
    if (!ok)
        show(&pl);
    CHECK(ok);

    /* Each open stroke gains a run-on and a run-off, so it is sprayed from
     * clear of the work to clear of it again:
     * home; stroke 1: approach, down, 4 points, run-off, up;
     * stroke 2: approach, down, 2 points, run-off, up; home. */
    CHECK(pl.nmoves == 16);
    CHECK_NEAR(pl.lead_needed, 300.0 * 300.0 / (2.0 * 2000.0) + 6.0, 1e-9);
    CHECK_NEAR(pl.lead_used, pl.lead_needed, 1e-12);
    if (pl.nmoves == 16) {
        CHECK(pl.moves[1].kind == RG_MV_JOINT && pl.moves[1].group == 0);
        CHECK_NEAR(pl.moves[1].tcp.pos.z, 250.0, 1e-9);           /* standoff + approach */
        CHECK_NEAR(pl.moves[2].tcp.pos.z, 150.0, 1e-9);
        CHECK(pl.moves[0].after == RG_ACT_READY);   /* asked once, before the cycles */
        /* the run-on starts the lead back along the first segment */
        CHECK_NEAR(pl.moves[2].tcp.pos.x, 50.0 - pl.lead_used, 1e-9);
        CHECK_NEAR(pl.moves[2].tcp.pos.y, 50.0, 1e-9);
        /* nothing stops on the work: rounded corners until the gun is clear */
        CHECK(pl.moves[3].zone == RG_Z_SMALL && pl.moves[6].zone == RG_Z_SMALL);
        CHECK(pl.moves[7].zone == RG_Z_TRAVEL);
        CHECK(pl.moves[7].after == RG_ACT_NONE);                  /* a torch is not switched */
        CHECK_NEAR(pl.moves[3].tcp.rot.m[2][2], -1.0, 1e-12);     /* gun into the part */
    }
    check_moves_consistent(&j, &pl);
    CHECK_NEAR(pl.stroke_length, 1580.0, 1e-9);
    CHECK(pl.cycles == 6);
    CHECK_NEAR(pl.spray_time, pl.cycle_time * 6, 1e-9);
    CHECK(pl.sharp_corners == 2);
    CHECK(has_issue(&pl, RG_NOTE, "more than 45 deg at 2 points"));
    /* A round spot has no long axis, so no stroke can run the wrong way. */
    CHECK_NEAR(pl.along_fan_length, 0.0, 1e-12);
    CHECK(!has_issue(&pl, RG_WARN, "runs within 30 deg"));
    CHECK_NEAR(pl.passes_per_point, 2.0, 1e-12);
    CHECK_NEAR(pl.thickness_total, 300.0, 1e-9);
    CHECK(pl.min_clearance >= j.clearance);
    CHECK_NEAR(pl.min_clearance, 150.0, 1e-6);                /* the gun tip, at the standoff */
    /* The torch runs while the gun hops between strokes, over the part. */
    CHECK(pl.transit_drops > 0);
    CHECK(has_issue(&pl, RG_WARN, "the gun runs continuously"));
    rg_plan_free(&pl);
    rg_job_free(&j);

    /* A fan, on the other hand, does have a wrong way round. */
    flat_with(&j, "pattern = fan\nfan_width = 80\nfan_along = 90");
    CHECK(rg_plan_build(&j, NULL, &pl));
    CHECK_NEAR(pl.along_fan_length, 80.0, 1e-9);
    CHECK(has_issue(&pl, RG_WARN, "runs within 30 deg"));
    rg_plan_free(&pl);
    rg_job_free(&j);

    /* Switched off between strokes, nothing is coated in between. */
    flat_with(&j, "gun = switched\ngun_signal = doGunOn");
    CHECK(rg_plan_build(&j, NULL, &pl));
    CHECK(!has_issue(&pl, RG_WARN, "the gun runs continuously"));
    rg_plan_free(&pl);
    rg_job_free(&j);

    flat_with(&j, "plane = 2600 -300 200");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "at the approach to stroke 1"));
    rg_plan_free(&pl);
    rg_job_free(&j);

    /* Turn the fan along X, with the gun mounted to match, and it is the long
     * runs that suffer instead. */
    flat_with(&j, "pattern = fan\nfan_width = 80\nfan_along = 0\n"
                  "tool_rot = 0.866025 0.5 0 0");
    ok = rg_plan_build(&j, NULL, &pl);
    if (!ok)
        show(&pl);
    CHECK(ok);
    CHECK_NEAR(pl.along_fan_length, 1500.0, 1e-9);
    CHECK(has_issue(&pl, RG_WARN, "1500 mm of the 1580 mm painted (95 %)"));
    CHECK_NEAR(pl.moves[3].tcp.rot.m[0][0], 1.0, 1e-12);
    rg_plan_free(&pl);
    rg_job_free(&j);

    /* Turning the fan without turning the gun on its mount turns the wrist
     * with it, and here that lands on the wrist singularity. */
    flat_with(&j, "pattern = fan\nfan_width = 80\nfan_along = 0");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "axis 5 comes within"));
    rg_plan_free(&pl);
    rg_job_free(&j);

    flat_with(&j, "spray_speed = 1200");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "spray_speed 1200 mm/s is above max_spray_speed 1000"));
    rg_plan_free(&pl);
    rg_job_free(&j);

    /* A standoff under the clearance puts the gun tip too close: first on the
     * way down into stroke 1, and at worst the standoff itself. */
    flat_with(&j, "standoff = 30");
    CHECK(!rg_plan_build(&j, NULL, &pl));
    CHECK(has_issue(&pl, RG_REFUSE, "the gun tip comes within"));
    CHECK(has_issue(&pl, RG_REFUSE, "(clearance 50 mm)"));
    CHECK_NEAR(pl.min_clearance, 30.0, 1e-6);
    rg_plan_free(&pl);
    rg_job_free(&j);
}

TEST_MAIN("test_plan",
    test_template_plan();
    test_bands();
    test_drawings();
    test_robot_rules();
    test_flat();
)
