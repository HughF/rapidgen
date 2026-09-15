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
/* test_robot.c — IRB 2400 forward and inverse kinematics. */
#include "rg_test.h"
#include "rg_robot.h"

static double wrap_diff(double a, double b)
{
    double d = fmod(a - b, 360.0);
    if (d > 180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return fabs(d);
}

static double rot_err(RgMat3 a, RgMat3 b)
{
    double e = 0.0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            e = fmax(e, fabs(a.m[i][j] - b.m[i][j]));
    return e;
}

/* A fixed sequence, so a failure reproduces. */
static unsigned long seed = 12345;
static double rnd(double lo, double hi)
{
    seed = seed * 1103515245ul + 12345ul;
    return lo + (hi - lo) * (double)((seed >> 8) & 0xffffff) / (double)0xffffff;
}

static void test_calibration_pose(void)
{
    const RgRobot *r = rg_robot_find("irb2400_16");
    CHECK(r != NULL);
    double j[6] = { 0, 0, 0, 0, 0, 0 };
    RgPose t0;
    RgVec3 wc;
    rg_robot_fk(r, j, &t0, &wc);
    /* Wrist centre a1 + d4 forward and d1 + a2 + a3 up; flange d6 further. */
    CHECK_NEAR(t0.pos.x, 940.0, 1e-9);
    CHECK_NEAR(t0.pos.y, 0.0, 1e-9);
    CHECK_NEAR(t0.pos.z, 1455.0, 1e-9);
    CHECK_NEAR(wc.x, 855.0, 1e-9);
    RgQuat q = rg_quat_from_m3(t0.rot);
    CHECK_NEAR(q.q1, 0.707107, 1e-6);
    CHECK_NEAR(q.q2, 0.0, 1e-9);
    CHECK_NEAR(q.q3, 0.707107, 1e-6);
    CHECK_NEAR(q.q4, 0.0, 1e-9);
    int cf[4];
    rg_robot_confdata(r, j, cf);
    CHECK(cf[0] == 0 && cf[1] == 0 && cf[2] == 0 && cf[3] == 0);
}

static void test_simple_poses(void)
{
    const RgRobot *r = rg_robot_find("irb2400_10");
    RgPose t0;

    double turned[6] = { 90, 0, 0, 0, 0, 0 };
    rg_robot_fk(r, turned, &t0, NULL);
    CHECK_NEAR(t0.pos.x, 0.0, 1e-9);
    CHECK_NEAR(t0.pos.y, 940.0, 1e-9);

    /* Lower arm leaning fully forward: the upper arm then points down. */
    double lean[6] = { 0, 90, 0, 0, 0, 0 };
    rg_robot_fk(r, lean, &t0, NULL);
    CHECK_NEAR(t0.pos.x, 100.0 + 705.0 + 135.0, 1e-9);
    CHECK_NEAR(t0.pos.z, 615.0 - 755.0 - 85.0, 1e-9);

    int cf[4];
    double quads[6] = { -45, 0, 0, 100, -30, -181 };
    rg_robot_confdata(r, quads, cf);
    CHECK(cf[0] == -1);
    CHECK(cf[1] == 1);
    CHECK(cf[2] == -3);
    CHECK(cf[3] == 1);     /* in front of everything, axis 5 negative */
}

static void test_round_trip(void)
{
    const RgRobot *r = rg_robot_find("irb2400_16");
    int found = 0, near_ok = 0, n = 2000;
    double worst_pos = 0.0, worst_rot = 0.0;

    for (int k = 0; k < n; k++) {
        double j[6];
        for (int a = 0; a < 6; a++)
            j[a] = rnd(r->lo[a] > -180 ? r->lo[a] : -179, r->hi[a] < 180 ? r->hi[a] : 179);
        if (fabs(j[4]) < 1.0)
            j[4] = 1.0;

        RgPose t0;
        rg_robot_fk(r, j, &t0, NULL);
        double sol[8][6], shortfall;
        int ns = rg_robot_ik_all(r, &t0, sol, &shortfall);

        bool match = false;
        for (int i = 0; i < ns; i++) {
            RgPose back;
            rg_robot_fk(r, sol[i], &back, NULL);
            worst_pos = fmax(worst_pos, rg_v3_len(rg_v3_sub(back.pos, t0.pos)));
            worst_rot = fmax(worst_rot, rot_err(back.rot, t0.rot));
            bool same = true;
            for (int a = 0; a < 6; a++)
                same = same && wrap_diff(sol[i][a], j[a]) < 1e-6;
            match = match || same;
        }
        found += match;

        double out[6];
        RgIkResult res;
        if (rg_robot_ik_near(r, &t0, j, out, &res) == RG_IK_OK) {
            bool same = true;
            for (int a = 0; a < 6; a++)
                same = same && fabs(out[a] - j[a]) < 1e-6;
            near_ok += same;
        }
    }
    CHECK(found == n);
    CHECK(near_ok == n);
    CHECK(worst_pos < 1e-6);
    CHECK(worst_rot < 1e-9);
}

static void test_out_of_reach(void)
{
    const RgRobot *r = rg_robot_find("irb2400_16");
    /* Tool Z along +X, so the wrist centre is 85 mm short of the point. */
    RgPose t0 = rg_pose(rg_rot_y(RG_PI / 2), rg_v3(3000, 0, 600));
    double sol[8][6], shortfall;
    CHECK(rg_robot_ik_all(r, &t0, sol, &shortfall) == 0);
    double rr = 2915.0 - 100.0, s = 600.0 - 615.0;
    double expect = hypot(rr, s) - (705.0 + hypot(755.0, 135.0));
    CHECK_NEAR(shortfall, expect, 1e-6);

    double seed6[6] = { 0 }, out[6];
    RgIkResult res;
    CHECK(rg_robot_ik_near(r, &t0, seed6, out, &res) == RG_IK_OUT_OF_REACH);
    CHECK_NEAR(res.shortfall_mm, expect, 1e-6);
}

static void test_limits(void)
{
    const RgRobot *r = rg_robot_find("irb2400_16");
    double j[6] = { 0, 0, 62, 0, 30, 0 };
    int axis;
    CHECK_NEAR(rg_robot_margin(r, j, &axis), 3.0, 1e-9);
    CHECK(axis == 2);
    CHECK(rg_robot_within_limits(r, j));
    j[2] = 66;
    CHECK(!rg_robot_within_limits(r, j));

    /* Axis 6 is taken round by a whole turn to stay nearest the seed. */
    double from[6] = { 10, 10, 10, 10, 40, 350 };
    RgPose t0;
    rg_robot_fk(r, from, &t0, NULL);
    double out[6];
    CHECK(rg_robot_ik_near(r, &t0, from, out, NULL) == RG_IK_OK);
    CHECK_NEAR(out[5], 350.0, 1e-6);
}

TEST_MAIN("test_robot",
    test_calibration_pose();
    test_simple_poses();
    test_round_trip();
    test_out_of_reach();
    test_limits();
)
