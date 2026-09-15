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
#include "rg_robot.h"
#include "rg_text.h"

#include <float.h>
#include <math.h>
#include <string.h>

/*
 * The /10 and /16 differ in payload, not geometry: same arm lengths, same
 * axis ranges. The IRB 2400L has a longer arm and is left out until its
 * dimensions have been checked against ABB's data sheet — a wrong arm length
 * would pass reach checks the real robot fails.
 */
static const RgRobot robots[] = {
    {
        "irb2400_16", "IRB 2400/16",
        615.0, 100.0, 705.0, 135.0, 755.0, 85.0,
        { -180.0, -100.0, -60.0, -200.0, -120.0, -400.0 },
        {  180.0,  110.0,  65.0,  200.0,  120.0,  400.0 },
    },
    {
        "irb2400_10", "IRB 2400/10",
        615.0, 100.0, 705.0, 135.0, 755.0, 85.0,
        { -180.0, -100.0, -60.0, -200.0, -120.0, -400.0 },
        {  180.0,  110.0,  65.0,  200.0,  120.0,  400.0 },
    },
};

#define N_ROBOTS ((int)(sizeof robots / sizeof robots[0]))

int rg_robot_count(void)
{
    return N_ROBOTS;
}

const RgRobot *rg_robot_at(int i)
{
    return (i >= 0 && i < N_ROBOTS) ? &robots[i] : NULL;
}

const RgRobot *rg_robot_find(const char *id)
{
    for (int i = 0; i < N_ROBOTS; i++)
        if (rg_streqi(robots[i].id, id))
            return &robots[i];
    return NULL;
}

static RgPose rot_only(RgMat3 r)
{
    return rg_pose(r, rg_v3(0, 0, 0));
}

static RgPose shift(double x, double y, double z)
{
    return rg_pose(rg_m3_identity(), rg_v3(x, y, z));
}

/* The flange frame of the last link has its X along the tool axis; tool0
 * turns that onto Z. */
static RgMat3 flange_to_tool0(void)
{
    return rg_rot_y(RG_PI / 2.0);
}

void rg_robot_fk(const RgRobot *r, const double j[RG_AXES], RgPose *tool0, RgVec3 *wrist)
{
    double q[RG_AXES];
    for (int i = 0; i < RG_AXES; i++)
        q[i] = j[i] * RG_DEG;

    RgPose t = rot_only(rg_rot_z(q[0]));
    t = rg_pose_mul(t, shift(r->a1, 0, r->d1));
    t = rg_pose_mul(t, rot_only(rg_rot_y(q[1])));
    t = rg_pose_mul(t, shift(0, 0, r->a2));
    t = rg_pose_mul(t, rot_only(rg_rot_y(q[2])));
    t = rg_pose_mul(t, shift(r->d4, 0, r->a3));
    if (wrist)
        *wrist = t.pos;
    t = rg_pose_mul(t, rot_only(rg_rot_x(q[3])));
    t = rg_pose_mul(t, rot_only(rg_rot_y(q[4])));
    t = rg_pose_mul(t, shift(r->d6, 0, 0));
    t = rg_pose_mul(t, rot_only(rg_rot_x(q[5])));
    t = rg_pose_mul(t, rot_only(flange_to_tool0()));
    if (tool0)
        *tool0 = t;
}

static double wrap180(double deg)
{
    deg = fmod(deg, 360.0);
    if (deg > 180.0)
        deg -= 360.0;
    else if (deg <= -180.0)
        deg += 360.0;
    return deg;
}

/*
 * Closed form for a spherical wrist on an arm with a shoulder offset (a1) and
 * an elbow offset (a3).
 *
 * In the vertical plane of axis 1, with r forward from axis 2 and s up from
 * it, the wrist centre is
 *     r = a2·sin q2 + L·cos ψ,   s = a2·cos q2 − L·sin ψ,
 * where L = |(d4, a3)|, δ = atan2(a3, d4) and ψ = q2 + q3 − δ. The law of
 * cosines gives sin(δ − q3) = (r² + s² − a2² − L²) / (2·a2·L) — two elbow
 * solutions — and q2 then follows linearly. The wrist is Rx(q4)·Ry(q5)·Rx(q6).
 */
int rg_robot_ik_all(const RgRobot *r, const RgPose *tool0, double sol[8][RG_AXES],
                    double *shortfall_mm)
{
    RgMat3 r6 = rg_m3_mul(tool0->rot, rg_m3_transpose(flange_to_tool0()));
    RgVec3 tool_z = rg_m3_col(tool0->rot, 2);
    RgVec3 wc = rg_v3_sub(tool0->pos, rg_v3_scale(tool_z, r->d6));

    double L = hypot(r->d4, r->a3);
    double delta = atan2(r->a3, r->d4);
    double h = hypot(wc.x, wc.y);
    double heading = atan2(wc.y, wc.x);
    double s = wc.z - r->d1;

    int n = 0;
    double best_short = DBL_MAX;

    for (int back = 0; back < 2; back++) {
        double q1 = back ? heading + RG_PI : heading;
        double rr = back ? -h - r->a1 : h - r->a1;
        double D = hypot(rr, s);
        double k = (D * D - r->a2 * r->a2 - L * L) / (2.0 * r->a2 * L);
        if (k > 1.0 || k < -1.0) {
            double miss = k > 1.0 ? D - (r->a2 + L) : fabs(r->a2 - L) - D;
            if (miss < best_short)
                best_short = miss;
            continue;
        }
        double as = asin(k);
        for (int elbow = 0; elbow < 2; elbow++) {
            double q3 = elbow ? delta - RG_PI + as : delta - as;
            double beta = q3 - delta;
            double A = r->a2 - L * sin(beta);
            double B = L * cos(beta);
            double q2 = atan2(A * rr - B * s, B * rr + A * s);

            RgMat3 r03 = rg_m3_mul(rg_rot_z(q1), rg_rot_y(q2 + q3));
            RgMat3 w = rg_m3_mul(rg_m3_transpose(r03), r6);

            double s5 = hypot(w.m[1][0], w.m[2][0]);
            double q4, q5, q6;
            if (s5 < 1e-9) {
                /* Axis 5 straight: only q4 + q6 is determined. */
                q4 = 0.0;
                if (w.m[0][0] > 0.0) {
                    q5 = 0.0;
                    q6 = atan2(w.m[2][1], w.m[1][1]);
                } else {
                    q5 = RG_PI;
                    q6 = atan2(-w.m[2][1], w.m[1][1]);
                }
            } else {
                q5 = atan2(s5, w.m[0][0]);
                q4 = atan2(w.m[1][0], -w.m[2][0]);
                q6 = atan2(w.m[0][1], w.m[0][2]);
            }

            for (int flip = 0; flip < 2; flip++) {
                double *o = sol[n++];
                o[0] = wrap180(q1 / RG_DEG);
                o[1] = wrap180(q2 / RG_DEG);
                o[2] = wrap180(q3 / RG_DEG);
                if (!flip) {
                    o[3] = wrap180(q4 / RG_DEG);
                    o[4] = wrap180(q5 / RG_DEG);
                    o[5] = wrap180(q6 / RG_DEG);
                } else {
                    o[3] = wrap180(q4 / RG_DEG + 180.0);
                    o[4] = wrap180(-q5 / RG_DEG);
                    o[5] = wrap180(q6 / RG_DEG + 180.0);
                }
            }
        }
    }

    if (shortfall_mm)
        *shortfall_mm = n ? 0.0 : best_short;
    return n;
}

/* The value of `v` plus whole turns nearest `seed` within [lo, hi]. False,
 * with the distance outside the range, when no whole turn fits. */
static bool fit_axis(double v, double seed, double lo, double hi, double *out, double *excess)
{
    bool found = false;
    double best = DBL_MAX, least = DBL_MAX;
    for (int k = -2; k <= 2; k++) {
        double c = v + 360.0 * k;
        if (c >= lo - 1e-9 && c <= hi + 1e-9) {
            double d = fabs(c - seed);
            if (d < best) {
                best = d;
                *out = c;
                found = true;
            }
        } else {
            double e = c < lo ? lo - c : c - hi;
            if (e < least)
                least = e;
        }
    }
    *excess = found ? 0.0 : least;
    return found;
}

RgIkStatus rg_robot_ik_near(const RgRobot *r, const RgPose *tool0, const double seed[RG_AXES],
                            double out[RG_AXES], RgIkResult *res)
{
    RgIkResult local;
    if (!res)
        res = &local;
    memset(res, 0, sizeof *res);

    double sol[8][RG_AXES];
    double shortfall;
    int n = rg_robot_ik_all(r, tool0, sol, &shortfall);
    if (n == 0) {
        res->status = RG_IK_OUT_OF_REACH;
        res->shortfall_mm = shortfall;
        return res->status;
    }

    double best_cost = DBL_MAX;
    double least_over = DBL_MAX;
    int least_axis = 0;

    for (int i = 0; i < n; i++) {
        double cand[RG_AXES];
        double worst = 0.0;
        int worst_axis = 0;
        for (int a = 0; a < RG_AXES; a++) {
            double excess;
            if (!fit_axis(sol[i][a], seed[a], r->lo[a], r->hi[a], &cand[a], &excess)) {
                if (excess > worst) {
                    worst = excess;
                    worst_axis = a;
                }
            }
        }
        if (worst > 0.0) {
            if (worst < least_over) {
                least_over = worst;
                least_axis = worst_axis;
            }
            continue;
        }
        double cost = 0.0;
        for (int a = 0; a < RG_AXES; a++)
            cost += fabs(cand[a] - seed[a]);
        if (cost < best_cost) {
            best_cost = cost;
            memcpy(out, cand, sizeof cand);
        }
    }

    if (best_cost == DBL_MAX) {
        res->status = RG_IK_JOINT_LIMIT;
        res->axis = least_axis;
        res->over_deg = least_over;
        return res->status;
    }
    res->status = RG_IK_OK;
    return res->status;
}

double rg_robot_margin(const RgRobot *r, const double j[RG_AXES], int *axis)
{
    double best = DBL_MAX;
    int which = 0;
    for (int a = 0; a < RG_AXES; a++) {
        double m = j[a] - r->lo[a];
        if (r->hi[a] - j[a] < m)
            m = r->hi[a] - j[a];
        if (m < best) {
            best = m;
            which = a;
        }
    }
    if (axis)
        *axis = which;
    return best;
}

bool rg_robot_within_limits(const RgRobot *r, const double j[RG_AXES])
{
    for (int a = 0; a < RG_AXES; a++)
        if (j[a] < r->lo[a] - 1e-9 || j[a] > r->hi[a] + 1e-9)
            return false;
    return true;
}

void rg_robot_confdata(const RgRobot *r, const double j[RG_AXES], int cf[4])
{
    cf[0] = (int)floor(j[0] / 90.0);
    cf[1] = (int)floor(j[3] / 90.0);
    cf[2] = (int)floor(j[5] / 90.0);

    double q2 = j[1] * RG_DEG, phi = (j[1] + j[2]) * RG_DEG;
    /* Wrist centre relative to axis 2, in the arm plane. */
    double dx = r->a2 * sin(q2) + r->d4 * cos(phi) + r->a3 * sin(phi);
    double dz = r->a2 * cos(q2) - r->d4 * sin(phi) + r->a3 * cos(phi);

    bool behind_axis1 = r->a1 + dx < 0.0;
    bool behind_lower_arm = dx * cos(q2) - dz * sin(q2) < 0.0;
    bool axis5_negative = j[4] < 0.0;
    cf[3] = (behind_axis1 ? 4 : 0) + (behind_lower_arm ? 2 : 0) + (axis5_negative ? 1 : 0);
}
