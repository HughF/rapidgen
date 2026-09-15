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
/*
 * rg_vec.h — vectors, rotations, quaternions and rigid poses.
 *
 * Millimetres and radians. Quaternions use ABB's order, scalar first
 * (q1 = w), because every one of them ends up in a RAPID robtarget or
 * tooldata and a reordering bug there turns a gun sideways.
 */
#ifndef RG_VEC_H
#define RG_VEC_H

#include <math.h>

#define RG_PI  3.14159265358979323846
#define RG_DEG (RG_PI / 180.0)

typedef struct { double x, y, z; } RgVec3;

/* m[row][col]; the columns are the rotated frame's axes. */
typedef struct { double m[3][3]; } RgMat3;

typedef struct { double q1, q2, q3, q4; } RgQuat;

/* A frame: `rot` and `pos` of the child expressed in the parent. */
typedef struct { RgMat3 rot; RgVec3 pos; } RgPose;

static inline RgVec3 rg_v3(double x, double y, double z)
{
    RgVec3 v = { x, y, z };
    return v;
}

static inline RgVec3 rg_v3_add(RgVec3 a, RgVec3 b) { return rg_v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline RgVec3 rg_v3_sub(RgVec3 a, RgVec3 b) { return rg_v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline RgVec3 rg_v3_scale(RgVec3 a, double s) { return rg_v3(a.x * s, a.y * s, a.z * s); }
static inline double rg_v3_dot(RgVec3 a, RgVec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline double rg_v3_len(RgVec3 a) { return sqrt(rg_v3_dot(a, a)); }

static inline RgVec3 rg_v3_cross(RgVec3 a, RgVec3 b)
{
    return rg_v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

static inline RgVec3 rg_v3_lerp(RgVec3 a, RgVec3 b, double t)
{
    return rg_v3_add(a, rg_v3_scale(rg_v3_sub(b, a), t));
}

static inline RgMat3 rg_m3_identity(void)
{
    RgMat3 r = { { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } };
    return r;
}

static inline RgMat3 rg_m3_from_cols(RgVec3 x, RgVec3 y, RgVec3 z)
{
    RgMat3 r = { { { x.x, y.x, z.x }, { x.y, y.y, z.y }, { x.z, y.z, z.z } } };
    return r;
}

static inline RgVec3 rg_m3_col(RgMat3 a, int c)
{
    return rg_v3(a.m[0][c], a.m[1][c], a.m[2][c]);
}

static inline RgMat3 rg_m3_mul(RgMat3 a, RgMat3 b)
{
    RgMat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}

static inline RgMat3 rg_m3_transpose(RgMat3 a)
{
    RgMat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[i][j] = a.m[j][i];
    return r;
}

static inline RgVec3 rg_m3_apply(RgMat3 a, RgVec3 v)
{
    return rg_v3(a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
                 a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
                 a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z);
}

static inline RgMat3 rg_rot_x(double a)
{
    double c = cos(a), s = sin(a);
    RgMat3 r = { { { 1, 0, 0 }, { 0, c, -s }, { 0, s, c } } };
    return r;
}

static inline RgMat3 rg_rot_y(double a)
{
    double c = cos(a), s = sin(a);
    RgMat3 r = { { { c, 0, s }, { 0, 1, 0 }, { -s, 0, c } } };
    return r;
}

static inline RgMat3 rg_rot_z(double a)
{
    double c = cos(a), s = sin(a);
    RgMat3 r = { { { c, -s, 0 }, { s, c, 0 }, { 0, 0, 1 } } };
    return r;
}

static inline RgPose rg_pose(RgMat3 rot, RgVec3 pos)
{
    RgPose p;
    p.rot = rot;
    p.pos = pos;
    return p;
}

static inline RgPose rg_pose_mul(RgPose a, RgPose b)
{
    return rg_pose(rg_m3_mul(a.rot, b.rot), rg_v3_add(rg_m3_apply(a.rot, b.pos), a.pos));
}

static inline RgPose rg_pose_inverse(RgPose a)
{
    RgMat3 rt = rg_m3_transpose(a.rot);
    return rg_pose(rt, rg_v3_scale(rg_m3_apply(rt, a.pos), -1.0));
}

static inline RgVec3 rg_pose_apply(RgPose a, RgVec3 v)
{
    return rg_v3_add(rg_m3_apply(a.rot, v), a.pos);
}

static inline RgQuat rg_quat_normalise(RgQuat q)
{
    double n = sqrt(q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3 + q.q4 * q.q4);
    RgQuat r = { q.q1 / n, q.q2 / n, q.q3 / n, q.q4 / n };
    return r;
}

/* The first non-zero component kept positive, so the same rotation always
 * prints the same way — a half turn included, where the scalar part is 0. */
static inline RgQuat rg_quat_from_m3(RgMat3 a)
{
    const double (*m)[3] = a.m;
    double t = m[0][0] + m[1][1] + m[2][2];
    RgQuat q;
    if (t > 0.0) {
        double s = 2.0 * sqrt(t + 1.0);
        q.q1 = 0.25 * s;
        q.q2 = (m[2][1] - m[1][2]) / s;
        q.q3 = (m[0][2] - m[2][0]) / s;
        q.q4 = (m[1][0] - m[0][1]) / s;
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        double s = 2.0 * sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]);
        q.q1 = (m[2][1] - m[1][2]) / s;
        q.q2 = 0.25 * s;
        q.q3 = (m[1][0] + m[0][1]) / s;
        q.q4 = (m[0][2] + m[2][0]) / s;
    } else if (m[1][1] > m[2][2]) {
        double s = 2.0 * sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]);
        q.q1 = (m[0][2] - m[2][0]) / s;
        q.q2 = (m[1][0] + m[0][1]) / s;
        q.q3 = 0.25 * s;
        q.q4 = (m[2][1] + m[1][2]) / s;
    } else {
        double s = 2.0 * sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]);
        q.q1 = (m[1][0] - m[0][1]) / s;
        q.q2 = (m[0][2] + m[2][0]) / s;
        q.q3 = (m[2][1] + m[1][2]) / s;
        q.q4 = 0.25 * s;
    }
    q = rg_quat_normalise(q);
    double lead = fabs(q.q1) > 1e-9 ? q.q1 : fabs(q.q2) > 1e-9 ? q.q2
                : fabs(q.q3) > 1e-9 ? q.q3 : q.q4;
    if (lead < 0.0) {
        q.q1 = -q.q1; q.q2 = -q.q2; q.q3 = -q.q3; q.q4 = -q.q4;
    }
    return q;
}

static inline RgMat3 rg_m3_from_quat(RgQuat q)
{
    q = rg_quat_normalise(q);
    double w = q.q1, x = q.q2, y = q.q3, z = q.q4;
    RgMat3 r = { {
        { 1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y) },
        { 2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x) },
        { 2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y) },
    } };
    return r;
}

static inline RgQuat rg_quat_slerp(RgQuat a, RgQuat b, double t)
{
    double d = a.q1 * b.q1 + a.q2 * b.q2 + a.q3 * b.q3 + a.q4 * b.q4;
    if (d < 0.0) {
        b.q1 = -b.q1; b.q2 = -b.q2; b.q3 = -b.q3; b.q4 = -b.q4;
        d = -d;
    }
    double wa = 1.0 - t, wb = t;
    if (d < 0.9995) {
        double th = acos(d), s = sin(th);
        wa = sin((1.0 - t) * th) / s;
        wb = sin(t * th) / s;
    }
    RgQuat r = { wa * a.q1 + wb * b.q1, wa * a.q2 + wb * b.q2,
                 wa * a.q3 + wb * b.q3, wa * a.q4 + wb * b.q4 };
    return rg_quat_normalise(r);
}

#endif /* RG_VEC_H */
