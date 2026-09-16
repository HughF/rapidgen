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
#include "rg_job.h"
#include "rg_rapid.h"
#include "rg_robot.h"
#include "rg_version.h"

#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Required values start out not-a-number, so "never given" can be told
 * apart from any real value. */
static const double UNSET = NAN;

void rg_job_default(RgJob *j)
{
    memset(j, 0, sizeof *j);
    rg_copy(j->robot, sizeof j->robot, "irb2400_16");
    j->part = RG_PART_CYLINDER;
    j->drawing_scale = 1.0;

    j->radius = UNSET;
    j->part_height = UNSET;
    j->axis = rg_v3(UNSET, UNSET, UNSET);
    j->azimuth = 0.0;
    j->rpm = UNSET;

    j->plane = rg_v3(UNSET, UNSET, UNSET);
    j->plane_rot.q1 = 1.0;
    j->spray_speed = UNSET;

    j->pattern = RG_PAT_SPOT;
    j->spot_diameter = UNSET;
    j->fan_width = UNSET;
    j->fan_along = 90.0;
    j->step_over = UNSET;
    j->overlap = 50.0;
    j->standoff = UNSET;
    j->coats = 1;
    j->cycles = 1;
    j->dwell = 0.0;
    j->thickness_per_pass = UNSET;
    j->target_thickness = UNSET;
    j->lead = UNSET;
    j->start_top = true;
    j->accel = 2000.0;
    j->approach = 150.0;
    j->travel_speed = 250.0;
    j->approach_speed = 100.0;
    j->max_spray_speed = 1000.0;

    j->tool_define = false;
    j->tool_tcp = rg_v3(UNSET, UNSET, UNSET);
    j->tool_rot.q1 = 1.0;
    j->tool_mass = UNSET;

    j->home[4] = 30.0;
    j->ready_prompt = true;
    j->gun = RG_GUN_CONTINUOUS;

    j->min_wrist = 10.0;
    j->min_margin = 5.0;
    j->clearance = 50.0;
    j->max_joint_step = 20.0;
    j->sample_step = 5.0;
    j->wrap_tolerance = 5.0;
    j->join_tolerance = 0.1;
    j->chord_tolerance = 0.2;
}

void rg_job_free(RgJob *j)
{
    for (int i = 0; i < j->nstrokes; i++)
        free(j->strokes[i].pts);
    free(j->strokes);
    j->strokes = NULL;
    j->nstrokes = 0;
}

bool rg_job_add_stroke(RgJob *j, const RgPt *pts, int n)
{
    if (n < 1)
        return true;
    RgStroke *s = realloc(j->strokes, (size_t)(j->nstrokes + 1) * sizeof *s);
    if (!s)
        return false;
    j->strokes = s;
    RgPt *copy = malloc((size_t)n * sizeof *copy);
    if (!copy)
        return false;
    memcpy(copy, pts, (size_t)n * sizeof *copy);
    s[j->nstrokes].pts = copy;
    s[j->nstrokes].n = n;
    j->nstrokes++;
    return true;
}

void rg_job_delete_stroke(RgJob *j, int index)
{
    if (index < 0 || index >= j->nstrokes)
        return;
    free(j->strokes[index].pts);
    memmove(&j->strokes[index], &j->strokes[index + 1],
            (size_t)(j->nstrokes - index - 1) * sizeof j->strokes[0]);
    j->nstrokes--;
}

bool rg_job_copy(RgJob *dst, const RgJob *src)
{
    *dst = *src;
    dst->strokes = NULL;
    dst->nstrokes = 0;
    for (int i = 0; i < src->nstrokes; i++)
        if (!rg_job_add_stroke(dst, src->strokes[i].pts, src->strokes[i].n)) {
            rg_job_free(dst);
            return false;
        }
    return true;
}

const char *rg_part_name(RgPartKind k)
{
    return k == RG_PART_FLAT ? "flat" : "cylinder";
}

double rg_job_width(const RgJob *j)
{
    return j->pattern == RG_PAT_FAN ? j->fan_width : j->spot_diameter;
}

double rg_job_step(const RgJob *j)
{
    if (!isnan(j->step_over))
        return j->step_over;
    return rg_job_width(j) * (1.0 - j->overlap / 100.0);
}

/* ---- the key table -------------------------------------------------- */

typedef enum {
    K_IDENT, K_TEXT, K_NUM, K_INT, K_BOOL, K_VEC3, K_QUAT, K_JOINTS, K_BAND,
    K_START, K_PART, K_STROKE, K_PATTERN, K_GUN
} KType;

typedef struct {
    const char *key;
    KType       type;
    size_t      off, cap;
    double      lo, hi;
    const char *group;     /* heading the writer puts above it */
} Key;

#define F(field)          offsetof(RgJob, field)
#define S(field)          sizeof(((RgJob *)0)->field)

static const char G_WHAT[]  = "what and where";
static const char G_CYL[]   = "a cylinder on a rotator";
static const char G_FLAT[]  = "a flat part";
static const char G_PROC[]  = "the process";
static const char G_GUN[]   = "the gun";
static const char G_PROG[]  = "around the program";
static const char G_RULES[] = "rules the plan must keep";
static const char G_PAINT[] = "the painted strokes, in order: x y pairs in drawing mm";

static const Key keys[] = {
    { "name",            K_IDENT,  F(name),            S(name),         0, 0, G_WHAT },
    { "controller",      K_TEXT,   F(controller),      S(controller),   0, 0, G_WHAT },
    { "robot",           K_TEXT,   F(robot),           S(robot),        0, 0, G_WHAT },
    { "part",            K_PART,   F(part),            0,               0, 0, G_WHAT },
    { "drawing",         K_TEXT,   F(drawing),         S(drawing),      0, 0, G_WHAT },
    { "layer",           K_TEXT,   F(layer),           S(layer),        0, 0, G_WHAT },
    { "drawing_scale",   K_NUM,    F(drawing_scale),   0,               1e-6, 1e6, G_WHAT },
    { "band",            K_BAND,   0,                  0,               -100000, 100000, G_WHAT },

    { "radius",          K_NUM,    F(radius),          0,               10, 5000, G_CYL },
    { "part_height",     K_NUM,    F(part_height),     0,               1, 10000, G_CYL },
    { "axis",            K_VEC3,   F(axis),            0,               -10000, 10000, G_CYL },
    { "azimuth",         K_NUM,    F(azimuth),         0,               -90, 90, G_CYL },
    { "rpm",             K_NUM,    F(rpm),             0,               0.1, 300, G_CYL },

    { "plane",           K_VEC3,   F(plane),           0,               -10000, 10000, G_FLAT },
    { "plane_rot",       K_QUAT,   F(plane_rot),       0,               -1, 1, G_FLAT },
    { "spray_speed",     K_NUM,    F(spray_speed),     0,               1, 2000, G_FLAT },

    { "pattern",         K_PATTERN, F(pattern),        0,               0, 0, G_PROC },
    { "spot_diameter",   K_NUM,    F(spot_diameter),   0,               0.5, 200, G_PROC },
    { "fan_width",       K_NUM,    F(fan_width),       0,               5, 1000, G_PROC },
    { "fan_along",       K_NUM,    F(fan_along),       0,               -180, 180, G_PROC },
    { "step_over",       K_NUM,    F(step_over),       0,               0.1, 1000, G_PROC },
    { "overlap",         K_NUM,    F(overlap),         0,               0, 90, G_PROC },
    { "standoff",        K_NUM,    F(standoff),        0,               10, 1000, G_PROC },
    { "coats",           K_INT,    F(coats),           0,               1, 50, G_PROC },
    { "cycles",          K_INT,    F(cycles),          0,               1, 999, G_PROC },
    { "dwell",           K_NUM,    F(dwell),           0,               0, 600, G_PROC },
    { "thickness_per_pass", K_NUM, F(thickness_per_pass), 0,            0.01, 5000, G_PROC },
    { "target_thickness", K_NUM,   F(target_thickness), 0,              0.1, 100000, G_PROC },
    { "lead",            K_NUM,    F(lead),            0,               0, 1000, G_PROC },
    { "start",           K_START,  F(start_top),       0,               0, 0, G_PROC },
    { "accel",           K_NUM,    F(accel),           0,               10, 20000, G_PROC },
    { "approach",        K_NUM,    F(approach),        0,               20, 1000, G_PROC },
    { "travel_speed",    K_NUM,    F(travel_speed),    0,               10, 2000, G_PROC },
    { "approach_speed",  K_NUM,    F(approach_speed),  0,               5, 1000, G_PROC },
    { "max_spray_speed", K_NUM,    F(max_spray_speed), 0,               1, 2000, G_PROC },

    { "tool",            K_IDENT,  F(tool),            S(tool),         0, 0, G_GUN },
    { "tool_define",     K_BOOL,   F(tool_define),     0,               0, 0, G_GUN },
    { "tool_tcp",        K_VEC3,   F(tool_tcp),        0,               -2000, 2000, G_GUN },
    { "tool_rot",        K_QUAT,   F(tool_rot),        0,               -1, 1, G_GUN },
    { "tool_mass",       K_NUM,    F(tool_mass),       0,               0.01, 50, G_GUN },
    { "tool_cog",        K_VEC3,   F(tool_cog),        0,               -2000, 2000, G_GUN },

    { "home",            K_JOINTS, F(home),            0,               -400, 400, G_PROG },
    { "ready_prompt",    K_BOOL,   F(ready_prompt),    0,               0, 0, G_PROG },
    { "gun",             K_GUN,    F(gun),             0,               0, 0, G_PROG },
    { "gun_signal",      K_IDENT,  F(gun_signal),      S(gun_signal),   0, 0, G_PROG },
    { "cool_signal",     K_IDENT,  F(cool_signal),     S(cool_signal),  0, 0, G_PROG },

    { "min_wrist",       K_NUM,    F(min_wrist),       0,               0, 60, G_RULES },
    { "min_margin",      K_NUM,    F(min_margin),      0,               0, 45, G_RULES },
    { "clearance",       K_NUM,    F(clearance),       0,               0, 1000, G_RULES },
    { "max_joint_step",  K_NUM,    F(max_joint_step),  0,               1, 90, G_RULES },
    { "sample_step",     K_NUM,    F(sample_step),     0,               0.5, 100, G_RULES },
    { "wrap_tolerance",  K_NUM,    F(wrap_tolerance),  0,               0, 100, G_RULES },
    { "join_tolerance",  K_NUM,    F(join_tolerance),  0,               0.001, 10, G_RULES },
    { "chord_tolerance", K_NUM,    F(chord_tolerance), 0,               0.01, 5, G_RULES },

    { "stroke",          K_STROKE, 0,                  0,               -100000, 100000, G_PAINT },
};

#define N_KEYS (sizeof keys / sizeof keys[0])

#undef F
#undef S

static char *trim(char *s)
{
    while (isspace((unsigned char)*s))
        s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1]))
        *--e = '\0';
    return s;
}

/* Every number in `v`, separated by spaces or commas: a malloc'd array and
 * its count, or NULL with *n = -1 when something is not a number. A value
 * with no numbers gives NULL and *n = 0. */
static double *numbers_all(const char *v, int *n)
{
    double *out = NULL;
    int count = 0, cap = 0;
    const char *p = v;
    for (;;) {
        while (*p == ' ' || *p == '\t' || *p == ',')
            p++;
        if (!*p)
            break;
        char *endp;
        double d = strtod(p, &endp);
        if (endp == p || !isfinite(d)) {
            free(out);
            *n = -1;
            return NULL;
        }
        if (count == cap) {
            cap = cap ? cap * 2 : 16;
            double *grown = realloc(out, (size_t)cap * sizeof *grown);
            if (!grown) {
                free(out);
                *n = -1;
                return NULL;
            }
            out = grown;
        }
        out[count++] = d;
        p = endp;
    }
    *n = count;
    return out;
}

/* Exactly `want` numbers into d; false otherwise. */
static bool numbers(const char *v, double *d, int want)
{
    int n;
    double *all = numbers_all(v, &n);
    bool ok = n == want;
    if (ok)
        memcpy(d, all, (size_t)want * sizeof *d);
    free(all);
    return ok;
}

static bool in_range(const Key *k, double v)
{
    return v >= k->lo && v <= k->hi;
}

static bool set_value(RgJob *j, const Key *k, const char *v, char *why, size_t whycap)
{
    void *field = (char *)j + k->off;
    double d[6];

    switch (k->type) {
    case K_IDENT:
        if (*v && !rg_rapid_ident_ok(v, (int)k->cap - 1)) {
            snprintf(why, whycap, "\"%s\" is not a RAPID name: a letter, then letters, "
                     "digits or _, at most %d characters, not a reserved word",
                     v, (int)k->cap - 1);
            return false;
        }
        rg_copy(field, k->cap, v);
        return true;

    case K_TEXT:
        if (strlen(v) >= k->cap) {
            snprintf(why, whycap, "longer than %d characters", (int)k->cap - 1);
            return false;
        }
        rg_copy(field, k->cap, v);
        return true;

    case K_NUM:
    case K_INT:
        if (!numbers(v, d, 1)) {
            snprintf(why, whycap, "expected a number");
            return false;
        }
        if (!in_range(k, d[0])) {
            snprintf(why, whycap, "%g is outside %g to %g", d[0], k->lo, k->hi);
            return false;
        }
        if (k->type == K_INT) {
            if (d[0] != floor(d[0])) {
                snprintf(why, whycap, "expected a whole number");
                return false;
            }
            *(int *)field = (int)d[0];
        } else {
            *(double *)field = d[0];
        }
        return true;

    case K_BOOL:
        if (rg_streqi(v, "yes") || rg_streqi(v, "true") || strcmp(v, "1") == 0)
            *(bool *)field = true;
        else if (rg_streqi(v, "no") || rg_streqi(v, "false") || strcmp(v, "0") == 0)
            *(bool *)field = false;
        else {
            snprintf(why, whycap, "expected yes or no");
            return false;
        }
        return true;

    case K_START:
        if (rg_streqi(v, "top"))
            *(bool *)field = true;
        else if (rg_streqi(v, "bottom"))
            *(bool *)field = false;
        else {
            snprintf(why, whycap, "expected top or bottom");
            return false;
        }
        return true;

    case K_PART:
        if (rg_streqi(v, "cylinder"))
            *(RgPartKind *)field = RG_PART_CYLINDER;
        else if (rg_streqi(v, "flat"))
            *(RgPartKind *)field = RG_PART_FLAT;
        else {
            snprintf(why, whycap, "expected cylinder or flat");
            return false;
        }
        return true;

    case K_PATTERN:
        if (rg_streqi(v, "spot"))
            *(RgPattern *)field = RG_PAT_SPOT;
        else if (rg_streqi(v, "fan"))
            *(RgPattern *)field = RG_PAT_FAN;
        else {
            snprintf(why, whycap, "expected spot or fan");
            return false;
        }
        return true;

    case K_GUN:
        if (rg_streqi(v, "continuous"))
            *(RgGunKind *)field = RG_GUN_CONTINUOUS;
        else if (rg_streqi(v, "switched"))
            *(RgGunKind *)field = RG_GUN_SWITCHED;
        else {
            snprintf(why, whycap, "expected continuous or switched");
            return false;
        }
        return true;

    case K_VEC3:
        if (!numbers(v, d, 3)) {
            snprintf(why, whycap, "expected three numbers: x y z");
            return false;
        }
        for (int i = 0; i < 3; i++)
            if (!in_range(k, d[i])) {
                snprintf(why, whycap, "%g is outside %g to %g", d[i], k->lo, k->hi);
                return false;
            }
        *(RgVec3 *)field = rg_v3(d[0], d[1], d[2]);
        return true;

    case K_QUAT: {
        if (!numbers(v, d, 4)) {
            snprintf(why, whycap, "expected four numbers: q1 q2 q3 q4");
            return false;
        }
        double n = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2] + d[3] * d[3]);
        if (fabs(n - 1.0) > 1e-3) {
            snprintf(why, whycap, "the quaternion's length is %.5f, not 1", n);
            return false;
        }
        RgQuat q = { d[0], d[1], d[2], d[3] };
        *(RgQuat *)field = rg_quat_normalise(q);
        return true;
    }

    case K_JOINTS:
        if (!numbers(v, d, 6)) {
            snprintf(why, whycap, "expected six joint angles");
            return false;
        }
        memcpy(field, d, sizeof d);
        return true;

    case K_BAND:
        if (j->nbands == RG_MAX_BANDS) {
            snprintf(why, whycap, "more than %d bands", RG_MAX_BANDS);
            return false;
        }
        if (!numbers(v, d, 2) || d[1] <= d[0]) {
            snprintf(why, whycap, "expected two heights, bottom then top");
            return false;
        }
        j->bands[j->nbands].y0 = d[0];
        j->bands[j->nbands].y1 = d[1];
        j->nbands++;
        return true;

    case K_STROKE: {
        int n;
        double *all = numbers_all(v, &n);
        if (n < 0) {
            snprintf(why, whycap, "expected numbers: x y pairs");
            return false;
        }
        if (n < 4 || n % 2) {
            free(all);
            snprintf(why, whycap, "expected x y pairs, at least two points");
            return false;
        }
        RgPt *pts = malloc((size_t)(n / 2) * sizeof *pts);
        bool ok = pts != NULL;
        for (int i = 0; ok && i < n / 2; i++) {
            pts[i].x = all[2 * i];
            pts[i].y = all[2 * i + 1];
            if (!in_range(k, pts[i].x) || !in_range(k, pts[i].y)) {
                snprintf(why, whycap, "point %d is outside %g to %g", i + 1, k->lo, k->hi);
                free(pts);
                free(all);
                return false;
            }
        }
        ok = ok && rg_job_add_stroke(j, pts, n / 2);
        free(pts);
        free(all);
        if (!ok)
            snprintf(why, whycap, "out of memory");
        return ok;
    }
    }
    return false;
}

bool rg_job_parse(RgJob *j, const char *text, char *err, size_t errcap)
{
    size_t len = strlen(text);
    char *buf = malloc(len + 1);
    if (!buf) {
        snprintf(err, errcap, "out of memory");
        return false;
    }
    memcpy(buf, text, len + 1);

    bool ok = true;
    int lineno = 0;
    char *p = buf;
    while (ok && *p) {
        char *nl = strchr(p, '\n');
        if (nl)
            *nl = '\0';
        char *line = p;
        p = nl ? nl + 1 : p + strlen(p);
        lineno++;

        char *hash = strchr(line, '#');
        if (hash)
            *hash = '\0';
        char *s = trim(line);
        if (!*s)
            continue;
        char *eq = strchr(s, '=');
        if (!eq) {
            snprintf(err, errcap, "line %d: expected key = value", lineno);
            ok = false;
            break;
        }
        *eq = '\0';
        char *key = trim(s), *val = trim(eq + 1);

        const Key *k = NULL;
        for (size_t i = 0; i < N_KEYS; i++)
            if (strcmp(keys[i].key, key) == 0)
                k = &keys[i];
        if (!k) {
            snprintf(err, errcap, "line %d: unknown setting \"%s\"", lineno, key);
            ok = false;
            break;
        }
        char why[256];
        if (!set_value(j, k, val, why, sizeof why)) {
            snprintf(err, errcap, "line %d: %s: %s", lineno, key, why);
            ok = false;
        }
    }
    free(buf);
    return ok;
}

bool rg_job_load(RgJob *j, const char *path, char *err, size_t errcap)
{
    char *text = rg_read_file(path, NULL, err, errcap);
    if (!text)
        return false;
    char why[384];
    bool ok = rg_job_parse(j, text, why, sizeof why);
    if (!ok)
        snprintf(err, errcap, "%s: %s", path, why);
    free(text);
    return ok;
}

/* ---- writing -------------------------------------------------------- */

static void write_nums(RgBuf *b, const double *v, int n, int dp)
{
    char t[32];
    for (int i = 0; i < n; i++)
        rg_buf_printf(b, "%s%s", i ? " " : "", rg_fmt(t, sizeof t, v[i], dp));
}

static void write_key(RgBuf *b, const RgJob *j, const Key *k)
{
    const char *field = (const char *)j + k->off;
    char t[32];

    switch (k->type) {
    case K_IDENT:
    case K_TEXT:
        if (field[0])
            rg_buf_printf(b, "%-15s = %s\n", k->key, field);
        return;
    case K_NUM: {
        double v = *(const double *)(void *)field;
        if (!isnan(v))
            rg_buf_printf(b, "%-15s = %s\n", k->key, rg_fmt(t, sizeof t, v, 6));
        return;
    }
    case K_INT:
        rg_buf_printf(b, "%-15s = %d\n", k->key, *(const int *)(void *)field);
        return;
    case K_BOOL:
        rg_buf_printf(b, "%-15s = %s\n", k->key, *(const bool *)(void *)field ? "yes" : "no");
        return;
    case K_START:
        rg_buf_printf(b, "%-15s = %s\n", k->key, *(const bool *)(void *)field ? "top" : "bottom");
        return;
    case K_PART:
        rg_buf_printf(b, "%-15s = %s\n", k->key, rg_part_name(*(const RgPartKind *)(void *)field));
        return;
    case K_PATTERN:
        rg_buf_printf(b, "%-15s = %s\n", k->key,
                      *(const RgPattern *)(void *)field == RG_PAT_FAN ? "fan" : "spot");
        return;
    case K_GUN:
        rg_buf_printf(b, "%-15s = %s\n", k->key,
                      *(const RgGunKind *)(void *)field == RG_GUN_SWITCHED ? "switched"
                                                                           : "continuous");
        return;
    case K_VEC3: {
        const RgVec3 *v = (const RgVec3 *)(void *)field;
        if (isnan(v->x))
            return;
        double d[3] = { v->x, v->y, v->z };
        rg_buf_printf(b, "%-15s = ", k->key);
        write_nums(b, d, 3, 4);
        rg_buf_puts(b, "\n");
        return;
    }
    case K_QUAT: {
        const RgQuat *q = (const RgQuat *)(void *)field;
        double d[4] = { q->q1, q->q2, q->q3, q->q4 };
        rg_buf_printf(b, "%-15s = ", k->key);
        write_nums(b, d, 4, 6);
        rg_buf_puts(b, "\n");
        return;
    }
    case K_JOINTS:
        rg_buf_printf(b, "%-15s = ", k->key);
        write_nums(b, (const double *)(void *)field, 6, 4);
        rg_buf_puts(b, "\n");
        return;
    case K_BAND:
        for (int i = 0; i < j->nbands; i++) {
            double d[2] = { j->bands[i].y0, j->bands[i].y1 };
            rg_buf_printf(b, "%-15s = ", k->key);
            write_nums(b, d, 2, 4);
            rg_buf_puts(b, "\n");
        }
        return;
    case K_STROKE:
        for (int i = 0; i < j->nstrokes; i++) {
            rg_buf_printf(b, "%-15s =", k->key);
            for (int p = 0; p < j->strokes[i].n; p++) {
                char x[32], y[32];
                rg_buf_printf(b, "  %s %s", rg_fmt(x, sizeof x, j->strokes[i].pts[p].x, 3),
                              rg_fmt(y, sizeof y, j->strokes[i].pts[p].y, 3));
            }
            rg_buf_puts(b, "\n");
        }
        return;
    }
}

void rg_job_write(const RgJob *j, RgBuf *b)
{
    rg_buf_printf(b, "# rapidgen job file, written by %s %s\n", RAPIDGEN_NAME, RAPIDGEN_VERSION);
    rg_buf_puts(b, "# Lines are  key = value ; # starts a comment. rapidgen --template\n"
                   "# lists every setting and what it means.\n");
    const char *group = NULL;
    for (size_t i = 0; i < N_KEYS; i++) {
        const Key *k = &keys[i];
        bool cyl_only = k->group == G_CYL || k->type == K_BAND || k->type == K_START ||
                        strcmp(k->key, "coats") == 0 || strcmp(k->key, "wrap_tolerance") == 0;
        bool flat_only = k->group == G_FLAT || k->type == K_STROKE ||
                         strcmp(k->key, "lead") == 0;
        bool fan_only = strcmp(k->key, "fan_width") == 0 || strcmp(k->key, "fan_along") == 0;
        bool spot_only = strcmp(k->key, "spot_diameter") == 0;
        if ((fan_only && j->pattern != RG_PAT_FAN) || (spot_only && j->pattern != RG_PAT_SPOT))
            continue;
        if ((cyl_only && j->part != RG_PART_CYLINDER) || (flat_only && j->part != RG_PART_FLAT))
            continue;
        if (k->type == K_STROKE && j->nstrokes == 0)
            continue;
        if (k->group != group) {
            group = k->group;
            rg_buf_printf(b, "\n# ---- %s\n", group);
        }
        write_key(b, j, k);
    }
}

bool rg_job_save(const RgJob *j, const char *path, char *err, size_t errcap)
{
    RgBuf b;
    rg_buf_init(&b);
    rg_job_write(j, &b);
    bool ok = !b.failed;
    if (!ok)
        snprintf(err, errcap, "out of memory");
    else
        ok = rg_write_file(path, b.s, b.len, err, errcap);
    rg_buf_free(&b);
    return ok;
}

/* ---- validation ----------------------------------------------------- */

static bool missing(double v)
{
    return isnan(v);
}

bool rg_job_validate(const RgJob *j, char *err, size_t errcap)
{
    #define NEED(cond, ...) do { if (!(cond)) { snprintf(err, errcap, __VA_ARGS__); return false; } } while (0)

    NEED(j->name[0], "name is required: the program's module and file name");
    const RgDialect *dl = rg_dialect_find(j->controller);
    NEED(j->controller[0], "controller is required: s4, s4c or s4c_plus");
    NEED(dl, "controller \"%s\" is not one of s4, s4c, s4c_plus", j->controller);
    NEED(!dl->dos_names || strlen(j->name) <= 8,
         "name \"%s\" is longer than 8 characters, which an %s floppy cannot hold",
         j->name, dl->name);
    NEED(rg_robot_find(j->robot), "robot \"%s\" is not known (rapidgen --robots lists them)",
         j->robot);

    if (j->part == RG_PART_CYLINDER) {
        NEED(j->drawing[0] || j->nbands, "give either drawing = FILE.dxf or one or more band lines");
        NEED(!(j->drawing[0] && j->nbands), "give drawing or band lines, not both");
        NEED(j->nstrokes == 0, "strokes are for flat parts: set part = flat, or remove them");
        NEED(!missing(j->radius), "radius is required: the sprayed surface's radius in mm");
        NEED(!missing(j->part_height), "part_height is required: table to top of the part in mm");
        NEED(!missing(j->axis.x), "axis is required: the rotator axis at table height, x y z in "
                                  "the robot base frame");
        NEED(!missing(j->rpm), "rpm is required: the rotator speed");
    } else {
        NEED(j->nbands == 0, "bands are for cylinders: a flat part is sprayed along strokes");
        NEED(!missing(j->plane.x), "plane is required: where the drawing's origin is on the "
                                   "part, x y z in the robot base frame");
        NEED(!missing(j->spray_speed), "spray_speed is required: the gun's speed along a "
                                       "stroke, in mm/s");
        NEED(j->nstrokes > 0, "there are no strokes: paint the pattern to spray");
    }

    if (j->pattern == RG_PAT_SPOT)
        NEED(!missing(j->spot_diameter), "spot_diameter is required: the circle the gun coats "
                                         "at the standoff, in mm");
    else
        NEED(!missing(j->fan_width), "fan_width is required: the fan's width at the standoff, "
                                     "in mm");
    NEED(!missing(j->standoff), "standoff is required: gun tip to surface in mm");
    NEED(rg_job_step(j) > 0.0, "step_over must be more than nothing");
    NEED(rg_job_step(j) <= rg_job_width(j) + 1e-9,
         "step_over %.2f mm is wider than the %.2f mm the gun covers: the passes would not "
         "meet", rg_job_step(j), rg_job_width(j));
    NEED(j->gun != RG_GUN_SWITCHED || j->gun_signal[0],
         "gun = switched needs gun_signal: the output the program switches the gun with");

    NEED(j->tool[0], "tool is required: the tooldata the program moves");
    NEED(!missing(j->tool_tcp.x), "tool_tcp is required: the gun tip in the flange frame, "
                                  "x y z mm");
    NEED(!j->tool_define || !missing(j->tool_mass),
         "tool_mass is required when tool_define = yes: the controller supervises the "
         "arm's load with it");

    NEED(!j->gun_signal[0] || strcmp(j->gun_signal, j->tool) != 0,
         "gun_signal and tool cannot have the same name");
    return true;
    #undef NEED
}

RgPose rg_job_tool(const RgJob *j)
{
    return rg_pose(rg_m3_from_quat(j->tool_rot), j->tool_tcp);
}

RgPose rg_job_wobj(const RgJob *j)
{
    if (j->part == RG_PART_FLAT)
        return rg_pose(rg_m3_from_quat(j->plane_rot), j->plane);
    double yaw = atan2(-j->axis.y, -j->axis.x);
    return rg_pose(rg_rot_z(yaw), j->axis);
}

bool rg_rapid_ident_ok(const char *s, int max)
{
    static const char *const reserved[] = {
        "ALIAS", "AND", "BACKWARD", "CASE", "CONNECT", "CONST", "DEFAULT", "DIV",
        "DO", "ELSE", "ELSEIF", "ENDFOR", "ENDFUNC", "ENDIF", "ENDMODULE",
        "ENDPROC", "ENDRECORD", "ENDTEST", "ENDTRAP", "ENDWHILE", "ERROR", "EXIT",
        "FALSE", "FOR", "FROM", "FUNC", "GOTO", "IF", "INOUT", "LOCAL", "MOD",
        "MODULE", "NOSTEPIN", "NOT", "NOVIEW", "OR", "PERS", "PROC", "RAISE",
        "READONLY", "RECORD", "RETRY", "RETURN", "STEP", "SYSMODULE", "TEST",
        "THEN", "TO", "TRAP", "TRUE", "TRYNEXT", "UNDO", "VAR", "VIEWONLY",
        "WHILE", "WITH", "XOR",
    };
    size_t n = strlen(s);
    if (n == 0 || (int)n > max || !isalpha((unsigned char)s[0]))
        return false;
    for (size_t i = 0; i < n; i++)
        if (!isalnum((unsigned char)s[i]) && s[i] != '_')
            return false;
    for (size_t i = 0; i < sizeof reserved / sizeof reserved[0]; i++)
        if (rg_streqi(s, reserved[i]))
            return false;
    return true;
}

const char *rg_job_template(void)
{
    return
"# rapidgen job file\n"
"#\n"
"# One cylinder, standing on a rotator that turns continuously on its own.\n"
"# The robot holds the gun square to the surface and traverses it up and down.\n"
"# Millimetres, degrees, rpm. Lines are  key = value ; # starts a comment.\n"
"# For a flat part painted with strokes, see  rapidgen --template flat\n"
"\n"
"# ---- what and where ------------------------------------------------------\n"
"name        = TANK01          # module and file name (8 characters for S4/S4C)\n"
"controller  = s4c_plus        # s4 | s4c | s4c_plus\n"
"robot       = irb2400_16      # irb2400_16 | irb2400_10\n"
"part        = cylinder        # cylinder | flat\n"
"\n"
"# Either a drawing of the unrolled surface: X round the circumference,\n"
"# Y up the part from the table. Every closed outline must go all the way\n"
"# round, because the robot cannot see which way the part is facing.\n"
"#drawing    = tank01.dxf\n"
"#layer      = SPRAY           # leave out to read every layer\n"
"#drawing_scale = 1            # drawing units to millimetres\n"
"# ...or the bands to coat, bottom and top height, one line each:\n"
"band        = 100 900\n"
"\n"
"# ---- the part and the cell -----------------------------------------------\n"
"radius      = 300             # sprayed surface\n"
"part_height = 1000            # table to top of part\n"
"axis        = 1450 0 300      # rotator axis at table height, robot base frame\n"
"azimuth     = 0               # gun position round the part, 0 = facing the robot\n"
"rpm         = 30              # rotator speed\n"
"\n"
"# ---- the process ---------------------------------------------------------\n"
"# A thermal-spray torch lays down a round spot, so the gun's rotation about\n"
"# its own axis does not matter. step_over is the advance per turn of the\n"
"# part; the spot covers each point spot_diameter / step_over times.\n"
"pattern     = spot            # spot (thermal spray) | fan (paint)\n"
"spot_diameter = 12            # the circle coated at the standoff\n"
"step_over   = 6               # between passes; leave out to use overlap\n"
"standoff    = 150             # gun tip to surface\n"
"coats       = 2               # traverses over each band, per cycle\n"
"cycles      = 8               # repeats of the whole pattern, building thickness\n"
"dwell       = 20              # seconds between cycles, to let the part cool\n"
"thickness_per_pass = 25       # microns a single pass lays down, as measured\n"
"target_thickness   = 350      # microns wanted\n"
"start       = top             # top | bottom\n"
"approach    = 150             # radial clearance before and after a band\n"
"accel       = 2000            # assumed robot acceleration, for the run-up\n"
"travel_speed    = 250\n"
"approach_speed  = 100\n"
"max_spray_speed = 1000\n"
"\n"
"# ---- the gun --------------------------------------------------------------\n"
"# The tool's Z axis is the spray direction; its X axis is the long axis of a\n"
"# fan, which a round spot does not use. tool_tcp is the gun tip, not the\n"
"# spray point: the standoff is added by rapidgen.\n"
"tool        = tSprayGun\n"
"tool_define = no              # yes: declare it in the program (needs tool_mass)\n"
"tool_tcp    = 0 -120 180\n"
"tool_rot    = 0.866025 0.5 0 0   # gun tilted 60 deg off the flange axis\n"
"#tool_mass  = 2.5\n"
"#tool_cog   = 0 -40 80\n"
"\n"
"# ---- around the program ---------------------------------------------------\n"
"home         = 0 0 0 0 30 0\n"
"ready_prompt = yes            # stop for the operator before the first traverse\n"
"gun          = continuous     # continuous (a torch) | switched (needs gun_signal)\n"
"#gun_signal  = doGunOn        # the output that switches the gun\n"
"#cool_signal = doCoolAir      # held on through the dwell between cycles\n"
"\n"
"# ---- rules the plan must keep ---------------------------------------------\n"
"min_wrist      = 10           # axis 5 at least this far from straight\n"
"min_margin     = 5            # every joint at least this far from its limit\n"
"clearance      = 50           # wrist, flange and gun tip from the part\n"
"max_joint_step = 20           # more between samples is a flip\n"
"sample_step    = 5            # check every this many mm along a move\n"
"wrap_tolerance = 5            # drawing width vs circumference\n"
"join_tolerance = 0.1\n"
"chord_tolerance = 0.2\n";
}

const char *rg_job_template_flat(void)
{
    return
"# rapidgen job file: a flat part\n"
"#\n"
"# The part lies still. The gun points along the surface normal, `standoff`\n"
"# above it, and follows the strokes in order. The editor paints strokes over\n"
"# a DXF of the part; they can also be written here by hand.\n"
"\n"
"# ---- what and where ------------------------------------------------------\n"
"name        = PANEL01\n"
"controller  = s4c_plus        # s4 | s4c | s4c_plus\n"
"robot       = irb2400_16\n"
"part        = flat\n"
"#drawing    = panel01.dxf     # the outline the strokes were traced over\n"
"drawing_scale = 1             # drawing units to millimetres\n"
"\n"
"# ---- the part -------------------------------------------------------------\n"
"# The drawing's origin on the part surface, in the robot base frame, and the\n"
"# drawing's axes: plane_rot turns the base frame's X, Y, Z onto the drawing's\n"
"# X, Y and the surface normal. 1 0 0 0 is a part lying flat, square to the\n"
"# robot.\n"
"plane       = 800 -300 200\n"
"plane_rot   = 1 0 0 0\n"
"spray_speed = 300             # along a stroke, mm/s\n"
"\n"
"# ---- the process ---------------------------------------------------------\n"
"# A thermal-spray torch lays down a round spot: which way round the gun sits\n"
"# does not matter, and a stroke may run any direction. The coating is built\n"
"# up over `cycles` repeats of the whole pattern.\n"
"pattern     = spot            # spot (thermal spray) | fan (paint)\n"
"spot_diameter = 12            # the circle coated at the standoff\n"
"step_over   = 6               # between passes; the fill tool uses it\n"
"standoff    = 150\n"
"cycles      = 6               # repeats of the whole pattern\n"
"dwell       = 15              # seconds between cycles, to let the part cool\n"
"thickness_per_pass = 25       # microns a single pass lays down, as measured\n"
"target_thickness   = 300      # microns wanted\n"
"#lead       = 70              # run-on and run-off past each stroke's ends;\n"
"                              # left out, it is worked out from speed and\n"
"                              # acceleration so the gun is up to speed on the work\n"
"approach    = 100             # lift before and after each stroke\n"
"accel       = 2000\n"
"travel_speed    = 250\n"
"approach_speed  = 100\n"
"max_spray_speed = 1000\n"
"\n"
"# ---- the gun --------------------------------------------------------------\n"
"tool        = tSprayGun\n"
"tool_define = no\n"
"tool_tcp    = 0 -120 180\n"
"# The gun tilted 60 deg off the flange axis, and turned 90 deg about its own\n"
"# axis on the mount, which keeps axis 5 clear of straight over this part. A\n"
"# round spot does not care how the gun is turned, so that angle is free to be\n"
"# chosen for the wrist.\n"
"tool_rot    = 0.612372 0.353553 -0.353553 0.612372\n"
"\n"
"# ---- around the program ---------------------------------------------------\n"
"home         = 0 0 0 0 30 0\n"
"ready_prompt = yes\n"
"gun          = continuous     # a torch cannot be switched stroke by stroke\n"
"#gun_signal  = doGunOn        # only for gun = switched\n"
"#cool_signal = doCoolAir      # held on through the dwell between cycles\n"
"\n"
"# ---- the painted strokes, in order: x y pairs in drawing mm ----------------\n"
"stroke = 50 50  550 50  550 130  50 130\n"
"stroke = 50 300  550 300\n";
}
