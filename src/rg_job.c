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
#include "rg_text.h"

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

    j->radius = UNSET;
    j->part_height = UNSET;
    j->axis = rg_v3(UNSET, UNSET, UNSET);
    j->azimuth = 0.0;
    j->rpm = UNSET;

    j->fan_width = UNSET;
    j->overlap = 50.0;
    j->standoff = UNSET;
    j->coats = 1;
    j->start_top = true;
    j->accel = 500.0;
    j->approach = 150.0;
    j->travel_speed = 250.0;
    j->approach_speed = 100.0;
    j->max_spray_speed = 500.0;

    j->tool_define = false;
    j->tool_tcp = rg_v3(UNSET, UNSET, UNSET);
    j->tool_rot.q1 = 1.0;
    j->tool_mass = UNSET;

    j->home[4] = 30.0;
    j->ready_prompt = true;

    j->min_wrist = 10.0;
    j->min_margin = 5.0;
    j->clearance = 50.0;
    j->max_joint_step = 20.0;
    j->sample_step = 5.0;
    j->wrap_tolerance = 5.0;
    j->join_tolerance = 0.1;
    j->chord_tolerance = 0.2;
}

/* ---- the key table -------------------------------------------------- */

typedef enum { K_IDENT, K_TEXT, K_NUM, K_INT, K_BOOL, K_VEC3, K_QUAT, K_JOINTS, K_BAND, K_START } KType;

typedef struct {
    const char *key;
    KType       type;
    size_t      off, cap;
    double      lo, hi;
} Key;

#define F(field)          offsetof(RgJob, field)
#define S(field)          sizeof(((RgJob *)0)->field)

static const Key keys[] = {
    { "name",            K_IDENT,  F(name),            S(name),         0, 0 },
    { "controller",      K_TEXT,   F(controller),      S(controller),   0, 0 },
    { "robot",           K_TEXT,   F(robot),           S(robot),        0, 0 },
    { "drawing",         K_TEXT,   F(drawing),         S(drawing),      0, 0 },
    { "layer",           K_TEXT,   F(layer),           S(layer),        0, 0 },
    { "band",            K_BAND,   0,                  0,               -100000, 100000 },

    { "radius",          K_NUM,    F(radius),          0,               10, 5000 },
    { "part_height",     K_NUM,    F(part_height),     0,               1, 10000 },
    { "axis",            K_VEC3,   F(axis),            0,               -10000, 10000 },
    { "azimuth",         K_NUM,    F(azimuth),         0,               -90, 90 },
    { "rpm",             K_NUM,    F(rpm),             0,               0.1, 300 },

    { "fan_width",       K_NUM,    F(fan_width),       0,               5, 1000 },
    { "overlap",         K_NUM,    F(overlap),         0,               0, 90 },
    { "standoff",        K_NUM,    F(standoff),        0,               10, 1000 },
    { "coats",           K_INT,    F(coats),           0,               1, 50 },
    { "start",           K_START,  F(start_top),       0,               0, 0 },
    { "accel",           K_NUM,    F(accel),           0,               10, 20000 },
    { "approach",        K_NUM,    F(approach),        0,               20, 1000 },
    { "travel_speed",    K_NUM,    F(travel_speed),    0,               10, 2000 },
    { "approach_speed",  K_NUM,    F(approach_speed),  0,               5, 1000 },
    { "max_spray_speed", K_NUM,    F(max_spray_speed), 0,               1, 2000 },

    { "tool",            K_IDENT,  F(tool),            S(tool),         0, 0 },
    { "tool_define",     K_BOOL,   F(tool_define),     0,               0, 0 },
    { "tool_tcp",        K_VEC3,   F(tool_tcp),        0,               -2000, 2000 },
    { "tool_rot",        K_QUAT,   F(tool_rot),        0,               -1, 1 },
    { "tool_mass",       K_NUM,    F(tool_mass),       0,               0.01, 50 },
    { "tool_cog",        K_VEC3,   F(tool_cog),        0,               -2000, 2000 },

    { "home",            K_JOINTS, F(home),            0,               -400, 400 },
    { "ready_prompt",    K_BOOL,   F(ready_prompt),    0,               0, 0 },
    { "gun_signal",      K_IDENT,  F(gun_signal),      S(gun_signal),   0, 0 },

    { "min_wrist",       K_NUM,    F(min_wrist),       0,               0, 60 },
    { "min_margin",      K_NUM,    F(min_margin),      0,               0, 45 },
    { "clearance",       K_NUM,    F(clearance),       0,               0, 1000 },
    { "max_joint_step",  K_NUM,    F(max_joint_step),  0,               1, 90 },
    { "sample_step",     K_NUM,    F(sample_step),     0,               0.5, 100 },
    { "wrap_tolerance",  K_NUM,    F(wrap_tolerance),  0,               0, 100 },
    { "join_tolerance",  K_NUM,    F(join_tolerance),  0,               0.001, 10 },
    { "chord_tolerance", K_NUM,    F(chord_tolerance), 0,               0.01, 5 },
};

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

/* Up to `max` numbers separated by spaces or commas; how many were read, or
 * -1 for something that is not a number. */
static int numbers(const char *v, double *out, int max)
{
    int n = 0;
    const char *p = v;
    for (;;) {
        while (*p == ' ' || *p == '\t' || *p == ',')
            p++;
        if (!*p)
            return n;
        if (n == max)
            return -1;
        char *endp;
        double d = strtod(p, &endp);
        if (endp == p || !isfinite(d))
            return -1;
        out[n++] = d;
        p = endp;
    }
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
        if (numbers(v, d, 1) != 1) {
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

    case K_VEC3:
        if (numbers(v, d, 3) != 3) {
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
        if (numbers(v, d, 4) != 4) {
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
        if (numbers(v, d, 6) != 6) {
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
        if (numbers(v, d, 2) != 2 || d[1] <= d[0]) {
            snprintf(why, whycap, "expected two heights, bottom then top");
            return false;
        }
        j->bands[j->nbands].y0 = d[0];
        j->bands[j->nbands].y1 = d[1];
        j->nbands++;
        return true;
    }
    return false;
}

bool rg_job_parse(RgJob *j, const char *text, char *err, size_t errcap)
{
    int lineno = 0;
    const char *p = text;
    while (*p) {
        const char *nl = strchr(p, '\n');
        size_t n = nl ? (size_t)(nl - p) : strlen(p);
        char line[1024];
        lineno++;
        if (n >= sizeof line) {
            snprintf(err, errcap, "line %d: too long", lineno);
            return false;
        }
        memcpy(line, p, n);
        line[n] = '\0';
        p = nl ? nl + 1 : p + n;

        char *hash = strchr(line, '#');
        if (hash)
            *hash = '\0';
        char *s = trim(line);
        if (!*s)
            continue;
        char *eq = strchr(s, '=');
        if (!eq) {
            snprintf(err, errcap, "line %d: expected key = value", lineno);
            return false;
        }
        *eq = '\0';
        char *key = trim(s), *val = trim(eq + 1);

        const Key *k = NULL;
        for (size_t i = 0; i < sizeof keys / sizeof keys[0]; i++)
            if (strcmp(keys[i].key, key) == 0)
                k = &keys[i];
        if (!k) {
            snprintf(err, errcap, "line %d: unknown setting \"%s\"", lineno, key);
            return false;
        }
        char why[256];
        if (!set_value(j, k, val, why, sizeof why)) {
            snprintf(err, errcap, "line %d: %s: %s", lineno, key, why);
            return false;
        }
    }
    return true;
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

    NEED(j->drawing[0] || j->nbands, "give either drawing = FILE.dxf or one or more band lines");
    NEED(!(j->drawing[0] && j->nbands), "give drawing or band lines, not both");

    NEED(!missing(j->radius), "radius is required: the sprayed surface's radius in mm");
    NEED(!missing(j->part_height), "part_height is required: table to top of the part in mm");
    NEED(!missing(j->axis.x), "axis is required: the rotator axis at table height, x y z in "
                              "the robot base frame");
    NEED(!missing(j->rpm), "rpm is required: the rotator speed");
    NEED(!missing(j->fan_width), "fan_width is required: the spray pattern's width at the "
                                 "standoff, in mm");
    NEED(!missing(j->standoff), "standoff is required: gun tip to surface in mm");

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
"\n"
"# ---- what and where ------------------------------------------------------\n"
"name        = TANK01          # module and file name (8 characters for S4/S4C)\n"
"controller  = s4c_plus        # s4 | s4c | s4c_plus\n"
"robot       = irb2400_16      # irb2400_16 | irb2400_10\n"
"\n"
"# Either a drawing of the unrolled surface: X round the circumference,\n"
"# Y up the part from the table. Every closed outline must go all the way\n"
"# round, because the robot cannot see which way the part is facing.\n"
"#drawing    = tank01.dxf\n"
"#layer      = SPRAY           # leave out to read every layer\n"
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
"fan_width   = 100             # spray pattern width at the standoff\n"
"overlap     = 50              # percent of the fan width covered again each turn\n"
"standoff    = 250             # gun tip to surface\n"
"coats       = 2               # traverses over each band\n"
"start       = top             # top | bottom\n"
"approach    = 150             # radial clearance before and after a band\n"
"accel       = 500             # assumed robot acceleration, for the run-up\n"
"travel_speed    = 250\n"
"approach_speed  = 100\n"
"max_spray_speed = 500\n"
"\n"
"# ---- the gun --------------------------------------------------------------\n"
"# The tool's Z axis is the spray direction and its X axis the long axis of\n"
"# the fan. tool_tcp is the gun tip, not the spray point: the standoff is\n"
"# added by rapidgen.\n"
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
"#gun_signal  = doGunOn        # a digital output; required for two or more bands\n"
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
