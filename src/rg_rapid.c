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
#include "rg_rapid.h"
#include "rg_plan.h"
#include "rg_version.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Assumed until checked against saved programs:
 *   - all three read a text program with the %%% VERSION:1 LANGUAGE:ENGLISH
 *     %%% header, one MODULE per file, from a .PRG file;
 *   - S4C and the original S4 load from floppy, so 8.3 names;
 *   - MoveAbsJ arrived with BaseWare 3, so the original S4 moves home with a
 *     MoveJ to a robtarget under configuration monitoring instead.
 */
static const RgDialect dialects[] = {
    { "s4",       "S4",   ".PRG", true,  false },
    { "s4c",      "S4C",  ".PRG", true,  true  },
    { "s4c_plus", "S4C+", ".PRG", false, true  },
};

#define N_DIALECTS ((int)(sizeof dialects / sizeof dialects[0]))

#define HOME_JOINTS "jRgHome"
#define HOME_TARGET "pRgHome"
#define MAX_TARGETS 9999

int rg_dialect_count(void)
{
    return N_DIALECTS;
}

const RgDialect *rg_dialect_at(int i)
{
    return (i >= 0 && i < N_DIALECTS) ? &dialects[i] : NULL;
}

const RgDialect *rg_dialect_find(const char *id)
{
    for (int i = 0; i < N_DIALECTS; i++)
        if (rg_streqi(dialects[i].id, id))
            return &dialects[i];
    return NULL;
}

void rg_rapid_filename(const RgJob *j, char *buf, size_t cap)
{
    const RgDialect *d = rg_dialect_find(j->controller);
    char name[RG_IDENT_CAP];
    rg_copy(name, sizeof name, j->name);
    if (d && d->dos_names)
        for (char *p = name; *p; p++)
            *p = (char)toupper((unsigned char)*p);
    snprintf(buf, cap, "%s%s", name, d ? d->ext : ".PRG");
}

/* ---- formatting ----------------------------------------------------- */

/*
 * Quaternions printed to 6 places and then made exactly unit length again,
 * by recomputing the largest component from the others: the controller
 * rejects an orientation that is not normalised, and rounding four
 * components independently can leave it just outside tolerance.
 */
static void quat_text(char *buf, size_t cap, RgQuat q)
{
    double v[4] = { q.q1, q.q2, q.q3, q.q4 };
    int big = 0;
    for (int i = 0; i < 4; i++) {
        v[i] = round(v[i] * 1e6) / 1e6;
        if (fabs(v[i]) > fabs(v[big]))
            big = i;
    }
    double rest = 0.0;
    for (int i = 0; i < 4; i++)
        if (i != big)
            rest += v[i] * v[i];
    double mag = sqrt(fmax(0.0, 1.0 - rest));
    v[big] = copysign(round(mag * 1e6) / 1e6, v[big]);

    char a[32], b[32], c[32], d[32];
    snprintf(buf, cap, "[%s,%s,%s,%s]", rg_fmt(a, sizeof a, v[0], 6), rg_fmt(b, sizeof b, v[1], 6),
             rg_fmt(c, sizeof c, v[2], 6), rg_fmt(d, sizeof d, v[3], 6));
}

static void vec_text(char *buf, size_t cap, RgVec3 p, int dp)
{
    char a[32], b[32], c[32];
    snprintf(buf, cap, "[%s,%s,%s]", rg_fmt(a, sizeof a, p.x, dp), rg_fmt(b, sizeof b, p.y, dp),
             rg_fmt(c, sizeof c, p.z, dp));
}

static void robtarget_text(char *buf, size_t cap, const RgPose *p, const int cf[4])
{
    char pos[96], q[96];
    vec_text(pos, sizeof pos, p->pos, 2);
    quat_text(q, sizeof q, rg_quat_from_m3(p->rot));
    snprintf(buf, cap, "[%s,%s,[%d,%d,%d,%d],[9E9,9E9,9E9,9E9,9E9,9E9]]",
             pos, q, cf[0], cf[1], cf[2], cf[3]);
}

/* ---- targets -------------------------------------------------------- */

typedef struct {
    char (*text)[256];
    int  n;
} Targets;

/* The target's number, adding it if it is new: identical points share one
 * name, which keeps the program short on a controller with little memory. */
static int target_index(Targets *t, const char *text)
{
    for (int i = 0; i < t->n; i++)
        if (strcmp(t->text[i], text) == 0)
            return i;
    rg_copy(t->text[t->n], sizeof t->text[0], text);
    return t->n++;
}

static const char *speed_name(RgSpeedKind s)
{
    switch (s) {
    case RG_SPD_SPRAY:    return "vRgSpray";
    case RG_SPD_APPROACH: return "vRgApproach";
    default:              return "vRgTravel";
    }
}

static void speeddata(RgBuf *b, const char *name, double v)
{
    char n[32];
    rg_buf_printf(b, "  CONST speeddata %s:=[%s,500,5000,1000];\n", name, rg_fmt(n, sizeof n, v, 2));
}

bool rg_rapid_write(const RgJob *j, const RgPlan *pl, const char *source,
                    const char *stamp, RgBuf *out, char *err, size_t errcap)
{
    const RgDialect *d = rg_dialect_find(j->controller);
    if (!d) {
        snprintf(err, errcap, "controller \"%s\" is not known", j->controller);
        return false;
    }
    if (pl->refused) {
        snprintf(err, errcap, "the plan was refused; no program is written");
        return false;
    }

    Targets t;
    t.n = 0;
    t.text = malloc((size_t)(pl->nmoves ? pl->nmoves : 1) * sizeof *t.text);
    int *index = malloc((size_t)(pl->nmoves ? pl->nmoves : 1) * sizeof *index);
    if (!t.text || !index) {
        free(t.text);
        free(index);
        snprintf(err, errcap, "out of memory");
        return false;
    }
    for (int i = 0; i < pl->nmoves; i++) {
        index[i] = -1;
        if (pl->moves[i].kind != RG_MV_HOME) {
            char text[256];
            robtarget_text(text, sizeof text, &pl->moves[i].tcp, pl->moves[i].cf);
            index[i] = target_index(&t, text);
        }
    }
    if (t.n > MAX_TARGETS) {
        snprintf(err, errcap, "the program would need %d targets, more than %d", t.n, MAX_TARGETS);
        free(t.text);
        free(index);
        return false;
    }

    char a[256], q[96], n1[32], n2[32], n3[32];
    const char *tool = j->tool;
    const char *wobj = j->part == RG_PART_FLAT ? "wRgPart" : "wRgCylinder";

    rg_buf_puts(out, "%%%\n  VERSION:1\n  LANGUAGE:ENGLISH\n%%%\n\n");
    rg_buf_printf(out, "MODULE %s\n", j->name);
    rg_buf_printf(out, "  ! Written by %s %s for an ABB %s\n", RAPIDGEN_NAME, RAPIDGEN_VERSION, d->name);
    rg_buf_printf(out, "  ! From %s, %s\n", source, stamp);
    if (j->part == RG_PART_FLAT) {
        rg_buf_printf(out, "  ! Flat part: %d stroke%s, %s mm at %s mm/s\n", j->nstrokes,
                      j->nstrokes == 1 ? "" : "s", rg_fmt(n1, sizeof n1, pl->stroke_length, 0),
                      rg_fmt(n2, sizeof n2, pl->spray_speed, 1));
        rg_buf_printf(out, "  ! Fan %s mm at %s mm standoff\n",
                      rg_fmt(n1, sizeof n1, j->fan_width, 1), rg_fmt(n2, sizeof n2, j->standoff, 1));
    } else {
        rg_buf_printf(out, "  ! Cylinder radius %s mm on a rotator at %s rpm\n",
                      rg_fmt(n1, sizeof n1, j->radius, 1), rg_fmt(n2, sizeof n2, j->rpm, 1));
        rg_buf_printf(out, "  ! Fan %s mm at %s mm standoff, pitch %s mm per turn\n",
                      rg_fmt(n1, sizeof n1, j->fan_width, 1), rg_fmt(n2, sizeof n2, j->standoff, 1),
                      rg_fmt(n3, sizeof n3, pl->pitch, 1));
    }
    rg_buf_puts(out, "  ! Not proven on a robot. Check it in simulation, then step\n"
                     "  ! through in manual reduced speed before running it.\n");

    if (j->tool_define) {
        vec_text(a, sizeof a, j->tool_tcp, 2);
        quat_text(q, sizeof q, j->tool_rot);
        char cog[96];
        vec_text(cog, sizeof cog, j->tool_cog, 1);
        rg_buf_printf(out, "  PERS tooldata %s:=[TRUE,[%s,%s],[%s,%s,[1,0,0,0],0,0,0]];\n",
                      tool, a, q, rg_fmt(n1, sizeof n1, j->tool_mass, 2), cog);
    }
    RgPose w = rg_job_wobj(j);
    vec_text(a, sizeof a, w.pos, 2);
    quat_text(q, sizeof q, rg_quat_from_m3(w.rot));
    rg_buf_printf(out, "  PERS wobjdata %s:=[FALSE,TRUE,\"\",[%s,%s],[[0,0,0],[1,0,0,0]]];\n",
                  wobj, a, q);

    speeddata(out, "vRgSpray", pl->spray_speed);
    speeddata(out, "vRgApproach", j->approach_speed);
    speeddata(out, "vRgTravel", j->travel_speed);

    if (d->moveabsj) {
        char h[6][32];
        for (int i = 0; i < 6; i++)
            rg_fmt(h[i], sizeof h[i], j->home[i], 2);
        rg_buf_printf(out, "  CONST jointtarget " HOME_JOINTS ":=[[%s,%s,%s,%s,%s,%s],"
                      "[9E9,9E9,9E9,9E9,9E9,9E9]];\n", h[0], h[1], h[2], h[3], h[4], h[5]);
    } else {
        robtarget_text(a, sizeof a, &pl->home_tcp, pl->home_cf);
        rg_buf_printf(out, "  CONST robtarget " HOME_TARGET ":=%s;\n", a);
    }
    for (int i = 0; i < t.n; i++)
        rg_buf_printf(out, "  CONST robtarget pRg%04d:=%s;\n", i + 1, t.text[i]);

    bool prompt = false;
    for (int i = 0; i < pl->nmoves; i++)
        prompt |= pl->moves[i].after == RG_ACT_READY;
    bool loop = pl->cycles > 1;

    rg_buf_puts(out, "\n  PROC main()\n");
    if (prompt)
        rg_buf_puts(out, "    VAR num nKey;\n");
    if (loop)
        rg_buf_puts(out, "    VAR num nCycle;\n");
    if (prompt || loop)
        rg_buf_puts(out, "\n");
    rg_buf_puts(out, "    ConfJ \\Off;\n    ConfL \\Off;\n");

    /* The pattern is written once and repeated, rather than its targets
     * written out again per cycle: a coating can take dozens of cycles, and
     * an S4's program memory is not large. */
    const char *in = loop ? "    " : "";
    for (int i = 0; i < pl->nmoves; i++) {
        const RgMove *m = &pl->moves[i];
        const char *zone = m->zone == RG_Z_FINE ? "fine" : m->zone == RG_Z_SMALL ? "z1" : "z10";
        if (loop && i == 1) {
            char n[32];
            rg_buf_printf(out, "    ! %s cycles, building the coating up\n",
                          rg_fmt(n, sizeof n, (double)pl->cycles, 0));
            rg_buf_printf(out, "    FOR nCycle FROM 1 TO %d DO\n", pl->cycles);
        }
        if (loop && m->kind == RG_MV_HOME && i > 0) {
            if (j->dwell > 0.0) {
                char secs[32];
                if (j->cool_signal[0])
                    rg_buf_printf(out, "      SetDO %s,1;\n", j->cool_signal);
                rg_buf_printf(out, "      WaitTime %s;\n", rg_fmt(secs, sizeof secs, j->dwell, 2));
                if (j->cool_signal[0])
                    rg_buf_printf(out, "      SetDO %s,0;\n", j->cool_signal);
            }
            rg_buf_puts(out, "    ENDFOR\n");
        }
        if (m->note[0])
            rg_buf_printf(out, "%s    ! %s\n", loop && i > 0 && i < pl->nmoves - 1 ? "  " : "",
                          m->note);
        (void)in;
        switch (m->kind) {
        case RG_MV_HOME:
            if (d->moveabsj) {
                rg_buf_printf(out, "    MoveAbsJ " HOME_JOINTS ",%s,fine,%s;\n",
                              speed_name(m->speed), tool);
            } else {
                rg_buf_printf(out, "    ConfJ \\On;\n    MoveJ " HOME_TARGET ",%s,fine,%s\\WObj:=%s;\n"
                              "    ConfJ \\Off;\n", speed_name(m->speed), tool, wobj);
            }
            break;
        case RG_MV_JOINT:
        case RG_MV_LINEAR:
            rg_buf_printf(out, "    %s pRg%04d,%s,%s,%s\\WObj:=%s;\n",
                          m->kind == RG_MV_JOINT ? "MoveJ" : "MoveL", index[i] + 1,
                          speed_name(m->speed), zone, tool, wobj);
            break;
        }
        switch (m->after) {
        case RG_ACT_READY:
            rg_buf_puts(out, "    TPReadFK nKey,\"Rotator turning and gun ready?\",\"\",\"\",\"\",\"\",\"Go\";\n");
            if (j->gun_signal[0])
                rg_buf_printf(out, "    SetDO %s,1;\n", j->gun_signal);
            break;
        case RG_ACT_GUN_ON:
            rg_buf_printf(out, "    SetDO %s,1;\n", j->gun_signal);
            break;
        case RG_ACT_GUN_OFF:
            rg_buf_printf(out, "    SetDO %s,0;\n", j->gun_signal);
            break;
        case RG_ACT_NONE:
            break;
        }
    }
    rg_buf_puts(out, "    ConfJ \\On;\n    ConfL \\On;\n  ENDPROC\nENDMODULE\n");

    free(t.text);
    free(index);
    if (out->failed) {
        snprintf(err, errcap, "out of memory");
        return false;
    }
    return true;
}
