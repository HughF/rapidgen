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
#include "rg_plan.h"
#include "rg_text.h"

#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* One point failing usually means its neighbours fail the same way; after
 * this many of a kind the rest are counted, not listed. */
#define MAX_LISTED 3

typedef enum { KIND_REACH, KIND_FLIP, KIND_CLEAR, KIND_COUNT } IssueKind;

typedef struct {
    const RgJob   *job;
    const RgRobot *robot;
    RgPlan        *pl;
    RgPose         wobj, wobj_inv, tool, tool_inv;
    double         joints[RG_AXES];
    int            counts[KIND_COUNT];
    bool           oom;
} Ctx;

static void issue(Ctx *c, RgSeverity sev, const char *fmt, ...) RG_PRINTF(3, 4);

static void issue(Ctx *c, RgSeverity sev, const char *fmt, ...)
{
    RgPlan *pl = c->pl;
    if (sev == RG_REFUSE)
        pl->refused = true;
    RgIssue *is = realloc(pl->issues, (size_t)(pl->nissues + 1) * sizeof *is);
    if (!is) {
        c->oom = true;
        pl->refused = true;
        return;
    }
    pl->issues = is;
    RgIssue *i = &is[pl->nissues++];
    i->sev = sev;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(i->text, sizeof i->text, fmt, ap);
    va_end(ap);
}

static bool listed(Ctx *c, IssueKind k)
{
    return c->counts[k]++ < MAX_LISTED;
}

int rg_plan_count(const RgPlan *pl, RgSeverity sev)
{
    int n = 0;
    for (int i = 0; i < pl->nissues; i++)
        n += pl->issues[i].sev == sev;
    return n;
}

void rg_plan_free(RgPlan *pl)
{
    free(pl->moves);
    free(pl->issues);
    memset(pl, 0, sizeof *pl);
}

/* ---- bands ---------------------------------------------------------- */

static int band_cmp(const void *pa, const void *pb)
{
    const RgBand *a = pa, *b = pb;
    return (a->y0 > b->y0) - (a->y0 < b->y0);
}

static void add_band(Ctx *c, double y0, double y1)
{
    RgPlan *pl = c->pl;
    if (pl->nbands && y0 <= pl->bands[pl->nbands - 1].y1 + 1e-6) {
        if (y1 > pl->bands[pl->nbands - 1].y1)
            pl->bands[pl->nbands - 1].y1 = y1;
        return;
    }
    if (pl->nbands == RG_MAX_BANDS) {
        issue(c, RG_REFUSE, "the drawing has more than %d separate bands", RG_MAX_BANDS);
        return;
    }
    pl->bands[pl->nbands].y0 = y0;
    pl->bands[pl->nbands].y1 = y1;
    pl->nbands++;
}

static void flush_partial(Ctx *c, bool *open, double y0, double y1, double lo, double hi)
{
    if (!*open)
        return;
    *open = false;
    double C = c->pl->circumference;
    issue(c, RG_REFUSE,
          "between heights %.1f and %.1f mm the drawing covers only %.0f-%.0f %% of the "
          "way round. A part turning continuously can only be sprayed in full bands: "
          "draw this as a band %.1f mm wide, or leave it out",
          y0, y1, 100.0 * lo / C, 100.0 * hi / C, C);
}

static void bands_from_shape(Ctx *c, const RgShape *s)
{
    const RgJob *j = c->job;
    double C = c->pl->circumference, tol = j->wrap_tolerance;

    RgPt hole;
    if (rg_shape_find_hole(s, &hole)) {
        issue(c, RG_REFUSE, "the outline through (%.1f, %.1f) lies inside another. Holes "
              "cannot be left bare on a part turning continuously", hole.x, hole.y);
        return;
    }

    RgSpanRun *runs;
    int n = rg_shape_runs(s, &runs);
    if (n < 0) {
        c->oom = true;
        issue(c, RG_REFUSE, "out of memory");
        return;
    }

    double widest = 0.0, widest_y = 0.0;
    bool partial = false;
    double p0 = 0, p1 = 0, plo = 0, phi = 0;

    for (int i = 0; i < n; i++) {
        const RgSpanRun *r = &runs[i];
        double lo = fmin(r->span0, r->span1), hi = fmax(r->span0, r->span1);
        if (hi > widest) {
            widest = hi;
            widest_y = r->span0 > r->span1 ? r->y0 : r->y1;
        }
        bool full = lo >= C - tol;
        bool empty = hi <= tol;
        if (!full && !empty) {
            if (!partial) {
                partial = true;
                p0 = r->y0;
                plo = lo;
                phi = hi;
            }
            p1 = r->y1;
            plo = fmin(plo, lo);
            phi = fmax(phi, hi);
            continue;
        }
        flush_partial(c, &partial, p0, p1, plo, phi);
        if (full)
            add_band(c, r->y0, r->y1);
    }
    flush_partial(c, &partial, p0, p1, plo, phi);
    free(runs);

    if (widest > C + tol)
        issue(c, RG_REFUSE, "at height %.1f mm the drawing is %.1f mm round, but a %.1f mm "
              "radius is only %.1f mm round (tolerance %.1f mm): the drawing and the radius "
              "do not describe the same part", widest_y, widest, j->radius, C, tol);
}

/* ---- geometry ------------------------------------------------------- */

/* The gun square to the surface, `out` beyond the standoff, at height z.
 * Tool Z points at the axis; tool X, the fan's long axis, points up the part,
 * across the direction the surface moves under the gun. */
static RgPose gun_pose(const RgJob *j, double z, double out)
{
    double th = j->azimuth * RG_DEG;
    RgVec3 u = rg_v3(cos(th), sin(th), 0.0);
    RgVec3 tz = rg_v3_scale(u, -1.0);
    RgVec3 tx = rg_v3(0.0, 0.0, 1.0);
    RgVec3 ty = rg_v3_cross(tz, tx);
    return rg_pose(rg_m3_from_cols(tx, ty, tz),
                   rg_v3_add(rg_v3_scale(u, j->radius + j->standoff + out), rg_v3(0, 0, z)));
}

static RgPose tool0_for(const Ctx *c, const RgPose *tcp)
{
    return rg_pose_mul(rg_pose_mul(c->wobj, *tcp), c->tool_inv);
}

static RgMove *add_move(Ctx *c, RgMoveKind kind, RgSpeedKind speed, bool fine,
                        RgPose tcp, int band, const char *note)
{
    RgPlan *pl = c->pl;
    RgMove *m = realloc(pl->moves, (size_t)(pl->nmoves + 1) * sizeof *m);
    if (!m) {
        c->oom = true;
        return NULL;
    }
    pl->moves = m;
    RgMove *mv = &m[pl->nmoves++];
    memset(mv, 0, sizeof *mv);
    mv->kind = kind;
    mv->speed = speed;
    mv->fine = fine;
    mv->tcp = tcp;
    mv->band = band;
    rg_copy(mv->note, sizeof mv->note, note ? note : "");
    return mv;
}

static void build_moves(Ctx *c)
{
    const RgJob *j = c->job;
    RgPlan *pl = c->pl;
    RgPose none = rg_pose(rg_m3_identity(), rg_v3(0, 0, 0));

    add_move(c, RG_MV_HOME, RG_SPD_TRAVEL, true, none, -1, "Home");
    for (int b = 0; b < pl->nbands && !c->oom; b++) {
        const RgBand *bd = &pl->bands[b];
        double lo = bd->y0 - pl->overrun, hi = bd->y1 + pl->overrun;
        double z_start = j->start_top ? hi : lo, z_other = j->start_top ? lo : hi;
        char note[80];
        snprintf(note, sizeof note, "Band %d: %.1f-%.1f mm, %d coat%s", b + 1, bd->y0, bd->y1,
                 j->coats, j->coats == 1 ? "" : "s");

        add_move(c, RG_MV_JOINT, RG_SPD_TRAVEL, false, gun_pose(j, z_start, j->approach), b, note);
        RgMove *in = add_move(c, RG_MV_LINEAR, RG_SPD_APPROACH, true, gun_pose(j, z_start, 0), b, NULL);
        if (!in)
            return;
        if (b == 0 && j->ready_prompt)
            in->after = RG_ACT_READY;
        else if (j->gun_signal[0])
            in->after = RG_ACT_GUN_ON;

        double z = z_start;
        RgMove *last = in;
        for (int k = 0; k < j->coats; k++) {
            z = (k % 2 == 0) ? z_other : z_start;
            last = add_move(c, RG_MV_LINEAR, RG_SPD_SPRAY, true, gun_pose(j, z, 0), b, NULL);
            if (!last)
                return;
        }
        if (j->gun_signal[0])
            last->after = RG_ACT_GUN_OFF;
        add_move(c, RG_MV_LINEAR, RG_SPD_APPROACH, false, gun_pose(j, z, j->approach), b, NULL);
    }
    add_move(c, RG_MV_HOME, RG_SPD_TRAVEL, true, none, -1, "Home");
}

/* ---- following the moves through the arm ---------------------------- */

/* Where on the program an arm position is, as a phrase that reads after a
 * finding: "...comes within 40 mm of the part surface <where>". `between`
 * is a position part-way along a joint move rather than its target. */
static void where(const Ctx *c, const RgMove *m, const RgPose *tcp, bool between,
                  char *buf, size_t cap)
{
    switch (m->kind) {
    case RG_MV_HOME:
        snprintf(buf, cap, between ? "on the way home" : "at the home position");
        return;
    case RG_MV_JOINT:
        if (between)
            snprintf(buf, cap, "on the way to band %d", m->band + 1);
        else
            snprintf(buf, cap, "at the approach to band %d, gun at height %.1f mm",
                     m->band + 1, tcp->pos.z);
        return;
    case RG_MV_LINEAR: {
        const RgBand *b = &c->pl->bands[m->band];
        snprintf(buf, cap, "in band %d (%.0f-%.0f mm), gun at height %.1f mm",
                 m->band + 1, b->y0, b->y1, tcp->pos.z);
        return;
    }
    }
}

/* Wrist centre, flange and gun tip against the part. */
static void check_clearance(Ctx *c, const RgMove *m, const double q[RG_AXES],
                            const RgPose *tcp_w, bool between)
{
    const RgJob *j = c->job;
    RgPose t0;
    RgVec3 wc;
    rg_robot_fk(c->robot, q, &t0, &wc);
    RgVec3 tip = rg_pose_apply(rg_pose_mul(t0, c->tool), rg_v3(0, 0, 0));
    RgVec3 pts[3] = { wc, t0.pos, tip };
    static const char *const names[3] = { "wrist centre", "flange", "gun tip" };

    for (int i = 0; i < 3; i++) {
        RgVec3 p = rg_pose_apply(c->wobj_inv, pts[i]);
        if (p.z < -j->clearance || p.z > j->part_height + j->clearance)
            continue;
        double gap = hypot(p.x, p.y) - j->radius;
        if (gap < c->pl->min_clearance) {
            c->pl->min_clearance = gap;
            char at[96];
            where(c, m, tcp_w, between, at, sizeof at);
            snprintf(c->pl->min_clearance_at, sizeof c->pl->min_clearance_at, "%s %s",
                     names[i], at);
        }
        if (gap < j->clearance && listed(c, KIND_CLEAR)) {
            char at[96];
            where(c, m, tcp_w, between, at, sizeof at);
            issue(c, RG_REFUSE, "the %s comes within %.0f mm of the part surface %s "
                  "(clearance %.0f mm)", names[i], gap, at, j->clearance);
        }
    }
}

static void record(Ctx *c, const RgMove *m, const double q[RG_AXES], const RgPose *tcp,
                   bool between)
{
    RgPlan *pl = c->pl;
    pl->samples++;
    if (fabs(q[4]) < pl->min_wrist) {
        pl->min_wrist = fabs(q[4]);
        where(c, m, tcp, between, pl->min_wrist_at, sizeof pl->min_wrist_at);
    }
    int axis;
    double margin = rg_robot_margin(c->robot, q, &axis);
    if (margin < pl->min_margin) {
        pl->min_margin = margin;
        pl->min_margin_axis = axis;
        where(c, m, tcp, between, pl->min_margin_at, sizeof pl->min_margin_at);
    }
    check_clearance(c, m, q, tcp, between);
}

/* The arm at one Cartesian point, starting from where it is. */
static bool reach(Ctx *c, const RgMove *m, const RgPose *tcp)
{
    RgPose t0 = tool0_for(c, tcp);
    double q[RG_AXES];
    RgIkResult res;
    char at[96];

    if (rg_robot_ik_near(c->robot, &t0, c->joints, q, &res) != RG_IK_OK) {
        if (listed(c, KIND_REACH)) {
            where(c, m, tcp, false, at, sizeof at);
            if (res.status == RG_IK_OUT_OF_REACH)
                issue(c, RG_REFUSE, "the robot cannot reach the point %s: the wrist centre is "
                      "%.0f mm outside its reach", at, res.shortfall_mm);
            else
                issue(c, RG_REFUSE, "the robot cannot reach the point %s without axis %d going "
                      "%.1f deg past its limit", at, res.axis + 1, res.over_deg);
        }
        return false;
    }

    double step = 0.0;
    int step_axis = 0;
    for (int a = 0; a < RG_AXES; a++)
        if (fabs(q[a] - c->joints[a]) > step) {
            step = fabs(q[a] - c->joints[a]);
            step_axis = a;
        }
    if (m->kind == RG_MV_LINEAR && step > c->job->max_joint_step && listed(c, KIND_FLIP)) {
        where(c, m, tcp, false, at, sizeof at);
        issue(c, RG_REFUSE, "axis %d turns %.0f deg in one %.1f mm step %s: the arm would "
              "flip configuration in the middle of a straight move", step_axis + 1, step,
              c->job->sample_step, at);
    }
    memcpy(c->joints, q, sizeof q);
    record(c, m, q, tcp, false);
    return true;
}

/* A joint move goes straight in joint space; check the arm along it. */
static void sweep_joints(Ctx *c, const RgMove *m, const double from[RG_AXES],
                         const double to[RG_AXES])
{
    double most = 0.0;
    for (int a = 0; a < RG_AXES; a++)
        most = fmax(most, fabs(to[a] - from[a]));
    int n = (int)ceil(most / 2.0);
    for (int i = 1; i < n; i++) {
        double q[RG_AXES];
        for (int a = 0; a < RG_AXES; a++)
            q[a] = from[a] + (to[a] - from[a]) * i / n;
        RgPose t0;
        rg_robot_fk(c->robot, q, &t0, NULL);
        RgPose tcp = rg_pose_mul(c->wobj_inv, rg_pose_mul(t0, c->tool));
        record(c, m, q, &tcp, true);
    }
}

static void follow(Ctx *c)
{
    const RgJob *j = c->job;
    RgPlan *pl = c->pl;
    RgPose prev = pl->home_tcp;

    for (int i = 0; i < pl->nmoves; i++) {
        RgMove *m = &pl->moves[i];
        bool ok = true;
        switch (m->kind) {
        case RG_MV_HOME: {
            double from[RG_AXES];
            memcpy(from, c->joints, sizeof from);
            memcpy(c->joints, j->home, sizeof c->joints);
            m->tcp = pl->home_tcp;
            /* The path from wherever the arm is before the program starts
             * cannot be known; the way back is checked up to home itself,
             * which the first move already checked. */
            if (i == 0)
                record(c, m, j->home, &m->tcp, false);
            else
                sweep_joints(c, m, from, j->home);
            break;
        }
        case RG_MV_JOINT: {
            double from[RG_AXES];
            memcpy(from, c->joints, sizeof from);
            ok = reach(c, m, &m->tcp);
            if (ok)
                sweep_joints(c, m, from, c->joints);
            break;
        }
        case RG_MV_LINEAR: {
            double len = rg_v3_len(rg_v3_sub(m->tcp.pos, prev.pos));
            int n = (int)ceil(len / j->sample_step);
            if (n < 1)
                n = 1;
            RgQuat qa = rg_quat_from_m3(prev.rot), qb = rg_quat_from_m3(m->tcp.rot);
            for (int k = 1; k <= n && ok; k++) {
                double t = (double)k / n;
                RgPose s = rg_pose(rg_m3_from_quat(rg_quat_slerp(qa, qb, t)),
                                   rg_v3_lerp(prev.pos, m->tcp.pos, t));
                if (k == n)
                    s = m->tcp;
                ok = reach(c, m, &s);
            }
            break;
        }
        }
        if (!ok)
            return;       /* everything after depends on where the arm got to */
        memcpy(m->joints, c->joints, sizeof m->joints);
        rg_robot_confdata(c->robot, m->joints, m->cf);
        prev = m->tcp;
    }
}

/* ---- the plan ------------------------------------------------------- */

static void check_process(Ctx *c)
{
    const RgJob *j = c->job;
    RgPlan *pl = c->pl;

    if (pl->spray_speed > j->max_spray_speed)
        issue(c, RG_REFUSE, "the traverse would have to run at %.1f mm/s, above "
              "max_spray_speed %.0f mm/s: slow the rotator or reduce the pitch",
              pl->spray_speed, j->max_spray_speed);
    else if (pl->spray_speed < 2.0)
        issue(c, RG_WARN, "the traverse runs at only %.2f mm/s; check the robot holds a "
              "steady speed that slow", pl->spray_speed);

    for (int b = 0; b < pl->nbands; b++) {
        const RgBand *bd = &pl->bands[b];
        if (bd->y0 < 0.0 || bd->y1 > j->part_height)
            issue(c, RG_REFUSE, "band %d (%.1f-%.1f mm) is not on the part, which runs from "
                  "0 to %.1f mm", b + 1, bd->y0, bd->y1, j->part_height);
        if (b + 1 < pl->nbands) {
            double gap = pl->bands[b + 1].y0 - bd->y1;
            double needed = 2.0 * pl->reach_past_edge;
            if (!j->gun_signal[0])
                issue(c, RG_REFUSE, "bands %d and %d are %.1f mm apart, and without "
                      "gun_signal the gun cannot be switched off between them: set "
                      "gun_signal, or make one program per band", b + 1, b + 2, gap);
            else if (gap < needed)
                issue(c, RG_REFUSE, "bands %d and %d are %.1f mm apart, but spray lands "
                      "%.1f mm past each edge, so the gap needs at least %.1f mm: draw "
                      "them as one band", b + 1, b + 2, gap, pl->reach_past_edge, needed);
        }
    }

    if (pl->nbands) {
        const RgBand *first = &pl->bands[0], *last = &pl->bands[pl->nbands - 1];
        if (last->y1 + pl->reach_past_edge > j->part_height + 1e-6)
            issue(c, RG_NOTE, "spray passes over the top of the part by up to %.1f mm",
                  last->y1 + pl->reach_past_edge - j->part_height);
        if (first->y0 - pl->reach_past_edge < -1e-6)
            issue(c, RG_WARN, "spray lands up to %.1f mm below the part, on the rotator table",
                  pl->reach_past_edge - first->y0);
        for (int b = 0; b < pl->nbands; b++) {
            const RgBand *bd = &pl->bands[b];
            if (bd->y0 > 1e-6 || bd->y1 < j->part_height - 1e-6)
                issue(c, RG_NOTE, "band %d: the part is coated up to %.1f mm beyond the "
                      "band's edges (half the fan, the overrun and the run-up)", b + 1,
                      pl->reach_past_edge);
        }
    }

    if (!j->gun_signal[0])
        issue(c, RG_WARN, "the gun is controlled outside the program: it must only spray "
              "while the robot traverses. Spraying while the arm waits at the start of a "
              "band puts a thick ring on the part at that height");

    double h = hypot(j->axis.x, j->axis.y);
    if (h < j->radius + j->standoff + j->clearance)
        issue(c, RG_REFUSE, "the rotator axis is only %.0f mm from the robot base, inside "
              "the part and gun (radius %.0f + standoff %.0f + clearance %.0f mm)",
              h, j->radius, j->standoff, j->clearance);
}

static void check_limits(Ctx *c)
{
    const RgJob *j = c->job;
    RgPlan *pl = c->pl;
    if (!pl->samples)
        return;

    if (pl->min_wrist < j->min_wrist)
        issue(c, RG_REFUSE, "axis 5 comes within %.1f deg of straight %s, and the rule is "
              "%.1f deg: near the wrist singularity axes 4 and 6 spin fast and the TCP "
              "speed is not held. Tilt the gun on its mount (tool_rot), or move azimuth",
              pl->min_wrist, pl->min_wrist_at, j->min_wrist);
    else if (pl->min_wrist < j->min_wrist + 5.0)
        issue(c, RG_WARN, "axis 5 comes within %.1f deg of straight %s; the rule is %.1f deg",
              pl->min_wrist, pl->min_wrist_at, j->min_wrist);

    if (pl->min_margin < j->min_margin)
        issue(c, RG_REFUSE, "axis %d comes within %.1f deg of its limit %s, and the rule is "
              "%.1f deg", pl->min_margin_axis + 1, pl->min_margin, pl->min_margin_at,
              j->min_margin);
    else if (pl->min_margin < j->min_margin + 5.0)
        issue(c, RG_WARN, "axis %d comes within %.1f deg of its limit %s; the rule is %.1f deg",
              pl->min_margin_axis + 1, pl->min_margin, pl->min_margin_at, j->min_margin);

    for (int k = 0; k < KIND_COUNT; k++)
        if (c->counts[k] > MAX_LISTED)
            issue(c, RG_REFUSE, "...and %d more points with the same problem",
                  c->counts[k] - MAX_LISTED);
}

bool rg_plan_build(const RgJob *job, const RgShape *shape, RgPlan *pl)
{
    memset(pl, 0, sizeof *pl);
    pl->min_wrist = DBL_MAX;
    pl->min_margin = DBL_MAX;
    pl->min_clearance = DBL_MAX;

    Ctx c;
    memset(&c, 0, sizeof c);
    c.job = job;
    c.pl = pl;
    c.robot = rg_robot_find(job->robot);
    if (!c.robot) {
        issue(&c, RG_REFUSE, "robot \"%s\" is not known", job->robot);
        return false;
    }
    c.wobj = rg_job_wobj(job);
    c.wobj_inv = rg_pose_inverse(c.wobj);
    c.tool = rg_job_tool(job);
    c.tool_inv = rg_pose_inverse(c.tool);

    pl->circumference = 2.0 * RG_PI * job->radius;
    pl->pitch = job->fan_width * (1.0 - job->overlap / 100.0);
    pl->spray_speed = pl->pitch * job->rpm / 60.0;
    pl->runup = pl->spray_speed * pl->spray_speed / (2.0 * job->accel);
    pl->overrun = 0.5 * job->fan_width + pl->runup;
    pl->reach_past_edge = pl->overrun + 0.5 * job->fan_width;

    if (shape) {
        bands_from_shape(&c, shape);
    } else {
        RgBand sorted[RG_MAX_BANDS];
        memcpy(sorted, job->bands, (size_t)job->nbands * sizeof sorted[0]);
        qsort(sorted, (size_t)job->nbands, sizeof sorted[0], band_cmp);
        for (int i = 0; i < job->nbands; i++)
            add_band(&c, sorted[i].y0, sorted[i].y1);
    }
    if (pl->nbands == 0 && !pl->refused)
        issue(&c, RG_REFUSE, "there is nothing to spray: no band goes all the way round");

    RgPose t0;
    rg_robot_fk(c.robot, job->home, &t0, NULL);
    pl->home_tcp = rg_pose_mul(c.wobj_inv, rg_pose_mul(t0, c.tool));
    rg_robot_confdata(c.robot, job->home, pl->home_cf);
    memcpy(c.joints, job->home, sizeof c.joints);
    if (!rg_robot_within_limits(c.robot, job->home))
        issue(&c, RG_REFUSE, "the home position is outside the robot's joint limits");

    check_process(&c);
    if (pl->refused)
        return false;

    build_moves(&c);
    if (!c.oom)
        follow(&c);
    check_limits(&c);

    for (int i = 0; i < pl->nmoves; i++)
        if (pl->moves[i].speed == RG_SPD_SPRAY && i > 0)
            pl->spray_time += rg_v3_len(rg_v3_sub(pl->moves[i].tcp.pos,
                                                  pl->moves[i - 1].tcp.pos)) / pl->spray_speed;
    if (c.oom)
        issue(&c, RG_REFUSE, "out of memory");
    return !pl->refused;
}
