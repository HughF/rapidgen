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

/* A change of direction along a stroke sharper than this makes the robot
 * slow noticeably through the corner. */
#define SHARP_CORNER_DEG 45.0

typedef enum { KIND_REACH, KIND_FLIP, KIND_CLEAR, KIND_COUNT } IssueKind;

typedef struct {
    const RgJob   *job;
    const RgRobot *robot;
    RgPlan        *pl;
    bool           flat;
    RgPose         wobj, wobj_inv, tool, tool_inv;
    double         joints[RG_AXES];
    int            counts[KIND_COUNT];
    bool           oom;
    double         fx0, fy0, fx1, fy1;   /* flat: where the part is taken to be */
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

/* Cylinder: the gun square to the surface, `out` beyond the standoff, at
 * height z. Tool Z points at the axis; tool X, the fan's long axis, points up
 * the part, across the direction the surface moves under the gun. */
static RgPose gun_cylinder(const RgJob *j, double z, double out)
{
    double th = j->azimuth * RG_DEG;
    RgVec3 u = rg_v3(cos(th), sin(th), 0.0);
    RgVec3 tz = rg_v3_scale(u, -1.0);
    RgVec3 tx = rg_v3(0.0, 0.0, 1.0);
    RgVec3 ty = rg_v3_cross(tz, tx);
    return rg_pose(rg_m3_from_cols(tx, ty, tz),
                   rg_v3_add(rg_v3_scale(u, j->radius + j->standoff + out), rg_v3(0, 0, z)));
}

/* Flat: the gun pointing into the surface, `out` above the standoff, over a
 * point of the drawing. Tool X is the fan's long axis, laid along
 * `fan_along`; the gun keeps that one orientation for the whole program. */
static RgPose gun_flat(const RgJob *j, RgPt p, double out)
{
    double a = j->fan_along * RG_DEG;
    RgVec3 tx = rg_v3(cos(a), sin(a), 0);
    RgVec3 tz = rg_v3(0, 0, -1);
    RgVec3 ty = rg_v3_cross(tz, tx);
    return rg_pose(rg_m3_from_cols(tx, ty, tz), rg_v3(p.x, p.y, j->standoff + out));
}

static RgPose tool0_for(const Ctx *c, const RgPose *tcp)
{
    return rg_pose_mul(rg_pose_mul(c->wobj, *tcp), c->tool_inv);
}

static RgMove *add_move(Ctx *c, RgMoveKind kind, RgSpeedKind speed, RgZone zone,
                        RgPose tcp, int group, const char *note)
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
    mv->zone = zone;
    mv->tcp = tcp;
    mv->group = group;
    rg_copy(mv->note, sizeof mv->note, note ? note : "");
    return mv;
}

static RgPose no_pose(void)
{
    return rg_pose(rg_m3_identity(), rg_v3(0, 0, 0));
}

static void build_cylinder(Ctx *c)
{
    const RgJob *j = c->job;
    RgPlan *pl = c->pl;
    bool switched = j->gun == RG_GUN_SWITCHED && j->gun_signal[0];

    add_move(c, RG_MV_HOME, RG_SPD_TRAVEL, RG_Z_FINE, no_pose(), -1, "Home");
    for (int b = 0; b < pl->nbands && !c->oom; b++) {
        const RgBand *bd = &pl->bands[b];
        double lo = bd->y0 - pl->overrun, hi = bd->y1 + pl->overrun;
        double z_start = j->start_top ? hi : lo, z_other = j->start_top ? lo : hi;
        char note[80];
        snprintf(note, sizeof note, "Band %d: %.1f-%.1f mm, %d coat%s", b + 1, bd->y0, bd->y1,
                 j->coats, j->coats == 1 ? "" : "s");

        add_move(c, RG_MV_JOINT, RG_SPD_TRAVEL, RG_Z_TRAVEL, gun_cylinder(j, z_start, j->approach),
                 b, note);
        RgMove *in = add_move(c, RG_MV_LINEAR, RG_SPD_APPROACH, RG_Z_FINE,
                              gun_cylinder(j, z_start, 0), b, NULL);
        if (!in)
            return;
        if (b == 0 && j->ready_prompt)
            in->after = RG_ACT_READY;
        else if (switched)
            in->after = RG_ACT_GUN_ON;

        double z = z_start;
        RgMove *last = in;
        for (int k = 0; k < j->coats; k++) {
            z = (k % 2 == 0) ? z_other : z_start;
            last = add_move(c, RG_MV_LINEAR, RG_SPD_SPRAY, RG_Z_FINE, gun_cylinder(j, z, 0), b, NULL);
            if (!last)
                return;
        }
        if (switched)
            last->after = RG_ACT_GUN_OFF;
        add_move(c, RG_MV_LINEAR, RG_SPD_APPROACH, RG_Z_TRAVEL, gun_cylinder(j, z, j->approach),
                 b, NULL);
    }
    add_move(c, RG_MV_HOME, RG_SPD_TRAVEL, RG_Z_FINE, no_pose(), -1, "Home");
}

/* A point `d` beyond `end`, carrying on the way the stroke was going. */
static RgPt extend_from(RgPt prev, RgPt end, double d)
{
    double dx = end.x - prev.x, dy = end.y - prev.y, len = hypot(dx, dy);
    if (len < 1e-9 || d <= 0.0)
        return end;
    RgPt p = { end.x + dx / len * d, end.y + dy / len * d };
    return p;
}

static void build_flat(Ctx *c)
{
    const RgJob *j = c->job;
    RgPlan *pl = c->pl;
    bool switched = j->gun == RG_GUN_SWITCHED && j->gun_signal[0];
    double lead = pl->lead_used;

    add_move(c, RG_MV_HOME, RG_SPD_TRAVEL, RG_Z_FINE, no_pose(), -1, "Home");
    for (int s = 0; s < j->nstrokes && !c->oom; s++) {
        const RgStroke *st = &j->strokes[s];
        bool closed = st->n > 2 && hypot(st->pts[st->n - 1].x - st->pts[0].x,
                                         st->pts[st->n - 1].y - st->pts[0].y) < 1e-6;
        double use_lead = closed ? 0.0 : lead;
        RgPt first = extend_from(st->pts[1], st->pts[0], use_lead);
        RgPt last_pt = extend_from(st->pts[st->n - 2], st->pts[st->n - 1], use_lead);

        char note[96];
        double len = 0.0;
        for (int k = 1; k < st->n; k++)
            len += hypot(st->pts[k].x - st->pts[k - 1].x, st->pts[k].y - st->pts[k - 1].y);
        snprintf(note, sizeof note, "Stroke %d: %d points, %.0f mm%s", s + 1, st->n, len,
                 use_lead > 0.0 ? ", run on and off" : closed ? ", closed" : "");

        add_move(c, RG_MV_JOINT, RG_SPD_TRAVEL, RG_Z_TRAVEL, gun_flat(j, first, j->approach),
                 s, note);
        RgMove *in = add_move(c, RG_MV_LINEAR, RG_SPD_APPROACH, RG_Z_FINE, gun_flat(j, first, 0),
                              s, NULL);
        if (!in)
            return;
        if (s == 0 && j->ready_prompt)
            in->after = RG_ACT_READY;
        else if (switched)
            in->after = RG_ACT_GUN_ON;

        /* The run-on, then the stroke, then the run-off: rounded corners
         * throughout so the gun never stops over the work. */
        RgMove *last = in;
        RgPt at = first;
        for (int k = 0; k < st->n; k++) {
            if (hypot(st->pts[k].x - at.x, st->pts[k].y - at.y) < 1e-6)
                continue;
            at = st->pts[k];
            last = add_move(c, RG_MV_LINEAR, RG_SPD_SPRAY, RG_Z_SMALL, gun_flat(j, at, 0), s, NULL);
            if (!last)
                return;
        }
        if (use_lead > 0.0 && hypot(last_pt.x - at.x, last_pt.y - at.y) > 1e-6) {
            at = last_pt;
            last = add_move(c, RG_MV_LINEAR, RG_SPD_SPRAY, RG_Z_SMALL, gun_flat(j, at, 0), s, NULL);
            if (!last)
                return;
        }
        /* Stop only once the gun is clear of the work. */
        last->zone = use_lead > 0.0 ? RG_Z_TRAVEL : RG_Z_FINE;
        if (switched)
            last->after = RG_ACT_GUN_OFF;
        add_move(c, RG_MV_LINEAR, RG_SPD_APPROACH, RG_Z_TRAVEL, gun_flat(j, at, j->approach),
                 s, NULL);
    }
    add_move(c, RG_MV_HOME, RG_SPD_TRAVEL, RG_Z_FINE, no_pose(), -1, "Home");
}

/* ---- following the moves through the arm ---------------------------- */

/* Where on the program an arm position is, as a phrase that reads after a
 * finding: "...comes within 40 mm of the part surface <where>". `between`
 * is a position part-way along a joint move rather than its target. */
static void where(const Ctx *c, const RgMove *m, const RgPose *tcp, bool between,
                  char *buf, size_t cap)
{
    const char *what = c->flat ? "stroke" : "band";
    switch (m->kind) {
    case RG_MV_HOME:
        snprintf(buf, cap, between ? "on the way home" : "at the home position");
        return;
    case RG_MV_JOINT:
        if (between)
            snprintf(buf, cap, "on the way to %s %d", what, m->group + 1);
        else if (c->flat)
            snprintf(buf, cap, "at the approach to stroke %d, above (%.1f, %.1f)",
                     m->group + 1, tcp->pos.x, tcp->pos.y);
        else
            snprintf(buf, cap, "at the approach to band %d, gun at height %.1f mm",
                     m->group + 1, tcp->pos.z);
        return;
    case RG_MV_LINEAR:
        if (c->flat) {
            snprintf(buf, cap, "in stroke %d at (%.1f, %.1f)", m->group + 1, tcp->pos.x,
                     tcp->pos.y);
        } else {
            const RgBand *b = &c->pl->bands[m->group];
            snprintf(buf, cap, "in band %d (%.0f-%.0f mm), gun at height %.1f mm",
                     m->group + 1, b->y0, b->y1, tcp->pos.z);
        }
        return;
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
        double gap;
        if (c->flat) {
            if (p.x < c->fx0 || p.x > c->fx1 || p.y < c->fy0 || p.y > c->fy1)
                continue;
            gap = p.z;
        } else {
            if (p.z < -j->clearance || p.z > j->part_height + j->clearance)
                continue;
            gap = hypot(p.x, p.y) - j->radius;
        }
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

static void check_cylinder(Ctx *c)
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
            if (j->gun == RG_GUN_CONTINUOUS)
                issue(c, RG_WARN, "bands %d and %d are %.1f mm apart and the gun runs "
                      "continuously, so the part is coated between them as the gun travels: "
                      "set gun = switched with a gun_signal, or treat it as one band",
                      b + 1, b + 2, gap);
            else if (!j->gun_signal[0])
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
                      "band's edges (half the %s, the overrun and the run-up)", b + 1,
                      pl->reach_past_edge, j->pattern == RG_PAT_SPOT ? "spot" : "fan");
        }
    }

    if (pl->surface_speed > 0.0 && (pl->surface_speed < 200.0 || pl->surface_speed > 3000.0))
        issue(c, RG_WARN, "the part's surface passes the gun at %.2f m/s, outside the 0.2 to "
              "3 m/s thermal spraying usually runs at: change the rotator's speed, or the "
              "coating will build unevenly", pl->surface_speed / 1000.0);

    if (j->gun == RG_GUN_CONTINUOUS)
        issue(c, RG_WARN, "the gun runs continuously: it coats the part from the moment it "
              "reaches the start of a band until it leaves the last one, the run-up and the "
              "approach included");

    double h = hypot(j->axis.x, j->axis.y);
    if (h < j->radius + j->standoff + j->clearance)
        issue(c, RG_REFUSE, "the rotator axis is only %.0f mm from the robot base, inside "
              "the part and gun (radius %.0f + standoff %.0f + clearance %.0f mm)",
              h, j->radius, j->standoff, j->clearance);
}

static void check_flat(Ctx *c)
{
    const RgJob *j = c->job;
    RgPlan *pl = c->pl;

    if (j->nstrokes == 0)
        issue(c, RG_REFUSE, "there is nothing to spray: paint at least one stroke");
    if (pl->spray_speed > j->max_spray_speed)
        issue(c, RG_REFUSE, "spray_speed %.0f mm/s is above max_spray_speed %.0f mm/s",
              pl->spray_speed, j->max_spray_speed);
    if (!isnan(j->lead) && j->lead + 1e-9 < pl->lead_needed)
        issue(c, RG_WARN, "lead %.1f mm is shorter than the %.1f mm the gun needs to reach "
              "%.0f mm/s and stop again (%.0f mm/s\xc2\xb2): it will still be changing speed "
              "over the work, which shows as a ridge in the coating",
              j->lead, pl->lead_needed, pl->spray_speed, j->accel);

    double x0 = DBL_MAX, y0 = DBL_MAX, x1 = -DBL_MAX, y1 = -DBL_MAX;
    double fan_a = j->fan_along * RG_DEG, fx = cos(fan_a), fy = sin(fan_a);
    double along_fan = 0.0;
    for (int s = 0; s < j->nstrokes; s++) {
        const RgStroke *st = &j->strokes[s];
        if (st->n < 2)
            issue(c, RG_REFUSE, "stroke %d has only one point", s + 1);
        for (int k = 0; k < st->n; k++) {
            RgPt p = st->pts[k];
            x0 = fmin(x0, p.x); x1 = fmax(x1, p.x);
            y0 = fmin(y0, p.y); y1 = fmax(y1, p.y);
            if (k > 0) {
                double dx = p.x - st->pts[k - 1].x, dy = p.y - st->pts[k - 1].y;
                double len = hypot(dx, dy);
                pl->stroke_length += len;
                /* A segment running along the fan paints a line the fan's
                 * thickness, not a band its width. */
                if (len > 1e-9 && fabs((dx * fx + dy * fy) / len) > cos(30.0 * RG_DEG))
                    along_fan += len;
            }
            if (k > 0 && k + 1 < st->n) {
                RgPt a = st->pts[k - 1], b = st->pts[k + 1];
                double ax = p.x - a.x, ay = p.y - a.y, bx = b.x - p.x, by = b.y - p.y;
                double la = hypot(ax, ay), lb = hypot(bx, by);
                if (la > 1e-6 && lb > 1e-6) {
                    double cosang = (ax * bx + ay * by) / (la * lb);
                    if (cosang < cos(SHARP_CORNER_DEG * RG_DEG))
                        pl->sharp_corners++;
                }
            }
        }
    }
    if (pl->sharp_corners)
        issue(c, RG_NOTE, "the gun turns more than %.0f deg at %d point%s along the strokes: "
              "the robot slows through each corner, so the coat is heavier there",
              SHARP_CORNER_DEG, pl->sharp_corners, pl->sharp_corners == 1 ? "" : "s");

    pl->along_fan_length = j->pattern == RG_PAT_FAN ? along_fan : 0.0;
    if (j->pattern == RG_PAT_FAN && along_fan > 10.0 && along_fan > 0.02 * pl->stroke_length)
        issue(c, RG_WARN, "%.0f mm of the %.0f mm painted (%.0f %%) runs within 30 deg of the "
              "fan's long axis, which lies at %.0f deg: there the gun lays a narrow line, not "
              "a band %.0f mm wide. Turn the fan (fan_along), or paint those parts across it",
              along_fan, pl->stroke_length, 100.0 * along_fan / pl->stroke_length, j->fan_along,
              j->fan_width);

    /* The part is taken to be under the pattern and its spray, plus the
     * clearance; the arm is kept above the surface anywhere over it. */
    double margin = 0.5 * rg_job_width(j) + j->clearance;
    c->fx0 = x0 - margin;
    c->fy0 = y0 - margin;
    c->fx1 = x1 + margin;
    c->fy1 = y1 + margin;

    /*
     * A torch that cannot be switched coats whatever passes under it. The
     * strokes' own footprint stands in for the part: travel across it between
     * strokes, and the descents and lifts over it, land coating where none
     * was asked for.
     */
    if (j->gun == RG_GUN_CONTINUOUS && j->nstrokes) {
        double px0 = x0 - 0.5 * rg_job_width(j), px1 = x1 + 0.5 * rg_job_width(j);
        double py0 = y0 - 0.5 * rg_job_width(j), py1 = y1 + 0.5 * rg_job_width(j);
        for (int s = 0; s < j->nstrokes; s++) {
            RgPt a = j->strokes[s].pts[0];
            RgPt z = j->strokes[s].pts[j->strokes[s].n - 1];
            if (a.x >= px0 && a.x <= px1 && a.y >= py0 && a.y <= py1)
                pl->transit_drops++;
            if (z.x >= px0 && z.x <= px1 && z.y >= py0 && z.y <= py1)
                pl->transit_drops++;
            if (s + 1 < j->nstrokes) {
                RgPt b = j->strokes[s + 1].pts[0];
                /* however much of the hop between strokes passes over the part */
                double n = 32.0, inside = 0.0;
                double seg = hypot(b.x - z.x, b.y - z.y) / n;
                for (int t = 0; t < (int)n; t++) {
                    double u = (t + 0.5) / n;
                    double mx = z.x + (b.x - z.x) * u, my = z.y + (b.y - z.y) * u;
                    if (mx >= px0 && mx <= px1 && my >= py0 && my <= py1)
                        inside += seg;
                }
                pl->transit_over_part += inside;
            }
        }
        if (pl->transit_over_part > 1.0 || pl->transit_drops)
            issue(c, RG_WARN, "the gun runs continuously: %.0f mm of travel between strokes "
                  "passes over the part, and it drops onto or lifts off the part %d time%s, "
                  "coating it there. Order the strokes so the gun leaves the work before it "
                  "moves, or set gun = switched with a gun_signal",
                  pl->transit_over_part, pl->transit_drops, pl->transit_drops == 1 ? "" : "s");
    }
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
              "speed is not held. Tilt the gun on its mount (tool_rot)%s",
              pl->min_wrist, pl->min_wrist_at, j->min_wrist,
              c->flat ? ", or move or turn the part" : ", or move azimuth");
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
    c.flat = job->part == RG_PART_FLAT;
    c.robot = rg_robot_find(job->robot);
    if (!c.robot) {
        issue(&c, RG_REFUSE, "robot \"%s\" is not known", job->robot);
        return false;
    }
    c.wobj = rg_job_wobj(job);
    c.wobj_inv = rg_pose_inverse(c.wobj);
    c.tool = rg_job_tool(job);
    c.tool_inv = rg_pose_inverse(c.tool);

    double width = rg_job_width(job);
    pl->pitch = rg_job_step(job);
    pl->cycles = job->cycles;
    pl->dwell = job->dwell;
    pl->passes_per_point = pl->pitch > 0.0 ? width / pl->pitch : 0.0;
    if (c.flat) {
        pl->spray_speed = job->spray_speed;
        /* Run on and off past each end far enough that the gun is up to speed
         * before it reaches the work, and has left it before it slows: a dip
         * in speed is a ridge in the coating. */
        pl->lead_needed = pl->spray_speed * pl->spray_speed / (2.0 * job->accel) + 0.5 * width;
        pl->lead_used = isnan(job->lead) ? pl->lead_needed : job->lead;
    } else {
        pl->circumference = 2.0 * RG_PI * job->radius;
        pl->spray_speed = pl->pitch * job->rpm / 60.0;
        pl->surface_speed = pl->circumference * job->rpm / 60.0;
        pl->runup = pl->spray_speed * pl->spray_speed / (2.0 * job->accel);
        pl->overrun = 0.5 * width + pl->runup;
        pl->reach_past_edge = pl->overrun + 0.5 * width;

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
    }

    RgPose t0;
    rg_robot_fk(c.robot, job->home, &t0, NULL);
    pl->home_tcp = rg_pose_mul(c.wobj_inv, rg_pose_mul(t0, c.tool));
    rg_robot_confdata(c.robot, job->home, pl->home_cf);
    memcpy(c.joints, job->home, sizeof c.joints);
    if (!rg_robot_within_limits(c.robot, job->home))
        issue(&c, RG_REFUSE, "the home position is outside the robot's joint limits");

    if (c.flat)
        check_flat(&c);
    else
        check_cylinder(&c);
    if (pl->refused)
        return false;

    if (c.flat)
        build_flat(&c);
    else
        build_cylinder(&c);
    if (!c.oom)
        follow(&c);
    check_limits(&c);

    for (int i = 1; i < pl->nmoves; i++)
        if (pl->moves[i].speed == RG_SPD_SPRAY)
            pl->cycle_time += rg_v3_len(rg_v3_sub(pl->moves[i].tcp.pos,
                                                  pl->moves[i - 1].tcp.pos)) / pl->spray_speed;
    pl->spray_time = pl->cycle_time * pl->cycles;

    /* Thickness from the operator's own measured figure, not a model of the
     * process: what one pass lays down, times the passes each point gets. */
    if (!isnan(job->thickness_per_pass)) {
        pl->thickness_cycle = job->thickness_per_pass * pl->passes_per_point;
        pl->thickness_total = pl->thickness_cycle * pl->cycles;
        if (!isnan(job->target_thickness) && pl->thickness_cycle > 0.0) {
            pl->cycles_for_target = (int)ceil(job->target_thickness / pl->thickness_cycle - 1e-9);
            if (pl->cycles_for_target > pl->cycles)
                issue(&c, RG_WARN, "%d cycle%s lay down about %.0f um, short of the %.0f um "
                      "wanted: %d cycles would reach it, at %.0f um a cycle",
                      pl->cycles, pl->cycles == 1 ? "" : "s", pl->thickness_total,
                      job->target_thickness, pl->cycles_for_target, pl->thickness_cycle);
        }
    }
    if (c.oom)
        issue(&c, RG_REFUSE, "out of memory");
    return !pl->refused;
}
