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
 * rg_plan.h — from a job to a checked sequence of robot moves.
 *
 * Planning and checking are one step. Every move is followed through the
 * robot's kinematics — sampled along linear moves, interpolated through
 * joint moves — the way the controller will execute it. A rule that is
 * broken refuses the plan: rapidgen writes no program at all rather than a
 * program with a warning attached. A refusal says where, and by how much.
 *
 * What is checked: reach at every sample; joint limits with a margin; the
 * wrist singularity with a margin; a flip mid-move; the wrist centre, flange
 * and gun tip staying clear of the part; the spray speed. For a cylinder,
 * also that the drawing's outlines go all the way round and match the
 * radius; for a flat part, that the gun can be switched between strokes.
 *
 * What is NOT checked, and is stated in every report: the gun body and the
 * arm's links against the part, the rotator and the cell; the IRB 2400's
 * axis 2/3 interaction limit; the controller's own path blending; paint film
 * thickness.
 */
#ifndef RG_PLAN_H
#define RG_PLAN_H

#include <stdbool.h>

#include "rg_job.h"
#include "rg_robot.h"
#include "rg_shape.h"

typedef enum { RG_NOTE, RG_WARN, RG_REFUSE } RgSeverity;

typedef struct {
    RgSeverity sev;
    char       text[400];
} RgIssue;

typedef enum { RG_MV_HOME, RG_MV_JOINT, RG_MV_LINEAR } RgMoveKind;
typedef enum { RG_SPD_TRAVEL, RG_SPD_APPROACH, RG_SPD_SPRAY } RgSpeedKind;
typedef enum { RG_ACT_NONE, RG_ACT_READY, RG_ACT_GUN_ON, RG_ACT_GUN_OFF } RgAction;

/* Stop on the point; round it by 1 mm (along a stroke); by 10 mm (in the air). */
typedef enum { RG_Z_FINE, RG_Z_SMALL, RG_Z_TRAVEL } RgZone;

typedef struct {
    RgMoveKind  kind;
    RgSpeedKind speed;
    RgZone      zone;
    RgPose      tcp;             /* in the work object                         */
    double      joints[RG_AXES]; /* where the checker expects the arm to be    */
    int         cf[4];
    RgAction    after;           /* done once the arm is there                 */
    bool        lap_begin;       /* a lap of a closed path starts here         */
    bool        lap_end;         /* ...and closes on this move                 */
    int         group;           /* the band or stroke; -1 for neither         */
    char        note[80];        /* a comment written before the move          */
} RgMove;

typedef struct RgPlan RgPlan;

struct RgPlan {
    bool refused;

    /* Derived */
    double circumference;       /* cylinder                                */
    double pitch;               /* between passes: fan less overlap        */
    double spray_speed;         /* mm/s                                    */
    double runup;               /* cylinder: distance to reach spray speed */
    double overrun;             /* cylinder: gun centre past each edge     */
    double reach_past_edge;     /* cylinder: furthest spray lands past an edge */
    double spray_time;          /* seconds spraying                        */
    double stroke_length;       /* flat: total sprayed length, one cycle   */
    double along_fan_length;    /* fan only: sprayed along the fan         */
    int    sharp_corners;       /* flat: turns of more than 45 degrees     */

    /* Building a coating takes many passes over many cycles. */
    int    cycles;
    int    laps;                /* flat: times round each closed path      */
    int    circuits;            /* flat: strokes driven round as a circuit */
    double dwell;               /* seconds between cycles                  */
    double cycle_time;          /* seconds spraying, one cycle             */
    double lead_used;           /* flat: run-on and run-off at each end    */
    double lead_needed;         /* ...the least the speed allows           */
    int    lead_ends;           /* flat: stroke ends run on by lead, no tab */
    double off_length;          /* flat: sprayed off the work, a cycle     */
    int    short_tabs;          /* ...runs off it shorter than lead_needed */
    double shortest_tab;        /* ...the shortest of those, mm            */
    double passes_per_point;    /* the gun's width over the step-over      */
    double thickness_cycle;     /* microns a cycle lays down, if given     */
    double thickness_total;     /* ...over every cycle                     */
    int    cycles_for_target;   /* to reach target_thickness               */
    double surface_speed;       /* cylinder: the part's surface under the gun, mm/s */
    double transit_over_part;   /* continuous gun: travel coated in passing */
    int    transit_drops;       /* ...and descents or lifts over the part  */

    RgBand bands[RG_MAX_BANDS];
    int    nbands;

    RgMove *moves;
    int     nmoves;

    RgIssue *issues;
    int      nissues;

    /* What the checks found */
    int    samples;
    double min_wrist;           /* smallest |axis 5|                       */
    char   min_wrist_at[128];
    double min_margin;
    int    min_margin_axis;
    char   min_margin_at[128];
    double min_clearance;       /* from the part surface                   */
    char   min_clearance_at[160];

    RgPose home_tcp;            /* home in the work object, for controllers without MoveAbsJ */
    int    home_cf[4];
};

/* `shape` is the cylinder's drawing, or NULL when the job gives its bands
 * directly (and always for a flat part). The plan is filled either way; the
 * result is !refused. */
bool rg_plan_build(const RgJob *job, const RgShape *shape, RgPlan *plan);
void rg_plan_free(RgPlan *plan);

int rg_plan_count(const RgPlan *plan, RgSeverity sev);

#endif /* RG_PLAN_H */
