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
 * rg_job.h — a spray job: the part, the cell, the gun and the process, and
 * the plain-text file it is kept in (.rgj).
 *
 * A job holds inputs only; the program is derived from it every time, so a
 * job file can never disagree with the program it makes. It is also the
 * hand-off point for text prompts: a request written in English is turned
 * into a job file, which a person can read and correct before anything is
 * generated from it.
 *
 * The process: a cylinder stands on a rotator that turns continuously and
 * independently of the robot. The robot holds the gun square to the outside
 * surface, `standoff` from it, at a fixed position round the part, and
 * traverses it up and down. Because the robot cannot know the part's angle,
 * only full bands round the part can be sprayed.
 *
 * Coordinates:
 *   robot base   ABB base frame: X forward, Z up, origin at the robot's foot.
 *   cylinder     the work object: origin on the rotator axis at table height,
 *                Z up the axis, X pointing horizontally towards the robot.
 *   drawing      the unrolled surface: X round the circumference, Y up the
 *                part from the table, millimetres.
 *
 * Units: millimetres, degrees, rpm, seconds.
 */
#ifndef RG_JOB_H
#define RG_JOB_H

#include <stdbool.h>
#include <stddef.h>

#include "plat.h"
#include "rg_vec.h"

#define RG_JOB_EXT     ".rgj"
#define RG_MAX_BANDS   32
#define RG_IDENT_CAP   17      /* S4 RAPID identifiers: 16 characters */

typedef struct { double y0, y1; } RgBand;

typedef struct {
    /* What and where */
    char   name[RG_IDENT_CAP];        /* module and file name              */
    char   controller[16];            /* dialect id: s4, s4c, s4c_plus     */
    char   robot[16];                 /* robot id: irb2400_16, irb2400_10  */
    char   drawing[PLAT_PATH_MAX];    /* DXF, relative to the job file     */
    char   layer[64];                 /* DXF layer; empty for all          */
    RgBand bands[RG_MAX_BANDS];       /* ...or the bands written directly  */
    int    nbands;

    /* The part and the cell */
    double radius;                    /* sprayed surface                   */
    double part_height;               /* table to top of part              */
    RgVec3 axis;                      /* rotator axis at table height, in the robot base frame */
    double azimuth;                   /* gun position round the part; 0 faces the robot */
    double rpm;                       /* rotator speed                     */

    /* The process */
    double fan_width;                 /* spray pattern width at the standoff */
    double overlap;                   /* percent of the fan width per turn */
    double standoff;                  /* gun tip to surface                */
    int    coats;                     /* traverses per band                */
    bool   start_top;                 /* first traverse runs downwards     */
    double accel;                     /* robot acceleration assumed for run-up */
    double approach;                  /* radial clearance before and after a band */
    double travel_speed, approach_speed, max_spray_speed;   /* mm/s */

    /* The gun */
    char   tool[RG_IDENT_CAP];
    bool   tool_define;               /* declare tooldata in the program   */
    RgVec3 tool_tcp;                  /* gun tip in the flange frame       */
    RgQuat tool_rot;                  /* spray direction = tool Z          */
    double tool_mass;                 /* kg; needed when tool_define       */
    RgVec3 tool_cog;

    /* Around the program */
    double home[6];                   /* joint angles, start and end       */
    bool   ready_prompt;              /* ask the operator before spraying  */
    char   gun_signal[RG_IDENT_CAP];  /* digital output; empty for none    */

    /* Rules the plan must keep */
    double min_wrist;                 /* |axis 5| from straight            */
    double min_margin;                /* from any joint limit              */
    double clearance;                 /* arm points from the part surface  */
    double max_joint_step;            /* per sample; more is a flip        */
    double sample_step;               /* along every linear move           */
    double wrap_tolerance;            /* drawing width vs circumference    */
    double join_tolerance, chord_tolerance;
} RgJob;

void rg_job_default(RgJob *j);

/* Parse a job file's text over a job (normally defaulted first). The error
 * names the line. */
bool rg_job_parse(RgJob *j, const char *text, char *err, size_t errcap);

bool rg_job_load(RgJob *j, const char *path, char *err, size_t errcap);

/* Everything required is present and consistent with its controller and
 * robot. Geometry and reach are the plan's business, not this. */
bool rg_job_validate(const RgJob *j, char *err, size_t errcap);

/* A complete, commented job file to start from. */
const char *rg_job_template(void);

/* The gun's tooldata frame, and the work object in the robot base frame. */
RgPose rg_job_tool(const RgJob *j);
RgPose rg_job_wobj(const RgJob *j);

/* A RAPID identifier of at most `max` characters that is not a reserved word. */
bool rg_rapid_ident_ok(const char *s, int max);

#endif /* RG_JOB_H */
