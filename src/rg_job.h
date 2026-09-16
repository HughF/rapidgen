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
 * hand-off point for text prompts and for the editor: a request written in
 * English, or a pattern painted over a drawing, becomes a job file that a
 * person can read and correct before anything is generated from it.
 *
 * Two kinds of part:
 *
 *   cylinder  The part stands on a rotator that turns continuously and
 *             independently of the robot. The robot holds the gun square to
 *             the outside surface, `standoff` from it, at a fixed position
 *             round the part, and traverses it up and down. Because the robot
 *             cannot know the part's angle, only full bands round the part
 *             can be sprayed.
 *
 *   flat      The part lies still on a plane. The gun points along the
 *             plane's normal, `standoff` above it, and follows strokes: paths
 *             painted in the plane of the drawing.
 *
 * Coordinates:
 *   robot base   ABB base frame: X forward, Z up, origin at the robot's foot.
 *   cylinder     the work object: origin on the rotator axis at table height,
 *                Z up the axis, X pointing horizontally towards the robot.
 *   flat         the work object: origin at the drawing's origin on the part
 *                surface, X and Y the drawing's axes, Z out of the surface.
 *   drawing      millimetres after `drawing_scale`. For a cylinder, the
 *                unrolled surface: X round the circumference, Y up the part.
 *
 * Units: millimetres, degrees, rpm, seconds.
 */
#ifndef RG_JOB_H
#define RG_JOB_H

#include <stdbool.h>
#include <stddef.h>

#include "plat.h"
#include "rg_dxf.h"
#include "rg_text.h"
#include "rg_vec.h"

#define RG_JOB_EXT     ".rgj"
#define RG_MAX_BANDS   32
#define RG_IDENT_CAP   17      /* S4 RAPID identifiers: 16 characters */

typedef enum { RG_PART_CYLINDER = 0, RG_PART_FLAT, RG_PART_COUNT } RgPartKind;

/*
 * What the gun lays down. A thermal-spray torch puts down a round spot, so
 * turning the gun about its own axis changes nothing and the sprayed strip is
 * the path swept by a disc. A paint gun's fan is a wide slot, which has to be
 * held across the direction of travel to lay a band its full width.
 */
typedef enum { RG_PAT_SPOT = 0, RG_PAT_FAN } RgPattern;

/*
 * A plasma or HVOF torch runs continuously — it cannot be switched stroke by
 * stroke, so whatever passes under it is coated. A paint gun, or a powder
 * feeder with its own valve, can be switched from the program.
 */
typedef enum { RG_GUN_CONTINUOUS = 0, RG_GUN_SWITCHED } RgGunKind;

typedef struct { double y0, y1; } RgBand;

/* A path the gun follows with the spray on, in drawing millimetres. */
/*
 * A painted stroke, and the tabs at its ends.
 *
 * The robot does not switch the torch, so a stroke has to come onto the work
 * already spraying and leave it still spraying: the first `tab_in` points and
 * the last `tab_out` points are the lead-in and the run-out, which are
 * travelled and sprayed but are not on the part. The points between them are
 * the work. Both zero means the whole stroke is work, and the plan falls back
 * to extending it by `lead`.
 */
typedef struct {
    RgPt *pts;
    int   n;
    int   tab_in, tab_out;
} RgStroke;

typedef struct {
    /* What and where */
    char       name[RG_IDENT_CAP];        /* module and file name              */
    char       controller[16];            /* dialect id: s4, s4c, s4c_plus     */
    char       robot[16];                 /* robot id: irb2400_16, irb2400_10  */
    RgPartKind part;
    char       drawing[PLAT_PATH_MAX];    /* DXF, relative to the job file     */
    char       layer[64];                 /* DXF layer; empty for all          */
    double     drawing_scale;             /* drawing units to millimetres      */
    RgBand     bands[RG_MAX_BANDS];       /* cylinder: bands written directly  */
    int        nbands;

    /* A cylinder and its cell */
    double radius;                    /* sprayed surface                   */
    double part_height;               /* table to top of part              */
    RgVec3 axis;                      /* rotator axis at table height, in the robot base frame */
    double azimuth;                   /* gun position round the part; 0 faces the robot */
    double rpm;                       /* rotator speed                     */

    /* A flat part */
    RgVec3 plane;                     /* the drawing's origin, robot base frame */
    RgQuat plane_rot;                 /* drawing X, Y and the surface normal    */
    double spray_speed;               /* along a stroke, mm/s                   */

    /* The process */
    RgPattern pattern;                /* spot (thermal spray) or fan (paint) */
    double spot_diameter;             /* spot: the coated circle at the standoff */
    double fan_width;                 /* fan: pattern width at the standoff  */
    double fan_along;                 /* fan: its long axis in the drawing
                                         plane, degrees from X. A stroke should
                                         run across it, not along it. */
    double step_over;                 /* between passes; unset: from overlap */
    double overlap;                   /* percent of the width covered again  */
    double standoff;                  /* gun tip to surface                */
    int    coats;                     /* cylinder: traverses per band      */
    int    cycles;                    /* repeats of the whole pattern      */
    int    laps;                      /* flat: times round a closed path,
                                         without the gun leaving it       */
    double dwell;                     /* seconds between cycles, to let the part cool */
    double thickness_per_pass;        /* microns a single pass lays down, measured */
    double target_thickness;          /* microns wanted, for working out cycles    */
    double lead;                      /* flat: run-on and run-off past a stroke's
                                         ends; unset: worked out from the speed  */
    bool   start_top;                 /* cylinder: first traverse downwards */
    double accel;                     /* robot acceleration assumed for run-up */
    double approach;                  /* clearance before and after spraying */
    double travel_speed, approach_speed, max_spray_speed;   /* mm/s */

    /* The gun */
    char   tool[RG_IDENT_CAP];
    bool   tool_define;               /* declare tooldata in the program   */
    RgVec3 tool_tcp;                  /* gun tip in the flange frame       */
    RgQuat tool_rot;                  /* spray direction = tool Z          */
    double tool_mass;                 /* kg; needed when tool_define       */
    RgVec3 tool_cog;

    /* Around the program */
    double    home[6];                /* joint angles, start and end       */
    bool      ready_prompt;           /* ask the operator before spraying  */
    RgGunKind gun;                    /* continuous, or switched by the program */
    char      gun_signal[RG_IDENT_CAP];  /* the output that switches it    */
    char      cool_signal[RG_IDENT_CAP]; /* held on through the dwell      */

    /* Rules the plan must keep */
    double min_wrist;                 /* |axis 5| from straight            */
    double min_margin;                /* from any joint limit              */
    double clearance;                 /* arm points from the part surface  */
    double max_joint_step;            /* per sample; more is a flip        */
    double sample_step;               /* along every linear move           */
    double wrap_tolerance;            /* cylinder drawing vs circumference */
    double join_tolerance, chord_tolerance;

    /* Flat: the painted pattern, in order. Owned: see rg_job_free. */
    RgStroke *strokes;
    int       nstrokes;
} RgJob;

void rg_job_default(RgJob *j);

/* Frees the strokes. The job is left valid and empty of strokes. */
void rg_job_free(RgJob *j);

/* A deep copy. `dst` is overwritten without being freed first. */
bool rg_job_copy(RgJob *dst, const RgJob *src);

/* Append a stroke (copied); false out of memory. */
bool rg_job_add_stroke(RgJob *j, const RgPt *pts, int n);

/* ...with its lead-in and run-out: the first `tab_in` and last `tab_out`
 * points are off the work. Tabs that would overlap, or swallow the whole
 * stroke, are dropped rather than believed. */
bool rg_job_add_stroke_tabs(RgJob *j, const RgPt *pts, int n, int tab_in, int tab_out);
void rg_job_delete_stroke(RgJob *j, int index);

/* Parse a job file's text over a job (normally defaulted first). The error
 * names the line. */
bool rg_job_parse(RgJob *j, const char *text, char *err, size_t errcap);

bool rg_job_load(RgJob *j, const char *path, char *err, size_t errcap);

/* The job as a job file. Parsing the result gives back the same job. */
void rg_job_write(const RgJob *j, RgBuf *out);

bool rg_job_save(const RgJob *j, const char *path, char *err, size_t errcap);

/* Everything required is present and consistent with its controller and
 * robot. Geometry and reach are the plan's business, not this. */
bool rg_job_validate(const RgJob *j, char *err, size_t errcap);

/* Complete, commented job files to start from. */
const char *rg_job_template(void);
const char *rg_job_template_flat(void);

const char *rg_part_name(RgPartKind k);

/* What the gun covers as it passes: the spot's diameter, or the fan's width. */
double rg_job_width(const RgJob *j);

/* Between one pass and the next: step_over, or what the overlap works out to. */
double rg_job_step(const RgJob *j);

/* The gun's tooldata frame, and the work object in the robot base frame. */
RgPose rg_job_tool(const RgJob *j);
RgPose rg_job_wobj(const RgJob *j);

/* A RAPID identifier of at most `max` characters that is not a reserved word. */
bool rg_rapid_ident_ok(const char *s, int max);

#endif /* RG_JOB_H */
