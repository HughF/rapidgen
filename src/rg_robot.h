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
 * rg_robot.h — robot arms: dimensions, joint limits, forward and inverse
 * kinematics, and ABB configuration data.
 *
 * The controller does its own inverse kinematics from the Cartesian targets
 * in a program; rapidgen's is there to *check* a program before it gets that
 * far — can every point be reached, does the wrist stay clear of its
 * singularity, does any joint come near a limit, does the arm flip mid-pass.
 *
 * Frames and signs follow ABB: base frame at the foot of the robot, X forward,
 * Z up; all axes at zero is the calibration pose (lower arm vertical, upper
 * arm horizontal, tool flange facing forward), where tool0 is at
 * orientation [0.707107, 0, 0.707107, 0]. Angles in degrees at this
 * interface, millimetres throughout.
 *
 * NOT YET VERIFIED against a real controller. The dimensions and limits come
 * from ABB's published product data as transcribed in ROS-Industrial's
 * abb_irb2400_support package. In particular the IRB 2400's parallel-arm
 * axis 2/3 interaction limit is not modelled, and the sign conventions and
 * the cfx encoding need a pendant check: jog to a few joint positions, read
 * the Cartesian position and compare with `rapidgen --fk`.
 */
#ifndef RG_ROBOT_H
#define RG_ROBOT_H

#include <stdbool.h>

#include "rg_vec.h"

#define RG_AXES 6

typedef struct {
    const char *id;       /* job-file name, e.g. "irb2400_16" */
    const char *name;     /* as ABB writes it                */
    /* Shoulder height and forward offset, upper arm, elbow offset, forearm
     * (axis 3 to wrist centre) and wrist centre to flange. */
    double d1, a1, a2, a3, d4, d6;
    double lo[RG_AXES], hi[RG_AXES];   /* joint limits, degrees */
} RgRobot;

int            rg_robot_count(void);
const RgRobot *rg_robot_at(int i);
const RgRobot *rg_robot_find(const char *id);

/* tool0 (the flange) in the base frame; the wrist centre too if asked. */
void rg_robot_fk(const RgRobot *r, const double j[RG_AXES], RgPose *tool0, RgVec3 *wrist);

/*
 * Every arm solution for a flange pose, each joint in (-180, 180]: up to
 * eight (shoulder front/back × elbow up/down × wrist flip). Joint limits are
 * not applied. Returns the count; when it is zero, *shortfall_mm is how far
 * the wrist centre lies outside the arm's reach (or inside its dead zone).
 */
int rg_robot_ik_all(const RgRobot *r, const RgPose *tool0, double sol[8][RG_AXES],
                    double *shortfall_mm);

typedef enum {
    RG_IK_OK = 0,
    RG_IK_OUT_OF_REACH,
    RG_IK_JOINT_LIMIT
} RgIkStatus;

typedef struct {
    RgIkStatus status;
    double     shortfall_mm;  /* RG_IK_OUT_OF_REACH                     */
    int        axis;          /* RG_IK_JOINT_LIMIT: the worst axis, 0-5 */
    double     over_deg;      /* ...and how far past its limit          */
} RgIkResult;

/*
 * The solution within limits closest to `seed`, taking axes 4 and 6 round by
 * whole turns where their range allows — the choice a controller makes when
 * configuration monitoring is off and it moves from `seed`.
 */
RgIkStatus rg_robot_ik_near(const RgRobot *r, const RgPose *tool0, const double seed[RG_AXES],
                            double out[RG_AXES], RgIkResult *res);

/* Smallest distance of any joint from its nearest limit, and which axis. */
double rg_robot_margin(const RgRobot *r, const double j[RG_AXES], int *axis);

bool rg_robot_within_limits(const RgRobot *r, const double j[RG_AXES]);

/*
 * robtarget confdata [cf1, cf4, cf6, cfx]: quadrants of axes 1, 4 and 6, and
 * cfx = 4·(wrist centre behind axis 1) + 2·(wrist centre behind the lower
 * arm) + (axis 5 negative). The cfx encoding is ABB's serial-arm convention
 * as documented for later RobotWare; it still needs confirming on an S4.
 */
void rg_robot_confdata(const RgRobot *r, const double j[RG_AXES], int cf[4]);

#endif /* RG_ROBOT_H */
