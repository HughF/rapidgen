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
#include "rg_report.h"
#include "rg_rapid.h"
#include "rg_robot.h"
#include "rg_version.h"

#include <float.h>

static void issues(RgBuf *b, const RgPlan *pl, RgSeverity sev, const char *title)
{
    if (!rg_plan_count(pl, sev))
        return;
    rg_buf_printf(b, "\n%s\n", title);
    for (int i = 0; i < pl->nissues; i++)
        if (pl->issues[i].sev == sev)
            rg_buf_printf(b, "  - %s\n", pl->issues[i].text);
}

void rg_report_write(const RgJob *j, const RgPlan *pl, const char *program_file,
                     const char *source, RgBuf *b)
{
    const RgDialect *d = rg_dialect_find(j->controller);
    const RgRobot *r = rg_robot_find(j->robot);

    rg_buf_printf(b, "%s %s - job %s\n\n", RAPIDGEN_NAME, RAPIDGEN_VERSION, j->name);
    if (pl->refused)
        rg_buf_puts(b, "RESULT      REFUSED - no program written. The reasons are below.\n");
    else
        rg_buf_printf(b, "RESULT      %s written\n", program_file);

    rg_buf_printf(b, "\nFrom        %s\n", source);
    rg_buf_printf(b, "Controller  ABB %s (file format not yet checked on a controller)\n",
                  d ? d->name : j->controller);
    rg_buf_printf(b, "Robot       %s (kinematics not yet checked on a controller)\n",
                  r ? r->name : j->robot);
    rg_buf_printf(b, "Cylinder    radius %.1f mm, %.1f mm round, %.1f mm tall\n",
                  j->radius, pl->circumference, j->part_height);
    rg_buf_printf(b, "Rotator     %.1f rpm, turning on its own\n", j->rpm);
    rg_buf_printf(b, "Spray       fan %.1f mm at %.1f mm standoff, %.0f %% overlap\n",
                  j->fan_width, j->standoff, j->overlap);
    rg_buf_printf(b, "            pitch %.1f mm per turn, traverse %.2f mm/s\n",
                  pl->pitch, pl->spray_speed);
    rg_buf_printf(b, "            run-up %.1f mm; the gun turns %.1f mm past each band edge\n",
                  pl->runup, pl->overrun);

    for (int i = 0; i < pl->nbands; i++)
        rg_buf_printf(b, "%s %2d: %.1f - %.1f mm, %d coat%s\n", i ? "           " : "Bands      ",
                      i + 1, pl->bands[i].y0, pl->bands[i].y1, j->coats, j->coats == 1 ? "" : "s");
    if (pl->spray_time > 0.0)
        rg_buf_printf(b, "Spraying    %.0f s, %.1f turns of the part\n",
                      pl->spray_time, pl->spray_time * j->rpm / 60.0);

    if (pl->samples) {
        rg_buf_printf(b, "\nChecked     %d arm positions%s\n", pl->samples,
                      pl->refused ? ", stopping at the first that failed"
                                  : " along every move");
        if (pl->min_wrist < DBL_MAX)
            rg_buf_printf(b, "            axis 5 no nearer straight than %.1f deg (rule %.1f),\n"
                             "              %s\n", pl->min_wrist, j->min_wrist, pl->min_wrist_at);
        if (pl->min_margin < DBL_MAX)
            rg_buf_printf(b, "            nearest a joint limit: axis %d, %.1f deg (rule %.1f),\n"
                             "              %s\n", pl->min_margin_axis + 1, pl->min_margin,
                          j->min_margin, pl->min_margin_at);
        if (pl->min_clearance < DBL_MAX)
            rg_buf_printf(b, "            nearest the part: %.0f mm (rule %.0f),\n"
                             "              %s\n", pl->min_clearance, j->clearance,
                          pl->min_clearance_at);
    }

    issues(b, pl, RG_REFUSE, "REFUSED BECAUSE");
    issues(b, pl, RG_WARN, "WARNINGS");
    issues(b, pl, RG_NOTE, "NOTES");

    rg_buf_puts(b,
        "\nNOT CHECKED\n"
        "  - the gun body and the arm's links against the part, the rotator and the\n"
        "    cell: only the wrist centre, flange and gun tip are kept clear of the part\n"
        "  - the IRB 2400's axis 2/3 interaction limit\n"
        "  - the controller's own corner blending and speed near the stops\n"
        "  - film thickness: pitch and coats set it, the spray process decides it\n"
        "  - that the program loads: the file format is not yet checked on an S4\n"
        "\nRun it in simulation first, then step through it in manual reduced speed.\n");
}
