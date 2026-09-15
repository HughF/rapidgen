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
 * rg_ui_pages.c — the Settings page (every value in the job file) and the
 * Program page (the checks, the report and the program)
 */
#include "rg_ui_int.h"
#include "rg_rapid.h"
#include "rg_robot.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Fields                                                              */
/* ------------------------------------------------------------------ */

static void label_cell(RgUi *ui, const char *label)
{
    nk_label_colored(ui->ctx, label, NK_TEXT_LEFT, ui->theme->text_dim);
}

static void text_field(RgUi *ui, const char *label, const char *tip, char *buf, int cap)
{
    ui_form_row(ui, ROW_H);
    label_cell(ui, label);
    ui_tip(ui, tip);
    nk_edit_string_zero_terminated(ui->ctx, NK_EDIT_FIELD, buf, cap, nk_filter_default);
}

/* A number that may not be set yet: shown as a button until it is. */
static void number(RgUi *ui, const char *label, const char *tip, double *v, double def,
                   double min, double max, double step, const char *unit)
{
    if (!isnan(*v)) {
        ui_prop(ui, label, tip, v, min, max, step, unit);
        return;
    }
    ui_form_row(ui, ROW_H);
    label_cell(ui, label);
    ui_tip(ui, "Not set. Click to give it a value");
    if (nk_button_label(ui->ctx, "not set \xe2\x80\x94 set it"))
        *v = def;
}

static void vector(RgUi *ui, const char *label, const char *tip, RgVec3 *v, RgVec3 def,
                   double lim, double step, const char *unit)
{
    struct nk_context *c = ui->ctx;
    nk_layout_row_template_begin(c, S(ui, ROW_H));
    nk_layout_row_template_push_static(c, S(ui, LABEL_W));
    nk_layout_row_template_push_dynamic(c);
    nk_layout_row_template_push_dynamic(c);
    nk_layout_row_template_push_dynamic(c);
    nk_layout_row_template_push_static(c, S(ui, UNIT_W));
    nk_layout_row_template_end(c);
    label_cell(ui, label);
    if (isnan(v->x)) {
        ui_tip(ui, "Not set. Click to give it a value");
        if (nk_button_label(c, "not set"))
            *v = def;
        nk_skip(c);
        nk_skip(c);
    } else {
        ui_tip(ui, tip);
        nk_property_double(c, "#x", -lim, &v->x, lim, step, (float)step / 4.0f);
        ui_tip(ui, tip);
        nk_property_double(c, "#y", -lim, &v->y, lim, step, (float)step / 4.0f);
        ui_tip(ui, tip);
        nk_property_double(c, "#z", -lim, &v->z, lim, step, (float)step / 4.0f);
    }
    nk_label_colored(c, unit, NK_TEXT_LEFT, ui->theme->text_faint);
}

/* Four components, kept unit length: a quaternion that is not would not
 * survive the job file. */
static void rotation(RgUi *ui, const char *label, const char *tip, RgQuat *q)
{
    struct nk_context *c = ui->ctx;
    nk_layout_row_template_begin(c, S(ui, ROW_H));
    nk_layout_row_template_push_static(c, S(ui, LABEL_W));
    for (int i = 0; i < 4; i++)
        nk_layout_row_template_push_dynamic(c);
    nk_layout_row_template_push_static(c, S(ui, UNIT_W));
    nk_layout_row_template_end(c);
    label_cell(ui, label);
    RgQuat before = *q;
    ui_tip(ui, tip); nk_property_double(c, "#q1", -1, &q->q1, 1, 0.01, 0.002f);
    ui_tip(ui, tip); nk_property_double(c, "#q2", -1, &q->q2, 1, 0.01, 0.002f);
    ui_tip(ui, tip); nk_property_double(c, "#q3", -1, &q->q3, 1, 0.01, 0.002f);
    ui_tip(ui, tip); nk_property_double(c, "#q4", -1, &q->q4, 1, 0.01, 0.002f);
    nk_skip(c);
    if (memcmp(&before, q, sizeof before) != 0) {
        double n = sqrt(q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3 + q->q4 * q->q4);
        if (n > 1e-9)
            *q = rg_quat_normalise(*q);
        else
            *q = before;
    }
}

static int choose(RgUi *ui, const char *label, const char *tip, const char **items, int n, int cur)
{
    struct nk_context *c = ui->ctx;
    ui_form_row(ui, ROW_H);
    label_cell(ui, label);
    ui_tip(ui, tip);
    int chosen = cur;
    struct nk_rect b = nk_widget_bounds(c);
    if (nk_combo_begin_label(c, cur >= 0 ? items[cur] : "choose...",
                             nk_vec2(b.w, S(ui, 30) * (float)(n + 1)))) {
        for (int i = 0; i < n; i++) {
            nk_layout_row_dynamic(c, S(ui, 26), 1);
            if (nk_combo_item_label(c, items[i], NK_TEXT_LEFT))
                chosen = i;
        }
        nk_combo_end(c);
    }
    return chosen;
}

/* A part changed kind: fill what the new kind needs from its template, so the
 * form never shows a half-empty job. */
static void adopt_part(RgUi *ui, RgPartKind part)
{
    RgJob *j = &ui->job;
    RgJob tpl;
    rg_job_default(&tpl);
    char err[256];
    rg_job_parse(&tpl, part == RG_PART_FLAT ? rg_job_template_flat() : rg_job_template(),
                 err, sizeof err);
    j->part = part;
    if (part == RG_PART_FLAT) {
        if (isnan(j->plane.x)) j->plane = tpl.plane;
        if (isnan(j->spray_speed)) j->spray_speed = tpl.spray_speed;
        j->nbands = 0;
    } else {
        if (isnan(j->radius)) j->radius = tpl.radius;
        if (isnan(j->part_height)) j->part_height = tpl.part_height;
        if (isnan(j->axis.x)) j->axis = tpl.axis;
        if (isnan(j->rpm)) j->rpm = tpl.rpm;
        if (!j->nbands && !j->drawing[0]) {
            j->bands[0] = tpl.bands[0];
            j->nbands = 1;
        }
        if (j->nstrokes)
            ui_message(ui, true, "The %d painted stroke%s stay in the job, but a cylinder cannot "
                       "use them: change back, or clear them on the Draw page", j->nstrokes,
                       j->nstrokes == 1 ? "" : "s");
    }
    rg_job_free(&tpl);
}

/* ------------------------------------------------------------------ */
/* Settings page                                                       */
/* ------------------------------------------------------------------ */

static void settings_job(RgUi *ui)
{
    RgJob *j = &ui->job;
    ui_section(ui, "The job");
    text_field(ui, "Name", "The RAPID module and the program file's name: a letter, then "
               "letters, digits or _. Eight characters at most for S4 and S4C floppies",
               j->name, sizeof j->name);

    const char *ctl[8];
    int nctl = rg_dialect_count(), cur = -1;
    char ctl_names[8][48];
    for (int i = 0; i < nctl && i < 8; i++) {
        snprintf(ctl_names[i], sizeof ctl_names[i], "ABB %s", rg_dialect_at(i)->name);
        ctl[i] = ctl_names[i];
        if (rg_streqi(rg_dialect_at(i)->id, j->controller))
            cur = i;
    }
    int pick = choose(ui, "Controller", "The controller the program is loaded on: it decides "
                      "the file format and name", ctl, nctl, cur);
    if (pick != cur && pick >= 0)
        rg_copy(j->controller, sizeof j->controller, rg_dialect_at(pick)->id);

    const char *rob[8];
    int nrob = rg_robot_count();
    cur = -1;
    for (int i = 0; i < nrob && i < 8; i++) {
        rob[i] = rg_robot_at(i)->name;
        if (rg_streqi(rg_robot_at(i)->id, j->robot))
            cur = i;
    }
    pick = choose(ui, "Robot", "The arm: its dimensions and joint limits check every move",
                  rob, nrob, cur);
    if (pick != cur && pick >= 0)
        rg_copy(j->robot, sizeof j->robot, rg_robot_at(pick)->id);

    static const char *parts[] = { "Cylinder on a rotator", "Flat part" };
    pick = choose(ui, "Part", "A cylinder turning on a rotator, sprayed in bands; or a flat "
                  "part sprayed along painted strokes", parts, 2, (int)j->part);
    if (pick != (int)j->part && pick >= 0)
        adopt_part(ui, (RgPartKind)pick);

    text_field(ui, "Drawing layer", "Read only this layer of the DXF; empty reads every layer",
               j->layer, sizeof j->layer);
}

static void settings_cylinder(RgUi *ui)
{
    struct nk_context *c = ui->ctx;
    RgJob *j = &ui->job;
    ui_gap(ui, 6);
    ui_section(ui, "The cylinder and the rotator");
    number(ui, "Radius", "The radius of the surface sprayed", &j->radius, 300, 10, 5000, 1, "mm");
    number(ui, "Part height", "Table to the top of the part", &j->part_height, 1000, 1, 10000, 5, "mm");
    vector(ui, "Rotator axis", "Where the rotator's axis meets the table, in the robot's base "
           "frame", &j->axis, rg_v3(1450, 0, 300), 10000, 5, "mm");
    number(ui, "Gun position", "Where round the part the gun sprays; 0 faces the robot",
           &j->azimuth, 0, -90, 90, 1, "deg");
    number(ui, "Rotator speed", "The rotator's speed", &j->rpm, 30, 0.1, 300, 1, "rpm");
    ui_prop_int(ui, "Coats", "Traverses over each band", &j->coats, 1, 50, "");
    ui_form_row(ui, ROW_H);
    label_cell(ui, "First traverse");
    nk_layout_row_dynamic(c, S(ui, ROW_H), 2);
    if (nk_option_label(c, "Downwards, from the top", j->start_top))
        j->start_top = true;
    if (nk_option_label(c, "Upwards, from the bottom", !j->start_top))
        j->start_top = false;
    number(ui, "Acceleration", "The robot's acceleration assumed for the run-up",
           &j->accel, 500, 10, 20000, 10, "mm/s\xc2\xb2");

    if (j->drawing[0]) {
        ui_label_wrap(ui, "The bands come from the drawing: every closed outline has to go all "
                      "the way round.", ui->theme->text_dim);
        return;
    }
    for (int i = 0; i < j->nbands; i++) {
        nk_layout_row_template_begin(c, S(ui, ROW_H));
        nk_layout_row_template_push_static(c, S(ui, LABEL_W));
        nk_layout_row_template_push_dynamic(c);
        nk_layout_row_template_push_dynamic(c);
        nk_layout_row_template_push_static(c, S(ui, 80));
        nk_layout_row_template_end(c);
        char lab[32];
        snprintf(lab, sizeof lab, "Band %d", i + 1);
        label_cell(ui, lab);
        ui_tip(ui, "Bottom of the band, above the table");
        nk_property_double(c, "#from", 0, &j->bands[i].y0, 10000, 5, 1.0f);
        ui_tip(ui, "Top of the band, above the table");
        nk_property_double(c, "#to", 0, &j->bands[i].y1, 10000, 5, 1.0f);
        ui_tip(ui, "Remove this band");
        if (nk_button_label(c, "Remove")) {
            memmove(&j->bands[i], &j->bands[i + 1], (size_t)(j->nbands - i - 1) * sizeof j->bands[0]);
            j->nbands--;
            break;
        }
        if (j->bands[i].y1 < j->bands[i].y0 + 1)
            j->bands[i].y1 = j->bands[i].y0 + 1;
    }
    ui_form_row(ui, ROW_H);
    nk_skip(c);
    ui_tip(ui, "Add a band above the last one");
    if (nk_button_label(c, "Add a band") && j->nbands < RG_MAX_BANDS) {
        double top = j->nbands ? j->bands[j->nbands - 1].y1 + 250 : 100;
        j->bands[j->nbands].y0 = top;
        j->bands[j->nbands].y1 = top + 200;
        j->nbands++;
    }
}

static void settings_flat(RgUi *ui)
{
    RgJob *j = &ui->job;
    ui_gap(ui, 6);
    ui_section(ui, "The flat part");
    vector(ui, "Drawing origin", "Where the drawing's 0,0 is on the part's surface, in the "
           "robot's base frame", &j->plane, rg_v3(800, -300, 200), 10000, 5, "mm");
    rotation(ui, "Drawing axes", "Turns the robot base frame's X, Y and Z onto the drawing's X, "
             "Y and the surface normal. 1 0 0 0: the part lies flat, square to the robot",
             &j->plane_rot);
    number(ui, "Spray speed", "The gun's speed along a stroke", &j->spray_speed, 300, 1, 2000,
           5, "mm/s");
}

static void settings_process(RgUi *ui)
{
    RgJob *j = &ui->job;
    ui_gap(ui, 6);
    ui_section(ui, "The process");
    number(ui, "Fan width", "The spray pattern's width at the standoff", &j->fan_width, 80, 5,
           1000, 1, "mm");
    ui_prop(ui, "Overlap", "How much of the fan each pass covers again", &j->overlap, 0, 90, 1, "%");
    number(ui, "Standoff", "Gun tip to the surface", &j->standoff, 200, 10, 1000, 5, "mm");
    ui_prop(ui, "Approach", "How far the gun stands back before and after spraying", &j->approach,
            20, 1000, 5, "mm");
    ui_prop(ui, "Travel speed", "Between the parts of the program, in the air",
            &j->travel_speed, 10, 2000, 10, "mm/s");
    ui_prop(ui, "Approach speed", "Onto and off the part", &j->approach_speed, 5, 1000, 5, "mm/s");
    ui_prop(ui, "Most spray speed", "Refuse a program that would spray faster than this",
            &j->max_spray_speed, 1, 2000, 10, "mm/s");
}

static void settings_gun(RgUi *ui)
{
    RgJob *j = &ui->job;
    ui_gap(ui, 6);
    ui_section(ui, "The gun");
    text_field(ui, "Tool name", "The tooldata the program moves", j->tool, sizeof j->tool);
    ui_check(ui, "Declare the tool in the program", "Write the tooldata into the program. Leave "
             "off when the controller already has this tool: two definitions will not load",
             &j->tool_define);
    vector(ui, "Gun tip", "The gun tip in the flange frame, as calibrated on the pendant",
           &j->tool_tcp, rg_v3(0, 0, 200), 2000, 1, "mm");
    rotation(ui, "Gun direction", "The tool frame's rotation on the flange: tool Z is the spray "
             "direction, tool X the fan's long axis", &j->tool_rot);
    if (j->tool_define) {
        number(ui, "Tool mass", "The gun's mass, which the controller needs to supervise the "
               "arm's load", &j->tool_mass, 2.0, 0.01, 50, 0.1, "kg");
        vector(ui, "Centre of mass", "The gun's centre of mass in the flange frame", &j->tool_cog,
               rg_v3(0, 0, 80), 2000, 1, "mm");
    }
}

static void settings_program(RgUi *ui)
{
    struct nk_context *c = ui->ctx;
    RgJob *j = &ui->job;
    ui_gap(ui, 6);
    ui_section(ui, "Around the program");
    static const char *axis[6] = { "#1", "#2", "#3", "#4", "#5", "#6" };
    for (int row = 0; row < 2; row++) {
        nk_layout_row_template_begin(c, S(ui, ROW_H));
        nk_layout_row_template_push_static(c, S(ui, LABEL_W));
        for (int i = 0; i < 3; i++)
            nk_layout_row_template_push_dynamic(c);
        nk_layout_row_template_push_static(c, S(ui, UNIT_W));
        nk_layout_row_template_end(c);
        label_cell(ui, row == 0 ? "Home, axes 1-3" : "Home, axes 4-6");
        for (int i = 0; i < 3; i++) {
            ui_tip(ui, "The joint angles the program starts and ends at");
            nk_property_double(c, axis[row * 3 + i], -400, &j->home[row * 3 + i], 400, 1, 0.25f);
        }
        nk_label_colored(c, "deg", NK_TEXT_LEFT, ui->theme->text_faint);
    }
    ui_check(ui, "Ask the operator before spraying", "Stop at the start of the first band or "
             "stroke with a pendant prompt, so the rotator and gun can be started", &j->ready_prompt);
    text_field(ui, "Gun output", "A digital output the program sets while spraying. Needed for "
               "more than one band or stroke; empty when the gun is switched from outside",
               j->gun_signal, sizeof j->gun_signal);
}

static void settings_rules(RgUi *ui)
{
    RgJob *j = &ui->job;
    ui_gap(ui, 6);
    ui_section(ui, "Rules every program is checked against");
    ui_prop(ui, "Wrist", "Axis 5 at least this far from straight: near the singularity the "
            "wrist spins and the gun's speed is not held", &j->min_wrist, 0, 60, 1, "deg");
    ui_prop(ui, "Joint margin", "Every joint at least this far from its limit", &j->min_margin,
            0, 45, 1, "deg");
    ui_prop(ui, "Clearance", "The wrist, flange and gun tip at least this far from the part",
            &j->clearance, 0, 1000, 5, "mm");
    ui_prop(ui, "Flip", "A joint turning more than this between samples is a flip",
            &j->max_joint_step, 1, 90, 1, "deg");
    ui_prop(ui, "Check every", "How often moves are checked along their length",
            &j->sample_step, 0.5, 100, 0.5, "mm");
    if (j->part == RG_PART_CYLINDER)
        ui_prop(ui, "Wrap tolerance", "How far a drawing's width may differ from the "
                "circumference", &j->wrap_tolerance, 0, 100, 0.5, "mm");
    ui_prop(ui, "Joining", "Line ends closer than this are joined into outlines",
            &j->join_tolerance, 0.001, 10, 0.01, "mm");
    ui_prop(ui, "Arc accuracy", "Arcs become straight pieces no further than this from the curve",
            &j->chord_tolerance, 0.01, 5, 0.01, "mm");
}

void page_settings(RgUi *ui, struct nk_rect inner)
{
    (void)inner;
    settings_job(ui);
    if (ui->job.part == RG_PART_FLAT)
        settings_flat(ui);
    else
        settings_cylinder(ui);
    settings_process(ui);
    settings_gun(ui);
    settings_program(ui);
    settings_rules(ui);
    ui_gap(ui, 12);
}

/* ------------------------------------------------------------------ */
/* Program page                                                        */
/* ------------------------------------------------------------------ */

static void text_pane(RgUi *ui, const char *id, const char *title, const char *text)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;
    if (!nk_group_begin_titled(c, id, title, NK_WINDOW_BORDER | NK_WINDOW_TITLE))
        return;
    float lh = c->style.font->height + S(ui, 3);
    int lines = 0;
    const char *p = text;
    while (p && *p && lines < 6000) {
        const char *nl = strchr(p, '\n');
        int len = nl ? (int)(nl - p) : (int)strlen(p);
        nk_layout_row_static(c, lh, (int)S(ui, 1500), 1);
        nk_text_colored(c, p, len, NK_TEXT_LEFT, t->text);
        p = nl ? nl + 1 : p + len;
        lines++;
    }
    nk_group_end(c);
}

void page_program(RgUi *ui, struct nk_rect inner)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;

    nk_layout_row_template_begin(c, S(ui, 34));
    nk_layout_row_template_push_dynamic(c);
    nk_layout_row_template_push_static(c, S(ui, 130));
    nk_layout_row_template_push_static(c, S(ui, 150));
    nk_layout_row_template_end(c);

    char head[256], name[64];
    rg_rapid_filename(&ui->job, name, sizeof name);
    struct nk_color col;
    if (!ui->plan_built) {
        snprintf(head, sizeof head, "Not ready: the job is not complete");
        col = t->warn;
    } else if (ui->plan.refused) {
        int n = rg_plan_count(&ui->plan, RG_REFUSE);
        snprintf(head, sizeof head, "Refused for %d reason%s: no program can be written", n,
                 n == 1 ? "" : "s");
        col = t->alarm;
    } else {
        snprintf(head, sizeof head, "Ready: %s and its report will be written beside the job", name);
        col = t->ok;
    }
    nk_label_colored(c, head, NK_TEXT_LEFT, col);
    ui_tip(ui, "Run every check again now");
    if (nk_button_label(c, "Check again"))
        ui_check_now(ui);
    bool can = ui->plan_built && !ui->plan.refused;
    if (!can)
        nk_widget_disable_begin(c);
    ui_tip(ui, can ? "Write the program and its report beside the job file"
                   : "Nothing can be written until every refusal is fixed");
    if (ui_primary_button(ui, "Write program") && can)
        ui_write_program(ui);
    if (!can)
        nk_widget_disable_end(c);

    if (!ui->plan_built && ui->plan_err[0])
        ui_label_wrap(ui, ui->plan_err, t->warn);
    if (ui->plan_built) {
        int shown = 0;
        for (int pass = 0; pass < 2; pass++)
            for (int i = 0; i < ui->plan.nissues && shown < 8; i++) {
                const RgIssue *is = &ui->plan.issues[i];
                if (is->sev != (pass == 0 ? RG_REFUSE : RG_WARN))
                    continue;
                char line[460];
                snprintf(line, sizeof line, "%s %s", is->sev == RG_REFUSE ? "\xe2\x9c\x97" : "!",
                         is->text);
                ui_label_wrap(ui, line, is->sev == RG_REFUSE ? t->alarm : t->warn);
                shown++;
            }
    }

    float y = nk_widget_bounds(c).y;
    float rest = inner.y + inner.h - y - c->style.window.padding.y;
    if (rest < S(ui, 140))
        rest = S(ui, 140);
    nk_layout_row_dynamic(c, rest, 2);
    text_pane(ui, "report", "Report", ui->report.s ? ui->report.s
              : (ui->plan_err[0] ? ui->plan_err : "The checks have not run yet."));
    text_pane(ui, "program", "Program preview", ui->program.s ? ui->program.s
              : "No program: the checks have to pass first.");
}
