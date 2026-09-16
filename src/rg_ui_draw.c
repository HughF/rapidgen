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
 * rg_ui_draw.c — the Draw page: the part's drawing, the pattern painted over
 * it, and the tools that paint it
 *
 * The canvas shows drawing millimetres with Y up, as CAD does. Each stroke is
 * drawn twice: a band as wide as the spray over the stretches on the work -
 * what will actually be coated - and its centre line, with a number at its
 * start and arrows along it, because the order and direction are the program.
 * Lead-ins, run-outs and turnarounds, off the work, are drawn faint and thin.
 *
 * Tools:
 *   Select  pick a stroke; drag to pan, scroll to zoom
 *   Line    click points, snapping to the drawing's corners and lines
 *   Draw    freehand, thinned to what matters when the button is let go
 *   Trace   round an outline, inset by an amount
 *   Fill    the region clicked, previewed on hover: rings following the
 *           outline inward, or parallel passes across it
 *   Scale   two points a known distance apart set the drawing's scale
 */
#include "rg_ui_int.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SNAP_PX      10.0f     /* snapping reach, in design pixels          */
#define PICK_PX       8.0f     /* how near a stroke or outline a click counts */
#define FREE_STEP_PX  2.0f     /* freehand samples at least this far apart  */
#define ZOOM_MIN   1e-4
#define ZOOM_MAX   500.0

static const char *TOOL_NAME[TOOL_COUNT] = {
    "Select", "Line", "Draw", "Trace", "Fill", "Scale"
};

static const char *TOOL_TIP[TOOL_COUNT] = {
    "Select a stroke by clicking it; drag to move round, scroll to zoom, Home to "
    "fit everything in view (S)",
    "Paint a stroke point by point. Points snap to the drawing's corners and "
    "lines; hold Shift not to snap. Double-click or Enter to finish (L)",
    "Paint a stroke freehand. It is smoothed when you let go (D)",
    "Trace an outline: click near it, and the gun goes round once from that "
    "point, inset by the amount set (T)",
    "Fill a region: click inside an outline to cover it with passes a step-over "
    "apart, turning round off the work, or with rings following it inward. Hover to "
    "preview (F)",
    "Scale the drawing: click two points a known distance apart, then type the "
    "true distance (M)",
};

static double dist(RgPt a, RgPt b)
{
    return hypot(a.x - b.x, a.y - b.y);
}

static double seg_dist(RgPt p, RgPt a, RgPt b)
{
    double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
    double t = len2 > 0.0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
    t = fmax(0.0, fmin(1.0, t));
    RgPt q = { a.x + t * dx, a.y + t * dy };
    return dist(p, q);
}

static void clear_preview(RgUi *ui)
{
    rg_strokes_free(ui->preview, ui->npreview);
    ui->preview = NULL;
    ui->npreview = 0;
    ui->preview_loop = -1;
}

void draw_forget(RgUi *ui)
{
    ui->ndraft = 0;
    ui->freehand = false;
    ui->panning = false;
    ui->selected = ui->hover_stroke = ui->hover_loop = -1;
    ui->scale_clicks = 0;
    ui->gen_loop = -1;
    ui->gen_count = 0;
    clear_preview(ui);
}

void draw_shutdown(RgUi *ui)
{
    clear_preview(ui);
    free(ui->draft);
    free(ui->scratch);
    ui->draft = NULL;
    ui->scratch = NULL;
}

static bool painting(const RgUi *ui)
{
    return ui->job.part == RG_PART_FLAT;
}

static double fill_pitch(const RgUi *ui)
{
    return rg_job_step(&ui->job);
}

static double spray_width(const RgUi *ui)
{
    return rg_job_width(&ui->job);
}

/*
 * How far off the work the gun runs before it turns round or leaves: the
 * shop's rule of 5 x the spot, or further if the gun needs more than that to
 * reach spray speed and stop again. The robot does not switch the torch, so
 * this is where the gun comes on and goes off the work.
 */
static double turn_length(const RgUi *ui)
{
    const RgJob *j = &ui->job;
    double w = spray_width(ui);
    if (!(w > 0.0))
        return 0.0;
    double t = 5.0 * w;
    if (j->spray_speed > 0.0 && j->accel > 0.0)
        t = fmax(t, j->spray_speed * j->spray_speed / (2.0 * j->accel) + 0.5 * w);
    return t;
}

/* ------------------------------------------------------------------ */
/* View                                                                */
/* ------------------------------------------------------------------ */

static float to_sx(const RgUi *ui, double x)
{
    return ui->canvas.x + ui->canvas.w * 0.5f + (float)((x - ui->cx) * ui->zoom);
}

static float to_sy(const RgUi *ui, double y)
{
    return ui->canvas.y + ui->canvas.h * 0.5f - (float)((y - ui->cy) * ui->zoom);
}

static RgPt to_mm(const RgUi *ui, float px, float py)
{
    RgPt p;
    p.x = ui->cx + (px - ui->canvas.x - ui->canvas.w * 0.5f) / ui->zoom;
    p.y = ui->cy - (py - ui->canvas.y - ui->canvas.h * 0.5f) / ui->zoom;
    return p;
}

static void grow_bounds(double *b, RgPt p)
{
    b[0] = fmin(b[0], p.x);
    b[1] = fmin(b[1], p.y);
    b[2] = fmax(b[2], p.x);
    b[3] = fmax(b[3], p.y);
}

static bool drawing_bounds(const RgUi *ui, double *b)
{
    b[0] = b[1] = DBL_MAX;
    b[2] = b[3] = -DBL_MAX;
    if (ui->have_raw)
        for (int i = 0; i < ui->drawing.n; i++)
            for (int k = 0; k < ui->drawing.paths[i].n; k++)
                grow_bounds(b, ui->drawing.paths[i].pts[k]);
    return b[0] <= b[2];
}

static void fit_view(RgUi *ui)
{
    if (ui->canvas.w < 20 || ui->canvas.h < 20)
        return;
    double b[4];
    drawing_bounds(ui, b);
    for (int s = 0; s < ui->job.nstrokes; s++)
        for (int k = 0; k < ui->job.strokes[s].n; k++)
            grow_bounds(b, ui->job.strokes[s].pts[k]);
    if (b[0] > b[2]) {
        b[0] = 0; b[1] = 0; b[2] = 600; b[3] = 400;
    }
    double w = fmax(b[2] - b[0], 1.0), h = fmax(b[3] - b[1], 1.0);
    ui->zoom = fmin(ui->canvas.w / (w * 1.15), ui->canvas.h / (h * 1.15));
    ui->zoom = fmax(ZOOM_MIN, fmin(ZOOM_MAX, ui->zoom));
    ui->cx = 0.5 * (b[0] + b[2]);
    ui->cy = 0.5 * (b[1] + b[3]);
    ui->fit_pending = false;
}

/* ------------------------------------------------------------------ */
/* Drawing primitives                                                  */
/* ------------------------------------------------------------------ */

static void text_at(RgUi *ui, struct nk_command_buffer *cb, float x, float y, const char *s,
                    struct nk_color col, int align)
{
    const struct nk_user_font *f = ui->ctx->style.font;
    int len = (int)strlen(s);
    float w = ui_text_width(ui, s, len), h = f->height;
    if (align == 0)
        x -= w / 2.0f;
    else if (align > 0)
        x -= w;
    nk_draw_text(cb, nk_rect(x, y - h / 2.0f, w + 2.0f, h), s, len, f, nk_rgba(0, 0, 0, 0), col);
}

static struct nk_color alpha(struct nk_color c, nk_byte a)
{
    c.a = a;
    return c;
}

/*
 * A colour blended against the canvas once, and opaque afterwards.
 *
 * The band a stroke covers is drawn as many overlapping quads and discs.
 * Translucent, every overlap darkens the one beneath it, so passes a
 * step-over apart stack up into visible boxes instead of the one continuous
 * area they really coat. Blended here and filled opaque, the pieces paint the
 * same colour over themselves and merge into a single even shape.
 */
static struct nk_color over(struct nk_color bg, struct nk_color c, nk_byte a)
{
    struct nk_color r;
    r.r = (nk_byte)((int)bg.r + ((int)c.r - (int)bg.r) * (int)a / 255);
    r.g = (nk_byte)((int)bg.g + ((int)c.g - (int)bg.g) * (int)a / 255);
    r.b = (nk_byte)((int)bg.b + ((int)c.b - (int)bg.b) * (int)a / 255);
    r.a = 255;
    return r;
}

static void polyline(RgUi *ui, struct nk_command_buffer *cb, const RgPt *p, int n, bool closed,
                     float thick, struct nk_color col)
{
    int m = n + (closed && n > 2 ? 1 : 0);
    if (m < 2)
        return;
    if (2 * m > ui->scratch_cap) {
        int cap = 2 * m + 256;
        float *v = realloc(ui->scratch, (size_t)cap * sizeof *v);
        if (!v)
            return;
        ui->scratch = v;
        ui->scratch_cap = cap;
    }
    for (int i = 0; i < m; i++) {
        RgPt q = p[i % n];
        ui->scratch[2 * i] = to_sx(ui, q.x);
        ui->scratch[2 * i + 1] = to_sy(ui, q.y);
    }
    nk_stroke_polyline(cb, ui->scratch, m, thick, col);
}

static void dot(struct nk_command_buffer *cb, float x, float y, float r, struct nk_color col)
{
    nk_fill_circle(cb, nk_rect(x - r, y - r, 2 * r, 2 * r), col);
}

/*
 * What the spray actually covers.
 *
 * A thermal-spray torch lays down a round spot, so the covered strip is the
 * path swept by a disc: a rectangle square to each segment, with a disc in
 * the corners, and it is the same width whichever way the stroke runs.
 *
 * A paint fan is a slot lying along `fan_along` that does not turn with the
 * path, so each segment covers a parallelogram instead, and a stroke running
 * along the fan covers barely more than a line — which is the point of
 * drawing the two differently.
 *
 * Neither is one thick polyline: Nuklear mitres a thick line's joins and
 * throws out wedges far outside the sprayed area at every corner.
 */
static void band(RgUi *ui, struct nk_command_buffer *cb, const RgPt *p, int n, float w,
                 struct nk_color col)
{
    if (w < 2.0f || n < 2)
        return;
    bool spot = ui->job.pattern != RG_PAT_FAN;
    double a = ui->job.fan_along * RG_DEG;
    float hx = (float)cos(a) * w * 0.5f, hy = -(float)sin(a) * w * 0.5f;   /* screen Y is down */
    float r = w * 0.5f;

    for (int k = 1; k < n; k++) {
        float ax = to_sx(ui, p[k - 1].x), ay = to_sy(ui, p[k - 1].y);
        float bx = to_sx(ui, p[k].x), by = to_sy(ui, p[k].y);
        if (spot) {
            float dx = bx - ax, dy = by - ay, len = (float)hypot(dx, dy);
            if (len < 1e-3f)
                continue;
            hx = -dy / len * r;
            hy = dx / len * r;
        }
        float quad[8] = { ax + hx, ay + hy, bx + hx, by + hy,
                          bx - hx, by - hy, ax - hx, ay - hy };
        nk_fill_polygon(cb, quad, 4, col);
    }
    if (!spot)
        return;
    /* Round the corners, where the disc pivots — and the ends, which are
     * where the spot sits when the gun arrives and leaves. */
    for (int k = 0; k < n; k++) {
        if (k > 0 && k + 1 < n) {
            double ax = p[k].x - p[k - 1].x, ay = p[k].y - p[k - 1].y;
            double bx = p[k + 1].x - p[k].x, by = p[k + 1].y - p[k].y;
            double la = hypot(ax, ay), lb = hypot(bx, by);
            if (la < 1e-9 || lb < 1e-9 || (ax * bx + ay * by) / (la * lb) > 0.985)
                continue;
        }
        dot(cb, to_sx(ui, p[k].x), to_sy(ui, p[k].y), r, col);
    }
}

static void grid(RgUi *ui, struct nk_command_buffer *cb, struct nk_rect r)
{
    const RgTheme *t = ui->theme;
    nk_fill_rect(cb, r, 0, t->plot_bg);

    double target = S(ui, 40) / ui->zoom;
    double p = pow(10.0, floor(log10(target)));
    double step = p * 10.0;
    static const double mult[] = { 1.0, 2.0, 5.0, 10.0 };
    for (int i = 0; i < 4; i++)
        if (p * mult[i] >= target) {
            step = p * mult[i];
            break;
        }

    RgPt lo = to_mm(ui, r.x, r.y + r.h), hi = to_mm(ui, r.x + r.w, r.y);
    struct nk_color minor = t->plot_grid, major = alpha(t->plot_axis, 90);
    char lab[32];

    for (double x = floor(lo.x / step) * step; x <= hi.x; x += step) {
        long idx = lround(x / step);
        float sx = to_sx(ui, x);
        bool big = idx % 5 == 0;
        nk_stroke_line(cb, sx, r.y, sx, r.y + r.h, 1.0f, big ? major : minor);
        if (big) {
            snprintf(lab, sizeof lab, "%g", fabs(x) < step * 1e-6 ? 0.0 : x);
            text_at(ui, cb, sx + S(ui, 3), r.y + r.h - S(ui, 10), lab, t->text_faint, -1);
        }
    }
    for (double y = floor(lo.y / step) * step; y <= hi.y; y += step) {
        long idx = lround(y / step);
        float sy = to_sy(ui, y);
        bool big = idx % 5 == 0;
        nk_stroke_line(cb, r.x, sy, r.x + r.w, sy, 1.0f, big ? major : minor);
        if (big) {
            snprintf(lab, sizeof lab, "%g", fabs(y) < step * 1e-6 ? 0.0 : y);
            text_at(ui, cb, r.x + S(ui, 4), sy - S(ui, 8), lab, t->text_faint, -1);
        }
    }
    float ox = to_sx(ui, 0), oy = to_sy(ui, 0);
    nk_stroke_line(cb, ox, r.y, ox, r.y + r.h, S(ui, 1.5f), t->plot_axis);
    nk_stroke_line(cb, r.x, oy, r.x + r.w, oy, S(ui, 1.5f), t->plot_axis);
}

static void arrow(RgUi *ui, struct nk_command_buffer *cb, float ax, float ay, float bx, float by,
                  struct nk_color col)
{
    float dx = bx - ax, dy = by - ay, len = (float)hypot(dx, dy);
    if (len < S(ui, 56))
        return;
    dx /= len;
    dy /= len;
    float mx = 0.5f * (ax + bx), my = 0.5f * (ay + by), s = S(ui, 6);
    nk_fill_triangle(cb, mx + dx * s, my + dy * s,
                     mx - dx * s - dy * s * 0.7f, my - dy * s + dx * s * 0.7f,
                     mx - dx * s + dy * s * 0.7f, my - dy * s - dx * s * 0.7f, col);
}

/* What the stroke coats on the part: the band follows only the stretches on
 * the work. Drawn under the drawing, because it is opaque. */
static void draw_stroke_band(RgUi *ui, struct nk_command_buffer *cb, const RgStroke *st,
                             bool selected)
{
    const RgTheme *t = ui->theme;
    struct nk_color line = selected ? t->warn : t->trace_a;
    float fan = (float)(spray_width(ui) * ui->zoom);
    if (!isfinite(fan) || fan <= S(ui, 3))
        return;
    struct nk_color col = over(t->plot_bg, line, selected ? 70 : 42);
    for (int a = 0; a < st->n;) {
        if (rg_stroke_off(st, a)) {
            a++;
            continue;
        }
        int b = a;
        while (b + 1 < st->n && rg_stroke_work_seg(st, b + 1))
            b++;
        if (b > a)
            band(ui, cb, st->pts + a, b - a + 1, fan, col);
        a = b + 1;
    }
}

/*
 * A path with its stretches off the work drawn apart from the work: thinner
 * and faint, so a lead-in, run-out or turnaround reads as the gun passing
 * by, not as coating.
 */
static void draw_path(RgUi *ui, struct nk_command_buffer *cb, const RgStroke *st, float thick,
                      struct nk_color col, bool arrows)
{
    if (!st->off) {
        polyline(ui, cb, st->pts, st->n, false, thick, col);
    } else {
        for (int k = 1; k < st->n; k++) {
            bool work = rg_stroke_work_seg(st, k);
            polyline(ui, cb, st->pts + k - 1, 2, false, work ? thick : thick * 0.6f,
                     work ? col : alpha(col, 110));
        }
    }
    for (int k = 1; arrows && k < st->n; k++)
        arrow(ui, cb, to_sx(ui, st->pts[k - 1].x), to_sy(ui, st->pts[k - 1].y),
              to_sx(ui, st->pts[k].x), to_sy(ui, st->pts[k].y),
              rg_stroke_work_seg(st, k) ? col : alpha(col, 110));
}

static void draw_stroke(RgUi *ui, struct nk_command_buffer *cb, const RgStroke *st, int index,
                        bool selected, bool hovered)
{
    const RgTheme *t = ui->theme;
    struct nk_color line = selected ? t->warn : t->trace_a;
    draw_path(ui, cb, st, S(ui, selected || hovered ? 2.6f : 1.8f), line, true);

    float x0 = to_sx(ui, st->pts[0].x), y0 = to_sy(ui, st->pts[0].y);
    dot(cb, x0, y0, S(ui, 4.5f), line);
    char num[16];
    snprintf(num, sizeof num, "%d", index + 1);
    text_at(ui, cb, x0 + S(ui, 8), y0 - S(ui, 11), num, line, -1);
}

/* ------------------------------------------------------------------ */
/* Painting                                                            */
/* ------------------------------------------------------------------ */

static void draft_add(RgUi *ui, RgPt p)
{
    if (ui->ndraft && dist(p, ui->draft[ui->ndraft - 1]) < 1e-6)
        return;
    if (ui->ndraft == ui->cap_draft) {
        int cap = ui->cap_draft ? ui->cap_draft * 2 : 256;
        RgPt *d = realloc(ui->draft, (size_t)cap * sizeof *d);
        if (!d)
            return;
        ui->draft = d;
        ui->cap_draft = cap;
    }
    ui->draft[ui->ndraft++] = p;
}

static void commit_stroke(RgUi *ui, const RgPt *pts, int n, const unsigned char *off,
                          const char *how)
{
    if (n < 2) {
        ui_message(ui, true, "A stroke needs at least two points.");
        return;
    }
    if (!rg_job_add_stroke_off(&ui->job, pts, n, off)) {
        ui_message(ui, true, "Out of memory.");
        return;
    }
    ui->selected = ui->job.nstrokes - 1;
    ui_message(ui, false, "%s stroke %d: %d points, %.0f mm", how, ui->job.nstrokes, n,
               rg_stroke_length(pts, n));
}

/*
 * A drawn open stroke gets a straight lead-in and run-out along its end
 * segments, off the work. The robot does not switch the torch, so the gun has
 * to be up to speed before it reaches the work and clear of it before it
 * slows. A closed stroke is a circuit and has no end to add them to.
 */
static void commit_drawn(RgUi *ui, const RgPt *pts, int n, const char *how)
{
    double t = turn_length(ui);
    if (n < 2 || !(t > 0.0) || (n > 2 && dist(pts[0], pts[n - 1]) < 1e-6)) {
        commit_stroke(ui, pts, n, NULL, how);
        return;
    }
    double d0 = dist(pts[0], pts[1]), d1 = dist(pts[n - 2], pts[n - 1]);
    if (d0 < 1e-9 || d1 < 1e-9) {
        commit_stroke(ui, pts, n, NULL, how);
        return;
    }
    RgPt *p = malloc((size_t)(n + 2) * sizeof *p);
    unsigned char *off = calloc((size_t)(n + 2), 1);
    if (!p || !off) {
        free(p);
        free(off);
        ui_message(ui, true, "Out of memory.");
        return;
    }
    p[0].x = pts[0].x + (pts[0].x - pts[1].x) / d0 * t;
    p[0].y = pts[0].y + (pts[0].y - pts[1].y) / d0 * t;
    memcpy(p + 1, pts, (size_t)n * sizeof *p);
    p[n + 1].x = pts[n - 1].x + (pts[n - 1].x - pts[n - 2].x) / d1 * t;
    p[n + 1].y = pts[n - 1].y + (pts[n - 1].y - pts[n - 2].y) / d1 * t;
    off[0] = off[n + 1] = 1;
    commit_stroke(ui, p, n + 2, off, how);
    free(p);
    free(off);
}

/*
 * Clicking a region a second time refines its path rather than laying
 * another one over the top. What the last Fill or Trace made is remembered,
 * and checked against the job before anything is removed — Delete, Clear all
 * and Undo can all have moved or removed it since.
 */
static void drop_generated(RgUi *ui, int loop)
{
    if (ui->gen_loop != loop || ui->gen_tool != (int)ui->tool || ui->gen_count <= 0)
        return;
    if (ui->gen_first < 0 || ui->gen_first + ui->gen_count > ui->job.nstrokes)
        return;
    const RgStroke *st = &ui->job.strokes[ui->gen_first];
    if (st->n != ui->gen_n || st->n < 1 || dist(st->pts[0], ui->gen_p0) > 1e-9)
        return;
    for (int i = ui->gen_count - 1; i >= 0; i--)
        rg_job_delete_stroke(&ui->job, ui->gen_first + i);
    ui->gen_count = 0;
}

static void remember_generated(RgUi *ui, int loop, int first, int count)
{
    ui->gen_loop = loop;
    ui->gen_tool = (int)ui->tool;
    ui->gen_first = first;
    ui->gen_count = count;
    ui->gen_n = ui->job.strokes[first].n;
    ui->gen_p0 = ui->job.strokes[first].pts[0];
}

static void finish_line(RgUi *ui)
{
    if (ui->ndraft >= 2)
        commit_drawn(ui, ui->draft, ui->ndraft, "Painted");
    else if (ui->ndraft)
        ui_message(ui, true, "A stroke needs at least two points.");
    ui->ndraft = 0;
}

static void finish_freehand(RgUi *ui)
{
    ui->freehand = false;
    if (ui->ndraft >= 2) {
        RgPt *out = malloc((size_t)ui->ndraft * sizeof *out);
        if (out) {
            int n = rg_simplify(ui->draft, ui->ndraft, ui->smoothing, out);
            commit_drawn(ui, out, n, "Drew");
            free(out);
        }
    }
    ui->ndraft = 0;
}

static void trace_loop(RgUi *ui, int loop, RgPt near)
{
    RgPt *pts;
    int n;
    if (!rg_pattern_trace(&ui->shape.loops[loop], near, ui->inset, &pts, &n)) {
        ui_message(ui, true, "That outline could not be traced.");
        return;
    }
    drop_generated(ui, loop);
    int before = ui->job.nstrokes;
    commit_stroke(ui, pts, n, NULL, "Traced an outline as");
    if (ui->job.nstrokes == before + 1)
        remember_generated(ui, loop, before, 1);
    else
        ui->gen_count = 0;
    free(pts);
}

static void fill_loop(RgUi *ui, int loop)
{
    double pitch = fill_pitch(ui);
    if (!(pitch > 0.5)) {
        ui_message(ui, true, "Set the spot and the step-over first: they space the passes.");
        return;
    }
    /* The gun covers nothing until it has a width, and NaN would spiral the
     * rings off to nowhere. */
    double half = spray_width(ui) > 0.0 ? 0.5 * spray_width(ui) : 0.0;
    RgStroke *st;
    int n;
    if (ui->fill_spiral) {
        n = rg_pattern_spiral(&ui->shape, loop, half, pitch, &st);
        if (n <= 0) {
            ui_message(ui, n < 0, n < 0 ? "Out of memory."
                       : "No spiral fits there: the region is narrower than the spray, or it "
                         "has a hole in it.");
            return;
        }
    } else {
        RgFillOpts o = { pitch, ui->fill_angle, ui->fill_extend ? half : 0.0, turn_length(ui) };
        n = rg_pattern_fill(&ui->shape, loop, &o, &st);
        if (n <= 0) {
            ui_message(ui, n < 0, n < 0 ? "Out of memory." : "There is nothing to fill there.");
            return;
        }
    }
    drop_generated(ui, loop);
    int before = ui->job.nstrokes, added = 0;
    for (int i = 0; i < n; i++)
        added += rg_job_add_stroke_off(&ui->job, st[i].pts, st[i].n, st[i].off);
    rg_strokes_free(st, n);
    ui->selected = ui->job.nstrokes - 1;
    if (added > 0)
        remember_generated(ui, loop, before, added);
    else
        ui->gen_count = 0;
    if (ui->fill_spiral)
        ui_message(ui, false, "Spiralled inward as %d stroke%s, rings %.1f mm apart", added,
                   added == 1 ? "" : "s", pitch);
    else
        ui_message(ui, false, "Filled with %d stroke%s, passes %.1f mm apart", added,
                   added == 1 ? "" : "s", pitch);
}

static void update_fill_preview(RgUi *ui)
{
    double pitch = fill_pitch(ui);
    /* The spiral flag belongs in the key: without it, flipping the toggle
     * leaves the previous pattern on screen, and the preview is what the
     * region is aimed with. */
    double key[6] = { pitch, ui->fill_angle, ui->fill_extend ? spray_width(ui) : 0.0,
                      ui->shown_scale, ui->fill_spiral ? 1.0 : 0.0, turn_length(ui) };
    bool want = ui->tool == TOOL_FILL && ui->hover_loop >= 0 && pitch > 0.5 && painting(ui);
    if (!want) {
        clear_preview(ui);
        return;
    }
    if (ui->preview_loop == ui->hover_loop && memcmp(key, ui->preview_key, sizeof key) == 0)
        return;
    clear_preview(ui);
    double half = spray_width(ui) > 0.0 ? 0.5 * spray_width(ui) : 0.0;
    int n;
    if (ui->fill_spiral) {
        n = rg_pattern_spiral(&ui->shape, ui->hover_loop, half, pitch, &ui->preview);
    } else {
        RgFillOpts o = { pitch, ui->fill_angle, ui->fill_extend ? half : 0.0, turn_length(ui) };
        n = rg_pattern_fill(&ui->shape, ui->hover_loop, &o, &ui->preview);
    }
    ui->npreview = n > 0 ? n : 0;
    ui->preview_loop = ui->hover_loop;
    memcpy(ui->preview_key, key, sizeof key);
}

static int stroke_at(const RgUi *ui, RgPt p, double radius)
{
    int best = -1;
    double bd = radius;
    for (int s = 0; s < ui->job.nstrokes; s++) {
        const RgStroke *st = &ui->job.strokes[s];
        for (int k = 1; k < st->n; k++) {
            double d = seg_dist(p, st->pts[k - 1], st->pts[k]);
            if (d <= bd) {
                bd = d;
                best = s;
            }
        }
    }
    return best;
}

static void set_tool(RgUi *ui, Tool t)
{
    if (ui->tool != t) {
        if (ui->tool == TOOL_LINE && ui->ndraft >= 2)
            finish_line(ui);
        ui->ndraft = 0;
        ui->freehand = false;
        ui->scale_clicks = 0;
    }
    ui->tool = t;
}

/* ------------------------------------------------------------------ */
/* Canvas                                                              */
/* ------------------------------------------------------------------ */

static void canvas_input(RgUi *ui, struct nk_rect r)
{
    const struct nk_input *in = &ui->ctx->input;
    struct nk_vec2 mp = in->mouse.pos;
    bool over = ui->dialog == DLG_NONE && nk_input_is_mouse_hovering_rect(in, r);
    bool left_down = in->mouse.buttons[NK_BUTTON_LEFT].down;

    ui->mouse_in = over;
    ui->snap_kind = RG_SNAP_NONE;
    ui->hover_stroke = ui->hover_loop = -1;

    if (ui->freehand && !left_down)
        finish_freehand(ui);

    if (over && in->mouse.scroll_delta.y != 0.0f) {
        RgPt before = to_mm(ui, mp.x, mp.y);
        ui->zoom = fmax(ZOOM_MIN, fmin(ZOOM_MAX, ui->zoom * pow(1.25, in->mouse.scroll_delta.y)));
        RgPt after = to_mm(ui, mp.x, mp.y);
        ui->cx += before.x - after.x;
        ui->cy += before.y - after.y;
    }

    if (over && (nk_input_is_mouse_pressed(in, NK_BUTTON_RIGHT) ||
                 nk_input_is_mouse_pressed(in, NK_BUTTON_MIDDLE))) {
        ui->panning = true;
        ui->pan_last = mp;
    }
    if (ui->panning) {
        bool held = in->mouse.buttons[NK_BUTTON_RIGHT].down ||
                    in->mouse.buttons[NK_BUTTON_MIDDLE].down ||
                    (ui->tool == TOOL_PAN && left_down);
        if (!held) {
            ui->panning = false;
        } else {
            ui->cx -= (mp.x - ui->pan_last.x) / ui->zoom;
            ui->cy += (mp.y - ui->pan_last.y) / ui->zoom;
            ui->pan_last = mp;
            return;
        }
    }

    RgPt m = to_mm(ui, mp.x, mp.y);
    ui->mouse_mm = m;
    if (!over)
        return;

    double per_px = 1.0 / ui->zoom;
    bool shift = (SDL_GetModState() & KMOD_SHIFT) != 0;
    RgPt at = m;
    if ((ui->tool == TOOL_LINE || ui->tool == TOOL_SCALE) && ui->snap && !shift) {
        double rad = S(ui, SNAP_PX) * per_px;
        if (ui->tool == TOOL_LINE && ui->ndraft >= 2 && dist(m, ui->draft[0]) <= rad) {
            ui->snap_kind = RG_SNAP_VERTEX;           /* close the shape */
            ui->snap_pt = ui->draft[0];
        } else if (ui->have_raw) {
            ui->snap_kind = rg_snap_drawing(&ui->drawing, m, rad, &ui->snap_pt);
        }
        if (ui->snap_kind != RG_SNAP_NONE)
            at = ui->snap_pt;
    }

    bool press = nk_input_is_mouse_pressed(in, NK_BUTTON_LEFT);
    bool twice = nk_input_is_mouse_pressed(in, NK_BUTTON_DOUBLE);

    switch (ui->tool) {
    case TOOL_PAN:
        ui->hover_stroke = stroke_at(ui, m, S(ui, PICK_PX) * per_px);
        if (press) {
            ui->selected = ui->hover_stroke;
            if (ui->hover_stroke < 0) {
                ui->panning = true;
                ui->pan_last = mp;
            }
        }
        break;
    case TOOL_LINE:
        if (!painting(ui))
            break;
        if (twice)
            finish_line(ui);
        else if (press)
            draft_add(ui, at);
        break;
    case TOOL_FREE:
        if (!painting(ui))
            break;
        if (press) {
            ui->ndraft = 0;
            draft_add(ui, m);
            ui->freehand = true;
        } else if (ui->freehand && left_down && ui->ndraft &&
                   dist(m, ui->draft[ui->ndraft - 1]) * ui->zoom >= S(ui, FREE_STEP_PX)) {
            draft_add(ui, m);
        }
        break;
    case TOOL_TRACE:
        if (!ui->have_raw)
            break;
        ui->hover_loop = rg_shape_loop_near(&ui->shape, m, S(ui, PICK_PX) * 1.5 * per_px, NULL);
        if (press && painting(ui) && ui->hover_loop >= 0)
            trace_loop(ui, ui->hover_loop, m);
        break;
    case TOOL_FILL:
        if (!ui->have_raw)
            break;
        ui->hover_loop = rg_shape_loop_at(&ui->shape, m);
        if (press && painting(ui) && ui->hover_loop >= 0)
            fill_loop(ui, ui->hover_loop);
        break;
    case TOOL_SCALE:
        if (press) {
            if (ui->scale_clicks != 1) {
                ui->scale_a = at;
                ui->scale_clicks = 1;
            } else {
                ui->scale_b = at;
                ui->scale_clicks = 2;
                ui->scale_true = dist(ui->scale_a, ui->scale_b);
            }
        }
        break;
    default:
        break;
    }
}

static void centred_note(RgUi *ui, struct nk_command_buffer *cb, struct nk_rect r, float dy,
                         const char *s, struct nk_color col)
{
    text_at(ui, cb, r.x + r.w / 2.0f, r.y + r.h / 2.0f + dy, s, col, 0);
}

static void canvas_paint(RgUi *ui, struct nk_rect r)
{
    const RgTheme *t = ui->theme;
    struct nk_command_buffer *cb = nk_window_get_canvas(ui->ctx);
    struct nk_rect old = cb->clip;
    nk_push_scissor(cb, r);

    grid(ui, cb, r);

    /* The coating goes down first, under the drawing: it is opaque so that
     * overlapping passes merge into one area, and the outline has to stay
     * visible through it. */
    for (int s = 0; s < ui->job.nstrokes; s++)
        draw_stroke_band(ui, cb, &ui->job.strokes[s], s == ui->selected);

    if (ui->have_raw) {
        for (int i = 0; i < ui->drawing.n; i++)
            polyline(ui, cb, ui->drawing.paths[i].pts, ui->drawing.paths[i].n,
                     ui->drawing.paths[i].closed, S(ui, 1.3f), t->text_dim);
        if (ui->hover_loop >= 0 && ui->hover_loop < ui->shape.n) {
            const RgLoop *l = &ui->shape.loops[ui->hover_loop];
            polyline(ui, cb, l->pts, l->n, true, S(ui, 3.0f), alpha(t->accent, 200));
        }
    }

    if (ui->tool == TOOL_TRACE && ui->hover_loop >= 0 && painting(ui)) {
        RgPt *pts;
        int n;
        if (rg_pattern_trace(&ui->shape.loops[ui->hover_loop], ui->mouse_mm, ui->inset, &pts, &n)) {
            /* what the gun covers, which for a spot is not the fan width */
            float fan = (float)(spray_width(ui) * ui->zoom);
            if (isfinite(fan) && fan > S(ui, 3))
                band(ui, cb, pts, n, fan, alpha(t->trace_a, 28));
            polyline(ui, cb, pts, n, false, S(ui, 1.5f), alpha(t->trace_a, 150));
            dot(cb, to_sx(ui, pts[0].x), to_sy(ui, pts[0].y), S(ui, 4), t->trace_a);
            free(pts);
        }
    }
    for (int i = 0; i < ui->npreview; i++)
        draw_path(ui, cb, &ui->preview[i], S(ui, 1.3f), alpha(t->trace_a, 150), false);

    for (int s = 0; s < ui->job.nstrokes; s++)
        draw_stroke(ui, cb, &ui->job.strokes[s], s, s == ui->selected, s == ui->hover_stroke);

    if (ui->ndraft) {
        polyline(ui, cb, ui->draft, ui->ndraft, false, S(ui, 2.0f), t->accent);
        for (int i = 0; i < ui->ndraft && ui->tool == TOOL_LINE; i++)
            dot(cb, to_sx(ui, ui->draft[i].x), to_sy(ui, ui->draft[i].y), S(ui, 3), t->accent);
        if (ui->tool == TOOL_LINE && ui->mouse_in) {
            RgPt to = ui->snap_kind != RG_SNAP_NONE ? ui->snap_pt : ui->mouse_mm;
            RgPt last = ui->draft[ui->ndraft - 1];
            nk_stroke_line(cb, to_sx(ui, last.x), to_sy(ui, last.y), to_sx(ui, to.x), to_sy(ui, to.y),
                           S(ui, 1.2f), alpha(t->accent, 140));
        }
    }

    if (ui->mouse_in && ui->snap_kind != RG_SNAP_NONE) {
        float x = to_sx(ui, ui->snap_pt.x), y = to_sy(ui, ui->snap_pt.y), h = S(ui, 5);
        if (ui->snap_kind == RG_SNAP_VERTEX)
            nk_stroke_rect(cb, nk_rect(x - h, y - h, 2 * h, 2 * h), 0, S(ui, 1.6f), t->trace_c);
        else
            nk_stroke_circle(cb, nk_rect(x - h, y - h, 2 * h, 2 * h), S(ui, 1.6f), t->trace_c);
    }

    if (ui->scale_clicks >= 1) {
        RgPt b = ui->scale_clicks == 2 ? ui->scale_b
               : ui->snap_kind != RG_SNAP_NONE ? ui->snap_pt : ui->mouse_mm;
        float ax = to_sx(ui, ui->scale_a.x), ay = to_sy(ui, ui->scale_a.y);
        float bx = to_sx(ui, b.x), by = to_sy(ui, b.y);
        nk_stroke_line(cb, ax, ay, bx, by, S(ui, 2.0f), t->trace_d);
        dot(cb, ax, ay, S(ui, 4), t->trace_d);
        dot(cb, bx, by, S(ui, 4), t->trace_d);
        char lab[48];
        snprintf(lab, sizeof lab, "%.2f mm", dist(ui->scale_a, b));
        text_at(ui, cb, 0.5f * (ax + bx), 0.5f * (ay + by) - S(ui, 14), lab, t->trace_d, 0);
    }

    if (ui->drawing_err[0])
        text_at(ui, cb, r.x + S(ui, 12), r.y + S(ui, 16), ui->drawing_err, t->alarm, -1);
    if (!ui->have_raw && !ui->loaded_from[0] && ui->job.nstrokes == 0 && !ui->ndraft)
        centred_note(ui, cb, r, 0, painting(ui)
                     ? "Import the part's DXF from the rail, then paint over it"
                     : "Import the unrolled surface's DXF, or set the bands on the Settings page",
                     t->text_faint);
    if (!painting(ui) && ui->tool != TOOL_PAN && ui->tool != TOOL_SCALE)
        centred_note(ui, cb, r, S(ui, 24),
                     "Painting is for flat parts: this job is a cylinder (Settings page)", t->warn);

    if (ui->mouse_in) {
        char xy[64];
        snprintf(xy, sizeof xy, "x %.1f   y %.1f mm", ui->mouse_mm.x, ui->mouse_mm.y);
        text_at(ui, cb, r.x + r.w - S(ui, 12), r.y + S(ui, 16), xy, t->text_dim, 1);
    }

    nk_push_scissor(cb, old);
}

/* ------------------------------------------------------------------ */
/* Inspector                                                           */
/* ------------------------------------------------------------------ */

static void button_row(RgUi *ui, int n)
{
    nk_layout_row_dynamic(ui->ctx, S(ui, 28), n);
}

static bool button(RgUi *ui, const char *label, const char *tip, bool enabled)
{
    struct nk_context *c = ui->ctx;
    if (!enabled)
        nk_widget_disable_begin(c);
    ui_tip(ui, tip);
    bool hit = nk_button_label(c, label) && enabled;
    if (!enabled)
        nk_widget_disable_end(c);
    return hit;
}

static void inspect_drawing(RgUi *ui)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;

    ui_section(ui, "Drawing");
    if (ui->loaded_from[0]) {
        ui_form_row(ui, 22);
        nk_label_colored(c, "File", NK_TEXT_LEFT, t->text_dim);
        ui_tip(ui, ui->loaded_from);
        nk_label(c, plat_path_leaf(ui->loaded_from), NK_TEXT_LEFT);
        if (ui->drawing_err[0]) {
            ui_label_wrap(ui, ui->drawing_err, t->alarm);
        } else if (ui->have_raw) {
            double b[4];
            if (drawing_bounds(ui, b)) {
                ui_tip(ui, "Everything in the drawing, centre lines and borders included: "
                       "not necessarily the part itself");
                ui_info_rowf(ui, "Extents", "%.1f \xc3\x97 %.1f mm", b[2] - b[0], b[3] - b[1]);
            }
            if (ui->skipped)
                ui_info_rowf(ui, "Outlines", "%d closed, %d open line%s", ui->shape.n, ui->skipped,
                             ui->skipped == 1 ? "" : "s");
            else
                ui_info_rowf(ui, "Outlines", "%d closed", ui->shape.n);
            if (ui->drawing.unsupported) {
                char buf[200];
                snprintf(buf, sizeof buf, "%d item%s not shown (%s first): convert them to "
                         "polylines in CAD to trace them.", ui->drawing.unsupported,
                         ui->drawing.unsupported == 1 ? "" : "s", ui->drawing.unsupported_kind);
                ui_label_wrap(ui, buf, t->warn);
            }
            if (ui->drawing.degenerate) {
                char buf[220];
                snprintf(buf, sizeof buf, "%d %s%s left out: the drawing gives them no real "
                         "coordinates. A program cannot be generated from this drawing until "
                         "they are deleted in CAD.", ui->drawing.degenerate,
                         ui->drawing.degenerate_kind[0] ? ui->drawing.degenerate_kind : "entity",
                         ui->drawing.degenerate == 1 ? "" : "s");
                ui_label_wrap(ui, buf, t->warn);
            }
        }
    } else {
        ui_label_wrap(ui, "No drawing. Import the part's DXF to paint over it, or paint on "
                      "the grid in millimetres.", t->text_faint);
    }
    button_row(ui, 2);
    if (button(ui, "Import DXF...", "Show a part's drawing to paint over (Ctrl+I)", true))
        ui_open_file_dialog(ui, FILE_IMPORT_DXF);
    if (button(ui, "Fit view", "Fit the drawing and every stroke in view (Home)", true))
        ui->fit_pending = true;

    double s = ui->job.drawing_scale;
    if (ui_prop(ui, "Scale", "Drawing units to millimetres. The strokes scale with the "
                "drawing so they stay where they were painted. The Scale tool works this out "
                "from a known dimension", &s, 1e-6, 1e6, 0.001, "\xc3\x97") &&
        s > 0.0 && ui->job.drawing_scale > 0.0)
        ui_rescale(ui, s / ui->job.drawing_scale);
}

static void inspect_tool(RgUi *ui)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;
    char head[64];
    snprintf(head, sizeof head, "Tool: %s", TOOL_NAME[ui->tool]);
    ui_gap(ui, 4);
    ui_section(ui, head);

    if (!painting(ui) && ui->tool != TOOL_PAN && ui->tool != TOOL_SCALE)
        ui_label_wrap(ui, "This job is a cylinder: strokes are painted only on flat parts. "
                      "Change the part on the Settings page.", t->warn);

    switch (ui->tool) {
    case TOOL_PAN:
        ui_label_wrap(ui, "Click a stroke to select it. Drag or right-drag to move round, "
                      "scroll to zoom, Home to fit.", t->text_dim);
        break;
    case TOOL_LINE:
        ui_check(ui, "Snap to the drawing", "Put points exactly on the drawing's corners "
                 "and lines. Hold Shift to place one point freely", &ui->snap);
        ui_label_wrap(ui, "Click to add points. Double-click or Enter finishes the stroke, "
                      "Backspace takes back a point, Escape cancels.", t->text_dim);
        if (ui->ndraft) {
            ui_info_rowf(ui, "So far", "%d point%s, %.0f mm", ui->ndraft, ui->ndraft == 1 ? "" : "s",
                         rg_stroke_length(ui->draft, ui->ndraft));
            button_row(ui, 2);
            if (button(ui, "Finish", "Add this stroke to the pattern (Enter)", ui->ndraft >= 2))
                finish_line(ui);
            if (button(ui, "Cancel", "Throw this stroke away (Escape)", true))
                ui->ndraft = 0;
        }
        break;
    case TOOL_FREE:
        ui_prop(ui, "Smoothing", "How far the stroke may stray from where you drew, to "
                "leave fewer points: more is smoother and quicker for the robot",
                &ui->smoothing, 0.05, 50, 0.1, "mm");
        ui_label_wrap(ui, "Hold the button down and draw.", t->text_dim);
        break;
    case TOOL_TRACE: {
        /* Latched, the inset follows the spot, so changing the gun does not
         * leave yesterday's figure sitting in the box. */
        bool half_ok = spray_width(ui) > 0.0;
        if (ui->inset_half && half_ok)
            ui->inset = 0.5 * spray_width(ui);
        if (ui_prop(ui, "Inset", "How far inside the outline the gun runs. Half the gun's width "
                    "puts the edge of the spray on the outline; 0 runs on the line; negative runs "
                    "outside it", &ui->inset, -1000, 1000, 1, "mm"))
            ui->inset_half = false;          /* set by hand: stop following */
        button_row(ui, 2);
        ui_tip(ui, "Hold the inset at half what the gun covers, so the edge of the spray follows "
               "the outline. Stays on, and follows the spot if you change it");
        nk_bool latched = ui->inset_half ? nk_true : nk_false;
        if (!half_ok)
            nk_widget_disable_begin(ui->ctx);
        if (nk_selectable_label(ui->ctx, "Half the width", NK_TEXT_CENTERED, &latched) && half_ok) {
            ui->inset_half = latched != 0;
            if (ui->inset_half)
                ui->inset = 0.5 * spray_width(ui);
        }
        if (!half_ok)
            nk_widget_disable_end(ui->ctx);
        if (button(ui, "On the line", "Run the gun on the outline itself", true)) {
            ui->inset = 0.0;
            ui->inset_half = false;
        }
        ui_label_wrap(ui, "Click near an outline: the stroke starts at the nearest point and "
                      "goes round once, and clicking it again replaces that path. A large "
                      "inset on a tight shape can cross itself: check the preview.",
                      t->text_dim);
        break;
    }
    case TOOL_FILL: {
        ui_check(ui, "Spiral", "Follow the outline inward instead of weaving back and forth: "
                 "one continuous path, driven round the work, with no square turn at the end "
                 "of every pass. A region with a hole in it cannot be spiralled",
                 &ui->fill_spiral);
        if (!ui->fill_spiral)
            ui_prop(ui, "Angle", ui->job.pattern == RG_PAT_FAN
                    ? "The direction of the passes, from the drawing's X axis. The passes should "
                      "run across the fan, which lies at the Fan angle below"
                    : "The direction of the passes, from the drawing's X axis. A round spot is "
                      "the same width whichever way they run",
                    &ui->fill_angle, -180, 180, 5, "deg");
        double p = fill_pitch(ui);
        if (p > 0.0)
            ui_info_rowf(ui, "Spacing", "%.1f mm step-over, %.1f passes per point", p,
                         spray_width(ui) / p);
        else
            ui_info_row(ui, "Spacing", "set the spot and step-over");
        if (!ui->fill_spiral && turn_length(ui) > 0.0)
            ui_info_rowf(ui, "Turns", "%.0f mm off the work, then back", turn_length(ui));
        if (!ui->fill_spiral)
            ui_check(ui, "Run past the edge", "Carry each pass half the gun's width past the "
                     "outline so the edge gets a full coat. Passes stop at the edge of a hole, "
                     "so nothing inside it is sprayed", &ui->fill_extend);
        ui_label_wrap(ui, "Hover inside an outline to preview, click to fill it. Outlines "
                      "inside it are left bare.", t->text_dim);
        break;
    }
    case TOOL_SCALE: {
        ui_label_wrap(ui, "Click two points a known distance apart, snapping to the drawing, "
                      "then type the true distance.", t->text_dim);
        if (ui->scale_clicks == 2) {
            double measured = dist(ui->scale_a, ui->scale_b);
            ui_info_rowf(ui, "Measured", "%.3f mm", measured);
            ui_prop(ui, "True length", "What that distance really is on the part",
                    &ui->scale_true, 1e-3, 1e6, 0.1, "mm");
            button_row(ui, 1);
            if (ui_primary_button(ui, "Apply scale") && measured > 1e-9) {
                double f = ui->scale_true / measured;
                ui_rescale(ui, f);
                ui->scale_clicks = 0;
                ui_message(ui, false, "Drawing scaled by %.6g: it is now %.6g mm per unit", f,
                           ui->job.drawing_scale);
            }
        }
        double b[4];
        if (drawing_bounds(ui, b) && b[2] > b[0]) {
            double wide = b[2] - b[0];
            if (ui->width_true <= 0.0)
                ui->width_true = wide;
            ui_gap(ui, 4);
            ui_info_rowf(ui, "Width now", "%.3f mm", wide);
            ui_prop(ui, "True width", "The drawing's overall width on the real part",
                    &ui->width_true, 1e-3, 1e6, 1, "mm");
            button_row(ui, 1);
            if (button(ui, "Set width", "Scale the drawing so its overall width is the true width",
                       true)) {
                ui_rescale(ui, ui->width_true / wide);
                ui_message(ui, false, "Drawing is now %.1f mm wide", ui->width_true);
            }
        }
        break;
    }
    default:
        break;
    }
    (void)c;
}

static void reverse_stroke(RgStroke *st)
{
    for (int a = 0, b = st->n - 1; a < b; a++, b--) {
        RgPt tmp = st->pts[a];
        st->pts[a] = st->pts[b];
        st->pts[b] = tmp;
        /* the lead-in becomes the run-out: the flags go with their points */
        if (st->off) {
            unsigned char f = st->off[a];
            st->off[a] = st->off[b];
            st->off[b] = f;
        }
    }
}

static void swap_strokes(RgJob *j, int a, int b)
{
    RgStroke tmp = j->strokes[a];
    j->strokes[a] = j->strokes[b];
    j->strokes[b] = tmp;
}

static void inspect_strokes(RgUi *ui)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;
    RgJob *j = &ui->job;
    char head[64];
    snprintf(head, sizeof head, "Strokes (%d)", j->nstrokes);
    ui_gap(ui, 4);
    ui_section(ui, head);

    nk_layout_row_dynamic(c, S(ui, 170), 1);
    if (nk_group_begin(c, "strokes", NK_WINDOW_BORDER)) {
        double total = 0.0;
        for (int i = 0; i < j->nstrokes; i++) {
            const RgStroke *st = &j->strokes[i];
            double len = rg_stroke_length(st->pts, st->n);
            total += len;
            char lab[96];
            snprintf(lab, sizeof lab, "%3d    %4d pts    %7.0f mm", i + 1, st->n, len);
            nk_layout_row_dynamic(c, S(ui, 22), 1);
            nk_bool on = ui->selected == i;
            if (nk_selectable_label(c, lab, NK_TEXT_LEFT, &on))
                ui->selected = on ? i : -1;
        }
        if (!j->nstrokes) {
            nk_layout_row_dynamic(c, S(ui, 22), 1);
            nk_label_colored(c, "None yet.", NK_TEXT_LEFT, t->text_faint);
        } else if (!isnan(j->spray_speed) && j->spray_speed > 0) {
            char lab[96];
            snprintf(lab, sizeof lab, "%.0f mm in all, %.0f s at %.0f mm/s", total,
                     total / j->spray_speed, j->spray_speed);
            nk_layout_row_dynamic(c, S(ui, 22), 1);
            nk_label_colored(c, lab, NK_TEXT_LEFT, t->text_dim);
        }
        nk_group_end(c);
    }

    int s = ui->selected;
    bool have = s >= 0 && s < j->nstrokes;
    button_row(ui, 4);
    if (button(ui, "Up", "Spray this stroke earlier", have && s > 0)) {
        swap_strokes(j, s, s - 1);
        ui->selected = s - 1;
    }
    if (button(ui, "Down", "Spray this stroke later", have && s + 1 < j->nstrokes)) {
        swap_strokes(j, s, s + 1);
        ui->selected = s + 1;
    }
    if (button(ui, "Reverse", "Spray this stroke the other way round (R)", have))
        reverse_stroke(&j->strokes[s]);
    if (button(ui, "Delete", "Remove this stroke (Delete)", have)) {
        rg_job_delete_stroke(j, s);
        ui->selected = -1;
    }
    button_row(ui, 1);
    if (button(ui, "Clear all strokes", "Remove every stroke. Undo brings them back", j->nstrokes > 0)) {
        rg_job_free(j);
        ui->selected = -1;
    }
}

static bool prop_set(RgUi *ui, const char *label, const char *tip, double *v, double def,
                     double min, double max, double step, const char *unit)
{
    double x = isnan(*v) ? def : *v;
    if (ui_prop(ui, label, tip, &x, min, max, step, unit)) {
        *v = x;
        return true;
    }
    return false;
}

static void inspect_spray(RgUi *ui)
{
    RgJob *j = &ui->job;
    ui_gap(ui, 4);
    ui_section(ui, "Spray");
    if (j->pattern == RG_PAT_FAN) {
        prop_set(ui, "Fan width", "The fan's width at the standoff: the band drawn along each "
                 "stroke", &j->fan_width, 80, 5, 1000, 1, "mm");
        if (j->part == RG_PART_FLAT)
            ui_prop(ui, "Fan angle", "Which way the fan's long axis lies in the drawing. A "
                    "stroke must run across the fan to lay down a band its full width; one "
                    "running along it paints a narrow line, as the band on the canvas shows",
                    &j->fan_along, -180, 180, 5, "deg");
    } else {
        prop_set(ui, "Spot", "The circle the gun coats at the standoff. A round spot does not "
                 "care which way the gun is turned, so a stroke may run any direction",
                 &j->spot_diameter, 12, 0.5, 200, 0.5, "mm");
    }
    double step = rg_job_step(j);
    if (!isnan(step) && ui_prop(ui, "Step-over", "The advance between one pass and the next. "
                                "Each point is covered spot / step-over times", &step, 0.1,
                                1000, 0.5, "mm"))
        j->step_over = step;
    if (j->part == RG_PART_FLAT)
        prop_set(ui, "Speed", "The gun's speed along a stroke. It must be held steady: a dip "
                 "in speed is a ridge in the coating", &j->spray_speed, 500, 1, 2000, 10, "mm/s");
    prop_set(ui, "Standoff", "Gun tip to the part's surface", &j->standoff, 150, 10, 1000, 5, "mm");
    ui_prop_int(ui, "Cycles", "Repeats of the whole pattern. A coating is built up over many "
                "of them", &j->cycles, 1, 999, "");
    ui_prop(ui, "Dwell", "Seconds between cycles, to let the part cool", &j->dwell, 0, 600, 1, "s");
}

/* ------------------------------------------------------------------ */
/* Page                                                                */
/* ------------------------------------------------------------------ */

void page_draw(RgUi *ui, struct nk_rect inner)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;
    float tw = S(ui, TOOLS_W), iw = S(ui, INSPECTOR_W);

    nk_layout_row_template_begin(c, inner.h);
    nk_layout_row_template_push_static(c, tw);
    nk_layout_row_template_push_dynamic(c);
    nk_layout_row_template_push_static(c, iw);
    nk_layout_row_template_end(c);

    nk_style_push_style_item(c, &c->style.window.fixed_background, nk_style_item_color(t->rail));
    nk_style_push_vec2(c, &c->style.window.group_padding, nk_vec2(S(ui, 6), S(ui, 10)));
    if (nk_group_begin(c, "tools", NK_WINDOW_NO_SCROLLBAR)) {
        nk_style_push_vec2(c, &c->style.window.spacing, nk_vec2(S(ui, 6), S(ui, 6)));
        for (int i = 0; i < TOOL_COUNT; i++) {
            nk_layout_row_dynamic(c, S(ui, 42), 1);
            ui_tip(ui, TOOL_TIP[i]);
            nk_bool on = ui->tool == (Tool)i;
            if (nk_selectable_label(c, TOOL_NAME[i], NK_TEXT_CENTERED, &on))
                set_tool(ui, (Tool)i);
        }
        nk_style_pop_vec2(c);
        nk_group_end(c);
    }
    nk_style_pop_vec2(c);
    nk_style_pop_style_item(c);

    struct nk_rect r;
    nk_widget(&r, c);
    ui->canvas = r;
    if (ui->fit_pending)
        fit_view(ui);
    canvas_input(ui, r);
    update_fill_preview(ui);
    canvas_paint(ui, r);

    nk_style_push_style_item(c, &c->style.window.fixed_background, nk_style_item_color(t->panel));
    nk_style_push_vec2(c, &c->style.window.group_padding, nk_vec2(S(ui, 12), S(ui, 10)));
    if (nk_group_begin(c, "inspector", 0)) {
        nk_style_push_vec2(c, &c->style.window.spacing, nk_vec2(S(ui, 8), S(ui, 6)));
        inspect_drawing(ui);
        inspect_tool(ui);
        inspect_strokes(ui);
        inspect_spray(ui);
        nk_style_pop_vec2(c);
        nk_group_end(c);
    }
    nk_style_pop_vec2(c);
    nk_style_pop_style_item(c);
}

bool draw_handle_key(RgUi *ui, SDL_Keycode key, Uint16 mod)
{
    (void)mod;
    int s = ui->selected;
    bool have = s >= 0 && s < ui->job.nstrokes;
    switch (key) {
    case SDLK_RETURN:
    case SDLK_KP_ENTER:
        if (ui->tool == TOOL_LINE && ui->ndraft) {
            finish_line(ui);
            return true;
        }
        return false;
    case SDLK_ESCAPE:
        if (ui->ndraft || ui->scale_clicks) {
            ui->ndraft = 0;
            ui->freehand = false;
            ui->scale_clicks = 0;
            return true;
        }
        if (have) {
            ui->selected = -1;
            return true;
        }
        return false;
    case SDLK_BACKSPACE:
        if (ui->ndraft && !ui->freehand) {
            ui->ndraft--;
            return true;
        }
        return false;
    case SDLK_DELETE:
        if (have) {
            rg_job_delete_stroke(&ui->job, s);
            ui->selected = -1;
            return true;
        }
        return false;
    case SDLK_HOME:
        ui->fit_pending = true;
        return true;
    case SDLK_r:
        if (have) {
            reverse_stroke(&ui->job.strokes[s]);
            return true;
        }
        return false;
    case SDLK_s: set_tool(ui, TOOL_PAN);   return true;
    case SDLK_l: set_tool(ui, TOOL_LINE);  return true;
    case SDLK_d: set_tool(ui, TOOL_FREE);  return true;
    case SDLK_t: set_tool(ui, TOOL_TRACE); return true;
    case SDLK_f: set_tool(ui, TOOL_FILL);  return true;
    case SDLK_m: set_tool(ui, TOOL_SCALE); return true;
    default:
        return false;
    }
}
