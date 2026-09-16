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
 * rg_ui.c — window shell, shared helpers, the job file, undo, the drawing
 * and the checks
 */
#define NK_IMPLEMENTATION
#define NK_SDL_RENDERER_IMPLEMENTATION
#include "nk.h"
#include "nuklear_sdl_renderer.h"

#include "rg_ui_int.h"
#include "rg_rapid.h"
#include "rg_report.h"
#include "rg_version.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WIN_MIN_W 1180
#define WIN_MIN_H 720
#define WIN_MAX_FRAC 0.88f

/* An edit is recorded for undo once it has been still this long, so typing a
 * number or dragging a value is one step, not a hundred. */
#define UNDO_SETTLE_MS 450

/* The checks run this long after the job last changed. */
#define PLAN_DELAY_MS 250

static const char *PAGE_NAME[PAGE_COUNT] = { "Draw", "Settings", "Program" };

static const char *PAGE_HINT[PAGE_COUNT] = {
    "The part's drawing and the pattern painted over it: import a DXF, scale "
    "it, then paint, trace or fill the strokes the gun follows",
    "The part, the cell, the gun, and the rules every program is checked against",
    "The checks, the report, and the RAPID program, written once nothing is refused",
};

static char *dup_text(const char *s)
{
    size_t n = strlen(s) + 1;
    char *d = malloc(n);
    if (d)
        memcpy(d, s, n);
    return d;
}

/* ------------------------------------------------------------------ */
/* Tooltips                                                            */
/*                                                                     */
/* Every control that does something says what it does when the pointer */
/* rests on it. ui_tip() is called immediately before the widget,       */
/* because the bounds tested are the ones the next widget occupies.     */
/* Nuklear's own nk_tooltip() clips at the window edge and only answers */
/* for the focused window; this draws into the overlay buffer instead.  */
/* ------------------------------------------------------------------ */

#define TIP_DELAY_MS  700
#define TIP_MAX_W     360.0f
#define TIP_MAX_LINES 6

typedef struct { int off, len; } TipLine;

float ui_text_width(const RgUi *ui, const char *s, int len)
{
    const struct nk_user_font *f = ui->ctx->style.font;
    if (!f || len <= 0)
        return 0.0f;
    return f->width(f->userdata, f->height, s, len);
}

static int wrap_text(const RgUi *ui, const char *s, float w, TipLine *out, int max_out)
{
    float space = ui_text_width(ui, " ", 1);
    int lines = 0, start = 0, i = 0;
    float cur = 0.0f;

    while (s[i]) {
        int ws = i;
        while (s[i] && s[i] != ' ')
            i++;
        float word = ui_text_width(ui, s + ws, i - ws);

        if (cur > 0.0f && cur + space + word > w) {
            if (out && lines < max_out) {
                out[lines].off = start;
                out[lines].len = ws - 1 - start;
            }
            lines++;
            start = ws;
            cur = word;
        } else {
            cur += (cur > 0.0f ? space : 0.0f) + word;
        }
        while (s[i] == ' ')
            i++;
    }
    if (out && lines < max_out) {
        out[lines].off = start;
        out[lines].len = i - start;
    }
    return lines + 1;
}

float ui_wrap_height(RgUi *ui, const char *text, float w)
{
    int n = wrap_text(ui, text, w, NULL, 0);
    return (float)n * (ui->ctx->style.font->height + S(ui, 4));
}

void ui_label_wrap(RgUi *ui, const char *text, struct nk_color col)
{
    struct nk_context *c = ui->ctx;
    struct nk_rect region = nk_window_get_content_region(c);
    float w = region.w - c->style.window.padding.x * 2.0f - S(ui, 8);
    if (w < S(ui, 60))
        w = S(ui, 60);
    nk_layout_row_dynamic(c, ui_wrap_height(ui, text, w) + S(ui, 2), 1);
    nk_label_colored_wrap(c, text, col);
}

void ui_tip(RgUi *ui, const char *text)
{
    struct nk_context *c = ui->ctx;
    if (!text || !text[0] || !c->current || !c->current->layout)
        return;
    if (c->current->popup.active)
        return;
    if (c->input.mouse.buttons[NK_BUTTON_LEFT].down)
        return;

    struct nk_rect b = nk_widget_bounds(c);
    struct nk_rect clip = c->current->layout->clip;
    if (!nk_input_is_mouse_hovering_rect(&c->input, b) ||
        !nk_input_is_mouse_hovering_rect(&c->input, clip))
        return;

    snprintf(ui->tip_text, sizeof ui->tip_text, "%s", text);
    ui->tip_over = b;
}

void ui_tip_rect(RgUi *ui, struct nk_rect over, const char *text)
{
    if (!text || !text[0])
        return;
    snprintf(ui->tip_text, sizeof ui->tip_text, "%s", text);
    ui->tip_over = over;
}

static void tip_draw(RgUi *ui, int w, int h)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;

    if (!ui->tip_text[0]) {
        ui->tip_shown[0] = '\0';
        return;
    }
    if (strcmp(ui->tip_text, ui->tip_shown) != 0) {
        snprintf(ui->tip_shown, sizeof ui->tip_shown, "%s", ui->tip_text);
        ui->tip_since_ms = plat_now_ms();
        return;
    }
    if (plat_now_ms() - ui->tip_since_ms < TIP_DELAY_MS)
        return;

    TipLine ln[TIP_MAX_LINES];
    int n = wrap_text(ui, ui->tip_shown, S(ui, TIP_MAX_W), ln, TIP_MAX_LINES);
    if (n > TIP_MAX_LINES)
        n = TIP_MAX_LINES;

    float lh = c->style.font->height + S(ui, 3);
    float pad_x = S(ui, 9), pad_y = S(ui, 6);
    float tw = 0.0f;
    for (int i = 0; i < n; i++) {
        float lw = ui_text_width(ui, ui->tip_shown + ln[i].off, ln[i].len);
        if (lw > tw)
            tw = lw;
    }

    struct nk_rect r;
    r.w = tw + pad_x * 2.0f;
    r.h = (float)n * lh + pad_y * 2.0f;
    r.x = ui->tip_over.x;
    r.y = ui->tip_over.y + ui->tip_over.h + S(ui, 6);
    if (r.y + r.h > (float)h - S(ui, 4))
        r.y = ui->tip_over.y - r.h - S(ui, 6);
    if (r.y < S(ui, 4)) r.y = S(ui, 4);
    if (r.x + r.w > (float)w - S(ui, 4)) r.x = (float)w - r.w - S(ui, 4);
    if (r.x < S(ui, 4)) r.x = S(ui, 4);

    struct nk_command_buffer *cb = &c->overlay;
    nk_command_buffer_init(cb, &c->memory, NK_CLIPPING_ON);
    nk_start_buffer(c, cb);
    nk_push_scissor(cb, nk_rect(0, 0, (float)w, (float)h));

    float rad = S(ui, 3);
    nk_fill_rect(cb, nk_rect(r.x + S(ui, 2), r.y + S(ui, 2), r.w, r.h), rad,
                 nk_rgba(0, 0, 0, t->is_light ? 36 : 90));
    nk_fill_rect(cb, r, rad, t->panel_alt);
    nk_stroke_rect(cb, r, rad, 1.0f, t->border);
    for (int i = 0; i < n; i++) {
        struct nk_rect lr = nk_rect(r.x + pad_x, r.y + pad_y + (float)i * lh, tw, lh);
        nk_draw_text(cb, lr, ui->tip_shown + ln[i].off, ln[i].len, c->style.font,
                     t->panel_alt, t->text);
    }
    nk_finish_buffer(c, cb);
}

/* ------------------------------------------------------------------ */
/* Form helpers                                                        */
/* ------------------------------------------------------------------ */

void ui_form_row(RgUi *ui, float h)
{
    nk_layout_row_template_begin(ui->ctx, S(ui, h));
    nk_layout_row_template_push_static(ui->ctx, S(ui, LABEL_W));
    nk_layout_row_template_push_dynamic(ui->ctx);
    nk_layout_row_template_end(ui->ctx);
}

void ui_section(RgUi *ui, const char *title)
{
    nk_layout_row_dynamic(ui->ctx, S(ui, 26.0f), 1);
    nk_label_colored(ui->ctx, title, NK_TEXT_LEFT, ui->theme->accent);
}

void ui_gap(RgUi *ui, float h)
{
    nk_layout_row_dynamic(ui->ctx, S(ui, h), 1);
    nk_skip(ui->ctx);
}

bool ui_primary_button(RgUi *ui, const char *label)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;

    nk_style_push_style_item(c, &c->style.button.normal, nk_style_item_color(t->accent));
    nk_style_push_style_item(c, &c->style.button.hover, nk_style_item_color(t->accent_hover));
    nk_style_push_style_item(c, &c->style.button.active, nk_style_item_color(t->accent_dim));
    nk_style_push_color(c, &c->style.button.text_normal, t->text_on_accent);
    nk_style_push_color(c, &c->style.button.text_hover, t->text_on_accent);
    nk_style_push_color(c, &c->style.button.border_color, t->accent);

    bool hit = nk_button_label(c, label) != 0;

    nk_style_pop_color(c);
    nk_style_pop_color(c);
    nk_style_pop_color(c);
    nk_style_pop_style_item(c);
    nk_style_pop_style_item(c);
    nk_style_pop_style_item(c);
    return hit;
}

void ui_info_row(RgUi *ui, const char *k, const char *v)
{
    ui_form_row(ui, 22.0f);
    nk_label_colored(ui->ctx, k, NK_TEXT_LEFT, ui->theme->text_dim);
    nk_label(ui->ctx, v, NK_TEXT_LEFT);
}

void ui_info_rowf(RgUi *ui, const char *k, const char *fmt, ...)
{
    char buf[320];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    ui_info_row(ui, k, buf);
}

static void prop_row(RgUi *ui)
{
    nk_layout_row_template_begin(ui->ctx, S(ui, ROW_H));
    nk_layout_row_template_push_static(ui->ctx, S(ui, LABEL_W));
    nk_layout_row_template_push_dynamic(ui->ctx);
    nk_layout_row_template_push_static(ui->ctx, S(ui, UNIT_W));
    nk_layout_row_template_end(ui->ctx);
}

bool ui_prop(RgUi *ui, const char *label, const char *tip, double *v,
             double min, double max, double step, const char *unit)
{
    struct nk_context *c = ui->ctx;
    prop_row(ui);
    nk_label_colored(c, label, NK_TEXT_LEFT, ui->theme->text_dim);
    /* A bare "#": Nuklear draws whatever follows the '#' inside the field,
     * and keys a '#' name by its order in the frame instead of its text. */
    double before = *v;
    ui_tip(ui, tip);
    nk_property_double(c, "#", min, v, max, step, (float)(step / 4.0));
    nk_label_colored(c, unit ? unit : "", NK_TEXT_LEFT, ui->theme->text_faint);
    return *v != before;
}

bool ui_prop_int(RgUi *ui, const char *label, const char *tip, int *v,
                 int min, int max, const char *unit)
{
    struct nk_context *c = ui->ctx;
    prop_row(ui);
    nk_label_colored(c, label, NK_TEXT_LEFT, ui->theme->text_dim);
    int before = *v;
    ui_tip(ui, tip);
    nk_property_int(c, "#", min, v, max, 1, 0.05f);
    nk_label_colored(c, unit ? unit : "", NK_TEXT_LEFT, ui->theme->text_faint);
    return *v != before;
}

bool ui_check(RgUi *ui, const char *label, const char *tip, bool *v)
{
    ui_form_row(ui, ROW_H);
    nk_skip(ui->ctx);
    nk_bool b = *v;
    ui_tip(ui, tip);
    nk_checkbox_label(ui->ctx, label, &b);
    bool changed = (b != 0) != *v;
    *v = b != 0;
    return changed;
}

void ui_message(RgUi *ui, bool error, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(ui->msg, sizeof ui->msg, fmt, ap);
    va_end(ap);
    ui->msg_error = error;
}

/* True while a text or number field has the keyboard. Not
 * nk_item_is_any_active(): that is also true whenever the pointer is over
 * any window, which here is always, and it silently eats every shortcut. */
bool ui_typing(const struct nk_context *ctx)
{
    for (const struct nk_window *w = ctx->begin; w; w = w->next)
        if (!(w->flags & NK_WINDOW_HIDDEN) && (w->edit.active || w->property.active))
            return true;
    return false;
}

/* ------------------------------------------------------------------ */
/* The job, its file, and undo                                         */
/* ------------------------------------------------------------------ */

static char *job_text(const RgJob *j)
{
    RgBuf b;
    rg_buf_init(&b);
    rg_job_write(j, &b);
    if (b.failed || !b.s) {
        rg_buf_free(&b);
        return NULL;
    }
    return b.s;
}

static void undo_clear(RgUi *ui)
{
    for (int i = 0; i < ui->undo_n; i++)
        free(ui->undo[i]);
    ui->undo_n = ui->undo_pos = 0;
    ui->change_waiting = false;
}

static void undo_reset(RgUi *ui)
{
    undo_clear(ui);
    ui->undo[0] = dup_text(ui->cur_text);
    ui->undo_n = ui->undo[0] ? 1 : 0;
}

/* Takes the job over (its strokes included). */
static void set_job(RgUi *ui, RgJob *job, const char *path)
{
    rg_job_free(&ui->job);
    ui->job = *job;
    snprintf(ui->path, sizeof ui->path, "%s", path ? path : "");

    char *text = job_text(&ui->job);
    free(ui->cur_text);
    ui->cur_text = text ? text : dup_text("");
    free(ui->saved_text);
    ui->saved_text = dup_text(path && path[0] ? ui->cur_text : "");

    undo_reset(ui);
    draw_forget(ui);
    ui->fit_pending = true;
    ui->plan_due = 1;
    ui->loaded_from[0] = '\0';        /* re-read the drawing, relative to the new path */
    ui->loaded_layer[0] = '\1';
}

static void new_job(RgUi *ui, RgPartKind part)
{
    RgJob j;
    rg_job_default(&j);
    char err[256];
    rg_job_parse(&j, part == RG_PART_FLAT ? rg_job_template_flat() : rg_job_template(),
                 err, sizeof err);
    rg_job_free(&j);                  /* the template's example strokes */
    set_job(ui, &j, NULL);
    ui->page = PAGE_DRAW;
    ui_message(ui, false, part == RG_PART_FLAT
               ? "New flat part. Import its DXF, scale it, then paint the strokes."
               : "New cylinder job. Set the bands on the Settings page, or import a DXF.");
}

static void remember_dir(char *dst, size_t cap, const char *path)
{
    char dir[PLAT_PATH_MAX];
    snprintf(dir, sizeof dir, "%s", path);
    if (plat_path_parent(dir))
        snprintf(dst, cap, "%s", dir);
}

bool ui_open_path(RgUi *ui, const char *path)
{
    RgJob j;
    rg_job_default(&j);
    char err[512];
    if (!rg_job_load(&j, path, err, sizeof err)) {
        rg_job_free(&j);
        ui_message(ui, true, "%s", err);
        return false;
    }
    set_job(ui, &j, path);
    rg_settings_add_recent(&ui->settings, path);
    snprintf(ui->settings.last_job, sizeof ui->settings.last_job, "%s", path);
    remember_dir(ui->settings.job_dir, sizeof ui->settings.job_dir, path);
    rg_settings_save(&ui->settings);
    ui_message(ui, false, "Opened %s", plat_path_leaf(path));
    return true;
}

static bool is_absolute(const char *p)
{
    return p[0] == '/' || p[0] == '\\' || (p[0] && p[1] == ':');
}

/* A drawing path as the job file should hold it: its name alone when it
 * sits beside the job, otherwise in full. */
static void drawing_path_for(const char *job_path, const char *abs, char *out, size_t cap)
{
    char dir[PLAT_PATH_MAX];
    snprintf(dir, sizeof dir, "%s", job_path);
    size_t n = 0;
    if (plat_path_parent(dir))
        n = strlen(dir);
    if (n && strncmp(abs, dir, n) == 0 && (abs[n] == '/' || abs[n] == '\\') &&
        !strchr(abs + n + 1, '/') && !strchr(abs + n + 1, '\\'))
        snprintf(out, cap, "%s", abs + n + 1);
    else
        snprintf(out, cap, "%s", abs);
}

bool ui_save_to(RgUi *ui, const char *path)
{
    if (ui->loaded_from[0] && ui->job.drawing[0])
        drawing_path_for(path, ui->loaded_from, ui->job.drawing, sizeof ui->job.drawing);

    char err[512];
    if (!rg_job_save(&ui->job, path, err, sizeof err)) {
        ui_message(ui, true, "%s", err);
        return false;
    }
    snprintf(ui->path, sizeof ui->path, "%s", path);
    char *text = job_text(&ui->job);
    if (text) {
        free(ui->saved_text);
        ui->saved_text = text;
        free(ui->cur_text);
        ui->cur_text = dup_text(text);
    }
    rg_settings_add_recent(&ui->settings, path);
    snprintf(ui->settings.last_job, sizeof ui->settings.last_job, "%s", path);
    remember_dir(ui->settings.job_dir, sizeof ui->settings.job_dir, path);
    rg_settings_save(&ui->settings);
    ui->plan_due = 1;
    ui_message(ui, false, "Saved %s", plat_path_leaf(path));
    return true;
}

bool ui_save(RgUi *ui, bool save_as)
{
    if (save_as || !ui->path[0]) {
        ui_open_file_dialog(ui, FILE_SAVE_JOB);
        return false;                      /* finishes when the dialog does */
    }
    return ui_save_to(ui, ui->path);
}

void ui_continue_pending(RgUi *ui)
{
    Pending p = ui->pending;
    ui->pending = PENDING_NONE;
    ui->save_then_pending = false;
    switch (p) {
    case PENDING_NEW_FLAT:     new_job(ui, RG_PART_FLAT); break;
    case PENDING_NEW_CYLINDER: new_job(ui, RG_PART_CYLINDER); break;
    case PENDING_OPEN:         ui_open_file_dialog(ui, FILE_OPEN_JOB); break;
    case PENDING_OPEN_PATH:    ui_open_path(ui, ui->pending_path); break;
    case PENDING_QUIT:         ui->quit = true; break;
    default: break;
    }
}

void ui_request(RgUi *ui, Pending what)
{
    ui->pending = what;
    if (ui->modified)
        ui_open_confirm(ui, what);
    else
        ui_continue_pending(ui);
}

bool ui_can_undo(const RgUi *ui) { return ui->undo_pos > 0 || ui->change_waiting; }
bool ui_can_redo(const RgUi *ui) { return ui->undo_pos < ui->undo_n - 1; }

static void undo_commit(RgUi *ui)
{
    if (!ui->change_waiting)
        return;
    ui->change_waiting = false;
    char *snap = dup_text(ui->cur_text);
    if (!snap)
        return;
    for (int i = ui->undo_pos + 1; i < ui->undo_n; i++)
        free(ui->undo[i]);
    ui->undo_n = ui->undo_pos + 1;
    if (ui->undo_n >= UNDO_MAX) {
        free(ui->undo[0]);
        memmove(&ui->undo[0], &ui->undo[1], (UNDO_MAX - 1) * sizeof ui->undo[0]);
        ui->undo_n--;
    }
    ui->undo[ui->undo_n++] = snap;
    ui->undo_pos = ui->undo_n - 1;
}

static void restore(RgUi *ui, const char *text)
{
    RgJob j;
    rg_job_default(&j);
    char err[256];
    if (!rg_job_parse(&j, text, err, sizeof err)) {
        rg_job_free(&j);
        ui_message(ui, true, "Could not restore: %s", err);
        return;
    }
    rg_job_free(&ui->job);
    ui->job = j;
    free(ui->cur_text);
    ui->cur_text = dup_text(text);
    if (ui->selected >= ui->job.nstrokes)
        ui->selected = ui->job.nstrokes - 1;
    ui->plan_due = 1;
}

void ui_undo(RgUi *ui)
{
    undo_commit(ui);
    if (ui->undo_pos > 0) {
        ui->undo_pos--;
        restore(ui, ui->undo[ui->undo_pos]);
        ui_message(ui, false, "Undone");
    }
}

void ui_redo(RgUi *ui)
{
    if (ui->undo_pos < ui->undo_n - 1) {
        ui->undo_pos++;
        restore(ui, ui->undo[ui->undo_pos]);
        ui_message(ui, false, "Redone");
    }
}

/* Once a frame: notice a change to the job, fold settled edits into the
 * undo history, and schedule the checks. */
static void sync_job(RgUi *ui)
{
    char *text = job_text(&ui->job);
    if (!text)
        return;
    if (strcmp(text, ui->cur_text) != 0) {
        free(ui->cur_text);
        ui->cur_text = text;
        text = NULL;
        ui->plan_due = plat_now_ms() + PLAN_DELAY_MS;
        bool at_snapshot = ui->undo_n > 0 && strcmp(ui->undo[ui->undo_pos], ui->cur_text) == 0;
        ui->change_waiting = !at_snapshot;
        ui->change_ms = plat_now_ms();
    }
    free(text);

    bool mouse_down = ui->ctx->input.mouse.buttons[NK_BUTTON_LEFT].down;
    if (ui->change_waiting && !mouse_down && plat_now_ms() - ui->change_ms > UNDO_SETTLE_MS)
        undo_commit(ui);

    ui->modified = strcmp(ui->cur_text, ui->saved_text) != 0;
}

void ui_rescale(RgUi *ui, double factor)
{
    if (!(factor > 0.0) || !isfinite(factor))
        return;
    ui->job.drawing_scale *= factor;
    for (int i = 0; i < ui->job.nstrokes; i++)
        for (int k = 0; k < ui->job.strokes[i].n; k++) {
            ui->job.strokes[i].pts[k].x *= factor;
            ui->job.strokes[i].pts[k].y *= factor;
        }
    ui->cx *= factor;
    ui->cy *= factor;
    ui->zoom /= factor;
}

/* ------------------------------------------------------------------ */
/* The drawing                                                         */
/* ------------------------------------------------------------------ */

static void resolve_drawing(const RgUi *ui, char *out, size_t cap)
{
    out[0] = '\0';
    const char *d = ui->job.drawing;
    if (!d[0])
        return;
    char dir[PLAT_PATH_MAX];
    snprintf(dir, sizeof dir, "%s", ui->path);
    if (is_absolute(d) || !ui->path[0] || !plat_path_parent(dir) ||
        !plat_path_join(out, cap, dir, d))
        snprintf(out, cap, "%s", d);
}

static void forget_drawing(RgUi *ui)
{
    rg_drawing_free(&ui->raw);
    rg_drawing_free(&ui->drawing);
    rg_shape_free(&ui->shape);
    ui->have_raw = false;
    ui->skipped = 0;
    ui->shown_scale = NAN;
}

/* Read the drawing when the job's drawing or layer changes, and rebuild its
 * scaled copy and outlines when the scale does. */
static void drawing_update(RgUi *ui)
{
    char abs[PLAT_PATH_MAX];
    resolve_drawing(ui, abs, sizeof abs);
    if (strcmp(abs, ui->loaded_from) != 0 || strcmp(ui->job.layer, ui->loaded_layer) != 0) {
        forget_drawing(ui);
        ui->drawing_err[0] = '\0';
        snprintf(ui->loaded_from, sizeof ui->loaded_from, "%s", abs);
        snprintf(ui->loaded_layer, sizeof ui->loaded_layer, "%s", ui->job.layer);
        if (abs[0]) {
            RgDxfOptions opt = { ui->job.chord_tolerance, ui->job.layer, true };
            char err[512];
            if (rg_dxf_load(abs, &opt, &ui->raw, err, sizeof err))
                ui->have_raw = true;
            else
                snprintf(ui->drawing_err, sizeof ui->drawing_err, "%s", err);
        }
        ui->fit_pending = true;
        ui->plan_due = 1;
    }
    if (ui->have_raw && ui->shown_scale != ui->job.drawing_scale) {
        rg_drawing_free(&ui->drawing);
        rg_shape_free(&ui->shape);
        if (rg_drawing_copy(&ui->drawing, &ui->raw)) {
            rg_drawing_scale(&ui->drawing, ui->job.drawing_scale);
            rg_shape_build_lenient(&ui->drawing, ui->job.join_tolerance, &ui->shape, &ui->skipped);
        }
        ui->shown_scale = ui->job.drawing_scale;
        ui->plan_due = 1;
    }
}

bool ui_import_dxf(RgUi *ui, const char *path)
{
    RgDxfOptions opt = { ui->job.chord_tolerance, NULL, true };
    RgDrawing probe;
    char err[300];
    if (!rg_dxf_load(path, &opt, &probe, err, sizeof err)) {
        ui_message(ui, true, "%s", err);
        return false;
    }
    int unsupported = probe.unsupported;
    char kind[16];
    snprintf(kind, sizeof kind, "%s", probe.unsupported_kind);
    rg_drawing_free(&probe);

    snprintf(ui->job.drawing, sizeof ui->job.drawing, "%s", path);
    ui->job.layer[0] = '\0';
    ui->job.drawing_scale = 1.0;
    remember_dir(ui->settings.dxf_dir, sizeof ui->settings.dxf_dir, path);
    rg_settings_save(&ui->settings);
    drawing_update(ui);
    ui->fit_pending = true;
    ui->page = PAGE_DRAW;
    if (unsupported)
        ui_message(ui, true, "Imported %s, but %d item%s (%s first) could not be shown: "
                   "convert them to lines, arcs or polylines to trace them",
                   plat_path_leaf(path), unsupported, unsupported == 1 ? "" : "s", kind);
    else
        ui_message(ui, false, "Imported %s. Check its size, and scale it if it is not in mm.",
                   plat_path_leaf(path));
    return true;
}

/* ------------------------------------------------------------------ */
/* The checks                                                          */
/* ------------------------------------------------------------------ */

static const char *source_name(const RgUi *ui)
{
    return ui->path[0] ? plat_path_leaf(ui->path) : "an unsaved job";
}

static void plan_run(RgUi *ui)
{
    ui->plan_due = 0;
    if (ui->plan_built)
        rg_plan_free(&ui->plan);
    ui->plan_built = false;
    rg_buf_free(&ui->report);
    rg_buf_free(&ui->program);

    char err[360];
    if (!rg_job_validate(&ui->job, err, sizeof err)) {
        snprintf(ui->plan_err, sizeof ui->plan_err, "%s", err);
        return;
    }
    RgShape strict;
    bool have_shape = false;
    if (ui->job.part == RG_PART_CYLINDER && ui->job.drawing[0]) {
        if (!ui->have_raw) {
            snprintf(ui->plan_err, sizeof ui->plan_err, "%s",
                     ui->drawing_err[0] ? ui->drawing_err : "the drawing is not loaded");
            return;
        }
        if (ui->drawing.unsupported) {
            snprintf(ui->plan_err, sizeof ui->plan_err, "the drawing has %d item%s rapidgen "
                     "cannot read (%s first): convert them to lines, arcs or polylines",
                     ui->drawing.unsupported, ui->drawing.unsupported == 1 ? "" : "s",
                     ui->drawing.unsupported_kind);
            return;
        }
        if (ui->drawing.degenerate) {
            snprintf(ui->plan_err, sizeof ui->plan_err, "the drawing has %d %s%s with no real "
                     "coordinates, which the editor leaves out: delete them in CAD, because a "
                     "program must not be generated from a drawing with pieces missing",
                     ui->drawing.degenerate,
                     ui->drawing.degenerate_kind[0] ? ui->drawing.degenerate_kind : "entity",
                     ui->drawing.degenerate == 1 ? "" : "s");
            return;
        }
        if (!rg_shape_build(&ui->drawing, ui->job.join_tolerance, &strict, err, sizeof err)) {
            snprintf(ui->plan_err, sizeof ui->plan_err, "%s", err);
            return;
        }
        have_shape = true;
    }
    ui->plan_err[0] = '\0';

    /* A flat part's outline is the lenient shape the tools paint over: it
     * lets the plan check the gun comes on and leaves clear of the part. */
    const RgShape *outline = have_shape ? &strict
                           : ui->job.part == RG_PART_FLAT && ui->have_raw ? &ui->shape : NULL;
    rg_plan_build(&ui->job, outline, &ui->plan);
    ui->plan_built = true;
    if (have_shape)
        rg_shape_free(&strict);

    char name[64];
    rg_rapid_filename(&ui->job, name, sizeof name);
    rg_report_write(&ui->job, &ui->plan, name, source_name(ui), &ui->report);
    if (!ui->plan.refused)
        rg_rapid_write(&ui->job, &ui->plan, source_name(ui), "not yet written", &ui->program,
                       err, sizeof err);
}

void ui_check_now(RgUi *ui)
{
    plan_run(ui);
}

bool ui_write_program(RgUi *ui)
{
    if (!ui->path[0]) {
        ui_message(ui, true, "Save the job first: the program is written beside it.");
        ui_save(ui, false);
        return false;
    }
    plan_run(ui);
    if (!ui->plan_built) {
        ui_message(ui, true, "The job is not complete: %s", ui->plan_err);
        return false;
    }
    if (ui->plan.refused) {
        ui_message(ui, true, "Refused: nothing written. The Program page says why.");
        return false;
    }

    PlatDate d;
    plat_date_now(&d);
    char stamp[32], err[512], name[64], dir[PLAT_PATH_MAX], out[PLAT_PATH_MAX], txt[64];
    snprintf(stamp, sizeof stamp, "%04d-%02d-%02d %02d:%02d", d.year, d.month, d.day, d.hour, d.minute);
    rg_rapid_filename(&ui->job, name, sizeof name);
    snprintf(dir, sizeof dir, "%s", ui->path);
    if (!plat_path_parent(dir))
        snprintf(dir, sizeof dir, ".");

    RgBuf prog;
    rg_buf_init(&prog);
    bool ok = rg_rapid_write(&ui->job, &ui->plan, source_name(ui), stamp, &prog, err, sizeof err);
    if (ok && !plat_path_join(out, sizeof out, dir, name)) {
        snprintf(err, sizeof err, "the path is too long");
        ok = false;
    }
    if (ok)
        ok = rg_write_file(out, prog.s, prog.len, err, sizeof err);
    rg_buf_free(&prog);
    snprintf(txt, sizeof txt, "%s.txt", ui->job.name);
    if (ok && plat_path_join(out, sizeof out, dir, txt))
        ok = rg_write_file(out, ui->report.s, ui->report.len, err, sizeof err);
    if (!ok) {
        ui_message(ui, true, "%s", err);
        return false;
    }
    ui_message(ui, false, "Wrote %s and %s beside the job", name, txt);
    return true;
}

/* ------------------------------------------------------------------ */
/* Status strip                                                        */
/* ------------------------------------------------------------------ */

static void draw_status(RgUi *ui, struct nk_rect r)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;
    const RgJob *j = &ui->job;

    nk_style_push_style_item(c, &c->style.window.fixed_background, nk_style_item_color(t->rail));
    nk_style_push_vec2(c, &c->style.window.padding, nk_vec2(S(ui, 14), S(ui, 8)));

    if (nk_begin(c, "status", r, NK_WINDOW_NO_SCROLLBAR)) {
        nk_layout_row_template_begin(c, S(ui, 30));
        nk_layout_row_template_push_dynamic(c);
        nk_layout_row_template_push_static(c, S(ui, 230));
        nk_layout_row_template_push_static(c, S(ui, 210));
        nk_layout_row_template_push_static(c, S(ui, 250));
        nk_layout_row_template_end(c);

        char buf[256];
        snprintf(buf, sizeof buf, "%s%s", j->name[0] ? j->name : "Untitled",
                 ui->modified ? " *" : "");
        ui_tip(ui, ui->path[0] ? ui->path : "Not saved yet");
        nk_label(c, buf, NK_TEXT_LEFT);

        if (j->part == RG_PART_FLAT)
            snprintf(buf, sizeof buf, "Flat part  \xc2\xb7  %d stroke%s", j->nstrokes,
                     j->nstrokes == 1 ? "" : "s");
        else
            snprintf(buf, sizeof buf, "Cylinder  \xc2\xb7  %s", j->drawing[0] ? "bands from drawing"
                     : j->nbands == 1 ? "1 band" : "bands");
        nk_label_colored(c, buf, NK_TEXT_LEFT, t->text_dim);

        const RgDialect *d = rg_dialect_find(j->controller);
        const RgRobot *rb = rg_robot_find(j->robot);
        snprintf(buf, sizeof buf, "%s  \xc2\xb7  %s", d ? d->name : "no controller",
                 rb ? rb->name : "no robot");
        nk_label_colored(c, buf, NK_TEXT_LEFT, t->text_dim);

        if (ui->plan_due) {
            nk_label_colored(c, "checking\xe2\x80\xa6", NK_TEXT_RIGHT, t->text_faint);
        } else if (!ui->plan_built) {
            ui_tip(ui, ui->plan_err);
            nk_label_colored(c, "\xe2\x97\x8f job not complete", NK_TEXT_RIGHT, t->warn);
        } else if (ui->plan.refused) {
            int n = rg_plan_count(&ui->plan, RG_REFUSE);
            snprintf(buf, sizeof buf, "\xe2\x97\x8f refused: %d reason%s", n, n == 1 ? "" : "s");
            ui_tip(ui, "No program can be written until these are fixed. See the Program page");
            nk_label_colored(c, buf, NK_TEXT_RIGHT, t->alarm);
        } else {
            int w = rg_plan_count(&ui->plan, RG_WARN);
            if (w)
                snprintf(buf, sizeof buf, "\xe2\x9c\x93 ready, %d warning%s", w, w == 1 ? "" : "s");
            else
                snprintf(buf, sizeof buf, "\xe2\x9c\x93 ready to write");
            ui_tip(ui, "Every check passed: the program can be written from the Program page");
            nk_label_colored(c, buf, NK_TEXT_RIGHT, w ? t->warn : t->ok);
        }
    }
    nk_end(c);
    nk_style_pop_vec2(c);
    nk_style_pop_style_item(c);
}

/* ------------------------------------------------------------------ */
/* Navigation rail                                                     */
/* ------------------------------------------------------------------ */

static bool rail_button(RgUi *ui, const char *label, const char *tip)
{
    nk_layout_row_dynamic(ui->ctx, S(ui, 30), 1);
    ui_tip(ui, tip);
    return nk_button_label(ui->ctx, label) != 0;
}

static void draw_rail(RgUi *ui, struct nk_rect r)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;

    nk_style_push_style_item(c, &c->style.window.fixed_background, nk_style_item_color(t->rail));
    nk_style_push_vec2(c, &c->style.window.padding, nk_vec2(S(ui, 8), S(ui, 12)));

    if (nk_begin(c, "rail", r, NK_WINDOW_NO_SCROLLBAR)) {
        for (int i = 0; i < PAGE_COUNT; i++) {
            nk_layout_row_dynamic(c, S(ui, 32), 1);
            ui_tip(ui, PAGE_HINT[i]);
            nk_bool on = ui->page == (Page)i;
            if (nk_selectable_label(c, PAGE_NAME[i], NK_TEXT_LEFT, &on) && on)
                ui->page = (Page)i;
        }

        ui_gap(ui, 10);
        if (rail_button(ui, "New flat part", "Start a job for a flat part: import its drawing "
                        "and paint the strokes (Ctrl+N)"))
            ui_request(ui, PENDING_NEW_FLAT);
        if (rail_button(ui, "New cylinder", "Start a job for a cylinder on a rotator"))
            ui_request(ui, PENDING_NEW_CYLINDER);
        if (rail_button(ui, "Open...", "Open a saved job (Ctrl+O)"))
            ui_request(ui, PENDING_OPEN);
        if (rail_button(ui, "Save", ui->path[0] ? "Save the job (Ctrl+S)"
                                                : "Save the job: it has no file yet (Ctrl+S)"))
            ui_save(ui, false);
        if (rail_button(ui, "Save as...", "Save the job under a new name (Ctrl+Shift+S)"))
            ui_save(ui, true);
        if (rail_button(ui, "Import DXF...", "Show a part's drawing to paint over (Ctrl+I)"))
            ui_open_file_dialog(ui, FILE_IMPORT_DXF);

        ui_gap(ui, 8);
        nk_layout_row_dynamic(c, S(ui, 30), 2);
        bool cu = ui_can_undo(ui), cr = ui_can_redo(ui);
        if (!cu) nk_widget_disable_begin(c);
        ui_tip(ui, "Undo the last change (Ctrl+Z)");
        if (nk_button_label(c, "Undo") && cu)
            ui_undo(ui);
        if (!cu) nk_widget_disable_end(c);
        if (!cr) nk_widget_disable_begin(c);
        ui_tip(ui, "Redo (Ctrl+Y)");
        if (nk_button_label(c, "Redo") && cr)
            ui_redo(ui);
        if (!cr) nk_widget_disable_end(c);

        ui_gap(ui, 8);
        if (rail_button(ui, "About...", "Version, build and licence")) {
            ui->dlg_error[0] = '\0';
            ui->dialog = DLG_ABOUT;
        }
        if (rail_button(ui, ui->dark ? "Light theme" : "Dark theme",
                        ui->dark ? "Switch to the light theme" : "Switch to the dark theme"))
            ui->theme_toggle = true;
    }
    nk_end(c);
    nk_style_pop_vec2(c);
    nk_style_pop_style_item(c);
}

/* ------------------------------------------------------------------ */
/* Action bar                                                          */
/* ------------------------------------------------------------------ */

static void draw_action(RgUi *ui, struct nk_rect r)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;

    nk_style_push_style_item(c, &c->style.window.fixed_background, nk_style_item_color(t->rail));
    nk_style_push_vec2(c, &c->style.window.padding, nk_vec2(S(ui, 14), S(ui, 6)));

    if (nk_begin(c, "action", r, NK_WINDOW_NO_SCROLLBAR)) {
        nk_layout_row_template_begin(c, S(ui, 20));
        nk_layout_row_template_push_dynamic(c);
        nk_layout_row_template_push_static(c, S(ui, 520));
        nk_layout_row_template_end(c);

        if (ui->msg[0]) {
            ui_tip(ui, ui->msg);
            nk_label_colored(c, ui->msg, NK_TEXT_LEFT, ui->msg_error ? t->alarm : t->text_dim);
        } else {
            nk_label_colored(c, RAPIDGEN_TAGLINE, NK_TEXT_LEFT, t->text_faint);
        }
        ui_tip(ui, ui->path[0] ? ui->path : NULL);
        nk_label_colored(c, ui->path[0] ? ui->path : "not saved", NK_TEXT_RIGHT, t->text_faint);
    }
    nk_end(c);
    nk_style_pop_vec2(c);
    nk_style_pop_style_item(c);
}

/* ------------------------------------------------------------------ */
/* Frame                                                               */
/* ------------------------------------------------------------------ */

static void update_title(RgUi *ui)
{
    char want[320];
    snprintf(want, sizeof want, "%s%s - %s %s", ui->job.name[0] ? ui->job.name : "Untitled",
             ui->modified ? " *" : "", RAPIDGEN_NAME, RAPIDGEN_VERSION);
    if (strcmp(want, ui->title) != 0) {
        snprintf(ui->title, sizeof ui->title, "%s", want);
        SDL_SetWindowTitle(ui->win, ui->title);
    }
}

void rg_ui_frame(RgUi *ui, int w, int h)
{
    struct nk_context *c = ui->ctx;

    if (ui->theme_toggle) {
        ui->theme_toggle = false;
        ui->dark = !ui->dark;
        ui->theme = ui->dark ? &RG_THEME_DARK : &RG_THEME_LIGHT;
        rg_theme_apply(c, ui->theme, ui->scale);
        ui->settings.dark = ui->dark;
        rg_settings_save(&ui->settings);
    }

    drawing_update(ui);
    sync_job(ui);
    if (ui->plan_due && plat_now_ms() >= ui->plan_due &&
        !c->input.mouse.buttons[NK_BUTTON_LEFT].down)
        plan_run(ui);
    update_title(ui);
    ui->tip_text[0] = '\0';

    float sh = S(ui, STATUS_H), ah = S(ui, ACTION_H), rw = S(ui, RAIL_W);
    float body_h = (float)h - sh - ah;
    if (body_h < 100.0f)
        body_h = 100.0f;

    draw_status(ui, nk_rect(0, 0, (float)w, sh));
    draw_rail(ui, nk_rect(0, sh, rw, body_h));
    draw_action(ui, nk_rect(0, (float)h - ah, (float)w, ah));

    struct nk_rect content = nk_rect(rw, sh, (float)w - rw, body_h);
    bool draw_page = ui->page == PAGE_DRAW;

    nk_style_push_style_item(c, &c->style.window.fixed_background, nk_style_item_color(ui->theme->bg));
    if (draw_page) {
        nk_style_push_vec2(c, &c->style.window.padding, nk_vec2(0, 0));
        nk_style_push_vec2(c, &c->style.window.spacing, nk_vec2(0, 0));
    }
    /* The form reads as a column, not as fields stretched across a wide
     * screen: centre it and cap its width. */
    bool settings = ui->page == PAGE_SETTINGS;
    if (settings) {
        float pad = (content.w - S(ui, 900)) * 0.5f;
        if (pad < S(ui, 14))
            pad = S(ui, 14);
        nk_style_push_vec2(c, &c->style.window.padding, nk_vec2(pad, S(ui, 12)));
    }
    nk_flags flags = settings ? 0 : NK_WINDOW_NO_SCROLLBAR;
    if (nk_begin(c, "content", content, flags)) {
        if (ui->page != ui->last_page) {
            nk_window_set_scroll(c, 0, 0);
            ui->last_page = ui->page;
        }
        struct nk_rect inner = nk_window_get_content_region(c);
        switch (ui->page) {
        case PAGE_DRAW:     page_draw(ui, inner);     break;
        case PAGE_SETTINGS: page_settings(ui, inner); break;
        case PAGE_PROGRAM:  page_program(ui, inner);  break;
        default: break;
        }
    }
    nk_end(c);
    if (draw_page) {
        nk_style_pop_vec2(c);
        nk_style_pop_vec2(c);
    }
    if (settings)
        nk_style_pop_vec2(c);
    nk_style_pop_style_item(c);

    if (ui->dialog != DLG_NONE)
        ui_draw_dialog(ui, w, h);

    tip_draw(ui, w, h);
}

/* ------------------------------------------------------------------ */
/* Lifetime                                                            */
/* ------------------------------------------------------------------ */

void rg_ui_output_size(const RgUi *ui, int *w, int *h)
{
    *w = *h = 0;
    SDL_GetRendererOutputSize(ui->ren, w, h);
}

static float detect_scale(const RgUi *ui)
{
    const char *env = getenv("RAPIDGEN_SCALE");
    if (env && atof(env) > 0.1)
        return (float)atof(env);

    int ww = 0, wh = 0, dw = 0, dh = 0;
    SDL_GetWindowSize(ui->win, &ww, &wh);
    rg_ui_output_size(ui, &dw, &dh);
    float scale = ww > 0 ? (float)dw / (float)ww : 1.0f;

    float ddpi = 0;
    if (SDL_GetDisplayDPI(SDL_GetWindowDisplayIndex(ui->win), &ddpi, NULL, NULL) == 0) {
        float dpi_scale = ddpi / 96.0f;
        if (dpi_scale > scale)
            scale = dpi_scale;
    }
    if (scale < 1.0f) scale = 1.0f;
    if (scale > 4.0f) scale = 4.0f;
    return scale;
}

static void load_font(RgUi *ui)
{
    struct nk_font_atlas *atlas = NULL;
    nk_sdl_font_stash_begin(&atlas);

    static const nk_rune ranges[] = {
        0x0020, 0x00FF, 0x2010, 0x2027, 0x2190, 0x2193, 0x2212, 0x2212,
        0x25A0, 0x25FF, 0x2713, 0x2713, 0
    };
    struct nk_font_config cfg = nk_font_config(14.0f * ui->scale);
    cfg.range = ranges;
    cfg.oversample_h = 2;
    cfg.oversample_v = 1;
    cfg.pixel_snap = 0;

    struct nk_font *font = NULL;

    /* A bundled face first, relative to the executable, so the interface
     * reads the same on a machine with no fonts. SDL_GetBasePath because an
     * AppImage mounts somewhere new every run. */
    char bundled[2][512];
    bundled[0][0] = bundled[1][0] = '\0';
    char *base = SDL_GetBasePath();
    if (base) {
        snprintf(bundled[0], sizeof bundled[0], "%sDejaVuSans.ttf", base);
        snprintf(bundled[1], sizeof bundled[1], "%s../share/rapidgen/DejaVuSans.ttf", base);
        SDL_free(base);
    }
    const char *candidates[] = {
        bundled[0], bundled[1],
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/Library/Fonts/Arial.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "C:\\Windows\\Fonts\\segoeui.ttf",
        NULL
    };
    for (int i = 0; candidates[i] && !font; i++) {
        if (!candidates[i][0] || !plat_file_exists(candidates[i]))
            continue;
        font = nk_font_atlas_add_from_file(atlas, candidates[i], 14.0f * ui->scale, &cfg);
    }
    nk_sdl_font_stash_end();
    if (font)
        nk_style_set_font(ui->ctx, &font->handle);
}

RgUi *rg_ui_create(SDL_Window *win, SDL_Renderer *ren, const char *job)
{
    RgUi *ui = calloc(1, sizeof *ui);
    if (!ui)
        return NULL;
    ui->win = win;
    ui->ren = ren;
    rg_job_default(&ui->job);
    ui->saved_text = dup_text("");
    ui->cur_text = dup_text("");
    ui->entries = calloc(FILE_ENTRIES, sizeof *ui->entries);
    if (!ui->saved_text || !ui->cur_text || !ui->entries) {
        rg_ui_destroy(ui);
        return NULL;
    }
    rg_buf_init(&ui->report);
    rg_buf_init(&ui->program);

    rg_settings_load(&ui->settings);
    ui->dark = ui->settings.dark;
    ui->theme = ui->dark ? &RG_THEME_DARK : &RG_THEME_LIGHT;
    ui->scale = detect_scale(ui);
    ui->page = PAGE_DRAW;
    ui->last_page = PAGE_COUNT;
    ui->shown_scale = NAN;
    ui->zoom = 1.0;
    ui->selected = ui->hover_stroke = ui->hover_loop = ui->preview_loop = -1;
    ui->snap = true;
    ui->smoothing = 1.0;
    ui->fill_extend = true;

    ui->ctx = nk_sdl_init(ui->win, ui->ren);
    if (!ui->ctx) {
        rg_ui_destroy(ui);
        return NULL;
    }
    load_font(ui);
    rg_theme_apply(ui->ctx, ui->theme, ui->scale);

    new_job(ui, RG_PART_FLAT);
    ui->msg[0] = '\0';

    const char *open = job;
    if (!open && ui->settings.last_job[0] && plat_file_exists(ui->settings.last_job))
        open = ui->settings.last_job;
    if (open)
        ui_open_path(ui, open);
    return ui;
}

void rg_ui_fit_window(RgUi *ui, int base_w, int base_h)
{
    int ww = 0, wh = 0, dw = 0, dh = 0;
    SDL_GetWindowSize(ui->win, &ww, &wh);
    rg_ui_output_size(ui, &dw, &dh);
    float px_per_unit = (ww > 0 && dw > 0) ? (float)dw / (float)ww : 1.0f;

    float unit_scale = ui->scale / px_per_unit;
    float w = (float)base_w * unit_scale, h = (float)base_h * unit_scale;

    SDL_Rect usable;
    if (SDL_GetDisplayUsableBounds(SDL_GetWindowDisplayIndex(ui->win), &usable) == 0 &&
        usable.w > 0 && usable.h > 0) {
        float max_w = (float)usable.w * WIN_MAX_FRAC, max_h = (float)usable.h * WIN_MAX_FRAC;
        float shrink = 1.0f;
        if (w > max_w) shrink = max_w / w;
        if (h > max_h && max_h / h < shrink) shrink = max_h / h;
        w *= shrink;
        h *= shrink;
    }
    int min_w = (int)((float)WIN_MIN_W * unit_scale);
    int min_h = (int)((float)WIN_MIN_H * unit_scale);
    SDL_SetWindowMinimumSize(ui->win, min_w, min_h);
    if (w < (float)min_w) w = (float)min_w;
    if (h < (float)min_h) h = (float)min_h;
    SDL_SetWindowSize(ui->win, (int)w, (int)h);
    SDL_SetWindowPosition(ui->win, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
}

void rg_ui_destroy(RgUi *ui)
{
    if (!ui)
        return;
    if (ui->ctx) {
        rg_settings_save(&ui->settings);
        nk_sdl_shutdown();
    }
    if (ui->plan_built)
        rg_plan_free(&ui->plan);
    rg_buf_free(&ui->report);
    rg_buf_free(&ui->program);
    forget_drawing(ui);
    draw_shutdown(ui);
    rg_job_free(&ui->job);
    undo_clear(ui);
    free(ui->saved_text);
    free(ui->cur_text);
    free(ui->entries);
    free(ui);
}

void rg_ui_input_begin(RgUi *ui) { nk_input_begin(ui->ctx); }
void rg_ui_input_end(RgUi *ui)   { nk_input_end(ui->ctx); }

bool rg_ui_handle_event(RgUi *ui, SDL_Event *e)
{
    if (e->type == SDL_QUIT) {
        if (ui->dialog == DLG_NONE)
            ui_request(ui, PENDING_QUIT);
        return true;
    }
    if (e->type == SDL_KEYDOWN) {
        SDL_Keycode k = e->key.keysym.sym;
        Uint16 mod = e->key.keysym.mod;
        bool ctrl = (mod & KMOD_CTRL) != 0;

        if (k == SDLK_ESCAPE && ui->dialog != DLG_NONE && ui->dialog != DLG_CONFIRM) {
            ui->dialog = DLG_NONE;
            return true;
        }
        if (ui->dialog == DLG_NONE && ctrl) {
            /* Text fields keep Ctrl+Z for themselves while they have the
             * keyboard; everywhere else it is the job's undo. */
            bool editing = ui_typing(ui->ctx);
            switch (k) {
            case SDLK_z:
                if (editing) break;
                if (mod & KMOD_SHIFT) ui_redo(ui); else ui_undo(ui);
                return true;
            case SDLK_y:
                if (editing) break;
                ui_redo(ui);
                return true;
            case SDLK_s: ui_save(ui, (mod & KMOD_SHIFT) != 0); return true;
            case SDLK_o: ui_request(ui, PENDING_OPEN); return true;
            case SDLK_n: ui_request(ui, PENDING_NEW_FLAT); return true;
            case SDLK_i: ui_open_file_dialog(ui, FILE_IMPORT_DXF); return true;
            default: break;
            }
        }
        if (ui->dialog == DLG_NONE && ui->page == PAGE_DRAW && !ctrl &&
            !ui_typing(ui->ctx) && draw_handle_key(ui, k, mod))
            return true;
    }
    return nk_sdl_handle_event(e) != 0;
}

void rg_ui_present(RgUi *ui)
{
    const struct nk_color bg = ui->theme->bg;
    SDL_SetRenderDrawColor(ui->ren, bg.r, bg.g, bg.b, 255);
    SDL_RenderClear(ui->ren);
    nk_sdl_render(NK_ANTI_ALIASING_ON);
    SDL_RenderPresent(ui->ren);
}

bool rg_ui_quit_requested(const RgUi *ui) { return ui->quit; }

bool rg_ui_busy(const RgUi *ui) { return ui->plan_due != 0 || ui->freehand || ui->panning; }
