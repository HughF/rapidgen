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
 * rg_ui_int.h — what the interface's own files share
 *
 * rg_ui.c is the shell (status strip, rail, action bar, tooltips, the job
 * file, undo, the drawing and the checks), rg_ui_draw.c the Draw page with
 * its canvas and painting tools, rg_ui_pages.c the Settings and Program
 * pages, rg_ui_dialogs.c the file browser and the dialogs. Not for use
 * outside those files.
 *
 * Same layout rule as pipegen and svpview: the shell computes an exact pixel
 * rect for every region, and each region is its own Nuklear window at it.
 *
 * The job is the only state that matters. Every frame it is written out as
 * job-file text; a change in that text is what marks the job modified,
 * records an undo step once it settles, and re-runs the checks. Undo
 * restores a job by parsing an old text, so it can never restore something a
 * job file could not hold.
 */
#ifndef RG_UI_INT_H
#define RG_UI_INT_H

#include <SDL2/SDL.h>
#include <stdarg.h>

#include "nk.h"
#include "plat.h"
#include "rg_config.h"
#include "rg_job.h"
#include "rg_pattern.h"
#include "rg_plan.h"
#include "rg_theme.h"
#include "rg_ui.h"

/* Unscaled metrics, multiplied by the HiDPI scale at use. */
#define STATUS_H     46.0f
#define ACTION_H     32.0f
#define RAIL_W      150.0f
#define ROW_H        26.0f
#define LABEL_W     150.0f
#define UNIT_W       46.0f
#define BTN_W        96.0f
#define BTN_H        32.0f
#define TOOLS_W      76.0f
#define INSPECTOR_W 350.0f
#define DLG_BOTTOM_MARGIN 12.0f

typedef enum { PAGE_DRAW = 0, PAGE_SETTINGS, PAGE_PROGRAM, PAGE_COUNT } Page;

typedef enum { DLG_NONE = 0, DLG_FILE, DLG_CONFIRM, DLG_ABOUT } Dialog;

typedef enum { DLG_R_NONE, DLG_R_CANCEL, DLG_R_OK, DLG_R_ALT } DlgResult;

typedef enum { FILE_OPEN_JOB, FILE_SAVE_JOB, FILE_IMPORT_DXF } FileMode;

/* What to do once an unsaved-changes question is answered. */
typedef enum {
    PENDING_NONE = 0, PENDING_NEW_FLAT, PENDING_NEW_CYLINDER, PENDING_OPEN,
    PENDING_OPEN_PATH, PENDING_QUIT
} Pending;

typedef enum {
    TOOL_PAN = 0, TOOL_LINE, TOOL_FREE, TOOL_TRACE, TOOL_FILL, TOOL_SCALE, TOOL_COUNT
} Tool;

#define UNDO_MAX     100
#define FILE_ENTRIES 512

struct RgUi {
    struct nk_context *ctx;
    SDL_Window        *win;
    SDL_Renderer      *ren;

    RgSettings     settings;
    const RgTheme *theme;
    bool           dark, theme_toggle;
    float          scale;
    Page           page, last_page;
    bool           quit;

    /* the job, and the file it lives in */
    RgJob    job;
    char     path[PLAT_PATH_MAX];
    char    *saved_text;             /* what the file holds; "" if never saved */
    char    *cur_text;               /* the job, as text                       */
    bool     modified;

    /* undo: job texts, oldest first */
    char    *undo[UNDO_MAX];
    int      undo_n, undo_pos;
    uint64_t change_ms;
    bool     change_waiting;

    char     msg[320];
    bool     msg_error;

    /* the drawing: as read, then scaled, then its outlines */
    char      loaded_from[PLAT_PATH_MAX];   /* absolute; "" for none */
    char      loaded_layer[64];
    RgDrawing raw;
    bool      have_raw;
    RgDrawing drawing;
    RgShape   shape;
    int       skipped;                      /* lines closing no outline */
    double    shown_scale;
    char      drawing_err[520];

    /* the canvas view: drawing millimetres at the centre, pixels per mm */
    double         cx, cy, zoom;
    bool           fit_pending;
    struct nk_rect canvas;
    bool           panning;
    struct nk_vec2 pan_last;

    /* painting */
    Tool     tool;
    RgPt    *draft;
    int      ndraft, cap_draft;
    bool     freehand;             /* a freehand drag is under way   */
    int      selected;             /* stroke, or -1                  */
    int      hover_stroke, hover_loop;
    bool     snap;
    double   smoothing, inset, fill_angle;
    bool     fill_extend, fill_spiral;
    int      scale_clicks;
    RgPt     scale_a, scale_b;
    double   scale_true, width_true;
    bool     mouse_in;
    RgPt     mouse_mm;
    RgSnap   snap_kind;
    RgPt     snap_pt;

    /* the fill tool's preview of the region under the pointer */
    RgStroke *preview;
    int       npreview, preview_loop;
    double    preview_key[5];   /* pitch, angle, extend, scale, spiral */

    float    *scratch;             /* screen points for one polyline */
    int       scratch_cap;

    /* the checks, re-run a moment after the job stops changing */
    RgPlan   plan;
    bool     plan_built;
    char     plan_err[560];
    uint64_t plan_due;             /* 0: nothing waiting */
    RgBuf    report, program;

    /* dialogs */
    Dialog   dialog;
    char     dlg_error[400];
    Dialog   dlg_measured;
    float    dlg_natural_h;

    FileMode      file_mode;
    char          file_dir[PLAT_PATH_MAX];
    char          file_name[160];
    PlatDirEntry *entries;
    int           n_entries;
    bool          file_listed;

    Pending  pending;
    char     pending_path[PLAT_PATH_MAX];
    bool     save_then_pending;

    /* tooltips: this frame's candidate, and the one the dwell counts on */
    char     tip_text[320];
    struct nk_rect tip_over;
    char     tip_shown[320];
    uint64_t tip_since_ms;

    char     title[320];
};

static inline float S(const RgUi *ui, float v) { return v * ui->scale; }

/* Leave one cell of the current row empty. Not nk_spacing(ctx, 1): that
 * wraps onto a new row when it fills the last column, and the next layout
 * call then advances by the row's height a second time. */
static inline void nk_skip(struct nk_context *c)
{
    struct nk_rect unused;
    nk_widget(&unused, c);
}

/* ---- rg_ui.c ------------------------------------------------------- */

float ui_text_width(const RgUi *ui, const char *s, int len);
float ui_wrap_height(RgUi *ui, const char *text, float w);
void  ui_label_wrap(RgUi *ui, const char *text, struct nk_color col);
void  ui_tip(RgUi *ui, const char *text);
void  ui_tip_rect(RgUi *ui, struct nk_rect over, const char *text);
void  ui_form_row(RgUi *ui, float h);
void  ui_section(RgUi *ui, const char *title);
void  ui_gap(RgUi *ui, float h);
bool  ui_primary_button(RgUi *ui, const char *label);
void  ui_info_row(RgUi *ui, const char *k, const char *v);
void  ui_info_rowf(RgUi *ui, const char *k, const char *fmt, ...);
bool  ui_prop(RgUi *ui, const char *label, const char *tip, double *v,
              double min, double max, double step, const char *unit);
bool  ui_prop_int(RgUi *ui, const char *label, const char *tip, int *v,
                  int min, int max, const char *unit);
bool  ui_check(RgUi *ui, const char *label, const char *tip, bool *v);
void  ui_message(RgUi *ui, bool error, const char *fmt, ...);
bool  ui_typing(const struct nk_context *ctx);

bool  ui_open_path(RgUi *ui, const char *path);
bool  ui_save_to(RgUi *ui, const char *path);
bool  ui_save(RgUi *ui, bool save_as);
void  ui_request(RgUi *ui, Pending what);
void  ui_continue_pending(RgUi *ui);
bool  ui_import_dxf(RgUi *ui, const char *path);
bool  ui_write_program(RgUi *ui);
void  ui_check_now(RgUi *ui);

bool  ui_can_undo(const RgUi *ui);
bool  ui_can_redo(const RgUi *ui);
void  ui_undo(RgUi *ui);
void  ui_redo(RgUi *ui);

/* Multiply the drawing scale by `factor` and move the strokes with it, so
 * they stay where they were painted on the drawing. */
void  ui_rescale(RgUi *ui, double factor);

/* ---- rg_ui_draw.c -------------------------------------------------- */

void  page_draw(RgUi *ui, struct nk_rect inner);
bool  draw_handle_key(RgUi *ui, SDL_Keycode key, Uint16 mod);
void  draw_forget(RgUi *ui);         /* a different job: drop the draft and selection */
void  draw_shutdown(RgUi *ui);

/* ---- rg_ui_pages.c ------------------------------------------------- */

void  page_settings(RgUi *ui, struct nk_rect inner);
void  page_program(RgUi *ui, struct nk_rect inner);

/* ---- rg_ui_dialogs.c ----------------------------------------------- */

void  ui_open_file_dialog(RgUi *ui, FileMode mode);
void  ui_open_confirm(RgUi *ui, Pending pending);
void  ui_draw_dialog(RgUi *ui, int w, int h);

#endif /* RG_UI_INT_H */
