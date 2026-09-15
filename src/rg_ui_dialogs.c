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
 * rg_ui_dialogs.c — the file browser, the unsaved-changes question and About
 *
 * SDL2 has no native file dialog, and pulling in a toolkit for one would
 * break the SDL-only build, so the browser is drawn here: a folder, its
 * folders and the files of the kind wanted, and a name.
 */
#include "rg_ui_int.h"
#include "rg_version.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *mode_ext(FileMode m)
{
    return m == FILE_IMPORT_DXF ? ".dxf" : RG_JOB_EXT;
}

static bool has_ext(const char *name, const char *ext)
{
    size_t n = strlen(name), e = strlen(ext);
    return n > e && rg_streqi(name + n - e, ext);
}

/* ------------------------------------------------------------------ */
/* Openers                                                             */
/* ------------------------------------------------------------------ */

static void list_dir(RgUi *ui)
{
    ui->n_entries = plat_dir_list(ui->file_dir, ui->entries, FILE_ENTRIES);
    if (ui->n_entries < 0) {
        ui->n_entries = 0;
        snprintf(ui->dlg_error, sizeof ui->dlg_error, "Cannot read %.180s", ui->file_dir);
    } else {
        ui->dlg_error[0] = '\0';
    }
    ui->file_listed = true;
}

void ui_open_file_dialog(RgUi *ui, FileMode mode)
{
    ui->file_mode = mode;
    const char *dir = mode == FILE_IMPORT_DXF ? ui->settings.dxf_dir : ui->settings.job_dir;
    char here[PLAT_PATH_MAX];
    if (mode == FILE_SAVE_JOB && ui->path[0]) {
        snprintf(here, sizeof here, "%s", ui->path);
        if (plat_path_parent(here))
            dir = here;
    }
    if (!dir[0] || !plat_is_dir(dir)) {
        static char docs[PLAT_PATH_MAX];
        if (!plat_documents_dir(docs, sizeof docs))
            snprintf(docs, sizeof docs, ".");
        dir = docs;
    }
    snprintf(ui->file_dir, sizeof ui->file_dir, "%s", dir);
    if (mode == FILE_SAVE_JOB) {
        if (ui->path[0])
            snprintf(ui->file_name, sizeof ui->file_name, "%s", plat_path_leaf(ui->path));
        else
            snprintf(ui->file_name, sizeof ui->file_name, "%s%s",
                     ui->job.name[0] ? ui->job.name : "job", RG_JOB_EXT);
    } else {
        ui->file_name[0] = '\0';
    }
    list_dir(ui);
    ui->dialog = DLG_FILE;
}

void ui_open_confirm(RgUi *ui, Pending pending)
{
    ui->pending = pending;
    ui->dlg_error[0] = '\0';
    ui->dialog = DLG_CONFIRM;
}

/* ------------------------------------------------------------------ */
/* Dialog chrome                                                       */
/* ------------------------------------------------------------------ */

static DlgResult dialog_buttons(RgUi *ui, bool can_ok, const char *primary, const char *alt)
{
    struct nk_context *c = ui->ctx;
    DlgResult res = DLG_R_NONE;

    nk_layout_row_template_begin(c, S(ui, BTN_H));
    nk_layout_row_template_push_dynamic(c);
    nk_layout_row_template_push_static(c, S(ui, BTN_W));
    if (alt)
        nk_layout_row_template_push_static(c, S(ui, BTN_W));
    nk_layout_row_template_push_static(c, S(ui, BTN_W));
    nk_layout_row_template_end(c);

    nk_skip(c);
    ui_tip(ui, "Close without doing it. Escape does the same");
    if (nk_button_label(c, "Cancel"))
        res = DLG_R_CANCEL;
    if (alt && nk_button_label(c, alt))
        res = DLG_R_ALT;
    if (!can_ok) nk_widget_disable_begin(c);
    if (ui_primary_button(ui, primary) && can_ok)
        res = DLG_R_OK;
    if (!can_ok) nk_widget_disable_end(c);
    return res;
}

/* ------------------------------------------------------------------ */
/* File browser                                                        */
/* ------------------------------------------------------------------ */

static bool file_commit(RgUi *ui)
{
    if (!ui->file_name[0]) {
        snprintf(ui->dlg_error, sizeof ui->dlg_error, "Give a file name.");
        return false;
    }
    char name[200];
    snprintf(name, sizeof name, "%s", ui->file_name);
    if (ui->file_mode == FILE_SAVE_JOB && !has_ext(name, RG_JOB_EXT))
        strncat(name, RG_JOB_EXT, sizeof name - strlen(name) - 1);

    char path[PLAT_PATH_MAX];
    if (!plat_path_join(path, sizeof path, ui->file_dir, name)) {
        snprintf(ui->dlg_error, sizeof ui->dlg_error, "That path is too long.");
        return false;
    }

    if (ui->file_mode != FILE_SAVE_JOB && !plat_file_exists(path)) {
        snprintf(ui->dlg_error, sizeof ui->dlg_error, "No such file.");
        return false;
    }
    bool ok;
    switch (ui->file_mode) {
    case FILE_OPEN_JOB:
        ok = ui_open_path(ui, path);
        break;
    case FILE_IMPORT_DXF:
        ok = ui_import_dxf(ui, path);
        break;
    default:
        ok = ui_save_to(ui, path);
        break;
    }
    if (!ok) {
        snprintf(ui->dlg_error, sizeof ui->dlg_error, "%s", ui->msg);
        return false;
    }
    ui->dialog = DLG_NONE;
    if (ui->file_mode == FILE_SAVE_JOB && ui->save_then_pending)
        ui_continue_pending(ui);
    return true;
}

static void file_body(RgUi *ui)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;
    const char *ext = mode_ext(ui->file_mode);

    if (!ui->file_listed)
        list_dir(ui);

    nk_layout_row_template_begin(c, S(ui, ROW_H));
    nk_layout_row_template_push_static(c, S(ui, 60));
    nk_layout_row_template_push_dynamic(c);
    nk_layout_row_template_push_static(c, S(ui, 60));
    nk_layout_row_template_end(c);
    nk_label_colored(c, "Folder", NK_TEXT_LEFT, t->text_dim);
    ui_tip(ui, "Type a folder and press Enter");
    nk_flags f = nk_edit_string_zero_terminated(c, NK_EDIT_FIELD | NK_EDIT_SIG_ENTER,
                                                ui->file_dir, sizeof ui->file_dir,
                                                nk_filter_default);
    if (f & NK_EDIT_COMMITED)
        list_dir(ui);
    ui_tip(ui, "Up one folder");
    if (nk_button_label(c, "Up") && plat_path_parent(ui->file_dir))
        list_dir(ui);

    if (ui->file_mode == FILE_OPEN_JOB && ui->settings.recent[0][0]) {
        ui_section(ui, "Recent");
        for (int i = 0; i < RG_RECENT; i++) {
            const char *rp = ui->settings.recent[i];
            if (!rp[0] || !plat_file_exists(rp))
                continue;
            nk_layout_row_dynamic(c, S(ui, 22), 1);
            ui_tip(ui, rp);
            if (nk_button_label(c, plat_path_leaf(rp))) {
                ui->dialog = DLG_NONE;
                ui_open_path(ui, rp);
                return;
            }
        }
    }

    ui_section(ui, ui->file_mode == FILE_IMPORT_DXF ? "Drawings"
                   : ui->file_mode == FILE_OPEN_JOB ? "Jobs" : "In this folder");
    nk_layout_row_dynamic(c, S(ui, 250), 1);
    if (nk_group_begin(c, "files", NK_WINDOW_BORDER)) {
        int shown = 0;
        for (int i = 0; i < ui->n_entries; i++) {
            PlatDirEntry *e = &ui->entries[i];
            if (!e->is_dir && !has_ext(e->name, ext))
                continue;
            shown++;
            nk_layout_row_dynamic(c, S(ui, 22), 1);
            char lab[300];
            snprintf(lab, sizeof lab, "%s%s", e->name, e->is_dir ? "/" : "");
            nk_bool on = !e->is_dir && strcmp(e->name, ui->file_name) == 0;
            if (nk_selectable_label(c, lab, NK_TEXT_LEFT, &on)) {
                if (e->is_dir) {
                    char next[PLAT_PATH_MAX];
                    if (plat_path_join(next, sizeof next, ui->file_dir, e->name)) {
                        snprintf(ui->file_dir, sizeof ui->file_dir, "%s", next);
                        list_dir(ui);
                    }
                    break;              /* the list just changed under us */
                }
                snprintf(ui->file_name, sizeof ui->file_name, "%s", e->name);
            }
        }
        if (!shown) {
            nk_layout_row_dynamic(c, S(ui, 22), 1);
            char none[80];
            snprintf(none, sizeof none, "No folders or %s files here.", ext);
            nk_label_colored(c, none, NK_TEXT_LEFT, t->text_faint);
        }
        nk_group_end(c);
    }

    nk_layout_row_template_begin(c, S(ui, ROW_H));
    nk_layout_row_template_push_static(c, S(ui, 60));
    nk_layout_row_template_push_dynamic(c);
    nk_layout_row_template_end(c);
    nk_label_colored(c, "Name", NK_TEXT_LEFT, t->text_dim);
    ui_tip(ui, ui->file_mode == FILE_SAVE_JOB ? "The job file's name; .rgj is added"
                                              : "The file to open");
    f = nk_edit_string_zero_terminated(c, NK_EDIT_FIELD | NK_EDIT_SIG_ENTER, ui->file_name,
                                       sizeof ui->file_name, nk_filter_default);
    if (f & NK_EDIT_COMMITED)
        file_commit(ui);
}

/* ------------------------------------------------------------------ */
/* Confirm and About                                                   */
/* ------------------------------------------------------------------ */

static void confirm_body(RgUi *ui)
{
    char buf[256];
    snprintf(buf, sizeof buf, "\"%s\" has changes that are not saved. Save them first?",
             ui->job.name[0] ? ui->job.name : "Untitled");
    ui_label_wrap(ui, buf, ui->theme->text);
}

static void about_body(RgUi *ui)
{
    struct nk_context *c = ui->ctx;
    nk_layout_row_dynamic(c, S(ui, 26), 1);
    nk_label_colored(c, RAPIDGEN_NAME "  " RAPIDGEN_VERSION, NK_TEXT_LEFT, ui->theme->accent);
    nk_layout_row_dynamic(c, S(ui, 20), 1);
    nk_label_colored(c, RAPIDGEN_TAGLINE, NK_TEXT_LEFT, ui->theme->text);
    ui_gap(ui, 6);
    ui_label_wrap(ui, "Writes RAPID spray programs for an ABB IRB 2400 on S4, S4C and S4C+ "
                  "controllers: flat parts sprayed along a pattern painted over their drawing, "
                  "and cylinders on a rotator sprayed in bands. Every move is followed through "
                  "the robot's kinematics, and a program that breaks a rule is refused, with "
                  "the reason and the margin.", ui->theme->text_dim);
    ui_label_wrap(ui, "Not yet proven on a robot. Simulate, then step through in manual reduced "
                  "speed.", ui->theme->warn);
    ui_gap(ui, 6);
    ui_section(ui, "This build");
    ui_info_row(ui, "Version", RAPIDGEN_VERSION);
    ui_info_row(ui, "Built", __DATE__ " " __TIME__);
    SDL_version linked;
    SDL_GetVersion(&linked);
    ui_info_rowf(ui, "SDL", "%d.%d.%d at runtime, built against %d.%d.%d", linked.major,
                 linked.minor, linked.patch, SDL_MAJOR_VERSION, SDL_MINOR_VERSION, SDL_PATCHLEVEL);
    ui_info_rowf(ui, "UI scale", "%.2f  (override with RAPIDGEN_SCALE)", (double)ui->scale);
    ui_gap(ui, 6);
    ui_section(ui, "Licence");
    ui_info_row(ui, "Licence", "GPL-3.0-or-later");
    ui_info_row(ui, "Copyright", "(C) 2026 Hugh Frater");
    ui_label_wrap(ui, "Free software with ABSOLUTELY NO WARRANTY; redistributable under the GNU "
                  "General Public License v3 or later. Nuklear, in third_party/, is MIT or "
                  "public domain; SDL2 is zlib-licensed.", ui->theme->text_faint);
}

/* ------------------------------------------------------------------ */
/* Frame                                                               */
/* ------------------------------------------------------------------ */

static const char *dialog_title(const RgUi *ui)
{
    switch (ui->dialog) {
    case DLG_FILE:
        return ui->file_mode == FILE_OPEN_JOB ? "Open a job"
             : ui->file_mode == FILE_SAVE_JOB ? "Save the job" : "Import a drawing";
    case DLG_CONFIRM: return "Unsaved changes";
    case DLG_ABOUT:   return "About " RAPIDGEN_NAME;
    default:          return "";
    }
}

static struct nk_vec2 dialog_size(Dialog d)
{
    switch (d) {
    case DLG_FILE:    return nk_vec2(640, 600);
    case DLG_CONFIRM: return nk_vec2(480, 200);
    case DLG_ABOUT:   return nk_vec2(620, 560);
    default:          return nk_vec2(500, 300);
    }
}

void ui_draw_dialog(RgUi *ui, int w, int h)
{
    struct nk_context *c = ui->ctx;
    const RgTheme *t = ui->theme;

    ui->tip_text[0] = '\0';

    nk_style_push_style_item(c, &c->style.window.fixed_background, nk_style_item_color(t->scrim));
    if (nk_begin(c, "scrim", nk_rect(0, 0, (float)w, (float)h),
                 NK_WINDOW_NO_SCROLLBAR | NK_WINDOW_NO_INPUT)) {}
    nk_end(c);
    nk_style_pop_style_item(c);

    struct nk_vec2 sz = dialog_size(ui->dialog);
    float dw = S(ui, sz.x), dh = S(ui, sz.y);
    bool autosize = ui->dialog == DLG_ABOUT || ui->dialog == DLG_CONFIRM;
    if (autosize && ui->dlg_measured == ui->dialog && ui->dlg_natural_h > 0.0f)
        dh = ui->dlg_natural_h;
    if (dw > w - S(ui, 40)) dw = w - S(ui, 40);
    if (dh > h - S(ui, 40)) dh = h - S(ui, 40);
    struct nk_rect r = nk_rect((w - dw) / 2, (h - dh) / 2, dw, dh);

    nk_style_push_float(c, &c->style.window.border, 1.0f);
    nk_style_push_color(c, &c->style.window.border_color, t->border);
    nk_style_push_style_item(c, &c->style.window.fixed_background, nk_style_item_color(t->panel));

    /* Each dialog is its own window name, so one closing and another opening
     * in the same frame cannot inherit its position or scroll. */
    char name[32];
    snprintf(name, sizeof name, "dialog%d", (int)ui->dialog);
    nk_window_set_bounds(c, name, r);

    Dialog showing = ui->dialog;
    if (nk_begin_titled(c, name, dialog_title(ui), r,
                        NK_WINDOW_BORDER | NK_WINDOW_TITLE | NK_WINDOW_CLOSABLE |
                        NK_WINDOW_NO_SCROLLBAR)) {
        nk_window_set_focus(c, name);

        float spacing = c->style.window.spacing.y;
        float pad_y = c->style.window.padding.y;
        struct nk_rect region = nk_window_get_content_region(c);
        float err_h = ui->dlg_error[0]
            ? ui_wrap_height(ui, ui->dlg_error, region.w - c->style.window.padding.x * 2.0f
                                                - S(ui, 8)) + S(ui, 2)
            : 0.0f;
        float msg_h = ui->dlg_error[0] ? err_h + spacing : 0.0f;
        float below = S(ui, BTN_H) + spacing + msg_h + pad_y + S(ui, DLG_BOTTOM_MARGIN);
        float body_h = region.h - below;
        /* An autosized dialog is already as tall as its body; a floor there
         * would push its buttons out of the window. */
        float floor_h = autosize ? 1.0f : S(ui, 60);
        if (body_h < floor_h)
            body_h = floor_h;
        float chrome = nk_window_get_bounds(c).h - region.h;
        float used = 0.0f;

        nk_layout_row_dynamic(c, body_h, 1);
        if (nk_group_begin(c, "dlgbody", autosize ? NK_WINDOW_NO_SCROLLBAR : 0)) {
            struct nk_rect topb = nk_layout_widget_bounds(c);
            switch (showing) {
            case DLG_FILE:    file_body(ui);    break;
            case DLG_CONFIRM: confirm_body(ui); break;
            case DLG_ABOUT:   about_body(ui);   break;
            default: break;
            }
            nk_layout_row_dynamic(c, 0.0f, 1);
            used = nk_layout_widget_bounds(c).y - topb.y - c->style.window.spacing.y;
            nk_group_end(c);
        }
        /* From `below`, not region.h - body_h: after a clamp that difference
         * is short, and feeding it back shrinks the dialog every frame. */
        if (autosize && used > 0.0f) {
            ui->dlg_natural_h = chrome + below + used + c->style.window.group_padding.y * 2.0f;
            ui->dlg_measured = showing;
        }

        if (ui->dlg_error[0]) {
            nk_layout_row_dynamic(c, err_h, 1);
            nk_label_colored_wrap(c, ui->dlg_error, t->alarm);
        }

        if (ui->dialog == showing) {
            DlgResult res;
            switch (showing) {
            case DLG_FILE:
                res = dialog_buttons(ui, ui->file_name[0] != '\0',
                                     ui->file_mode == FILE_OPEN_JOB ? "Open"
                                     : ui->file_mode == FILE_SAVE_JOB ? "Save" : "Import", NULL);
                if (res == DLG_R_CANCEL) {
                    ui->dialog = DLG_NONE;
                    ui->pending = PENDING_NONE;
                    ui->save_then_pending = false;
                } else if (res == DLG_R_OK) {
                    file_commit(ui);
                }
                break;
            case DLG_CONFIRM:
                res = dialog_buttons(ui, true, "Save", "Discard");
                if (res == DLG_R_CANCEL) {
                    ui->dialog = DLG_NONE;
                    ui->pending = PENDING_NONE;
                } else if (res == DLG_R_ALT) {
                    ui->dialog = DLG_NONE;
                    ui->modified = false;
                    ui_continue_pending(ui);
                } else if (res == DLG_R_OK) {
                    ui->dialog = DLG_NONE;
                    if (ui->path[0]) {
                        if (ui_save_to(ui, ui->path))
                            ui_continue_pending(ui);
                    } else {
                        ui->save_then_pending = true;
                        ui_open_file_dialog(ui, FILE_SAVE_JOB);
                    }
                }
                break;
            case DLG_ABOUT:
                nk_layout_row_template_begin(c, S(ui, BTN_H));
                nk_layout_row_template_push_dynamic(c);
                nk_layout_row_template_push_static(c, S(ui, BTN_W));
                nk_layout_row_template_end(c);
                nk_skip(c);
                if (ui_primary_button(ui, "OK"))
                    ui->dialog = DLG_NONE;
                break;
            default:
                break;
            }
        }
    } else if (ui->dialog == showing) {
        ui->dialog = DLG_NONE;
        ui->pending = PENDING_NONE;
    }
    nk_end(c);

    nk_style_pop_style_item(c);
    nk_style_pop_color(c);
    nk_style_pop_float(c);

    /* The close box hides the window rather than closing it; a hidden window
     * would stay hidden the next time the same dialog opens. */
    if (ui->dialog != showing)
        nk_window_show(c, name, NK_SHOWN);
}
