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
#include "rg_theme.h"

#define C(r,g,b)     { (r), (g), (b), 255 }
#define CA(r,g,b,a)  { (r), (g), (b), (a) }

/*
 * Dark is the default: it keeps the drawing and the pattern the brightest
 * things on screen. Light suits a bright workshop.
 */
const RgTheme RG_THEME_DARK = {
    .is_light        = false,

    .bg              = C( 18,  20,  24),
    .panel           = C( 26,  29,  35),
    .panel_alt       = C( 31,  35,  42),
    .rail            = C( 14,  16,  19),
    .border          = C( 46,  51,  60),
    .divider         = C( 38,  42,  50),

    .text            = C(226, 232, 240),
    .text_dim        = C(148, 160, 178),
    .text_faint      = C( 94, 104, 120),
    .text_on_accent  = C(255, 255, 255),

    .accent          = C( 56, 132, 222),
    .accent_hover    = C( 76, 152, 242),
    .accent_dim      = C( 34,  74, 124),

    .ok              = C( 64, 190, 120),
    .warn            = C(226, 170,  60),
    .alarm           = C(226,  86,  76),

    .plot_bg         = C( 22,  25,  30),
    .plot_grid       = C( 40,  45,  54),
    .plot_axis       = C( 82,  92, 108),
    .trace_a         = C(255, 196,  84),   /* strokes          */
    .trace_b         = C( 92, 178, 255),   /* drawing          */
    .trace_c         = C(122, 214, 154),   /* snap             */
    .trace_d         = C(198, 142, 232),   /* measure          */
    .scrim           = CA(  8,   9,  12, 190),
};

const RgTheme RG_THEME_LIGHT = {
    .is_light        = true,

    .bg              = C(242, 244, 248),
    .panel           = C(255, 255, 255),
    .panel_alt       = C(246, 248, 251),
    .rail            = C(233, 237, 243),
    .border          = C(206, 212, 222),
    .divider         = C(224, 229, 237),

    .text            = C( 26,  32,  44),
    .text_dim        = C( 92, 102, 118),
    .text_faint      = C(140, 150, 166),
    .text_on_accent  = C(255, 255, 255),

    .accent          = C( 26, 108, 204),
    .accent_hover    = C( 40, 126, 226),
    .accent_dim      = C(196, 218, 244),

    .ok              = C( 24, 142,  78),
    .warn            = C(178, 118,  10),
    .alarm           = C(196,  48,  40),

    .plot_bg         = C(252, 253, 255),
    .plot_grid       = C(226, 231, 239),
    .plot_axis       = C(140, 150, 166),
    .trace_a         = C(184, 116,   8),   /* strokes          */
    .trace_b         = C( 24,  96, 200),   /* drawing          */
    .trace_c         = C( 26, 132,  76),   /* snap             */
    .trace_d         = C(122,  60, 176),   /* measure          */
    .scrim           = CA( 30,  36,  46, 120),
};

void rg_theme_apply(struct nk_context *ctx, const RgTheme *t, float scale)
{
    struct nk_color tbl[NK_COLOR_COUNT];

    tbl[NK_COLOR_TEXT]                    = t->text;
    tbl[NK_COLOR_WINDOW]                  = t->panel;
    tbl[NK_COLOR_HEADER]                  = t->panel_alt;
    tbl[NK_COLOR_BORDER]                  = t->border;
    tbl[NK_COLOR_BUTTON]                  = t->panel_alt;
    tbl[NK_COLOR_BUTTON_HOVER]            = t->accent_dim;
    tbl[NK_COLOR_BUTTON_ACTIVE]           = t->accent;
    tbl[NK_COLOR_TOGGLE]                  = t->panel_alt;
    tbl[NK_COLOR_TOGGLE_HOVER]            = t->border;
    tbl[NK_COLOR_TOGGLE_CURSOR]           = t->accent;
    tbl[NK_COLOR_SELECT]                  = t->panel;
    tbl[NK_COLOR_SELECT_ACTIVE]           = t->accent;
    tbl[NK_COLOR_SLIDER]                  = t->panel_alt;
    tbl[NK_COLOR_SLIDER_CURSOR]           = t->accent;
    tbl[NK_COLOR_SLIDER_CURSOR_HOVER]     = t->accent_hover;
    tbl[NK_COLOR_SLIDER_CURSOR_ACTIVE]    = t->accent_hover;
    tbl[NK_COLOR_PROPERTY]                = t->panel_alt;
    tbl[NK_COLOR_EDIT]                    = t->is_light ? t->panel_alt : t->bg;
    tbl[NK_COLOR_EDIT_CURSOR]             = t->text;
    tbl[NK_COLOR_COMBO]                   = t->panel_alt;
    tbl[NK_COLOR_CHART]                   = t->plot_bg;
    tbl[NK_COLOR_CHART_COLOR]             = t->accent;
    tbl[NK_COLOR_CHART_COLOR_HIGHLIGHT]   = t->accent_hover;
    tbl[NK_COLOR_SCROLLBAR]               = t->is_light ? t->panel_alt : t->bg;
    tbl[NK_COLOR_SCROLLBAR_CURSOR]        = t->border;
    tbl[NK_COLOR_SCROLLBAR_CURSOR_HOVER]  = t->text_faint;
    tbl[NK_COLOR_SCROLLBAR_CURSOR_ACTIVE] = t->accent;
    tbl[NK_COLOR_TAB_HEADER]              = t->panel_alt;

    nk_style_from_table(ctx, tbl);

    struct nk_style *s = &ctx->style;
    float r = 3.0f * scale;

    /* Windows: no title bars anywhere — every region is positioned by the
     * shell, so window chrome would only be noise. */
    s->window.padding          = nk_vec2(14 * scale, 12 * scale);
    s->window.group_padding    = nk_vec2(10 * scale, 8 * scale);
    s->window.spacing          = nk_vec2(8 * scale, 7 * scale);
    s->window.border           = 0.0f;
    s->window.rounding         = 0.0f;
    s->window.group_border     = 1.0f;
    s->window.group_border_color = t->divider;
    s->window.combo_border     = 1.0f;
    s->window.combo_border_color = t->border;
    s->window.popup_border     = 1.0f;
    s->window.popup_border_color = t->border;
    s->window.background       = t->panel;
    s->window.fixed_background = nk_style_item_color(t->panel);

    /* Dialog title bar. */
    s->window.header.normal    = nk_style_item_color(t->panel_alt);
    s->window.header.hover     = nk_style_item_color(t->panel_alt);
    s->window.header.active    = nk_style_item_color(t->panel_alt);
    s->window.header.label_normal = t->text;
    s->window.header.label_hover  = t->text;
    s->window.header.label_active = t->text;
    s->window.header.padding   = nk_vec2(12 * scale, 8 * scale);
    s->window.header.label_padding = nk_vec2(2 * scale, 2 * scale);
    s->window.header.spacing   = nk_vec2(2 * scale, 2 * scale);
    s->window.header.close_button.normal =
        nk_style_item_color(t->panel_alt);
    s->window.header.close_button.hover  = nk_style_item_color(t->alarm);
    s->window.header.close_button.active = nk_style_item_color(t->alarm);
    s->window.header.close_button.text_normal = t->text_dim;
    s->window.header.close_button.text_hover  = t->text_on_accent;
    s->window.header.close_button.text_active = t->text_on_accent;

    s->button.rounding         = r;
    s->button.border           = 1.0f;
    s->button.border_color     = t->border;
    s->button.text_normal      = t->text;
    s->button.text_hover       = t->text;
    s->button.text_active      = t->text_on_accent;
    s->button.padding          = nk_vec2(10 * scale, 5 * scale);

    s->checkbox.padding        = nk_vec2(4 * scale, 4 * scale);
    s->checkbox.text_normal    = t->text;
    s->checkbox.text_hover     = t->text;
    s->checkbox.text_active    = t->text;
    s->checkbox.cursor_normal  = nk_style_item_color(t->accent);
    s->checkbox.cursor_hover   = nk_style_item_color(t->accent_hover);
    s->checkbox.border         = 1.0f;
    s->checkbox.border_color   = t->border;

    s->option.cursor_normal    = nk_style_item_color(t->accent);
    s->option.cursor_hover     = nk_style_item_color(t->accent_hover);
    s->option.text_normal      = t->text;
    s->option.text_hover       = t->text;
    s->option.text_active      = t->text;
    s->option.border           = 1.0f;
    s->option.border_color     = t->border;

    s->property.rounding       = r;
    s->property.border         = 1.0f;
    s->property.border_color   = t->border;
    s->property.label_normal   = t->text_dim;
    s->property.label_hover    = t->text;
    s->property.label_active   = t->text;
    s->property.padding        = nk_vec2(6 * scale, 4 * scale);

    s->edit.rounding           = r;
    s->edit.border             = 1.0f;
    s->edit.border_color       = t->border;
    s->edit.text_normal        = t->text;
    s->edit.text_hover         = t->text;
    s->edit.text_active        = t->text;
    s->edit.selected_normal    = t->accent;
    s->edit.selected_text_normal = t->text_on_accent;
    s->edit.cursor_normal      = t->text;
    s->edit.padding            = nk_vec2(8 * scale, 4 * scale);

    s->combo.rounding          = r;
    s->combo.border            = 1.0f;
    s->combo.border_color      = t->border;
    s->combo.label_normal      = t->text;
    s->combo.label_hover       = t->text;
    s->combo.label_active      = t->text;
    s->combo.content_padding   = nk_vec2(8 * scale, 5 * scale);
    s->combo.button_padding    = nk_vec2(2 * scale, 2 * scale);

    s->contextual_button.rounding = r;
    s->selectable.rounding     = r;
    s->selectable.text_normal  = t->text_dim;
    s->selectable.text_hover   = t->text;
    s->selectable.text_pressed = t->text_on_accent;
    s->selectable.text_normal_active  = t->text_on_accent;
    s->selectable.text_hover_active   = t->text_on_accent;
    s->selectable.text_pressed_active = t->text_on_accent;
    s->selectable.normal       = nk_style_item_color(t->panel);
    s->selectable.hover        = nk_style_item_color(t->panel_alt);
    s->selectable.pressed      = nk_style_item_color(t->accent);
    s->selectable.normal_active  = nk_style_item_color(t->accent);
    s->selectable.hover_active   = nk_style_item_color(t->accent_hover);
    s->selectable.pressed_active = nk_style_item_color(t->accent);
    s->selectable.padding      = nk_vec2(8 * scale, 5 * scale);

    s->scrollv.rounding        = r;
    s->scrollv.border          = 0.0f;
    s->scrollv.rounding_cursor = r;

    s->progress.rounding       = r;
    s->progress.border         = 1.0f;
    s->progress.border_color   = t->border;
    s->progress.normal         = nk_style_item_color(t->panel_alt);
    s->progress.cursor_normal  = nk_style_item_color(t->accent);
    s->progress.cursor_hover   = nk_style_item_color(t->accent_hover);
    s->progress.cursor_active  = nk_style_item_color(t->accent);

    s->tab.rounding            = r;

    /* Nuklear shows a disabled widget by scaling its colours. Halving them
     * suits the dark theme, but turns a light theme's disabled button into a
     * dark block that reads as pressed. */
    float dis = t->is_light ? 0.88f : NK_WIDGET_DISABLED_FACTOR;
    s->button.disabled_factor             = dis;
    s->contextual_button.disabled_factor  = dis;
    s->menu_button.disabled_factor        = dis;
    s->selectable.disabled_factor         = dis;
    s->checkbox.disabled_factor           = dis;
    s->option.disabled_factor             = dis;
    s->property.disabled_factor           = dis;
    s->edit.disabled_factor               = dis;
    s->combo.disabled_factor              = dis;
}
