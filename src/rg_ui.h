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
 * rg_ui.h — the editor: a window onto one job.
 *
 * Draw the pattern over the part's drawing, set the job up, and see the
 * checks and the program the job makes, all from the same job file the
 * command line reads.
 */
#ifndef RG_UI_H
#define RG_UI_H

#include <SDL2/SDL.h>
#include <stdbool.h>

typedef struct RgUi RgUi;

/* NULL on failure. `job` is opened at start, or NULL for the last job. */
RgUi *rg_ui_create(SDL_Window *win, SDL_Renderer *ren, const char *job);
void  rg_ui_destroy(RgUi *ui);

void  rg_ui_fit_window(RgUi *ui, int base_w, int base_h);

void  rg_ui_input_begin(RgUi *ui);
void  rg_ui_input_end(RgUi *ui);
bool  rg_ui_handle_event(RgUi *ui, SDL_Event *e);

/* The drawable's size in pixels, which the frame is laid out in. */
void  rg_ui_output_size(const RgUi *ui, int *w, int *h);
void  rg_ui_frame(RgUi *ui, int w, int h);
void  rg_ui_present(RgUi *ui);

bool  rg_ui_quit_requested(const RgUi *ui);

/* True while something is waiting to happen on a timer (a check about to
 * run), so the loop should not sleep long waiting for input. */
bool  rg_ui_busy(const RgUi *ui);

#endif /* RG_UI_H */
