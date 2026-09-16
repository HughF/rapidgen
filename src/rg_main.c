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
 * rg_main.c — the command line, and starting the editor.
 *
 *   rapidgen                        open the editor on the last job
 *   rapidgen JOB.rgj [--out DIR] [--check]
 *   rapidgen --template
 *   rapidgen --fk ROBOT J1 J2 J3 J4 J5 J6
 *   rapidgen --ik ROBOT X Y Z Q1 Q2 Q3 Q4
 *   rapidgen --robots | --controllers | --version | --help
 *
 * Exit status: 0 program written (or --check passed), 1 bad input or I/O,
 * 2 the plan was refused.
 */
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "plat.h"
#include "rg_ui.h"
#include "rg_dxf.h"
#include "rg_job.h"
#include "rg_plan.h"
#include "rg_rapid.h"
#include "rg_report.h"
#include "rg_robot.h"
#include "rg_shape.h"
#include "rg_text.h"
#include "rg_version.h"

/* A -mwindows program has no console; reattach to the one it was started
 * from, so the command-line options can print. */
static void console(void)
{
#ifdef _WIN32
    if (AttachConsole(ATTACH_PARENT_PROCESS)) {
        freopen("CONOUT$", "w", stdout);
        freopen("CONOUT$", "w", stderr);
    }
#endif
}

static void usage(FILE *f)
{
    fprintf(f,
        "%s %s - %s\n\n"
        "  rapidgen                   open the editor on the last job\n"
        "  rapidgen --gui [JOB.rgj]   open the editor\n"
        "  rapidgen JOB.rgj [--out DIR] [--check]\n"
        "        plan the job, check it through the robot's kinematics, and write\n"
        "        NAME.PRG and NAME.txt (the report) to DIR, default beside the job.\n"
        "        --check writes nothing and prints the report.\n"
        "  rapidgen --template [flat] print a commented job file to start from:\n"
        "                             a cylinder, or a flat part with strokes\n"
        "  rapidgen --fk ROBOT J1..J6 tool0 position and orientation for joint angles\n"
        "  rapidgen --ik ROBOT X Y Z Q1 Q2 Q3 Q4\n"
        "                             every joint solution for a tool0 pose\n"
        "  rapidgen --robots          robots known\n"
        "  rapidgen --controllers     controllers known\n\n"
        "Exit status: 0 written, 1 bad input, 2 refused.\n",
        RAPIDGEN_NAME, RAPIDGEN_VERSION, RAPIDGEN_TAGLINE);
}

static bool parse_doubles(char **argv, int n, double *out)
{
    for (int i = 0; i < n; i++) {
        char *end;
        out[i] = strtod(argv[i], &end);
        if (end == argv[i] || *end)
            return false;
    }
    return true;
}

static const RgRobot *robot_arg(const char *id)
{
    const RgRobot *r = rg_robot_find(id);
    if (!r)
        fprintf(stderr, "rapidgen: robot \"%s\" is not known (--robots lists them)\n", id);
    return r;
}

static int cmd_fk(int argc, char **argv)
{
    double j[6];
    if (argc != 8 || !parse_doubles(argv + 2, 6, j)) {
        fprintf(stderr, "rapidgen: --fk ROBOT J1 J2 J3 J4 J5 J6 (degrees)\n");
        return 1;
    }
    const RgRobot *r = robot_arg(argv[1]);
    if (!r)
        return 1;
    RgPose t0;
    RgVec3 wc;
    rg_robot_fk(r, j, &t0, &wc);
    RgQuat q = rg_quat_from_m3(t0.rot);
    int cf[4];
    rg_robot_confdata(r, j, cf);
    printf("tool0   [%.2f, %.2f, %.2f]  [%.6f, %.6f, %.6f, %.6f]\n",
           t0.pos.x, t0.pos.y, t0.pos.z, q.q1, q.q2, q.q3, q.q4);
    printf("confdata [%d, %d, %d, %d]\n", cf[0], cf[1], cf[2], cf[3]);
    printf("wrist centre [%.2f, %.2f, %.2f]\n", wc.x, wc.y, wc.z);
    if (!rg_robot_within_limits(r, j))
        printf("outside the joint limits\n");
    return 0;
}

static int cmd_ik(int argc, char **argv)
{
    double v[7];
    if (argc != 9 || !parse_doubles(argv + 2, 7, v)) {
        fprintf(stderr, "rapidgen: --ik ROBOT X Y Z Q1 Q2 Q3 Q4\n");
        return 1;
    }
    const RgRobot *r = robot_arg(argv[1]);
    if (!r)
        return 1;
    RgQuat q = { v[3], v[4], v[5], v[6] };
    RgPose t0 = rg_pose(rg_m3_from_quat(q), rg_v3(v[0], v[1], v[2]));
    double sol[8][6], shortfall;
    int n = rg_robot_ik_all(r, &t0, sol, &shortfall);
    if (!n) {
        printf("out of reach by %.1f mm\n", shortfall);
        return 2;
    }
    for (int i = 0; i < n; i++) {
        int cf[4];
        rg_robot_confdata(r, sol[i], cf);
        printf("[%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f]  cf [%d,%d,%d,%d]%s\n",
               sol[i][0], sol[i][1], sol[i][2], sol[i][3], sol[i][4], sol[i][5],
               cf[0], cf[1], cf[2], cf[3],
               rg_robot_within_limits(r, sol[i]) ? "" : "  outside limits");
    }
    return 0;
}

static void stamp_now(char *buf, size_t cap)
{
    PlatDate d;
    plat_date_now(&d);
    snprintf(buf, cap, "%04d-%02d-%02d %02d:%02d", d.year, d.month, d.day, d.hour, d.minute);
}

static int cmd_job(const char *job_path, const char *out_dir, bool check_only)
{
    char err[512];
    RgJob job;
    rg_job_default(&job);
    if (!rg_job_load(&job, job_path, err, sizeof err) ||
        !rg_job_validate(&job, err, sizeof err)) {
        fprintf(stderr, "rapidgen: %s\n", err);
        rg_job_free(&job);
        return 1;
    }

    char job_dir[PLAT_PATH_MAX];
    rg_copy(job_dir, sizeof job_dir, job_path);
    if (strcmp(plat_path_leaf(job_dir), job_dir) == 0)
        rg_copy(job_dir, sizeof job_dir, ".");
    else
        plat_path_parent(job_dir);

    char source[PLAT_PATH_MAX + 32];
    RgDrawing drawing = { 0 };
    RgShape shape = { 0 };
    bool have_shape = false;

    /* A flat part's drawing is only what its strokes were painted over. */
    if (job.drawing[0] && job.part == RG_PART_CYLINDER) {
        char path[PLAT_PATH_MAX];
        bool absolute = job.drawing[0] == '/' || job.drawing[0] == '\\' ||
                        (job.drawing[0] && job.drawing[1] == ':');
        if (absolute)
            rg_copy(path, sizeof path, job.drawing);
        else if (!plat_path_join(path, sizeof path, job_dir, job.drawing)) {
            fprintf(stderr, "rapidgen: the drawing's path is too long\n");
            rg_job_free(&job);
            return 1;
        }
        RgDxfOptions opt = { job.chord_tolerance, job.layer, false };
        if (!rg_dxf_load(path, &opt, &drawing, err, sizeof err)) {
            fprintf(stderr, "rapidgen: %s\n", err);
            rg_job_free(&job);
            return 1;
        }
        rg_drawing_scale(&drawing, job.drawing_scale);
        if (!rg_shape_build(&drawing, job.join_tolerance, &shape, err, sizeof err)) {
            fprintf(stderr, "rapidgen: %s: %s\n", job.drawing, err);
            rg_drawing_free(&drawing);
            rg_job_free(&job);
            return 1;
        }
        have_shape = true;
        snprintf(source, sizeof source, "%s", plat_path_leaf(job.drawing));
    } else if (job.drawing[0]) {
        snprintf(source, sizeof source, "%.200s, painted over %.200s", plat_path_leaf(job_path),
                 plat_path_leaf(job.drawing));
        /* A flat part's drawing is what its strokes were painted over. Its
         * outline lets the plan check that the gun comes on, turns round and
         * leaves clear of the part. Best effort: a drawing that will not load
         * does not stop the program, and the report says what was not checked. */
        char path[PLAT_PATH_MAX];
        bool absolute = job.drawing[0] == '/' || job.drawing[0] == '\\' ||
                        (job.drawing[0] && job.drawing[1] == ':');
        bool have_path = absolute ? (rg_copy(path, sizeof path, job.drawing), true)
                                  : plat_path_join(path, sizeof path, job_dir, job.drawing);
        /* Read leniently, as the editor does: this outline is only checked
         * against, and a drawing with a bad entity in it still has one. */
        RgDxfOptions opt = { job.chord_tolerance, job.layer, true };
        int skipped = 0;
        if (have_path && rg_dxf_load(path, &opt, &drawing, err, sizeof err)) {
            rg_drawing_scale(&drawing, job.drawing_scale);
            have_shape = rg_shape_build_lenient(&drawing, job.join_tolerance, &shape, &skipped);
            if (!have_shape)
                rg_drawing_free(&drawing);
        }
    } else {
        snprintf(source, sizeof source, "%s", plat_path_leaf(job_path));
    }

    RgPlan plan;
    bool ok = rg_plan_build(&job, have_shape ? &shape : NULL, &plan);
    if (have_shape) {
        rg_shape_free(&shape);
        rg_drawing_free(&drawing);
    }

    char program[64];
    rg_rapid_filename(&job, program, sizeof program);

    RgBuf report, text;
    rg_buf_init(&report);
    rg_buf_init(&text);
    rg_report_write(&job, &plan, program, source, &report);

    int status = ok ? 0 : 2;
    if (check_only) {
        fputs(report.s ? report.s : "", stdout);
    } else {
        const char *dir = out_dir ? out_dir : job_dir;
        char path[PLAT_PATH_MAX], stamp[32];
        stamp_now(stamp, sizeof stamp);

        if (ok && !rg_rapid_write(&job, &plan, source, stamp, &text, err, sizeof err)) {
            fprintf(stderr, "rapidgen: %s\n", err);
            status = 1;
        }
        if (status == 0) {
            if (!plat_path_join(path, sizeof path, dir, program) ||
                !rg_write_file(path, text.s, text.len, err, sizeof err)) {
                fprintf(stderr, "rapidgen: %s\n", err);
                status = 1;
            } else {
                printf("wrote %s\n", path);
            }
        }
        char leaf[64];
        snprintf(leaf, sizeof leaf, "%s.txt", job.name);
        if (!plat_path_join(path, sizeof path, dir, leaf) ||
            !rg_write_file(path, report.s, report.len, err, sizeof err)) {
            fprintf(stderr, "rapidgen: %s\n", err);
            if (status == 0)
                status = 1;
        } else {
            printf("wrote %s\n", path);
        }
        if (!ok) {
            fprintf(stderr, "rapidgen: refused, no program written:\n");
            for (int i = 0; i < plan.nissues; i++)
                if (plan.issues[i].sev == RG_REFUSE)
                    fprintf(stderr, "  - %s\n", plan.issues[i].text);
        }
    }

    rg_buf_free(&report);
    rg_buf_free(&text);
    rg_plan_free(&plan);
    rg_job_free(&job);
    return status;
}

#define BASE_W 1500
#define BASE_H 940

static int run_gui(const char *job)
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        console();
        fprintf(stderr, "rapidgen: SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "best");

    SDL_Window *win = SDL_CreateWindow(RAPIDGEN_NAME " " RAPIDGEN_VERSION,
                                       SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                       BASE_W, BASE_H,
                                       SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE |
                                       SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Renderer *ren = win ? SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED |
                                                 SDL_RENDERER_PRESENTVSYNC) : NULL;
    if (win && !ren)
        ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
    if (!win || !ren) {
        console();
        fprintf(stderr, "rapidgen: cannot open a window: %s\n", SDL_GetError());
        if (win)
            SDL_DestroyWindow(win);
        SDL_Quit();
        return 1;
    }
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);

    RgUi *ui = rg_ui_create(win, ren, job);
    if (!ui) {
        console();
        fprintf(stderr, "rapidgen: cannot start the editor\n");
        SDL_DestroyRenderer(ren);
        SDL_DestroyWindow(win);
        SDL_Quit();
        return 1;
    }
    rg_ui_fit_window(ui, BASE_W, BASE_H);
    SDL_ShowWindow(win);

    while (!rg_ui_quit_requested(ui)) {
        SDL_Event e;
        rg_ui_input_begin(ui);
        /* Nothing moves on its own, so sleep until the operator does
         * something — unless a check is due in a moment. */
        bool got = rg_ui_busy(ui) ? SDL_PollEvent(&e) != 0
                                  : SDL_WaitEventTimeout(&e, 200) != 0;
        if (got) {
            rg_ui_handle_event(ui, &e);
            while (SDL_PollEvent(&e))
                rg_ui_handle_event(ui, &e);
        }
        rg_ui_input_end(ui);

        int w = 0, h = 0;
        rg_ui_output_size(ui, &w, &h);
        rg_ui_frame(ui, w, h);
        rg_ui_present(ui);
    }

    rg_ui_destroy(ui);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2)
        return run_gui(NULL);
    if (strcmp(argv[1], "--gui") == 0)
        return run_gui(argc > 2 ? argv[2] : NULL);

    console();
    const char *a = argv[1];
    if (strcmp(a, "--help") == 0 || strcmp(a, "-h") == 0) {
        usage(stdout);
        return 0;
    }
    if (strcmp(a, "--version") == 0) {
        printf("%s %s\n", RAPIDGEN_NAME, RAPIDGEN_VERSION);
        return 0;
    }
    if (strcmp(a, "--template") == 0) {
        bool flat = argc > 2 && strcmp(argv[2], "flat") == 0;
        fputs(flat ? rg_job_template_flat() : rg_job_template(), stdout);
        return 0;
    }
    if (strcmp(a, "--robots") == 0) {
        for (int i = 0; i < rg_robot_count(); i++)
            printf("%-12s %s\n", rg_robot_at(i)->id, rg_robot_at(i)->name);
        return 0;
    }
    if (strcmp(a, "--controllers") == 0) {
        for (int i = 0; i < rg_dialect_count(); i++)
            printf("%-12s ABB %s\n", rg_dialect_at(i)->id, rg_dialect_at(i)->name);
        return 0;
    }
    if (strcmp(a, "--fk") == 0)
        return cmd_fk(argc - 1, argv + 1);
    if (strcmp(a, "--ik") == 0)
        return cmd_ik(argc - 1, argv + 1);
    if (a[0] == '-') {
        fprintf(stderr, "rapidgen: unknown option %s\n", a);
        usage(stderr);
        return 1;
    }

    const char *out_dir = NULL;
    bool check = false;
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out_dir = argv[++i];
        } else if (strcmp(argv[i], "--check") == 0) {
            check = true;
        } else {
            fprintf(stderr, "rapidgen: unknown option %s\n", argv[i]);
            return 1;
        }
    }
    return cmd_job(a, out_dir, check);
}
