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
#include "rg_dxf.h"
#include "rg_text.h"
#include "rg_vec.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- group-code reader ---------------------------------------------- */

typedef struct {
    const char *p, *end;
    int  line;           /* line of the value just read */
    int  code;
    char val[256];
    bool held;           /* hand the current pair out again */
} Reader;

static bool read_line(Reader *r, char *buf, size_t cap)
{
    if (r->p >= r->end)
        return false;
    const char *s = r->p;
    while (r->p < r->end && *r->p != '\n')
        r->p++;
    const char *e = r->p;
    if (r->p < r->end)
        r->p++;
    r->line++;
    while (s < e && isspace((unsigned char)*s))
        s++;
    while (e > s && isspace((unsigned char)e[-1]))
        e--;
    size_t n = (size_t)(e - s);
    if (n >= cap)
        n = cap - 1;
    memcpy(buf, s, n);
    buf[n] = '\0';
    return true;
}

/* 1 a pair, 0 the end of the text, -1 malformed. */
static int next_pair(Reader *r)
{
    if (r->held) {
        r->held = false;
        return 1;
    }
    char codebuf[32];
    if (!read_line(r, codebuf, sizeof codebuf))
        return 0;
    char *endp;
    long c = strtol(codebuf, &endp, 10);
    if (endp == codebuf || *endp)
        return -1;
    if (!read_line(r, r->val, sizeof r->val))
        return -1;
    r->code = (int)c;
    return 1;
}

static bool is(const Reader *r, int code, const char *val)
{
    return r->code == code && strcmp(r->val, val) == 0;
}

/* ---- entities ------------------------------------------------------- */

typedef struct { double x, y, bulge; } Vtx;

typedef struct {
    char   type[32];
    char   layer[64];
    int    line;
    double x10, y10, r40, a50, a51, b42, ez;
    double x11, y11;
    int    f70;
    Vtx   *v;            /* LWPOLYLINE vertices in order */
    int    nv, vcap;
    bool   bad;          /* a value that is not a number: nothing here is usable */
    char   bad_val[32];  /* ...as the drawing spelled it                        */
    int    bad_line;
} Ent;

typedef struct {
    Reader            rd;
    const RgDxfOptions *opt;
    RgDrawing         *out;
    Ent               *cur;      /* the entity being read, for marking it bad */
    char              *err;
    size_t             errcap;
} Ctx;

static bool fail(Ctx *c, const char *fmt, ...) RG_PRINTF(2, 3);

static bool fail(Ctx *c, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(c->err, c->errcap, fmt, ap);
    va_end(ap);
    return false;
}

/*
 * Not-a-number, as CAD writes it. The C library spells it "nan" and "inf",
 * which strtod reads; programs built against the Microsoft C runtime write
 * "1.#QNAN", "-1.#IND" and "1.#INF", which it does not. QCAD emits these for
 * an entity whose geometry has gone bad — an arc with no centre, say.
 */
static bool non_finite(const char *s, double parsed, const char *end)
{
    if (end != s && !*end)
        return !isfinite(parsed);
    return strstr(s, "#QNAN") || strstr(s, "#IND") || strstr(s, "#INF") ||
           strstr(s, "#SNAN") || strstr(s, "#NAN");
}

/*
 * A value that is not a number condemns its entity, not the whole drawing:
 * one dead arc among hundreds of good lines should not cost the operator the
 * import. The entity is marked and left out; read_entities decides whether
 * that is merely counted (the editor) or refused (generating a program).
 */
static bool number(Ctx *c, double *out)
{
    char *endp;
    double v = strtod(c->rd.val, &endp);
    if (endp != c->rd.val && !*endp && isfinite(v)) {
        *out = v;
        return true;
    }
    if (c->cur && non_finite(c->rd.val, v, endp)) {
        if (!c->cur->bad) {
            c->cur->bad = true;
            c->cur->bad_line = c->rd.line;
            rg_copy(c->cur->bad_val, sizeof c->cur->bad_val, c->rd.val);
        }
        *out = 0.0;
        return true;
    }
    return fail(c, "line %d: expected a number, found \"%s\"", c->rd.line, c->rd.val);
}

static bool push_vertex(Ctx *c, Ent *e, double x)
{
    if (e->nv == e->vcap) {
        int cap = e->vcap ? e->vcap * 2 : 16;
        Vtx *v = realloc(e->v, (size_t)cap * sizeof *v);
        if (!v)
            return fail(c, "out of memory reading the drawing");
        e->v = v;
        e->vcap = cap;
    }
    e->v[e->nv].x = x;
    e->v[e->nv].y = 0.0;
    e->v[e->nv].bulge = 0.0;
    e->nv++;
    return true;
}

/* Precondition: the (0, TYPE) pair has just been read. Consumes the entity's
 * groups and leaves the next (0, ...) pair held. */
static bool read_entity(Ctx *c, Ent *e)
{
    Vtx *keep = e->v;
    int keepcap = e->vcap;
    memset(e, 0, sizeof *e);
    e->v = keep;
    e->vcap = keepcap;
    e->ez = 1.0;
    e->line = c->rd.line;
    rg_copy(e->type, sizeof e->type, c->rd.val);
    rg_copy(e->layer, sizeof e->layer, "0");
    bool lw = strcmp(e->type, "LWPOLYLINE") == 0;
    c->cur = e;

    for (;;) {
        int k = next_pair(&c->rd);
        if (k == 0) {
            c->cur = NULL;
            return fail(c, "the drawing ends in the middle of a %s", e->type);
        }
        if (k < 0) {
            c->cur = NULL;
            return fail(c, "line %d: malformed group code", c->rd.line);
        }
        if (c->rd.code == 0) {
            c->rd.held = true;
            c->cur = NULL;
            return true;
        }
        double v = 0.0;
        switch (c->rd.code) {
        case 8:
            rg_copy(e->layer, sizeof e->layer, c->rd.val);
            break;
        case 10:
            if (!number(c, &v)) return false;
            if (lw) {
                if (!push_vertex(c, e, v)) return false;
            } else {
                e->x10 = v;
            }
            break;
        case 20:
            if (!number(c, &v)) return false;
            if (lw && e->nv) e->v[e->nv - 1].y = v; else e->y10 = v;
            break;
        case 11: if (!number(c, &e->x11)) return false; break;
        case 21: if (!number(c, &e->y11)) return false; break;
        case 40: if (!number(c, &e->r40)) return false; break;
        case 42:
            if (!number(c, &v)) return false;
            if (lw && e->nv) e->v[e->nv - 1].bulge = v; else e->b42 = v;
            break;
        case 50: if (!number(c, &e->a50)) return false; break;
        case 51: if (!number(c, &e->a51)) return false; break;
        case 70:
            if (!number(c, &v)) return false;
            e->f70 = (int)v;
            break;
        case 230: if (!number(c, &e->ez)) return false; break;
        default:
            break;
        }
    }
}

static bool on_layer(const Ctx *c, const char *layer)
{
    return !c->opt->layer || !c->opt->layer[0] || rg_streqi(c->opt->layer, layer);
}

/* ---- paths ---------------------------------------------------------- */

static RgPath *new_path(Ctx *c, const Ent *e, bool closed)
{
    RgDrawing *d = c->out;
    if (d->n == d->cap) {
        int cap = d->cap ? d->cap * 2 : 16;
        RgPath *p = realloc(d->paths, (size_t)cap * sizeof *p);
        if (!p) {
            fail(c, "out of memory reading the drawing");
            return NULL;
        }
        d->paths = p;
        d->cap = cap;
    }
    RgPath *p = &d->paths[d->n++];
    memset(p, 0, sizeof *p);
    p->closed = closed;
    rg_copy(p->layer, sizeof p->layer, e->layer);
    rg_copy(p->kind, sizeof p->kind, e->type);
    return p;
}

static bool add_pt(Ctx *c, RgPath *p, double x, double y)
{
    if (p->n && fabs(p->pts[p->n - 1].x - x) < 1e-9 && fabs(p->pts[p->n - 1].y - y) < 1e-9)
        return true;
    if (p->n == p->cap) {
        int cap = p->cap ? p->cap * 2 : 16;
        RgPt *q = realloc(p->pts, (size_t)cap * sizeof *q);
        if (!q)
            return fail(c, "out of memory reading the drawing");
        p->pts = q;
        p->cap = cap;
    }
    p->pts[p->n].x = x;
    p->pts[p->n].y = y;
    p->n++;
    return true;
}

/* Chords whose deviation from the arc, r·(1 − cos(Δ/2)), stays within tol. */
static int arc_segments(double r, double sweep, double tol)
{
    double step = tol < r ? 2.0 * acos(1.0 - tol / r) : RG_PI / 2.0;
    if (step > RG_PI / 2.0)
        step = RG_PI / 2.0;
    double n = ceil(fabs(sweep) / step);
    if (n < 1.0)
        n = 1.0;
    if (n > 100000.0)
        n = 100000.0;
    return (int)n;
}

/* Points after the start of an arc, up to and including its end. */
static bool add_arc(Ctx *c, RgPath *p, double cx, double cy, double r,
                    double a0, double sweep)
{
    int n = arc_segments(r, sweep, c->opt->chord_tol_mm);
    for (int i = 1; i <= n; i++) {
        double a = a0 + sweep * i / n;
        if (!add_pt(c, p, cx + r * cos(a), cy + r * sin(a)))
            return false;
    }
    return true;
}

/* A polyline segment from a to b; bulge = tan(included angle / 4), positive
 * counter-clockwise. Appends the points after a, ending at b. */
static bool add_bulge(Ctx *c, RgPath *p, Vtx a, Vtx b)
{
    double dx = b.x - a.x, dy = b.y - a.y;
    double chord = hypot(dx, dy);
    if (fabs(a.bulge) < 1e-12 || chord < 1e-12)
        return add_pt(c, p, b.x, b.y);
    double theta = 4.0 * atan(a.bulge);
    double r = chord / (2.0 * sin(fabs(theta) / 2.0));
    /* Centre on the left of the chord for a counter-clockwise arc. */
    double d = chord / (2.0 * tan(theta / 2.0));
    double mx = 0.5 * (a.x + b.x), my = 0.5 * (a.y + b.y);
    double cx = mx - dy / chord * d, cy = my + dx / chord * d;
    double a0 = atan2(a.y - cy, a.x - cx);
    return add_arc(c, p, cx, cy, r, a0, theta);
}

/* An entity drawn with its extrusion direction pointing down (a mirrored
 * view in CAD) has its OCS X reversed. Anything tilted out of the plane
 * cannot be a 2D outline at all. */
static bool plane_ok(Ctx *c, const Ent *e, bool *mirror)
{
    if (fabs(fabs(e->ez) - 1.0) > 1e-6)
        return fail(c, "line %d: the %s on layer %s is not drawn in the XY plane",
                    e->line, e->type, e->layer);
    *mirror = e->ez < 0.0;
    return true;
}

static bool finish_polyline(Ctx *c, const Ent *e, Vtx *v, int nv, bool closed)
{
    if (nv < 2)
        return true;
    RgPath *p = new_path(c, e, closed);
    if (!p || !add_pt(c, p, v[0].x, v[0].y))
        return false;
    for (int i = 0; i + 1 < nv; i++)
        if (!add_bulge(c, p, v[i], v[i + 1]))
            return false;
    if (closed) {
        if (!add_bulge(c, p, v[nv - 1], v[0]))
            return false;
        if (p->n > 1 && fabs(p->pts[p->n - 1].x - p->pts[0].x) < 1e-9 &&
            fabs(p->pts[p->n - 1].y - p->pts[0].y) < 1e-9)
            p->n--;
    }
    return true;
}

static void mirror_vertices(Vtx *v, int nv)
{
    for (int i = 0; i < nv; i++) {
        v[i].x = -v[i].x;
        v[i].bulge = -v[i].bulge;
    }
}

static bool do_lwpolyline(Ctx *c, Ent *e)
{
    bool mirror = false;
    if (!plane_ok(c, e, &mirror))
        return false;
    if (mirror)
        mirror_vertices(e->v, e->nv);
    return finish_polyline(c, e, e->v, e->nv, (e->f70 & 1) != 0);
}

/* R12-style POLYLINE: a header, VERTEX entities, SEQEND. */
static bool do_polyline(Ctx *c, Ent *head)
{
    Ent hdr = *head;
    hdr.v = NULL;
    hdr.vcap = 0;
    bool keep = on_layer(c, hdr.layer);

    if (keep && (hdr.f70 & (8 | 16 | 32 | 64)))
        return fail(c, "line %d: the POLYLINE on layer %s is a 3D polyline or mesh, "
                       "not a 2D outline", hdr.line, hdr.layer);

    Vtx *v = NULL;
    int nv = 0, vcap = 0;
    Ent ve;
    memset(&ve, 0, sizeof ve);
    bool ok = true;

    for (;;) {
        int k = next_pair(&c->rd);
        if (k <= 0 || c->rd.code != 0) {
            ok = fail(c, "line %d: POLYLINE without SEQEND", c->rd.line);
            break;
        }
        bool vertex = strcmp(c->rd.val, "VERTEX") == 0;
        bool seqend = strcmp(c->rd.val, "SEQEND") == 0;
        if (!vertex && !seqend) {
            ok = fail(c, "line %d: POLYLINE without SEQEND", c->rd.line);
            break;
        }
        if (!read_entity(c, &ve)) {
            ok = false;
            break;
        }
        if (ve.bad && !hdr.bad) {   /* one dead vertex spoils the polyline */
            hdr.bad = true;
            hdr.bad_line = ve.bad_line;
            rg_copy(hdr.bad_val, sizeof hdr.bad_val, ve.bad_val);
        }
        if (seqend)
            break;
        if (ve.f70 & 16)            /* spline frame control point */
            continue;
        if (nv == vcap) {
            vcap = vcap ? vcap * 2 : 16;
            Vtx *nvv = realloc(v, (size_t)vcap * sizeof *nvv);
            if (!nvv) {
                ok = fail(c, "out of memory reading the drawing");
                break;
            }
            v = nvv;
        }
        v[nv].x = ve.x10;
        v[nv].y = ve.y10;
        v[nv].bulge = ve.b42;
        nv++;
    }
    free(ve.v);

    if (ok && keep && hdr.bad) {
        *head = hdr;                /* let read_entities report or count it */
        head->v = NULL;
        head->vcap = 0;
    } else if (ok && keep) {
        bool mirror = false;
        ok = plane_ok(c, &hdr, &mirror);
        if (ok && mirror)
            mirror_vertices(v, nv);
        if (ok)
            ok = finish_polyline(c, &hdr, v, nv, (hdr.f70 & 1) != 0);
    }
    free(v);
    return ok;
}

static bool do_line(Ctx *c, const Ent *e)
{
    RgPath *p = new_path(c, e, false);
    return p && add_pt(c, p, e->x10, e->y10) && add_pt(c, p, e->x11, e->y11);
}

static bool do_arc(Ctx *c, const Ent *e, bool circle)
{
    bool mirror = false;
    if (!plane_ok(c, e, &mirror))
        return false;
    if (e->r40 <= 0.0)
        return fail(c, "line %d: %s with no radius", e->line, e->type);
    double cx = mirror ? -e->x10 : e->x10, cy = e->y10;
    double a0, sweep;
    if (circle) {
        a0 = 0.0;
        sweep = 2.0 * RG_PI;
    } else {
        double s = e->a50, t = e->a51;
        if (mirror) {             /* reflect, then swap to stay counter-clockwise */
            double s2 = 180.0 - t;
            t = 180.0 - s;
            s = s2;
        }
        sweep = fmod(t - s, 360.0);
        if (sweep <= 0.0)
            sweep += 360.0;
        a0 = s * RG_DEG;
        sweep *= RG_DEG;
    }
    RgPath *p = new_path(c, e, circle);
    if (!p || !add_pt(c, p, cx + e->r40 * cos(a0), cy + e->r40 * sin(a0)))
        return false;
    if (!add_arc(c, p, cx, cy, e->r40, a0, sweep))
        return false;
    if (circle && p->n > 1)
        p->n--;                   /* the last point repeats the first */
    return true;
}

/* Annotation: carries no outline, safe to pass over. */
static bool is_annotation(const char *t)
{
    static const char *const names[] = {
        "TEXT", "MTEXT", "DIMENSION", "POINT", "ATTDEF", "ATTRIB", "LEADER",
        "MLEADER", "MULTILEADER", "TOLERANCE", "VIEWPORT", "IMAGE", "WIPEOUT",
    };
    for (size_t i = 0; i < sizeof names / sizeof names[0]; i++)
        if (strcmp(t, names[i]) == 0)
            return true;
    return false;
}

static const char *advice_for(const char *t)
{
    if (strcmp(t, "SPLINE") == 0 || strcmp(t, "ELLIPSE") == 0)
        return "convert it to a polyline of lines and arcs in CAD";
    if (strcmp(t, "INSERT") == 0)
        return "explode the block in CAD";
    if (strcmp(t, "HATCH") == 0)
        return "delete the hatch: its boundary would count as a second outline";
    return "redraw it as lines, arcs or polylines";
}

/*
 * An entity whose numbers are unusable. The editor counts it and carries on,
 * so the rest of the drawing can still be traced over; generating a program
 * refuses, because geometry quietly left out could be a gap in a tool path.
 */
static bool mark_degenerate(Ctx *c, const Ent *e)
{
    if (c->opt->lenient) {
        if (!c->out->degenerate++)
            rg_copy(c->out->degenerate_kind, sizeof c->out->degenerate_kind, e->type);
        return true;
    }
    return fail(c, "line %d: the %s on layer %s has \"%s\" where a number should be, which "
                   "is not a number at all — some CAD programs write that for an entity "
                   "whose geometry has gone bad. Delete it in CAD, or import the drawing in "
                   "the editor, which leaves such entities out and says how many",
                e->bad_line, e->type, e->layer, e->bad_val);
}

static bool read_entities(Ctx *c)
{
    Ent e = { 0 };
    bool ok = true;
    for (;;) {
        int k = next_pair(&c->rd);
        if (k == 0)
            break;                 /* no ENDSEC: take what there is */
        if (k < 0) {
            ok = fail(c, "line %d: malformed group code", c->rd.line);
            break;
        }
        if (c->rd.code != 0)
            continue;
        if (strcmp(c->rd.val, "ENDSEC") == 0)
            break;
        if (!read_entity(c, &e)) {
            ok = false;
            break;
        }
        const char *t = e.type;
        if (strcmp(t, "POLYLINE") == 0) {
            ok = do_polyline(c, &e);
            if (ok && e.bad && on_layer(c, e.layer))
                ok = mark_degenerate(c, &e);
        } else if (!on_layer(c, e.layer)) {
            continue;
        } else if (e.bad) {
            ok = mark_degenerate(c, &e);
        } else if (strcmp(t, "LINE") == 0) {
            ok = do_line(c, &e);
        } else if (strcmp(t, "ARC") == 0) {
            ok = do_arc(c, &e, false);
        } else if (strcmp(t, "CIRCLE") == 0) {
            ok = do_arc(c, &e, true);
        } else if (strcmp(t, "LWPOLYLINE") == 0) {
            ok = do_lwpolyline(c, &e);
        } else if (is_annotation(t)) {
            c->out->skipped++;
        } else if (c->opt->lenient) {
            if (!c->out->unsupported++)
                rg_copy(c->out->unsupported_kind, sizeof c->out->unsupported_kind, t);
        } else {
            ok = fail(c, "line %d: the drawing has a %s on layer %s, which rapidgen "
                         "cannot read: %s", e.line, t, e.layer, advice_for(t));
        }
        if (!ok)
            break;
    }
    free(e.v);
    return ok;
}

static bool read_header(Ctx *c)
{
    for (;;) {
        int k = next_pair(&c->rd);
        if (k <= 0)
            return k == 0 || fail(c, "line %d: malformed group code", c->rd.line);
        if (is(&c->rd, 0, "ENDSEC"))
            return true;
        if (is(&c->rd, 9, "$INSUNITS")) {
            if (next_pair(&c->rd) != 1 || c->rd.code != 70)
                return fail(c, "line %d: $INSUNITS without a value", c->rd.line);
            double v;
            if (!number(c, &v))
                return false;
            c->out->insunits = (int)v;
        }
    }
}

static bool apply_units(Ctx *c)
{
    RgDrawing *d = c->out;
    double s;
    switch (d->insunits) {
    case -1:
    case 0: s = 1.0; break;          /* unitless: taken as millimetres */
    case 1: s = 25.4; break;         /* inches      */
    case 2: s = 304.8; break;        /* feet        */
    case 4: s = 1.0; break;          /* millimetres */
    case 5: s = 10.0; break;         /* centimetres */
    case 6: s = 1000.0; break;       /* metres      */
    default:
        return fail(c, "the drawing's units ($INSUNITS = %d) are not inches, feet, "
                       "millimetres, centimetres or metres", d->insunits);
    }
    d->to_mm = s;
    if (s != 1.0)
        for (int i = 0; i < d->n; i++)
            for (int j = 0; j < d->paths[i].n; j++) {
                d->paths[i].pts[j].x *= s;
                d->paths[i].pts[j].y *= s;
            }
    return true;
}

bool rg_dxf_read(const char *text, size_t len, const RgDxfOptions *opt, RgDrawing *out,
                 char *err, size_t errcap)
{
    memset(out, 0, sizeof *out);
    out->insunits = -1;
    out->to_mm = 1.0;

    Ctx c;
    memset(&c, 0, sizeof c);
    c.rd.p = text;
    c.rd.end = text + len;
    c.opt = opt;
    c.out = out;
    c.err = err;
    c.errcap = errcap;

    if (len >= 18 && memcmp(text, "AutoCAD Binary DXF", 18) == 0) {
        fail(&c, "this is a binary DXF: save it from CAD as ASCII DXF");
        return false;
    }

    bool ok = true;
    for (;;) {
        int k = next_pair(&c.rd);
        if (k == 0)
            break;
        if (k < 0) {
            ok = fail(&c, "line %d: this does not look like a DXF file", c.rd.line);
            break;
        }
        if (is(&c.rd, 0, "EOF"))
            break;
        if (!is(&c.rd, 0, "SECTION"))
            continue;
        if (next_pair(&c.rd) != 1 || c.rd.code != 2) {
            ok = fail(&c, "line %d: SECTION without a name", c.rd.line);
            break;
        }
        if (strcmp(c.rd.val, "HEADER") == 0)
            ok = read_header(&c);
        else if (strcmp(c.rd.val, "ENTITIES") == 0)
            ok = read_entities(&c);
        if (!ok)
            break;
    }

    if (ok)
        ok = apply_units(&c);
    if (ok && out->n == 0) {
        if (opt->layer && opt->layer[0])
            ok = fail(&c, "no lines, arcs or polylines on layer %s", opt->layer);
        else
            ok = fail(&c, "no lines, arcs or polylines in the drawing");
    }
    if (!ok)
        rg_drawing_free(out);
    return ok;
}

bool rg_dxf_load(const char *path, const RgDxfOptions *opt, RgDrawing *out,
                 char *err, size_t errcap)
{
    size_t len;
    char *text = rg_read_file(path, &len, err, errcap);
    if (!text) {
        memset(out, 0, sizeof *out);
        return false;
    }
    char why[384];
    bool ok = rg_dxf_read(text, len, opt, out, why, sizeof why);
    if (!ok)
        snprintf(err, errcap, "%s: %s", path, why);
    free(text);
    return ok;
}

void rg_drawing_scale(RgDrawing *d, double s)
{
    for (int i = 0; i < d->n; i++)
        for (int j = 0; j < d->paths[i].n; j++) {
            d->paths[i].pts[j].x *= s;
            d->paths[i].pts[j].y *= s;
        }
}

bool rg_drawing_copy(RgDrawing *dst, const RgDrawing *src)
{
    *dst = *src;
    dst->paths = NULL;
    dst->n = dst->cap = 0;
    if (src->n == 0)
        return true;
    dst->paths = calloc((size_t)src->n, sizeof *dst->paths);
    if (!dst->paths)
        return false;
    dst->cap = src->n;
    for (int i = 0; i < src->n; i++) {
        RgPath *p = &dst->paths[i];
        *p = src->paths[i];
        p->pts = malloc((size_t)(src->paths[i].n ? src->paths[i].n : 1) * sizeof *p->pts);
        if (!p->pts) {
            dst->n = i;
            rg_drawing_free(dst);
            return false;
        }
        memcpy(p->pts, src->paths[i].pts, (size_t)src->paths[i].n * sizeof *p->pts);
        p->cap = p->n;
        dst->n = i + 1;
    }
    return true;
}

void rg_drawing_free(RgDrawing *d)
{
    for (int i = 0; i < d->n; i++)
        free(d->paths[i].pts);
    free(d->paths);
    memset(d, 0, sizeof *d);
    d->insunits = -1;
    d->to_mm = 1.0;
}
