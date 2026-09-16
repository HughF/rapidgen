# Changelog

All notable changes to rapidgen. Versions are dates (YYYY.MM.DD), with .N for
a second release on the same day.

## [2026.09.16.2] — alpha, built for thermal spray

### Changed

- **Built for thermal spray, not paint.** The process is a torch laying down a
  round spot, built up over many cycles, not a paint fan in one or two coats.
  - `pattern = spot | fan`, spot by default. A round spot is the same width
    whichever way the stroke runs, so the gun's rotation about its own axis no
    longer constrains anything — `fan_along` and the "this stroke runs along
    the fan" warning apply only to `pattern = fan`. The canvas draws the strip
    a disc sweeps for a spot, and the slot a fan sweeps for a fan.
  - `spot_diameter` and `step_over` (the advance between passes) replace fan
    width and overlap as the working numbers; `overlap` still sets the
    step-over when it is not given. The report says how many passes each point
    gets: the width over the step-over.
  - `cycles` repeats the whole pattern, with `dwell` seconds between them and
    an optional `cool_signal` held on through the dwell. The program repeats
    the pattern in a `FOR` loop rather than writing its targets out again, so
    a coating of dozens of cycles still fits an S4's memory.
  - `thickness_per_pass` (your own measured microns) and `target_thickness`
    give an estimated thickness per cycle and in total, and how many cycles
    would reach the target. It is arithmetic on your figure, not a model of
    the process, and nothing is refused on the strength of it.
  - `gun = continuous | switched`. A torch cannot be switched stroke by
    stroke, so "more than one stroke needs a gun signal" is gone; instead,
    travel between strokes that passes over the part, and every descent and
    lift over it, is measured and warned about, because that is where unwanted
    coating lands.
  - **Run-on and run-off.** Each open stroke is extended past both ends by
    `lead` — by default far enough to reach spray speed and stop again, from
    the speed and the acceleration — and the gun no longer comes to a dead
    stop on the work: a dip in speed is a ridge in the coating. Corners are
    rounded throughout; the only stops are clear of the part.
  - The cylinder's report gives the **surface speed** the part passes the gun
    at, and warns outside the 0.2–3 m/s thermal spraying usually runs at.
  - Part temperature is stated as not modelled, next to the dwell that exists
    to control it.
  - The report, the editor's help text and the templates say *spot* where they
    used to say *fan*: the part is coated "half the spot" past a band edge,
    the Fill tool asks for the spot and the step-over, and the flat template
    explains the gun's 90° turn on its mount as what keeps axis 5 clear of
    straight, which is what it is for now that a round spot can run any way.
    The README's screenshot is the Draw page as it is now.

## [2026.09.16.1] — alpha, DXF import fix

### Fixed

- A DXF with `1.#QNAN`, `-1.#IND` or `1.#INF` in it — what a program built
  against the Microsoft C runtime writes for a value that is not a number, and
  what QCAD leaves behind for an entity whose geometry has gone bad — no
  longer costs the whole import. The entity is left out and counted, and the
  editor says how many and of what kind; generating a program still refuses,
  naming the entity, its layer and the line, because geometry quietly dropped
  could be a gap in a tool path. One dead vertex condemns its polyline, not
  the drawing. A value that is neither a number nor one of those spellings is
  still an error.

## [2026.09.16] — first alpha

An alpha: everything below is tested against unit tests and, for the editor,
by driving the real program and reading the screenshots back. **No program it
writes has been loaded onto an S4 controller, simulated, or run on a robot,**
and the file format and the IRB 2400's kinematics conventions have not been
checked against a real controller. See *What has and has not been proven* in
the README before loading anything.

### Added

- **Flat parts.** A part lying still on a plane, sprayed along *strokes*: the
  gun points along the surface normal at the standoff and follows each stroke
  in turn, stopping on the point at the end of one. Strokes live in the job
  file, so a pattern can be read, edited by hand, and kept in version control.
- **The editor** (`rapidgen` with no arguments): an SDL2 and Nuklear window in
  the house style, with three pages.
  - *Draw* shows the part's DXF with the pattern painted over it. Each stroke
    is drawn as a translucent band as wide as the spray fan — what will
    actually be coated — with its number at the start and arrows along it, so
    the order and direction are visible. Tools: Select, Line (snapping to the
    drawing's corners and lines), freehand Draw (thinned when the button is
    let go), Trace (round an outline, inset by an amount), Fill (parallel
    passes across a region, previewed on hover), and Scale.
  - *Settings* edits every value the job file holds.
  - *Program* runs the checks as the job changes, lists what was refused,
    and shows the report and the program before writing them.
- **The fan's direction** (`fan_along`) for flat parts, because a spray fan is
  a wide slot and the gun does not turn during a program: a stroke running
  across the fan lays down a band its full width, one running along it paints
  a narrow line. The canvas draws what the fan really sweeps, and the report
  warns with how much of the painted length runs within 30° of the fan.
- **Scaling a drawing to a dimension**: click two points a known distance
  apart and type the true distance, or set the drawing's overall width. The
  strokes scale with the drawing so they stay where they were painted.
- **Pattern tools** as a tested core (`rg_pattern`): Ramer–Douglas–Peucker
  thinning, snapping to a drawing's vertices and edges, tracing an outline
  with a mitred inset, and filling a region with parallel passes joined into
  zig-zags where the region allows. Fill passes run past the region's own
  outline so its edge gets a full coat, and stop exactly on the edge of a
  hole so nothing meant to stay bare is sprayed.
- **Flat-part checks**: several strokes need a gun signal, because the gun
  must be off between them; sharp corners are counted and reported, since the
  robot slows through each one and the coat builds up there; and reach, joint
  limits, wrist singularity, flips and clearance are checked as for a
  cylinder.
- Job files can be written as well as read, so the editor and the command
  line share one format; `rapidgen --template flat` prints a flat-part
  example.
- A tolerant DXF read and outline builder for the editor: centre lines,
  dimension lines and anything rapidgen cannot read are counted and left out
  rather than refused, so a real part drawing can be traced over. Generating
  a program still refuses them.

### Changed

- The program writer places a 1 mm zone on the points inside a stroke and
  stops exactly on its last point, and names the work object for the kind of
  part.

### Added before the editor: the command line and cylinders

- Spray programs for an ABB IRB 2400 (/10, /16) on S4, S4C and S4C+
  controllers, for a cylinder on a rotator that turns continuously and
  independently of the robot: gun square to the surface at a set standoff,
  traversed up and down at pitch × rpm / 60, turning round half a fan plus
  the run-up beyond each band edge.
- Job files (`.rgj`): plain `key = value`, unknown keys rejected, every
  required value named when missing. `rapidgen --template` prints a
  commented one.
- Bands written into the job, or read from an ASCII DXF of the unrolled
  surface (LINE, ARC, CIRCLE, LWPOLYLINE, 2D POLYLINE with bulges;
  `$INSUNITS`; layer filter). SPLINE, ELLIPSE, INSERT, HATCH and 3D
  polylines are refused with advice rather than skipped.
- Coverage checks: every region must wrap the full circumference and match
  the radius; holes, partial regions, self-crossing outlines and gaps
  between loose lines are refused with their coordinates.
- IRB 2400 forward and inverse kinematics (all eight solutions) and ABB
  confdata; `--fk` and `--ik` for checking against the pendant.
- Plan checks that refuse the program: reach (shortfall in mm), joint limit
  margin, wrist singularity margin, configuration flips mid-move, clearance
  of wrist centre, flange and gun tip from the part, traverse speed, bands
  off the part, bands too close together, several bands without a gun signal.
- RAPID writer: one module with named CONST robtargets, MoveAbsJ home
  (MoveJ under ConfJ on the original S4), optional tooldata, optional gun
  output, operator prompt before the first traverse, 8.3 names for floppy
  controllers, normalised quaternions.
- A plain-text report beside every program, and in place of a refused one,
  listing the derived process, the closest approach to every rule, warnings,
  and what is not checked.
- Windows cross-build (`make windows`, `make windows-dist`).

[2026.09.16.2]: https://github.com/HughF/rapidgen/releases/tag/2026.09.16.2
[2026.09.16.1]: https://github.com/HughF/rapidgen/releases/tag/2026.09.16.1
[2026.09.16]: https://github.com/HughF/rapidgen/releases/tag/2026.09.16
