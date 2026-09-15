# rapidgen

**ABB S4 spray program generator** — plain C99, Linux and Windows.

rapidgen writes RAPID programs for an ABB IRB 2400 on an S4, S4C or S4C+
controller that sprays the outside of a cylinder standing on a rotator. You
say which bands of the part to coat, in a job file or as a DXF of the unrolled
surface. rapidgen works out the traverse, follows every move through the
robot's kinematics, and then either writes the program or refuses with the
reason and how far off it was.

> **Status.** Tested only against its own unit tests (ASan + UBSan). The
> Windows build has been run under Wine. **No generated program has been
> loaded onto an S4 controller, simulated, or run on a robot.** The program
> file format and the IRB 2400 kinematics conventions have not been checked
> against a real controller. Read *What has and has not been proven* before
> you load anything.

## The process

- The part stands on a rotator that turns **continuously and independently**
  of the robot. The robot cannot tell which way the part is facing.
- The robot holds the gun **square to the surface** at a fixed **standoff**,
  at one position round the part, and traverses it up and down.
- Each turn of the part moves the spray band on by the pitch:
  `pitch = fan width × (1 − overlap)`. The traverse speed follows from that:
  `pitch × rpm / 60`.
- Because the part's angle is unknown, only **full bands round the part** can
  be coated. A DXF region that does not go all the way round, or a hole in
  one, is refused rather than coated wrong.
- The gun turns round `fan/2 + run-up` beyond each band edge, so every edge
  gets the full number of passes. The report says how far past the edge the
  spray lands.
- The gun is switched outside the program, unless you set `gun_signal`. The
  program stops at the start of the first band with a pendant prompt
  (`TPReadFK`) so the operator can start the rotator and the gun.

## Quick start

```sh
make
./rapidgen --template > tank.rgj     # a commented job file; edit it
./rapidgen tank.rgj --check          # plan and check, print the report
./rapidgen tank.rgj                  # write TANK01.PRG and TANK01.txt
```

Exit status: `0` written, `1` bad input, `2` refused (the report says why).

`make example` runs the two jobs in `examples/`: one with bands written into
the job, and one that reads a DXF.

## Job files

A job is plain text, one `key = value` per line (`rapidgen --template` lists
every key with its meaning). Unknown keys are errors, so a typo cannot
silently fall back to a default. The job is the only input. Programs are
regenerated from it, never edited by hand. A job is also what a request in
plain English turns into: something a person can read and correct before
anything is generated.

Coordinates:

| Frame | Origin | Axes |
|---|---|---|
| robot base | the robot's foot | ABB base frame: X forward, Z up |
| cylinder (work object `wRgCylinder`) | rotator axis at table height | Z up the axis, X towards the robot |
| drawing | as drawn | X round the circumference, Y up from the table, mm |

The tool's Z axis is the spray direction, and its X axis is the long axis of
the fan. `tool_tcp` is the gun tip, and rapidgen adds the standoff itself.

## Drawings

ASCII DXF, any AutoCAD version. rapidgen reads LINE, ARC, CIRCLE, LWPOLYLINE
and 2D POLYLINE (bulge arcs included). Loose lines and arcs are joined into
closed outlines. Arcs are flattened to within `chord_tolerance`. `$INSUNITS`
is honoured, and a drawing without units is taken as millimetres. Text,
dimensions and points are skipped.

Anything else that carries geometry is **refused with advice**, not skipped:
SPLINE and ELLIPSE (convert to polylines), INSERT (explode the block), HATCH
(delete it, or its boundary becomes a second outline), 3D polylines. Use
`layer` to read only one layer.

Overlapping outlines are merged. An outline that crosses itself, or an end
that doesn't meet another within `join_tolerance`, is an error that gives the
coordinates and the gap.

## What rapidgen checks

Every rule is a hard limit. A broken rule means no program, and the report
gives the place and the margin.

- **Drawing.** Every region wraps the full circumference, within
  `wrap_tolerance`, and the drawing is no wider than the radius allows. Holes
  and partial regions are refused.
- **Process.** The traverse speed stays under `max_spray_speed`. Bands lie on
  the part. Two or more bands need `gun_signal`, and the gap between them must
  be wider than the spray that lands past their edges.
- **Reach.** Every point on every move is checked, every `sample_step` mm
  along linear moves and every 2° along joint moves. A miss is reported in mm.
- **Joint limits.** No joint comes closer than `min_margin` to its limit.
- **Wrist singularity.** Axis 5 stays at least `min_wrist` from straight.
- **Flips.** No joint turns more than `max_joint_step` between samples on a
  straight move.
- **Clearance.** The wrist centre, flange and gun tip stay `clearance` mm
  from the part surface.

Warnings and notes cover things that are allowed but worth knowing: spray
landing on the table, over the top, or past a band edge, and the ring of
coating left if an externally controlled gun sprays while the arm waits.

## What rapidgen does not check

Every report lists these as well:

- the gun body and the arm's links against the part, rotator and cell; only
  three points on the arm are checked
- the IRB 2400's axis 2/3 interaction limit (parallel arm)
- the controller's own corner blending, and speed near the stops
- film thickness: pitch and coats set it, but the spray process decides it

## What has and has not been proven

| | Status |
|---|---|
| Kinematics agree with themselves (FK→IK round trip, 2000 random poses) | tested |
| IRB 2400 dimensions and joint limits | from ABB product data via ROS-Industrial's `abb_irb2400_support`; **not checked on a robot** |
| Joint sign conventions, `cfx` encoding | ABB's documented convention; **not checked on an S4** |
| Program file format (`%%%` header, `.PRG`, 8.3 names, MoveAbsJ on S4C/S4C+ only) | assumed; **not loaded on any controller** |
| Coverage, overrun and speed arithmetic | tested |
| Film build and coverage on a real part | **not tested** |

Before the first program runs on a robot:

1. **Check the kinematics on the pendant.** With `tool0` and `wobj0`
   selected, jog to three or four joint positions well away from zero. For
   each, compare the pendant's Cartesian position with
   `rapidgen --fk irb2400_16 J1 J2 J3 J4 J5 J6`. They should agree to within
   about a millimetre. If axis 3 or a sign disagrees, stop and report it.
2. **Check the file format.** Save a program from each controller type (S4C
   floppy, S4C+) and compare it with rapidgen's output. Then load a generated
   program and let the controller check its syntax, without running it.
3. **Calibrate the gun's TCP** on the pendant (4-point method) and put it in
   `tool_tcp` / `tool_rot`.
4. **Simulate.** Current RobotStudio has no S4 virtual controller, so use an
   IRC5 virtual controller with an IRB 2400 and the same program body.
5. **First run** in manual reduced speed, gun off, stepping through.

## Building

Needs a C99 compiler and libm, nothing else.

```sh
make            # ./rapidgen
make test       # unit tests under ASan + UBSan
make windows    # rapidgen.exe (mingw-w64: pacman -S mingw-w64-gcc / apt install mingw-w64)
make windows-dist
```

The Windows build links the UCRT, so it needs Windows 10 or 11 (Windows 7/8.1
with KB2999226).

## Source layout

| File | What it does |
|---|---|
| `rg_dxf` | DXF reader: entities into millimetre polylines |
| `rg_shape` | closing outlines, union coverage along a line, holes and crossings |
| `rg_robot` | IRB 2400 data, forward/inverse kinematics, confdata |
| `rg_job` | job file: keys, parsing, validation, frames |
| `rg_plan` | bands, moves, and every check that can refuse a plan |
| `rg_rapid` | controller dialects and the RAPID writer |
| `rg_report` | the text report |
| `rg_main` | command line |
| `plat_*` | the only OS-specific code |

## Planned

- An SDL2/Nuklear viewer in the house style: the drawing, the bands, the arm
  along the path, and the report beside them.
- An IRC5 export alongside the S4 program, plus a small RobotStudio add-in, so
  simulation becomes one step.
- A stopped-rotator mode that rasters the full DXF shape across the reachable
  arc, for parts indexed by hand.
- An IRB 2400L profile, once its dimensions are checked against ABB's data
  sheet.

## Licence

GPL-3.0-or-later. Copyright (C) 2026 Hugh Frater. See `LICENSE`.
