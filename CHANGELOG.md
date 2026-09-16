# Changelog

All notable changes to rapidgen. Versions are dates (YYYY.MM.DD), with .N for
a second release on the same day.

## [Unreleased]

### Added

- **Cover a track.** A wizard, from the Trace tool, for a closed track - a
  band between two edges, like a seal face - covered by passes driven round
  it. Step 1: click the outer edge. Step 2: click the inner edge, or type the
  track's width. Step 3: how far outside the outer edge the first pass runs,
  how far past the inner edge the last one runs, the step-over, the drift and
  the lead, previewed on the drawing as they change. The passes are spaced
  evenly, no further apart than the step-over, and each follows whichever
  edge is nearer, so a track that narrows is followed on both sides. The gun
  holds the spray speed on every pass, so the shorter inner passes take less
  time. It moves from one pass to the next by drifting across over a set
  distance rather than jumping, comes on along a lead-in tangent to the first
  pass and leaves along a run-out into the middle, both off the work. Create
  adds the path; changing anything and pressing Replace swaps it.
- **A seam that moves each cycle.** In the wizard the gun can step across at
  the same place every cycle - click near the track to move it - or at a new
  place each cycle. A new place each cycle writes one version of the path per
  cycle, up to 8 by default, spread round the track at random but never two
  together, and never where the lead-in or run-out would cut back across the
  passes (the waist of a figure-of-eight). Shuffle picks other places. In the
  job file each version is a `cycle_stroke = k : ...` line, and the program
  chooses one by the cycle's number with `TEST ... CASE`. Points the versions
  share are written once. The Seams row of the report counts them; deleting
  or reversing one version does the whole set.
- **An example track**, `examples/seal-track.dxf`: a figure-of-eight seal
  face on a plate, to try the wizard on.
- **Entities drawn twice over are ignored** when a drawing's outlines are
  joined, in either direction. Copies had made the joiner close tiny slivers
  and lose the real outlines, so a track's inner edge could not be clicked.
  The drawing panel counts them.

### Fixed

- **Save forgot the job's file.** Saving a job that already had a file wrote
  it correctly, then left the editor with no file name: the drawing, found
  relative to the job, stopped loading, and the next Save asked where to save.
  In every release so far.
- **Reopening the last job worked once.** Opening it at start-up also emptied
  the remembered path, and opening a job from the Recent list could remember
  the wrong one.
- **A run-out made exactly as long as the gun needs was warned as too short**
  once saved: the job file keeps points to 0.001 mm, and the check allowed no
  rounding at all.
- **The command line read a flat part's drawing strictly**, so one entity the
  editor would skip lost the outline checks altogether.

## [2026.09.16.4] — alpha, coming on and off the work

### Added

- **Stretches of a stroke off the work.** The robot does not switch the
  torch, so the gun comes onto the work already spraying and leaves it
  still spraying. Any stretch of a stroke can now be marked off the work -
  travelled and sprayed, but not on the part - and a segment is work only
  when both its ends are. In the job file bars alternate off and work,
  starting and ending off:

  ```
  stroke = -60 0 | 0 0  400 0 | 460 0  460 6 | 400 6  0 6 | -60 6
           lead-in |   pass   |  turnaround  |   pass    | run-out
  ```

  An end left empty is work; a line with no bars is all work, as before, so
  every existing job still loads. Coated length, corners and the
  continuous-gun check count only the work.
- **A weave that turns round off the work.** The Fill tool's passes carry on
  past the region's own outline by 5 x the spot - the shop's rule - or by
  however far the gun needs to reach spray speed and stop again, if that is
  more. The gun comes on at speed, stops and turns round clear of the part,
  and leaves the same way. Never into a hole: a pass that ends at a hole's
  edge stops on it.
- **Lead-ins and run-outs on drawn strokes.** A stroke drawn with Line or Draw
  gets a straight lead-in and run-out along its end segments, the same
  length. On the canvas the coverage band follows only the work, and
  stretches off it are drawn faint and thin.
- **Checked against the part's outline.** A flat part's drawing now reaches
  the plan, from the editor and from the command line. Everywhere the gun
  comes on, turns round or leaves is checked to be off the part itself, not
  just off the pattern, and the first one that is not is named and placed.
  A run-out that is shorter than the distance the gun needs to reach speed
  or stop is warned about, named as a lead-in, run-out or turnaround.

### Fixed

- **The program header said `Fan nan mm`.** Every program since the
  thermal-spray rebuild printed the fan width in its header comment, which
  a spot job does not have. It says `Spot 12 mm` now.
- **laps > 1 was reported as having no effect.** Circuits were counted after
  the notes about laps had already been decided, so a job with a racetrack
  in it was told "no stroke closes on itself".
- **The continuous-gun check tested a stroke's drawn ends**, not where the gun
  really comes down and lifts. With a run-on the gun has already left the
  work, so a pattern that was clean was warned about.

## [2026.09.16.3] — alpha, driving the path

### Added

- **A closed path is driven as a circuit.** Spraying is guiding the gun round
  a path, lap after lap — not stopping at the end of one and starting again.
  A stroke that closes on itself is now entered once, driven round `laps`
  times with the seam blended, and left once. Previously every lap stopped
  dead on the seam, lifted off and came back down, which laid a heavy patch
  there each time.
- **`laps`** for flat parts: times round a closed path before the gun leaves
  it. The program repeats it as `FOR nLap FROM 1 TO n DO … ENDFOR` inside the
  cycle loop, so the targets are written once however many laps are driven.
- **Spiral fill.** Covering a region by following its outline inward — a ring
  half a spot inside the edge, then one every step-over further in, each
  joined to the next — instead of weaving back and forth across it. The
  result is one continuous path with no square turn at the end of a pass,
  which is the shape a torch is actually driven in. Rings stop when one would
  turn itself inside out, and a region with a hole is refused rather than
  spiralled, since a ring would run across the hole.

### Fixed

- **The operator prompt was asked on every cycle.** `TPReadFK` sat inside the
  cycle loop, so a coating of thirty cycles stopped for the operator thirty
  times. It is now asked once, before the loop. For a switched gun the
  prompt no longer doubles as the gun's on signal; each stroke and band
  switches its own gun, which is what the second and later ones already did.
- The prompt said *"Rotator turning and gun ready?"* on a flat part, which
  has no rotator.
- The report's *Run on/off* line was printed even when every stroke was
  closed and therefore had no run-on at all.
- **Clicking a region again piled another path on top of the last one.**
  Fill and Trace now replace the path that region produced, so re-clicking
  after changing the spot or the step-over refines it instead of stacking
  overlapping copies. What was generated is checked against the job before
  anything is removed, so Delete, Clear all and Undo cannot make it drop
  somebody else's stroke.
- **"Half the width" would not stay put.** It set the inset once and forgot,
  so it could never show as on, and changing the spot afterwards left the old
  figure behind. It latches now and holds the inset at half the spot until
  the inset is set by hand or *On the line* is pressed.
- **Overlapping passes drew as stacked boxes.** The band showing what the
  spray covers was filled once per segment and translucent, so every overlap
  darkened the one under it — with passes a step-over apart, a spiral came
  out as a heap of rectangles instead of one coated area. The colour is
  blended against the canvas once and filled opaque, so the passes merge;
  the coating is drawn under the drawing, which keeps the outline visible
  through it.
- The Trace tool's preview measured its band with `fan_width`, which a
  round spot does not have, so no band was previewed at all.

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

[2026.09.16.4]: https://github.com/HughF/rapidgen/releases/tag/2026.09.16.4
[2026.09.16.3]: https://github.com/HughF/rapidgen/releases/tag/2026.09.16.3
[2026.09.16.2]: https://github.com/HughF/rapidgen/releases/tag/2026.09.16.2
[2026.09.16.1]: https://github.com/HughF/rapidgen/releases/tag/2026.09.16.1
[2026.09.16]: https://github.com/HughF/rapidgen/releases/tag/2026.09.16
