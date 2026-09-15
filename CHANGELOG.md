# Changelog

All notable changes to rapidgen. Versions are dates (YYYY.MM.DD), with .N for
a second release on the same day.

## [Unreleased]

### Added

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
