# FsMotion development validation — 2026-10-08

Local package: `FsMotion.0.1.0-local.2`; dependency: unchanged `FsIK.0.1.0-local.7`.
The initial `.1` package is retained. `.2` includes final node-budget and margin
guards. No source change was made to the FsIK library or controller.

- 14 FsMotion tests and 28 existing FsIK tests pass.
- Fantomas checks pass for the new library, tests and F# orchestration scripts.
- The unchanged frozen FsIK evaluator passes all nine gates and all 1,100 known
  reachable targets, including numerical regression and independent path checks.
- Seeded obstacle-detour example: 12 waypoints / 24 search nodes, 11,388 timed
  samples at 120 Hz. Conservative stop-at-waypoint duration: 94.892 seconds.
- Independent NumPy planar oracle verifies all 11,388 samples: minimum checked
  clearance 0.011116 m, maximum joint velocity 0.099846 rad/s, maximum acceleration
  0.115289 rad/s², maximum selected-tip speed 0.018179 m/s. It separately verifies
  endpoints, joint limits and unchanged inactive joint.
- A separate NuGet consumer loads `FsMotion` version `0.1.0-local.2` and plans the
  same 12-waypoint obstacle detour with the inactive joint unchanged.
- Full official Axol URDF/STL fixture loads; all 18 collision meshes resolve.

These results validate the generic offline component. They do not validate
physical Axol clearance. The generic shoulder-mount envelopes reject the normal
VR rest pose; see `FS_MOTION.md` for the SDK policy comparison and remaining
calibration work. No robot session, motor enable or physical movement was used.

Reproduce: set `FSIK_PYTHON` to a Python interpreter with NumPy, then run
`dotnet fsi scripts/Build-Motion.fsx`. Add `pack` only after choosing a new unused
FsMotion package version. `dotnet fsi scripts/Build.fsx validate` checks the frozen
FsIK corpus. The example JSON is in `artifacts/motion-example/detour.json`.
