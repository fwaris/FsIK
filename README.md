# FsIK

Standalone F#/.NET 10 geometry, bounded inverse kinematics and conservative trajectory planning. Production code has no dependency on Axol, cameras, SSH, a robot SDK or the controller. Runtime dependencies are FSharp.Core 11.0.100 and MathNet.Numerics 5.0.0.

The numerical implementation is the allocation campaign champion `12fd364727721b7554f6b7ff3fd81aa2d544bffa`; extraction changes its namespace to `FsIK`. Original and migrated source hashes are recorded in `provenance.json`. Immutable records, DUs and lists remain the public API; numerical scratch belongs to an individual call.

```fsharp
open FsIK

let chain = Geometry.load "robot.urdf" "base" "tool"
let seed = [ 0.0; 0.0; 0.0 ] // one angle per revolute joint
let current = Geometry.forward chain seed
let target = { current with Position = Geometry.add current.Position (Geometry.v 0.0 0.0 0.01) }
let solution = Solver.solve Solver.defaults chain target seed
let trajectory = Translation.plan chain seed (Geometry.v 0.0 0.0 0.01)
```

Distances are metres and angles are radians. The URDF loader supports fixed and bounded revolute joints. IK uses damped SVD with active joint bounds and deterministic restart seeds. Translation preserves orientation along Cartesian waypoints, accepts at most 10 cm, and retimes with the existing conservative profile: 120 Hz, 0.1 rad/s, 0.2 rad/s², 0.02 m/s tip speed and 150 seconds maximum duration. This library generates mathematical plans; applying motor commands, other-arm state and robot collision acceptance belong to the application.

## Build and local package

```text
dotnet fsi scripts/Build.fsx pack
```

This checks formatting, runs standalone tests and creates `artifacts/packages/FsIK.0.1.0-local.1.nupkg`. The local NuGet source is `artifacts/packages`. Increment `FsIKVersion` in `Directory.Build.props` before building a changed package; versions are immutable. The Axol controller uses this directory as its `FsIK-local` NuGet source and pins the version in central package management. No package is published remotely.

## Verify the actual NuGet package

Set `FSIK_PYTHON` to a Python interpreter with NumPy, then run:

```text
dotnet fsi scripts/Build.fsx validate
```

The frozen validation executable references the NuGet package, not the source project. It checks all 1,100 known reachable targets, all 1,164 previously successful endpoints, invalid inputs, finite-difference Jacobians, deterministic concurrent calls, four timed paths and six allocation measurements. An independent NumPy oracle verifies every successful endpoint and trajectory sample. Reports go to `artifacts/validation/latest`; failed gates return a nonzero exit code. Axol models and its independent mathematical oracle exist only as test fixtures; the URDF's MIT notice is retained in `validation/fixtures/AXOL_SDK_LICENSE`.

The source/test solution is `FsIK.slnx`. Package validation is separate so the first build can create its package before restoring the consumer. F# helper scripts are under `scripts`; temporary experiments belong in `temp`.
