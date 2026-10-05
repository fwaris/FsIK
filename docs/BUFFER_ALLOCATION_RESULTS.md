# Reusable .NET list trajectory experiment

The validated struct implementation is preserved on `codex/fsik-structs-baseline` (also `codex/extract-fsik`, commit `0eaaafc`). The new experiment is on `codex/fsik-trajectory-buffers`. The controller remains pinned to `FsIK.0.1.0-local.2`; the experiment creates the separate immutable local package `FsIK.0.1.0-local.3.nupkg`.

Trajectory retiming allocates one pre-sized `ResizeArray<float>` (.NET `List<double>`) for interpolated angles and reuses it with `Clear()`/`Add()`. It retains the previous value-type tip rather than building lists of points, pairs and distances for each 65-point check. Segment metadata uses a pre-sized .NET list of value tuples. Scratch belongs exclusively to one call; returned trajectory frames retain separate immutable lists and no buffer references escape.

Three complete frozen validation runs per package retain the original warmups, allocation counters, acceptance limits and data. The table reports medians across complete runs.

| Benchmark | Baseline bytes | Reusable list bytes | Reduction | Baseline ms | Reusable list ms |
|---|---:|---:|---:|---:|---:|
| left-up-10mm | 58866.6 | 58866.6 | 0.0% | 0.051340 | 0.050105 |
| left-up-50.8mm | 85170.6 | 85170.6 | 0.0% | 0.073457 | 0.074787 |
| right-up-50.8mm | 53754.6 | 53754.6 | 0.0% | 0.047952 | 0.048067 |
| left-straight-up-50.8mm | 1073802.6 | 1073802.6 | 0.0% | 0.902780 | 0.899320 |
| left-path-10mm | 2132997.0 | 1533493.0 | 28.1% | 1.546750 | 2.460900 |
| left-path-50.8mm | 6325821.0 | 3327597.0 | 47.4% | 5.783850 | 4.426375 |

The complete two-inch plan falls **6.33 MB → 3.33 MB (47.4%)**. The 10 mm plan falls **2.13 MB → 1.53 MB (28.1%)**. Endpoint IK is untouched and allocates exactly the same bytes. The mean of all six normalized allocations falls **12.6%**, because four cases exercise endpoint IK rather than trajectory retiming.

All nine frozen gates pass in each run. All **1,100 reachable targets**, **1,164 previously passing cases**, all **1,214 endpoint outputs**, and all samples of the four tested trajectories are preserved exactly. Public reflection signatures match. Nine standalone tests pass, including a new test checking that sequential segments, repeated calls and concurrent calls cannot overwrite input waypoints or previously returned frames. Packaged-source hashes and the validation consumer's DLL match the actual NuGet package; the previous package and frozen evaluator hashes are unchanged.

## Timing investigation

The original protocol's 10 mm path median increases **1.546750 ms → 2.460900 ms (59.1%)**. The two-inch path improves **5.783850 ms → 4.426375 ms (23.5%)**. All unchanged endpoint and path latency gates pass. A separate diagnostic using 100 warmups and five timed rounds measures the 10 mm path **1.413698 ms → 1.334604 ms**, and the two-inch path **3.860646 ms → 3.355351 ms**. This demonstrates sensitivity to warm-up; the canonical measurements remain the reported comparison and the regression is retained in this experiment's evidence.

The same component diagnostic measures retiming **4.50 MB → 1.52 MB**. The remaining retiming allocation largely represents output trajectory frames; solving the 100 waypoints still allocates about 1.79 MB. Detailed stage outputs and timings are under `artifacts/validation/buffers/timing`; canonical runs are under `{baseline,candidate}/{1,2,3}`. Reproduce the report with `dotnet fsi scripts/CompareBuffers.fsx` after generating the six frozen evaluation outputs.

## Reuse across plans

A prepared immutable robot model can be reused across plans: joint count/order, bounds, axes, fixed origin transforms and topology are fixed for a model identity. The numerical benchmarks already load and reuse their chain. Currently `Geometry.limits` still constructs a list, and the controller launches a fresh planner process which loads both URDF chains for each request. Reuse across controller requests would require a persistent worker and a cache keyed by verified model identity.

Measured start angles, forward kinematics, Jacobians, their numerical factorization and trajectories depend on the current state or target and must be recalculated when those change. Numerical scratch can also be reused by an exclusively owned workspace with reset/borrow semantics. The current experiment keeps workspace ownership within one call; it introduces no global mutable cache and does not change the controller lifecycle.
