# Reusable solver workspace allocation experiment

This experiment is on `codex/fsik-solver-workspace`, branching from the preserved `codex/fsik-trajectory-buffers` implementation (`0.1.0-local.3`). The controller remains pinned to `0.1.0-local.2`. The new immutable local package is `FsIK.0.1.0-local.4.nupkg`.

## Storage and ownership

The solver rents an exclusively owned workspace containing pre-sized native .NET mutable lists for current, candidate and best joint angles, active joints and blocked joints. Axis/origin arrays, the full Jacobian matrix, error/coefficient/scaled/delta vectors and reduced active-set matrices are retained and overwritten. The same workspaces can be reused across iterations, deterministic restarts, successive waypoints and later solves using the same loaded chain.

A `ConditionalWeakTable` associates each immutable chain with cached joint limits and a `ConcurrentBag` of available workspaces. Each concurrent request obtains separate storage. A `finally` block returns the lease after success, non-convergence or an exception; the next solve resets its angles from its own seed. Every matrix/vector entry needed for a computation is overwritten before use. Completed solutions copy their final angles to independent immutable F# lists. No targets, solved answers, old motor commands, SVD factors or model responses are cached. The cache and owned scratch can be collected when their chain is no longer used.

The algorithm retains the original SVD, five line-search values/order, active-bound decisions, seed perturbations, 64 restart seeds, tolerances, arithmetic order and trajectory sampling. MathNet's multiply overloads write results into owned vectors. SVD itself and completed immutable results still allocate. Cache/workspace initialization costs occur on first use; reported allocation measurements retain the frozen warmup protocol and include each complete synchronous public operation.

## Three complete runs per package

All data, measurements and acceptance limits are unchanged. Medians across the three complete baseline and three candidate runs:

| Benchmark | Baseline bytes | Workspace bytes | Reduction | Baseline ms | Workspace ms |
|---|---:|---:|---:|---:|---:|
| left-up-10mm | 58866.6 | 25714.6 | 56.3% | 0.050905 | 0.062190 |
| left-up-50.8mm | 85170.6 | 36946.6 | 56.6% | 0.077035 | 0.073400 |
| right-up-50.8mm | 53754.6 | 23194.6 | 56.9% | 0.049892 | 0.045665 |
| left-straight-up-50.8mm | 1073802.6 | 471498.6 | 56.1% | 0.908405 | 0.902930 |
| left-path-10mm | 1533493.0 | 938525.0 | 38.8% | 2.489150 | 1.350450 |
| left-path-50.8mm | 3327597.0 | 2345157.0 | 29.5% | 4.370350 | 3.379500 |

Mean normalized allocation falls **49.0%** against the previous mutable-trajectory package. Endpoint IK falls approximately **56–57%**. The two-inch plan falls **3.33 MB → 2.35 MB (29.5%)**, and the 10 mm plan **1.53 MB → 0.94 MB (38.8%)**. Against the original extracted package's 9.12 MB, cumulative two-inch allocation falls **74.3%**.

## Validation and timing

All nine gates pass in each complete run. All **1,100 reachable targets** and **1,164 previously passing cases** remain solved. All **1,214 endpoint outputs**, including angles, success flags, iteration counts and errors, and every sample of all four tested trajectories are **exactly identical**. Public reflection signatures match. All ten standalone tests pass, including retained-result ownership, mixed-target concurrent solves, non-convergence/invalid-input recovery, and trajectory scratch isolation. The frozen file hashes, previous package and current packaged-source hashes are unchanged/matching; the frozen validation consumer loads the actual new packaged DLL. Fantomas passes.

The original-protocol 10 mm left endpoint increases **0.050905 ms → 0.062190 ms (22.2%; 11.3 microseconds)**. The other five medians improve, including the complete two-inch plan **4.370350 ms → 3.379500 ms (22.7%)**. All original latency ceilings pass. A separate longer-warmup diagnostic (100 warmups, five timed rounds, 1,000 endpoint operations per round) measures that 10 mm endpoint **0.064158 ms → 0.048034 ms**. Its two-inch complete path measures **3.511038 ms → 3.161039 ms**. The original measurements remain the canonical comparison; the separate diagnostic demonstrates warm-up sensitivity and does not assert a universal speedup.

The component diagnostic measures 100 waypoint solves at **1.79 MB → 0.81 MB**, while trajectory retiming remains about 1.52 MB. Allocation now largely represents MathNet's SVD work and immutable output frames. The controller currently launches a fresh process per plan, so reuse across controller requests would additionally require a persistent planner worker; within one translation's 100 solves the workspace is already reused. This branch does not change that lifecycle or deploy to the robot.

Canonical evidence is under `artifacts/validation/workspace/{baseline,candidate}/{1,2,3}`; `comparison.json` contains all medians and `verified-experiment.json` records package checks. Separate diagnostic stages are under `timing`. Recompute the canonical comparison using `dotnet fsi scripts/CompareWorkspace.fsx` after producing the six evaluator outputs.
