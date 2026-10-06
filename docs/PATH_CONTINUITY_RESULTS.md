# Bounded path continuity — validated .5 package

The validated `.4` baseline is preserved on local/GitHub `main` at `19c164d118e3446f53cc803457756e8fc725d70b`. Existing experiment branches and all previous immutable packages remain intact. GitHub's default branch remains `codex/extract-fsik`. The fix is on `codex/fsik-path-continuity`.

## Behavior and API

`Solver.solveNear : Options -> float -> Chain -> Pose -> float list -> SolveResult` restricts every candidate to physical limits intersected with the supplied seed's absolute joint-change neighborhood. The original seed and deterministic per-joint +/-0.08 rad perturbations are tried; no global restart seeds or bounded-angle wrapping occur. Floating-point bounds are rounded inward where needed. Existing DUs and `Solver.solve` remain compatible.

`Translation.planWithContinuity` accepts immutable `TranslationContinuityOptions`; `Translation.plan` uses defaults of 0.1 rad, eight subdivision levels per original interval, and 4,096 accepted waypoints including the start. Original Cartesian spacing remains 0.508 mm. Local failures trigger deterministic first-half/second-half bisection, anchored to each preceding accepted solution. Depth/waypoint exhaustion returns an interval-specific error without any partial trajectory. The interval is a fraction of the original request (0..1). Original orientation, bounds, numerical tolerances, retiming and motion limits are retained.

Numerical scratch uses the existing exclusive workspace leases, including a reusable local-bound array; every solve returns its lease in `finally`. Returned solutions are independent immutable snapshots. This bounds local changes, not a preferred elbow posture. Tool configuration, continuous retiming and null-space posture objectives are separate work.

## Three complete runs per version

Median results across the fresh baseline and candidate runs, using unchanged warmups and allocation counters:

| Benchmark | .4 bytes | .5 bytes | .4 ms | .5 ms | Time change |
|---|---:|---:|---:|---:|---:|
| left-up-10mm | 25714.6 | 25714.6 | 0.046163 | 0.047045 | +1.9% |
| left-up-50.8mm | 36946.6 | 36946.6 | 0.065623 | 0.067212 | +2.4% |
| right-up-50.8mm | 23194.6 | 23194.6 | 0.043210 | 0.042690 | -1.2% |
| left-straight-up-50.8mm | 471498.6 | 471498.6 | 0.813990 | 0.821327 | +0.9% |
| left-path-10mm | 938525.0 | 942973.0 | 1.274850 | 1.290912 | +1.3% |
| left-path-50.8mm | 2345157.0 | 2377765.0 | 3.046250 | 3.698187 | +21.4% |

All four endpoint allocation measurements are unchanged. The 10 mm path adds 4,448 bytes (0.47%); the two-inch path adds 32,608 bytes (1.39%). Mean normalized allocation rises 0.31%. This is a continuity feature, not an allocation reduction experiment.

## Timing investigation

The canonical two-inch path increases **3.046250 ms -> 3.698188 ms (21.4%)**. All original latency gates pass. A separate 100-warmup/five-round diagnostic measures **2.945131 ms -> 3.132894 ms (6.4%)**. The 100 waypoint solves use the same **137 iterations**, with identical per-waypoint counts; their diagnostic time rises 1.366618 -> 1.422387 ms (4.1%). Retiming's code and allocations are unchanged, although its diagnostic elapsed time varies 1.592368 -> 1.692091 ms. The evidence shows additional continuity bookkeeping and measurement sensitivity; the canonical slowdown remains disclosed and is not replaced by the diagnostic.

## Validation and adoption

All nine frozen gates pass in each of the six complete runs. All **1,100 reachable** and **1,164 previously passing** cases remain solved; all **1,214 ordinary endpoint outputs** match exactly, including angles, iterations and errors. All four existing trajectory files remain identical and pass the independent NumPy endpoint/path oracle. Existing public reflection signatures remain present; new APIs are additive. Frozen files, old packages and packaged production-source hashes are verified.

All **25 standalone tests**, **77 controller tests** and **62 Python adapter tests** pass. New tests cover neighborhood refusal, unchanged/physical-boundary poses, rounding, local/global concurrent buffer reuse, both planar elbow configurations, subdivision from a straight chain, exhausted budgets, recorded left/right 10 mm and 50.8 mm translations, and straight Axol boundary targets. The adapter regression confirms that a continuity planning error removes the prior plan and cannot start motion.

The controller pins `0.1.0-local.5`. The planner, packaged-library integration tests and benchmark consumer load the new NuGet DLL; the Linux ARM64 build and embedded worker hashes match. Benchmark compilation and Fantomas pass. Only the controller package pin, solver paragraph and rejection regression were changed; other existing working changes were preserved. No robot deployment, connection, enabling or movement was performed.

Canonical evidence: `artifacts/validation/continuity/{baseline,candidate}/{1,2,3}`. `comparison.json`, `acceptance.json`, `verified-experiment.json` and `timing/` record compatibility, hashes and measurements. Recompute the comparison with `dotnet fsi scripts/CompareContinuity.fsx` after producing the six frozen evaluator runs.
