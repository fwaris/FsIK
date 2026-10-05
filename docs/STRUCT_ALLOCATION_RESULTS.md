# Double-precision struct allocation results

Compared immutable `FsIK.0.1.0-local.1` and `FsIK.0.1.0-local.2` on Windows x64 / .NET 10.0.401. Each version completed three full frozen validation runs. Each run retains the original ten warmups, three measurement rounds, allocation counters, and complete synchronous public operations. Values below are medians across the three full runs; bytes use decimal MB where summarised.

| Benchmark | Baseline bytes | Struct bytes | Reduction | Baseline ms | Struct ms |
|---|---:|---:|---:|---:|---:|
| left-up-10mm | 103330.6 | 58866.6 | 43.0% | 0.058142 | 0.051802 |
| left-up-50.8mm | 151682.6 | 85170.6 | 43.8% | 0.081055 | 0.074110 |
| right-up-50.8mm | 98218.6 | 53754.6 | 45.3% | 0.052143 | 0.048872 |
| left-straight-up-50.8mm | 1868842.6 | 1073802.6 | 42.5% | 0.934458 | 0.915077 |
| left-path-10mm | 3175253.0 | 2132997.0 | 32.8% | 1.552963 | 1.518012 |
| left-path-50.8mm | 9121405.0 | 6325821.0 | 30.6% | 4.168900 | 5.467487 |

Mean of the six candidate/baseline allocation ratios: **0.603057**, a **39.7% reduction**. Every case allocates less. The complete two-inch path decreases from 9.12 MB to 6.33 MB (30.6%). This comparison starts at the previous extracted package, rather than the earlier campaign's original solver.

All nine gates pass in every run: build, standalone tests, frozen contract, numerical correctness, regressions, reachability, independent NumPy validation, latency, and determinism. All 1,100 reachable targets and 1,164 previously passing cases remain solved. Public reflection signatures match the previous package. All 1,214 endpoint outputs (angles, success, iteration counts and errors) and all samples of the four tested trajectories are exactly unchanged. Standalone tests increased from five to eight, checking precision, generic analytic geometry and angle-count errors.

## Timing investigation and acceptance

The canonical two-inch path median increases **31.1%**, from 4.168900 ms to 5.467488 ms, with the unchanged ten-warmup measurement protocol. The other five medians improve. This increase was investigated before adoption with separate component benchmarks using 100 warmups, five timed rounds and 100 complete-path operations per round. Both immutable packages were loaded by separate F# processes. With that longer warmup the complete path measures **3.972900 ms → 3.645963 ms**; waypoint solving **1.695950 ms → 1.554050 ms**, and retiming **2.319759 ms → 2.147476 ms**. This shows warm-up sensitivity; it does not replace the canonical measurements or establish a universal speedup. The diagnostic public Jacobian-rows benchmark is slower (0.001864 ms → 0.002956 ms), despite lower allocation.

Accepted for the allocation reduction with the canonical timing tradeoff disclosed. The complete path remains far below the unchanged 500 ms limit; endpoints remain below 10 ms. No numerical algorithm, restart seeds, arithmetic order, joint bounds, tolerances, trajectory sampling or validation limits were changed. The public API remains immutable reference records/DUs/lists; vectors, quaternions, poses and joint-axis scratch use internal double-precision value types. Scratch is local to each call.

Machine-readable canonical evidence is in `artifacts/validation/structs/{baseline,candidate}/{1,2,3}`, with `comparison.json` and `acceptance.json`; separate timing diagnostics are under `timing`. Reproduce the canonical comparison with `dotnet fsi scripts/CompareStructs.fsx` after generating the six evaluation outputs. Each run can use `dotnet fsi validation/evaluate.fsx <output/evaluation.json>` with the appropriate pinned `FsIKVersion` and `FSIK_PYTHON`.

The controller adopts local package `0.1.0-local.2` and rebuilds its embedded Linux ARM64 planner. The previous package is retained. This is offline numerical validation and build integration; it does not measure ARM64 timing or physical robot stopping/motion.
