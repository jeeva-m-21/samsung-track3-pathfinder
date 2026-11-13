# Scorecard: EXP-E22 Spike Front-End (3-sample median filter)

**Generated:** 2026-11-13 15:06:08 | **Total Runs:** 50 (5 dev sequences x 10 seeds)

## 1. Overall Scorecard & Paired Ablation (Seeds 1–10)

| Metric | Baseline (Off) | Candidate (On) | Paired Delta | 95% Bootstrap CI | Rel Delta % | Target / Status |
|---|---|---|---|---|---|---|
| Waypoint ATE RMSE | 2.4089 m | 2.3093 m | -0.0996 m | [-0.2408, -0.0212] | -2.54% | Collateral guard <= +3% |
| Blind 5 s RMSE | 5.3899 m | 4.7832 m | -0.6068 m | [-1.2030, -0.2239] | **-7.83%** | Target >= -5% |
| Gap 2 s RMSE | 6.6250 m | 6.2004 m | -0.4245 m | [-0.9532, -0.1217] | **-5.42%** | Target >= -5% |
| Attitude Error | 74.47° | 74.73° | +0.25° | [+0.11, +0.40] | +0.45% | Guard <= +5% |

## 2. Budget Measurements (taskset -c 0)

- **Peak RSS:** 1780 KB (1.74 MB) vs budget target <= 4.0 MB (PASS ✅)
- **Compute:** 0.83 ms/s vs budget target <= 5.0 ms/s (PASS ✅)
- **Single Thread C++17:** Confirmed (PASS ✅)
- **Heap Allocation in Filter Loop:** 0 bytes (stack-only ring buffer) (PASS ✅)

## 3. Per-Sequence Breakdown

| Sequence | ATE (Base -> Cand) | Blind 5s (Base -> Cand) | Gap 2s (Base -> Cand) | Noise Floor 2x std (Blind/Gap) |
|---|---|---|---|---|
| **MH_01_easy** | 3.418 -> 3.417 (-0.0%) | 4.541 -> 4.521 (-0.5%) | 12.725 -> 12.707 (-0.1%) | Blind: 0.115m / Gap: 0.180m |
| **V1_01_easy** | 1.715 -> 1.717 (+0.2%) | 1.946 -> 1.905 (-2.1%) | 1.900 -> 1.834 (-3.5%) | Blind: 0.047m / Gap: 0.413m |
| **V2_02_medium** | 2.266 -> 2.101 (-7.3%) | 5.427 -> 3.732 (-31.2%) | 4.522 -> 3.722 (-17.7%) | Blind: 0.212m / Gap: 0.399m |
| **room1** | 1.403 -> 1.404 (+0.1%) | 3.357 -> 3.371 (+0.5%) | 3.162 -> 3.170 (+0.3%) | Blind: 2.112m / Gap: 1.296m |
| **room2** | 3.243 -> 2.907 (-5.6%) | 11.678 -> 10.387 (-5.9%) | 10.816 -> 9.570 (-6.0%) | Blind: 7.504m / Gap: 7.178m |