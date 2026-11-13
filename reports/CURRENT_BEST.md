# CURRENT BEST — Track 03 "Path Finder"

**Last updated:** 2026-11-13
**Current best tag:** `baseline-honest-eval`
**Commit:** `35c3334`
**Phase:** 0 (foundation)

---

## Scorecard (Dev Set, LOOCV, legacy seeds 42/123/456/789/999)

| Metric | Mean | Median | Worst | Notes |
|--------|------|--------|-------|-------|
| ATE RMSE (m) | 3.151 | 2.340 | 7.127 | V2_02 worst |
| Blind 5 s RMSE (m) | 8.340 | — | 23.49 | V2_02 pathological |
| Gap 2 s RMSE (m) | 8.027 | — | 15.40 | V2_02 pathological |
| RPE 1 s (m) | ~0.887 | — | — | |
| Peak RSS | 1,876 KB | — | — | ✅ well within 4 MB |
| Compute | 1.25 ms/s | — | — | ✅ well within 5 ms/s |

## Weighted Proxy Score (0.30×ATE + 0.20×blind + 0.10×gap, normalised to baseline=1.0)
**1.000** (this IS the baseline)

## Parameter File

`configs/params.json` at commit `35c3334`.

Key parameters for the blind/gap path:

| Param | Value | Role |
|-------|-------|------|
| `tau_v` | 3.0 s | Velocity decay time constant (blind) |
| `tau_omega` | 1.5 s | Angular rate decay time constant (blind) |
| `shrink_v` | 0.9 | Velocity scale factor |
| `stationary_v_thresh` | 0.05 m/s | Below this → hold position |
| `tau_gap` | 2.0 s | Gap propagation time |
| `kappa_gap` | 5.0 | Covariance inflation on gap |
| `recover_steps` | 100 | Post-gap recovery window |

## Per-Sequence Breakdown

| Sequence | ATE (m) | Blind 5s (m) | Gap 2s (m) |
|----------|---------|--------------|------------|
| MH_01_easy | 3.389 | 5.146 | 13.26 |
| V1_01_easy | 1.706 | 2.17 | 2.01 |
| V2_02_medium | 7.127 | 23.49 | 15.40 |
| room1 | 1.19 | 2.41 | 2.37 |
| room2 | 2.34 | 8.50 | 7.15 |

## Open Issues / Anomalies

1. V2_02 blind=23.5 m is 10× worse than V1_01. Root cause unknown — investigate Phase 0.
2. MH_01 gap=13.3 m is very high for a 2 s window. The filter's velocity at gap start is likely wrong.
3. room2 blind=8.5 m — handheld sequence, long trajectory (28 m span vs 16 m for room1).
4. The harness uses 5 legacy seeds; switch to seeds 1–10 for all future evaluations.
