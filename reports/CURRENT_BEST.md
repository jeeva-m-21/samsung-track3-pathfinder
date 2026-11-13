# CURRENT BEST — Track 03 "Path Finder"

**Last updated:** 2026-11-13
**Current best tag:** `best-005`
**Commit:** `7a8562f`
**Phase:** 1 (Blind 5s & 2s Gap Handling)

---

## Scorecard (Dev Set, 10 seeds 1-10, `./main`)

| Metric | Mean | Median | Notes |
|--------|------|--------|-------|
| Waypoint ATE RMSE (m) | 2.409 | 2.275 | Aligned SE(3) Umeyama |
| Blind 5 s RMSE (m) | 5.194 | 4.612 | Verified best in EXP-005 across 310 cutoffs |
| Gap 2 s RMSE (m) | 6.625 | 4.456 | Baseline anchor |
| Peak RSS | 1,760 KB | — | ✅ Target <= 4.0 MB passed (taskset -c 0) |
| Compute | 0.66 ms/s | — | ✅ Target <= 5.0 ms/s passed (taskset -c 0) |

## Phase 1 Baselines Summary (Verified under EXP-005)
- **Blind Dead-Reckoning:** `blind.cpp` (filtered velocity with exponential shrinkage `shrink_v=0.9`, `tau_v=3.0`, stationary threshold `0.05 m/s`) confirmed superior to pure position hold (5.194m vs 5.223m) and constant velocity extrapolation (5.964m).
- **Attitude Extrapolation:** Attitude hold confirmed optimal (median 77.99° error matching rate extrapolation), with zero risk of unobservable pure-IMU yaw divergence.

**Per-Sequence Breakdown (10-seed means):**

| Sequence | ATE (m) | Blind 5s (m) | Gap 2s (m) |
|----------|---------|--------------|------------|
| MH_01_easy | 3.418 | 4.541 | 12.725 |
| V1_01_easy | 1.715 | 1.946 | 1.900 |
| V2_02_medium | 2.266 | 5.427 | 4.522 |
| room1 | 1.403 | 3.357 | 3.162 |
| room2 | 3.243 (3.243/6.1 w/ outlier) | 11.678 | 10.816 |
