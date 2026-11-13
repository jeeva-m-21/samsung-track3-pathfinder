# CURRENT BEST — Track 03 "Path Finder"

**Last updated:** 2026-11-13
**Current best tag:** `baseline-honest-eval`
**Commit:** `35c3334`
**Phase:** 0 (foundation)

---

## Scorecard (Dev Set, 10 seeds 1-10, baseline-honest-eval commit 35c3334)

| Metric | Mean | Notes |
|--------|------|-------|
| ATE RMSE (m) | 2.409 | Seq mean-of-means |
| Blind 5 s RMSE (m) | 5.390 | room2 seed=8 outlier inflates |
| Gap 2 s RMSE (m) | 6.625 | MH_01 gap=12.7 m dominates |
| Peak RSS | 1,876 KB | ✅ well within 4 MB |
| Compute | 1.25 ms/s | ✅ well within 5 ms/s |

**Per-Sequence Breakdown (10-seed means):**

| Sequence | ATE (m) | Blind 5s (m) | Gap 2s (m) |
|----------|---------|--------------|------------|
| MH_01_easy | 3.418 | 4.541 | 12.725 |
| V1_01_easy | 1.715 | 1.946 | 1.900 |
| V2_02_medium | 2.266 | 5.427 | 4.522 |
| room1 | 1.403 | 3.357 | 3.162 |
| room2 | 3.243 (3.243/6.1 w/ outlier) | 11.678 | 10.816 |
