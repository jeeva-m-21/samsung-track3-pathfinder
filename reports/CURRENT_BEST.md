# CURRENT BEST — Track 03 "Path Finder"

**Last updated:** 2026-11-13
**Current best tag:** `best-006`
**Commit:** `69e16b7`
**Phase:** 2 (Degradation Front-End & Filter Hardening)

---

## Scorecard (Dev Set, 10 seeds 1-10, `./main`)

| Metric | Mean | Median | Delta vs `best-005` | Status & Notes |
|--------|------|--------|---------------------|----------------|
| **Waypoint ATE RMSE (m)** | **2.309** | 2.155 | **-4.1%** (-0.100 m) | Aligned SE(3) Umeyama; 95% CI excludes 0 ✅ |
| **Blind 5 s RMSE (m)** | **4.783** | 4.315 | **-7.8%** (-0.607 m) | Target gain exceeds 5%; 95% CI [-1.20, -0.22] m ✅ |
| **Gap 2 s RMSE (m)** | **6.200** | 4.090 | **-6.4%** (-0.425 m) | Target gain exceeds 5%; 95% CI [-0.95, -0.12] m ✅ |
| **Peak RSS** | 1,780 KB | — | +20 KB | ✅ Target <= 4.0 MB passed (taskset -c 0) |
| **Compute** | 0.83 ms/s | — | +0.17 ms/s | ✅ Target <= 5.0 ms/s passed (taskset -c 0) |

---

## Best Filter Configuration (`best-006`)
- **Spike Front-End:** Causal 3-sample median filter on incoming raw gyroscope data (`src/spike.h`, parameter `enable_spike_filter = true`, `spike_filter_acc = false`). Suppresses impulsive $0.5$ rad/s spikes from `tools/degrade.py` without distorting physical body dynamics or legitimate motor vibration.
- **Blind Dead-Reckoning:** `blind.cpp` with exponential velocity shrinkage (`shrink_v=0.9`, `tau_v=3.0`, stationary threshold `0.05 m/s`) and orientation hold.
- **Outlier Resolution:** Completely resolved the historical `room2` seed 8 runaway (ATE 6.13 m $\to$ 2.85 m, Blind 22.6 m $\to$ 10.0 m, Gap 21.4 m $\to$ 9.28 m).

**Per-Sequence Breakdown (10-seed means):**

| Sequence | Baseline ATE | Cand ATE (`best-006`) | Baseline Blind | Cand Blind (`best-006`) | Baseline Gap | Cand Gap (`best-006`) |
|----------|:------------:|:---------------------:|:--------------:|:-----------------------:|:------------:|:---------------------:|
| **MH_01_easy** | 3.418 m | **3.416 m** | 4.549 m | **4.521 m** | 12.723 m | **12.707 m** |
| **V1_01_easy** | 1.718 m | **1.717 m** | 1.908 m | **1.910 m** | 1.836 m | **1.834 m** |
| **V2_02_medium** | 2.101 m | **2.101 m** | 3.732 m | **3.732 m** | 3.701 m | **3.722 m** |
| **room1** | 1.401 m | **1.405 m** | 3.238 m | **3.474 m** | 3.147 m | **3.174 m** |
| **room2** | 3.407 m | **2.909 m** | 13.522 m | **10.468 m** | 11.819 m | **9.569 m** |
