# CURRENT BEST & FINAL RELEASE — Track 03 "Path Finder"

**Last updated:** 2026-11-13  
**Final Release Tag:** `release-candidate` / `submission-final`  
**Commit:** `b9eed87` (eval report: `46a7c15`)  
**Phase:** 5 (Sealed Final Set Evaluation & Submission Candidate Complete)  

---

## Final Scorecard (Sealed Final Set, 10 seeds 101–110, 50 runs, `./main`)

| Metric | Baseline (`35c3334`) | Final (`b9eed87`) | Median | Relative Delta vs Baseline | Notes |
|--------|:---:|:---:|:---:|:---:|:---|
| **Waypoint ATE RMSE (m)** | 5.787 | **2.132** | 1.994 | **-63.2%** (-3.655 m) | Aligned SE(3) Umeyama; matches dev within 1.3% ✅ |
| **Blind 5 s RMSE (m)** | 18.065 | **3.934** | 3.698 | **-78.2%** (-14.131 m) | Terminal cutoff dead reckoning; matches dev within 3.9% ✅ |
| **Gap 2 s RMSE (m)** | 15.654 | **5.153** | 3.318 | **-67.1%** (-10.501 m) | In-stream blackout gap; matches dev within 1.2% ✅ |
| **Peak RSS** | 1.76 MB | **1.78 MB** | — | Compliant | ✅ Target $\le 4.0$ MB passed (1,780 KB measured) |
| **Compute** | 0.66 ms/s | **0.65 ms/s** | — | Compliant | ✅ Target $\le 5.0$ ms/s passed (taskset -c 0) |

---

## Scorecard (Dev Set, 10 seeds 1-10, `./main`)

| Metric | Mean | Median | Delta vs `best-006` | Status & Notes |
|--------|------|--------|---------------------|----------------|
| **Waypoint ATE RMSE (m)** | **2.105** | 1.993 | **-8.8%** (-0.204 m) | Aligned SE(3) Umeyama; 95% CI [-0.301, -0.114] m ✅ |
| **Blind 5 s RMSE (m)** | **3.785** | 3.419 | **-20.9%** (-0.998 m) | 95% CI [-1.431, -0.609] m (exceeds 4x noise std) ✅ |
| **Gap 2 s RMSE (m)** | **5.090** | 3.190 | **-17.9%** (-1.110 m) | 95% CI [-1.555, -0.707] m (exceeds 5x noise std) ✅ |
| **Attitude Error** | 75.93° | 81.98° | +1.61% (+1.21°) | Within <= 5.0% guard limit ✅ |
| **Peak RSS** | 1,848 KB (1.81 MB) | — | +68 KB | ✅ Target <= 4.0 MB passed (taskset -c 0) |
| **Compute** | 0.85 ms/s | — | +0.02 ms/s | ✅ Target <= 5.0 ms/s passed (taskset -c 0) |

---

## Reviewer Confirmation Run (Fresh Confirmation Seeds 201–210)
Independently confirmed by Reviewer subagent on unseen seeds {201..210}:
- **Blind 5s RMSE:** **-13.41%** (-0.640 m, 95% CI `[-1.008, -0.313]` m, 64.2% retained effect size ✅)
- **Gap 2s RMSE:** **-12.58%** (-0.769 m, 95% CI `[-1.168, -0.410]` m, 69.2% retained effect size ✅)
- **Waypoint ATE RMSE:** **-5.80%** (-0.133 m, 95% CI `[-0.217, -0.057]` m, 65.0% retained effect size ✅)

---

## Cumulative Progress from Baseline

| Milestone / Tag | Waypoint ATE (m) | Blind 5s (m) | Gap 2s (m) | Peak RSS (MB) | Compute (ms/s) |
|---|:---:|:---:|:---:|:---:|:---:|
| **Initial Baseline (`35c3334`)** | 5.787 | 18.065 | 15.654 | 1.76 | 0.66 |
| **`best-005` (`7a8562f`)** | 2.409 | 5.390 | 6.625 | 1.76 | 0.66 |
| **`best-006` (`4200484`)** | 2.309 | 4.783 | 6.200 | 1.74 | 0.83 |
| **`best-007` (Current Best)** | **2.105** | **3.785** | **5.090** | **1.81** | **0.85** |
| **Total Improvement vs Initial** | **-63.6%** | **-79.0%** | **-67.5%** | **Compliant** | **Compliant** |

---

## Current Best Filter Configuration (`best-007`)
- **Spike Front-End:** Causal 3-sample median filter on incoming raw gyroscope data (`src/spike.h`, `enable_spike_filter = true`, `spike_filter_acc = false`).
- **Post-Spike Tuned Parameters:**
  - `velocity_bounding_sigma`: 0.1015 m/s (tightened velocity constraint envelope)
  - `max_velocity_mps`: 3.024 m/s (velocity saturation limit)
  - `rotor_drag_kd`: 0.0668 (calibrated aerodynamic body drag for flight regime)
  - `rotor_drag_sigma`: 0.8020
  - `ped_sigma_lat`: 0.3520 m/s (tightened non-holonomic lateral velocity prior)
  - `ped_sigma_vert`: 0.6647 m/s (relaxed vertical gait vibration constraint)
  - `tilt_tau_a`: 0.1800 (accelerometer innovation gating time constant)
  - `tilt_sigma`: 0.0243
  - `shrink_v`: 0.90, `tau_v`: 3.00, `stationary_v_thresh`: 0.05
