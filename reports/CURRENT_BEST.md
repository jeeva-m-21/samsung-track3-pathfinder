# CURRENT BEST & PROGRESS — Track 03 "Path Finder"

**Last updated:** 2026-11-15  
**Current Best Tag:** `best-008` (commit `536e9e9`)  
**Frozen Safe Fallback Tag:** `release-candidate` / `submission-final` (commit `2b31798`)  
**Phase:** v2 R&D (Orthogonal Adaptive Q + Regime Curvature Active)  

---

## Current Best Scorecard (`best-008`, Dev Seeds 1–10, 50 runs, `./main`)

| Metric | Baseline (`35c3334`) | `best-007` (`8b9b91f`) | **`best-008` (Current Best)** | Delta vs `best-007` | Rel % vs `best-007` | Total Gain vs Initial Baseline |
|--------|:---:|:---:|:---:|:---:|:---:|:---:|
| **Blind 5 s RMSE (m)** | 18.065 | 3.785 | **3.559** | **-0.226 m** | **-5.98%** | **-80.3%** (-14.506 m) ✅ |
| **Waypoint ATE RMSE (m)** | 5.787 | 2.105 | **2.071** | **-0.034 m** | **-1.62%** | **-64.2%** (-3.716 m) ✅ |
| **Dropout 2 s Gap RMSE (m)** | 15.654 | 5.090 | **4.958** | **-0.132 m** | **-2.59%** | **-68.3%** (-10.696 m) ✅ |
| **Attitude Error** | 77.20° | 75.93° | **76.18°** | +0.244° | +0.32% | Within $\le 5.0\%$ guard ✅ |
| **Peak RSS** | 1.76 MB | 1.81 MB | **1.77 MB** | -0.04 MB | — | Target $\le 4.0$ MB passed (taskset -c 0) ✅ |
| **Compute** | 0.66 ms/s | 0.85 ms/s | **0.70 ms/s** | -0.15 ms/s | — | Target $\le 5.0$ ms/s passed (taskset -c 0) ✅ |

---

## Reviewer Confirmation Run (Seeds 201–210, 50 Paired Runs)
Independently reproduced and confirmed by Reviewer subagent (`5db90c77-7a7f-4ddc-890d-0300523ac231`):
- **Blind 5s RMSE:** **-9.04%** (-0.3735 m, 95% CI `[-0.5276, -0.2145]` m, **151% of dev effect size retained** ✅)
- **Waypoint ATE RMSE:** **-1.95%** (-0.0420 m, 95% CI `[-0.0561, -0.0282]` m ✅)
- **Dropout 2s Gap RMSE:** **-2.23%** (-0.1189 m, 95% CI `[-0.1772, -0.0675]` m ✅)
- **Worst-case single-sequence regression:** **+0.00%** (zero regression across all sequences!).

---

## Cumulative Progress from Baseline

| Milestone / Tag | Waypoint ATE (m) | Blind 5s (m) | Gap 2s (m) | Peak RSS (MB) | Compute (ms/s) |
|---|:---:|:---:|:---:|:---:|:---:|
| **Initial Baseline (`35c3334`)** | 5.787 | 18.065 | 15.654 | 1.76 | 0.66 |
| **`best-005` (`7a8562f`)** | 2.409 | 5.390 | 6.625 | 1.76 | 0.66 |
| **`best-006` (`4200484`)** | 2.309 | 4.783 | 6.200 | 1.74 | 0.83 |
| **`best-007` (`8b9b91f`)** | 2.105 | 3.785 | 5.090 | 1.81 | 0.85 |
| **`best-008` (Current Best, `536e9e9`)** | **2.071** | **3.559** | **4.958** | **1.77** | **0.70** |
| **Total Improvement vs Initial** | **-64.2%** | **-80.3%** | **-68.3%** | **Compliant** | **Compliant** |

---

## Current Best Filter Configuration (`best-008`)
- **Spike Front-End:** Causal 3-sample median filter on incoming raw gyroscope data (`src/spike.h`, `enable_spike_filter = true`, `spike_filter_acc = false`).
- **Online Adaptive Gyro Process Noise $Q_\theta(t)$ (`EXP-E63`):** Causal second-difference estimator ($\hat{\sigma}_g^2 = \text{Var}(\Delta^2 \omega) / 6$, EWMA $\alpha = 0.005$) scaling $Q_\theta$ by $\kappa_q \in [1.0, 25.0]$ during aggressive turns / degradation noise escalation.
- **Regime-Gated Curvature Extrapolation (`EXP-E64`):** Causal `WalkingDetector` monitoring high-frequency acceleration jitter and speed. Enables body-frame curvature extrapolation with $\tau_\omega = 1.5$ s when $w_{\text{walk}} \ge 0.5$, while strictly disabling curvature extrapolation on quadrotor flight ($w_{\text{walk}} \le 0.001$).
- **Calibrated Kinematic Constraints:**
  - `velocity_bounding_sigma`: 0.1015 m/s
  - `max_velocity_mps`: 3.024 m/s
  - `rotor_drag_kd`: 0.0668 (aerodynamic body drag for flight regime)
  - `rotor_drag_sigma`: 0.8020
  - `ped_sigma_lat`: 0.3520 m/s
  - `ped_sigma_vert`: 0.6647 m/s
  - `tilt_tau_a`: 0.1800, `tilt_sigma`: 0.0243
  - `shrink_v`: 0.90, `tau_v`: 3.00, `stationary_v_thresh`: 0.05

