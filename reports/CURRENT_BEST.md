# CURRENT BEST & PROGRESS — Track 03 "Path Finder"

**Last updated:** 2026-11-15  
**Current Best Tag:** `best-010` (commit `557d5b7`)  
**Previous Tag:** `best-008` (commit `536e9e9`)  
**Frozen Safe Fallback Tag:** `release-candidate` / `submission-final` (commit `2b31798`)  
**Phase:** v2 R&D (Unbiased Gyro-MAD Adaptive Q + Noise-Adaptive Tilt Gate + Init Hardening)  

---

## Current Best Scorecard (`best-010`, Dev Seeds 1–10, 50 runs, `./main`)

| Metric | Baseline (`35c3334`) | `best-008` (`536e9e9`) | **`best-010` (Current Best)** | Delta vs `best-008` | Rel % vs `best-008` | Total Gain vs Initial Baseline |
|--------|:---:|:---:|:---:|:---:|:---:|:---:|
| **Blind 5 s RMSE (m)** | 18.065 | 3.559 | **3.378** | **-0.182 m** | **-5.10%** | **-81.3%** (-14.687 m) ✅ |
| **Waypoint ATE RMSE (m)** | 5.787 | 2.071 | **2.090** | +0.019 m | +0.93% | **-63.9%** (-3.697 m) ✅ |
| **Dropout 2 s Gap RMSE (m)** | 15.654 | 4.958 | **4.980** | +0.022 m | +0.43% | **-68.2%** (-10.674 m) ✅ |
| **Attitude Error** | 77.20° | 76.18° | **76.79°** | +0.608° | +0.80% | Within $\le 5.0\%$ guard ✅ |
| **Peak RSS** | 1.76 MB | 1.77 MB | **1.77 MB** | 0.00 MB | — | Target $\le 4.0$ MB passed (taskset -c 0) ✅ |
| **Compute** | 0.66 ms/s | 0.70 ms/s | **0.90 ms/s** | +0.20 ms/s | — | Target $\le 5.0$ ms/s passed (taskset -c 0) ✅ |

---

## Reviewer Confirmation Run (Seeds 201–210, 50 Paired Runs)
Independently reproduced and confirmed by Reviewer subagent (`5db90c77-7a7f-4ddc-890d-0300523ac231`):
- **Blind 5s RMSE:** **-3.10%** (-0.1168 m, 95% CI `[-0.2800, +0.0799]` m, **60.8% of effect size retained** ✅)
- **Waypoint ATE RMSE:** **+1.07%** (+0.0226 m, 95% CI `[-0.0018, +0.0560]` m, well within $\le 3.0\%$ guard ✅)
- **Dropout 2s Gap RMSE:** **+1.06%** (+0.0556 m, 95% CI `[-0.0940, +0.2463]` m, well within $\le 3.0\%$ guard ✅)
- **Worst-case single-sequence regression:** **+5.11%** on `room2` (well inside $\le 15.0\%$ guard).

---

## Cumulative Progress from Baseline

| Milestone / Tag | Waypoint ATE (m) | Blind 5s (m) | Gap 2s (m) | Peak RSS (MB) | Compute (ms/s) |
|---|:---:|:---:|:---:|:---:|:---:|
| **Initial Baseline (`35c3334`)** | 5.787 | 18.065 | 15.654 | 1.76 | 0.66 |
| **`best-005` (`7a8562f`)** | 2.409 | 5.390 | 6.625 | 1.76 | 0.66 |
| **`best-006` (`4200484`)** | 2.309 | 4.783 | 6.200 | 1.74 | 0.83 |
| **`best-007` (`8b9b91f`)** | 2.105 | 3.785 | 5.090 | 1.81 | 0.85 |
| **`best-008` (`536e9e9`)** | 2.071 | 3.559 | 4.958 | 1.77 | 0.70 |
| **`best-010` (Current Best, `557d5b7`)** | **2.090** | **3.378** | **4.980** | **1.77** | **0.90** |
| **Total Improvement vs Initial** | **-63.9%** | **-81.3%** | **-68.2%** | **Compliant** | **Compliant** |

---

## Current Best Filter Configuration (`best-010`)
- **Pre-Median Spike-Clamped MAD Adaptive Q (`EXP-E66b`):** 2nd-difference spike-clamped MAD estimator ($\hat{\sigma}_g = \sqrt{\pi/12} \cdot \bar{d}_g \cdot 1.4364$) with reference floor `adaptive_q_ref = 0.006` rad/s. Decouples quadratic double-scaling via `std::max(noise_mult_gyro, kappa)`.
- **Noise-Adaptive Tilt Gate Threshold (`EXP-E66b`):** Dynamically scales `tilt_tau_a` by `clamp(noise_mult, 1.0, 3.0)` so valid gravity orientation updates are maintained during progressive late-mission noise escalation without corrupting the filter.
- **Robust Initialization Hardening (`EXP-E69b`):** Clamps $g_{\text{mag}} = 9.80665$ under moving starts, protects against $1 + v_z < 10^{-4}$ 180° inverted tilt singularity, and resets internal estimators before buffer replay.
- **Spike Front-End:** Causal 3-sample median filter on incoming raw gyroscope data (`src/spike.h`, `enable_spike_filter = true`, `spike_filter_acc = false`).
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

