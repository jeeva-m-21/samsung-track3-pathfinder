# CURRENT BEST & PROGRESS — Track 03 "Path Finder"

**Last updated:** 2026-11-15  
**Current Best Tag:** `best-011`  
**Previous Tag:** `best-010` (commit `557d5b7`)  
**Frozen Safe Fallback Tag:** `release-candidate` / `submission-final` (commit `2b31798`)  
**Phase:** v2 R&D (Unbiased Gyro-MAD Adaptive Q + Noise-Adaptive Tilt Gate + Init Hardening + 5-Fold LOOCV Re-Balancing)  

---

## Current Best Scorecard (`best-011`, Dev Seeds 1–10, 50 runs, `./main`)

| Metric | Baseline (`35c3334`) | `best-010` (`557d5b7`) | **`best-011` (Current Best)** | Delta vs `best-010` | Rel % vs `best-010` | Total Gain vs Initial Baseline |
|--------|:---:|:---:|:---:|:---:|:---:|:---:|
| **Blind 5 s RMSE (m)** | 18.065 | 3.378 | **3.061** | **-0.316 m** | **-9.37%** | **-83.1%** (-15.004 m) ✅ |
| **Waypoint ATE RMSE (m)** | 5.787 | 2.090 | **2.006** | **-0.084 m** | **-4.02%** | **-65.3%** (-3.781 m) ✅ |
| **Dropout 2 s Gap RMSE (m)** | 15.654 | 4.980 | **4.652** | **-0.328 m** | **-6.58%** | **-70.3%** (-11.002 m) ✅ |
| **Attitude Error** | 77.20° | 76.79° | **78.89°** | +2.107° | +2.74% | Within $\le 5.0\%$ guard ✅ |
| **Peak RSS** | 1.76 MB | 1.77 MB | **1.77 MB** | 0.00 MB | — | Target $\le 4.0$ MB passed (taskset -c 0) ✅ |
| **Compute** | 0.66 ms/s | 0.90 ms/s | **0.90 ms/s** | 0.00 ms/s | — | Target $\le 5.0$ ms/s passed (taskset -c 0) ✅ |

---

## Reviewer Confirmation Run (Seeds 201–210, 50 Paired Runs)
Independently reproduced and confirmed by Reviewer subagent (`5db90c77-7a7f-4ddc-890d-0300523ac231`):
- **Blind 5s RMSE:** **-8.43%** (-0.3072 m, 95% CI `[-0.6471, -0.0182]` m, **90.0% of effect size retained** ✅)
- **Waypoint ATE RMSE:** **-3.52%** (-0.0752 m, 95% CI `[-0.1346, -0.0259]` m, **Gain preserved** ✅)
- **Dropout 2s Gap RMSE:** **-6.12%** (-0.3230 m, 95% CI `[-0.6375, -0.0599]` m, **Gain preserved** ✅)
- **Worst-case single-sequence regression:** **+10.86%** on `room1` (well inside $\le 15.0\%$ guard).

---

## Cumulative Progress from Baseline

| Milestone / Tag | Waypoint ATE (m) | Blind 5s (m) | Gap 2s (m) | Peak RSS (MB) | Compute (ms/s) |
|---|:---:|:---:|:---:|:---:|:---:|
| **Initial Baseline (`35c3334`)** | 5.787 | 18.065 | 15.654 | 1.76 | 0.66 |
| **`best-005` (`7a8562f`)** | 2.409 | 5.390 | 6.625 | 1.76 | 0.66 |
| **`best-006` (`4200484`)** | 2.309 | 4.783 | 6.200 | 1.74 | 0.83 |
| **`best-007` (`8b9b91f`)** | 2.105 | 3.785 | 5.090 | 1.81 | 0.85 |
| **`best-008` (`536e9e9`)** | 2.071 | 3.559 | 4.958 | 1.77 | 0.70 |
| **`best-010` (`557d5b7`)** | 2.090 | 3.378 | 4.980 | 1.77 | 0.90 |
| **`best-011` (Current Best)** | **2.006** | **3.061** | **4.652** | **1.77** | **0.90** |
| **Total Improvement vs Initial** | **-65.3%** | **-83.1%** | **-70.3%** | **Compliant** | **Compliant** |

---

## Current Best Filter Configuration (`best-011`)
- **5-Fold LOOCV Re-Balanced Kinematic Priors & Blind Decay (`EXP-E71`):**
  - `tau_ou`: 1.9457 s (Ornstein-Uhlenbeck drift correlation)
  - `tau_v`: 3.3678 s, `shrink_v`: 0.8573, `tau_omega_blind`: 1.8266 s, `stationary_v_thresh`: 0.0263 m/s
  - `adaptive_q_ref`: 0.003312 rad/s
  - `rotor_drag_kd`: 0.096694, `rotor_drag_sigma`: 0.622660
  - `ped_sigma_lat`: 0.214724 m/s, `ped_sigma_vert`: 0.647838 m/s
  - `max_velocity_mps`: 2.796122 m/s, `velocity_bounding_sigma`: 0.127493 m/s
  - `tilt_tau_a`: 0.188808 s
- **Pre-Median Spike-Clamped MAD Adaptive Q (`EXP-E66b`):** 2nd-difference spike-clamped MAD estimator ($\hat{\sigma}_g = \sqrt{\pi/12} \cdot \bar{d}_g \cdot 1.4364$) with reference floor `adaptive_q_ref = 0.003312` rad/s. Decouples quadratic double-scaling via `std::max(noise_mult_gyro, kappa)`.
- **Noise-Adaptive Tilt Gate Threshold (`EXP-E66b`):** Dynamically scales `tilt_tau_a` by `clamp(noise_mult, 1.0, 3.0)` so valid gravity orientation updates are maintained during progressive late-mission noise escalation without corrupting the filter.
- **Robust Initialization Hardening (`EXP-E69b`):** Clamps $g_{\text{mag}} = 9.80665$ under moving starts, protects against $1 + v_z < 10^{-4}$ 180° inverted tilt singularity, and resets internal estimators before buffer replay.
- **Spike Front-End:** Causal 3-sample median filter on incoming raw gyroscope data (`src/spike.h`, `enable_spike_filter = true`, `spike_filter_acc = false`).
- **Regime-Gated Curvature Extrapolation (`EXP-E64`):** Causal `WalkingDetector` monitoring high-frequency acceleration jitter and speed. Enables body-frame curvature extrapolation with $\tau_\omega = 1.8266$ s when $w_{\text{walk}} \ge 0.5$, while strictly disabling curvature extrapolation on quadrotor flight ($w_{\text{walk}} \le 0.001$).

