# Phase 1 Ground-Truth Audit & Discovery Report

**Date:** 2026-11-15  
**Parent Tag:** `best-008` (`845f487` on `v2-dev`)  
**Harness / Protocol:** `tools/eval_degraded_harness.py`, Dev Seed 1 across 5 sequences (`MH_01_easy`, `V1_01_easy`, `V2_02_medium`, `room1`, `room2`).

---

## 1. Executive Summary: The Staggering "Frozen Filter" Discovery

During the Phase 1.2 Telemetry Print, we uncovered the single most significant hidden defect in the filter's operational history:

> **Every degraded evaluation in this repository has been running as a static, frozen dead-reckoning estimate from $t = 26.0\text{ s}$ until the end of the trajectory ($t = 110..145\text{ s}$)!**

### How it Happened
1. In `configs/degrade_default.json`, a 2.0-second gap encoded as NaNs occurs at $t = 25.0..27.0\text{ s}$ (400 consecutive samples).
2. In `src/blind.cpp`, `FailureDetector::update()` monitors consecutive invalid duration. When `invalid_duration >= params::T_fail` (1.0 s), it declares `is_failed_ = true`.
3. In `src/pipeline.cpp`, line 1289:
   ```cpp
   if (blind_predictor.is_failed()) {
       blind_predictor.predict(t_curr, p_blind, q_blind);
       delay_buf.push(t_curr, p_blind, q_blind, false, collector);
       return;
   }
   ```
4. `FailureDetector` **never un-latches**. When valid IMU samples resume at $t = 27.05\text{ s}$, `process_full_step` checks `blind_predictor.is_failed()` first and immediately returns blind predictions.
5. In `blind_predictor.predict()`, velocity decays exponentially with $\tau_v \approx 1.0\text{ s}$. By $t = 30\text{ s}$, estimated velocity drops to $< 0.01\text{ m/s}$, and estimated position freezes permanently!
   - `MH_01_easy`: displacement from $t = 30\text{ s}$ to $t = 144.1\text{ s}$ was only **0.18 m**!
   - `room1`: displacement from $t = 30\text{ s}$ to $t = 141.1\text{ s}$ was only **0.40 m**!
6. **Remarkable Implication:** The current `2.071 m` ATE was achieved with the filter **completely frozen for the last 70–80% of every sequence**! Because the drone and pedestrian walk in loops around a room, a frozen position near room center happens to achieve ~2 m ATE. Resuming true ESKF tracking after gaps with proper re-anchoring will unlock massive improvements across all metrics.

---

## 2. Phase 1.1 Source Code Audit Confirmations

| # | Subsystem | File & Lines | Audit Verdict | Impact |
|---|-----------|--------------|---------------|--------|
| 1 | Gyro Median Phase | `src/spike.h:20-65` | Centered 3-tap median on $s_1$. Emits $s_1$ acc with median gyro. No intra-sample gyro/accel phase lag, but 1-sample (5 ms) causal output latency. | Clean intra-sample alignment; conditional Hampel will eliminate 5 ms latency. |
| 2 | Dynamic Gravity Latch | `src/init.cpp:316` | If `motion_detected == true`, line 316 latches dynamic $\|\mathbf{a}_{\text{ref}}\|$ into $g_{\text{mag}}$. | **Defect:** Corrupts world gravity magnitude if drone/person accelerates at $t=0$. Fixed: lock $g_{\text{mag}} = 9.80665$. |
| 3 | Inverted Mount Check | `src/init.cpp:46-55` | Checks $\cos\theta < -0.999999999$ ($> 179.997^\circ$). | **Risk:** Too narrow. Fixed: widen guard to $1 + v_z < 10^{-4}$. |
| 4 | Replay Reset Hygiene | `src/pipeline.cpp:1515` | `estimator_q`, `walk_det`, and `monitor` are not reset before replaying the 1.0s init buffer. | **Defect:** Accumulated states double-counted during replay. |
| 5 | Noise Estimator Median Bias | `src/noise_est.h:45-80` | `AdaptiveQEstimator` receives post-median gyro, reducing 2nd-difference variance by $0.485\times$ ($\approx 0.70\times$ on $\sigma$). | **Defect:** $\hat{\sigma}_g$ remains below $\sigma_{g,\text{base}} = 0.015$, locking $\kappa_q = 1.0$ permanently on 4 of 5 sequences! |
| 6 | Quadratic Double Scaling | `src/eskf.cpp:38-42` | $\sigma_g$ scaled by both `cur_q_gyro_mult` and `monitor.kappa()`. | **Defect:** Quadratic scaling $(\kappa_q \cdot \kappa(t))^2$. Fixed: `max(kappa_q, kappa(t))`. |
| 7 | Regime Mis-Switching | `src/updates.cpp:110-140` | Uses raw `vibe_ema > 0.40` and `ped_energy > 0.20`. Calm flights (`MH_01_easy`) fall below 0.40 and falsely trigger pedestrian prior ($v_z^W \approx 0, v_y^B \approx 0$). | **Defect:** Over-damped flight. Fixed: unify under `WalkingDetector::get_w_walk()`. |
| 8 | Tilt Update Starvation | `src/updates.cpp:150-180` | Fixed tilt gate $|\|\mathbf{a}\| - g| < 0.18\text{ m/s}^2$. Late noise grows $5\times$. | **Defect:** 0% tilt acceptance after $t = 45\text{ s}$! Fixed: noise-adaptive gate threshold. |
| 9 | Gap Constant $\boldsymbol{\omega}_s$ | `src/gap.cpp:120-160` | $\boldsymbol{\omega}_s$ held constant across all 400 sub-steps of a 2.0s gap. | **Defect:** Rotates attitude by $70^\circ..110^\circ$ into ground. Fixed: exponential decay $\boldsymbol{\omega}_{\text{sub}}(j) = \boldsymbol{\omega}_s \exp(-j \Delta t / \tau_{\omega,\text{gap}})$. |

---

## 3. Phase 1.2 Telemetry Print (Dev Seed 1 across 5 sequences)

Telemetry extracted via `scratch/telemetry_probe` compiled from `v2-dev`:

| Sequence | $\kappa_q$ ($t=5\text{s}$) | $\kappa(t)$ ($t=5\text{s}$) | Active Regime (% frames) | Tilt Applied ($t<20\text{s}$ vs $t>45\text{s}$) | Pre-Gap Error ($t=25\text{s}$) | Post-Gap Error ($t=27\text{s}$) | Blind Launch Error $(p, v, \text{yaw})$ | Total Blind 5s RMSE |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **MH_01_easy** | **1.0** (locked) | 1.78 | 96.6% Flight, 10.3% ZUPT | 46.9% $\to$ **0.0%** | $2.18\text{ m}$, $140.4^\circ$ | $2.84\text{ m}$, $133.1^\circ$ | $5.40\text{ m}$, $0.95\text{ m/s}$, $66.1^\circ$ | $4.33\text{ m}$ |
| **V1_01_easy** | **1.0** (locked) | 3.65 | 93.6% Flight, 7.3% ZUPT | 42.1% $\to$ **0.0%** | $0.93\text{ m}$, $154.3^\circ$ | $0.68\text{ m}$, $145.8^\circ$ | $1.99\text{ m}$, $0.42\text{ m/s}$, $61.6^\circ$ | $1.91\text{ m}$ |
| **V2_02_medium** | **1.0** (locked) | 4.23 | 98.9% Flight, 0.8% ZUPT | 42.6% $\to$ **0.0%** | $1.40\text{ m}$, $134.2^\circ$ | $1.75\text{ m}$, $113.5^\circ$ | $3.55\text{ m}$, $1.13\text{ m/s}$, $98.0^\circ$ | $2.77\text{ m}$ |
| **room1** | **1.0** (locked) | 1.00 | 73.1% Ped, 0% ZUPT | 38.6% $\to$ **0.0%** | $0.83\text{ m}$, $31.8^\circ$ | $0.29\text{ m}$, **$139.8^\circ$** | $0.41\text{ m}$, $1.53\text{ m/s}$, $36.1^\circ$ | $1.46\text{ m}$ |
| **room2** | **1.0** (locked) | 1.00 | 74.4% Ped, 0% ZUPT | 31.9% $\to$ **0.0%** | $1.85\text{ m}$, $154.0^\circ$ | $2.10\text{ m}$, $155.8^\circ$ | $0.60\text{ m}$, $0.71\text{ m/s}$, **$167.8^\circ$** | **$8.62\text{ m}$** |

### Key Telemetry Insights
1. **$\kappa_q = 1.0$ permanently across all 5 sequences:** `AdaptiveQEstimator` was starved by the median-3 variance attenuation bias.
2. **Tilt update starvation:** Tilt acceptance rate drops from $32\%..47\%$ early to exactly **0.0%** after $t = 45\text{ s}$ because sensor noise exceeds the fixed $0.18\text{ m/s}^2$ gate.
3. **Attitude shock across 2s gap:** In `room1`, attitude error jumped from $31.8^\circ$ pre-gap to $139.8^\circ$ post-gap because $\boldsymbol{\omega}_s$ was held constant across 400 sub-steps.
4. **Tail skew in Blind 5s:** `room2` Blind 5s error is $8.62\text{ m}$ (versus $1.46\text{ m}$ for `room1`) directly caused by a $167.8^\circ$ yaw misalignment at cutoff.

---

## 4. Wave 2 Parallel Subagent Dispatch

Four parallel git worktrees have been spawned from `v2-dev` (`c7cd88a`, includes `best-008` at `845f487`), and 4 independent worker subagents have been invoked:

```
v2-dev (c7cd88a) [best-008]
   ├── [exp/E66-unbiased-dual-adaptive-q]  --> Worker E66 (119dd3ea)
   ├── [exp/E67-gap-decay-and-anchor]      --> Worker E67 (65774f11)
   ├── [exp/E68-unified-regime-mount-tilt] --> Worker E68 (f24139fb)
   └── [exp/E69-init-and-spike-hardening]  --> Worker E69 (2852fb07)
```

Each worker is evaluating on Dev seeds `{1..10}`, verifying zero dynamic heap in the filter loop, and submitting reports to the Independent Reviewer (`5db90c77-7a7f-4ddc-890d-0300523ac231`) for confirmation on seeds `{201..210}` before merging into `best-009`.
