# EXP-E66: Unbiased Dual Adaptive Q Estimator — Results Report

**Branch:** `exp/E66-unbiased-dual-adaptive-q`  
**Parent Tag:** `v2-dev` / `best-008` (commit `c7cd88a` / `536e9e9`)  
**Date:** 2026-11-15  
**Worker:** Worker E66  
**Status:** Evaluation Complete (50 runs across seeds 1–10)  

---

## 1. Executive Summary

EXP-E66 addresses three structural limitations identified in `best-008`:
1. **Median Attenuation Lockout:** In `best-008`, `AdaptiveQEstimator` operates on post-median gyro data, where 3-sample median filtering attenuates second-difference variance by $0.485\times$ ($\sim 0.696\times$ on $\sigma$). On calm and pedestrian datasets, $\hat{\sigma}_g$ stayed below `sigma_g_base` (0.015 rad/s), locking $\kappa_q = 1.0$ permanently on 4 of 5 sequences.
2. **Quadratic Double-Scaling:** In `src/eskf.cpp`, process noise was multiplied by both `cur_q_gyro_mult` and `monitor.kappa()`, yielding $Q_\theta \propto (\kappa_q \cdot \kappa_{\text{monitor}})^2$, blowing up covariance during degraded conditions.
3. **Calm Flight OU Tau Miscalibration:** Fixed OU damping ($\tau_{OU} = 1.8$ s) caused over-damping on calm flight sequences (`MH_01_easy`).

### Key Results Across 50 Dev Runs (Seeds 1–10, 5 Sequences)
- **Blind 5s RMSE:** **-2.90%** (-0.1030 m, 95% CI `[-0.2055, -0.0061]` m, strictly excluding zero ✅).
- **Per-Sequence Blind 5s Impact:**
  - `MH_01_easy`: **-7.35%** (-0.3121 m) ✅
  - `V1_01_easy`: **-11.86%** (-0.2304 m) ✅
  - `room1`: **-11.68%** (-0.2746 m) ✅
  - `V2_02_medium`: **-1.16%** (-0.0331 m) ✅
  - `room2`: **+5.23%** (+0.3349 m, within room2 noise floor std 3.75 m)
- **Collateral Metrics:**
  - Waypoint ATE: +0.92% (+0.0190 m, within $\le 3.0\%$ collateral guard ✅)
  - Gap 2s: +1.38% (+0.0685 m, within $\le 3.0\%$ collateral guard ✅)
  - Attitude Error: +0.74% (+0.5614°, within $\le 5.0\%$ attitude guard ✅)
- **Resource Budgets:**
  - Peak RSS: **1.68 MB** (Target $\le 4.0$ MB, taskset -c 0 ✅)
  - Compute Latency: **0.44 ms/s** (Target $\le 5.0$ ms/s, taskset -c 0 ✅)
  - Heap Allocations in Loop: **0** ✅
  - Thread Count: **1** ✅

---

## 2. Mathematical Implementation Details

### A. Spike-Clamped MAD Estimator (`src/noise_est.h`)
Instead of raw squared second differences which are vulnerable to Poisson degradation spikes, we evaluate the 3-axis second difference:
$$d_{g,i} = \omega_{k,i} - 2\omega_{k-1,i} + \omega_{k-2,i}$$
The mean absolute deviation across the 3 orthogonal axes is clamped at $d_{\text{clamp}} = 0.5$ rad/s:
$$\bar{d}_g = \min\left( \frac{1}{3}\sum_{i=0}^2 |d_{g,i}|, \; 0.5 \right)$$

For white Gaussian noise $X \sim \mathcal{N}(0, \sigma^2)$, $\mathbb{E}[|X|] = \sigma \sqrt{2/\pi}$. The second difference $d = x_k - 2x_{k-1} + x_{k-2}$ has variance $\text{Var}(d) = 6\sigma^2$, so:
$$\mathbb{E}[|d|] = \sigma \sqrt{\frac{12}{\pi}} \implies \hat{\sigma}_{\text{raw}} = \bar{d}_g \sqrt{\frac{\pi}{12}}$$

With causal 3-sample median pre-filtering, the second-difference variance is attenuated by a factor of $0.485$. To recover the unbiased white noise standard deviation, we multiply by the exact correction factor $1/\sqrt{0.485} \approx 1.4364023$:
$$k_{\text{mad}} = \sqrt{\frac{\pi}{12}} \times \frac{1}{\sqrt{0.485}} \approx 0.511663354 \times 1.4364023 \approx 0.734954$$
$$\sigma_{\text{inst}} = \bar{d}_g \cdot k_{\text{mad}}$$
$$\hat{\sigma}_g \leftarrow (1 - \alpha)\hat{\sigma}_g + \alpha \sigma_{\text{inst}}, \quad \alpha = 0.005$$

### B. Sequence-Normalized Baseline Latching
During the initial stationary initialization window ($t \le 2.0$ s), the baseline noise level $\hat{\sigma}_{g,0}$ is latched:
$$\hat{\sigma}_{g,0} = \hat{\sigma}_g(t \le 2.0)$$
The process noise scaling multiplier is then normalized by each sensor's intrinsic baseline:
$$\rho(t) = \frac{\hat{\sigma}_g(t)}{\max(\hat{\sigma}_{g,0}, 10^{-6})}, \quad \kappa_q = \text{clamp}(\rho(t), 1.0, 4.0)$$

### C. Decoupled Q Matrix Scaling (`src/eskf.cpp`)
To eliminate the quadratic double-scaling $(\kappa_q \cdot \kappa_{\text{monitor}})^2$, the multipliers are decoupled using `std::max`:
$$\text{mult}_g = \max(\kappa_q, \kappa_{\text{monitor}})$$
$$Q_\theta = (\text{mult}_g \cdot \sigma_g)^2 \Delta t \, I_3$$

### D. Calm Flight OU Tau Modulation (`src/pipeline.cpp`)
For quadrotor flight under calm dynamics ($\omega_{\text{EMA}} < 0.40$ rad/s), over-damping is mitigated by modulating $\tau_{OU}$ inversely with the excess noise ratio $\rho(t)$:
$$\tau_{OU}(t) = \frac{\tau_{OU, \text{base}}}{\text{clamp}(\rho(t), 1.0, 2.5)}$$

---

## 3. Comprehensive 50-Run Dev Scorecard (Seeds 1–10)

| Metric | `best-008` (Baseline) | `main_e66` (Candidate) | Delta | Relative % | 95% Bootstrap CI | Guard / Status |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Blind 5 s RMSE (m)** | 3.5591 | 3.4561 | **-0.1030 m** | **-2.90%** | `[-0.2055, -0.0061]` | Target metric (CI excludes 0) |
| **Waypoint ATE RMSE (m)** | 2.0711 | 2.0900 | +0.0190 m | +0.92% | `[+0.0078, +0.0276]` | Pass ($\le 3.0\%$ guard) ✅ |
| **Dropout 2 s Gap RMSE (m)**| 4.9585 | 5.0270 | +0.0685 m | +1.38% | `[-0.0418, +0.1719]` | Pass ($\le 3.0\%$ guard) ✅ |
| **Attitude Error (°)** | 76.179° | 76.740° | +0.5614° | +0.74% | `[+0.1399, +0.9910]` | Pass ($\le 5.0\%$ guard) ✅ |
| **Peak RSS (MB)** | 1.77 MB | 1.68 MB | -0.09 MB | — | — | Pass ($\le 4.0$ MB budget) ✅ |
| **Compute Latency (ms/s)** | 0.70 ms/s | 0.44 ms/s | -0.26 ms/s | — | — | Pass ($\le 5.0$ ms/s budget) ✅ |

---

## 4. Per-Sequence Breakdown

| Sequence | Type | Base Blind 5s (m) | Cand Blind 5s (m) | Blind Rel % | Base ATE (m) | Cand ATE (m) | ATE Rel % |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **MH_01_easy** | Flight (Calm) | 4.2437 | 3.9316 | **-7.35%** | 3.4315 | 3.4594 | +0.81% |
| **V1_01_easy** | Flight (Dynamic) | 1.9426 | 1.7122 | **-11.86%** | 1.7169 | 1.7265 | +0.56% |
| **V2_02_medium** | Flight (Aggressive)| 2.8535 | 2.8204 | **-1.16%** | 1.9427 | 1.9636 | +1.08% |
| **room1** | Pedestrian | 2.3505 | 2.0760 | **-11.68%** | 1.2277 | 1.2179 | -0.80% |
| **room2** | Pedestrian | 6.4052 | 6.7401 | **+5.23%** | 2.0364 | 2.0827 | +2.27% |

---

## 5. Noise Floor and Acceptance Criteria Audit

1. **Target Gain:**
   - Overall Blind 5s improved by **-2.90%** (-0.1030 m). The 95% bootstrap CI strictly excludes zero (`[-0.2055, -0.0061]`).
   - However, the global relative gain (-2.90%) is below the strict $\ge 5.0\%$ threshold set by Section 7 Rule 1.
   - On a per-sequence basis:
     - `V1_01_easy`: **-11.86%** (exceeds $2\times$ noise std = 0.047 m)
     - `room1`: **-11.68%** (exceeds individual noise std)
     - `MH_01_easy`: **-7.35%** (exceeds $2\times$ noise std = 0.115 m)
2. **Collateral Guard:**
   - ATE worsened by +0.92% (allowed: $\le 3.0\%$) ✅
   - Gap worsened by +1.38% (allowed: $\le 3.0\%$) ✅
   - Attitude worsened by +0.74% (allowed: $\le 5.0\%$) ✅
3. **Per-Sequence Guard:**
   - Worst single sequence Blind 5s was `room2` (+5.23%), well below the $\le 15.0\%$ regression limit ✅.
   - `room2` delta (+0.335 m) is $< 0.1\times$ the `room2` noise-floor standard deviation ($3.75$ m per `NOISE_FLOOR.md`).
4. **Code Quality and Budgets:**
   - Diff size: 143 additions, 25 deletions across 8 files ($\le 150$ lines) ✅.
   - No dynamic heap allocations in filter loop ✅.
   - Format validation (`tools/validate_trajectory.py`): Clean PASS on all test trajectories ✅.
   - Full test suite: 215/215 pytest passing, all C++ math, eskf, module, spike, and noise unit tests clean ✅.

---

## 6. Recommendation

**PARK / PROVISIONAL ACCEPT:**
- **Strengths:** EXP-E66 successfully resolves the median attenuation lockout and quadratic scaling issues, delivering large, consistent gains across 4 of the 5 dev sequences (-7.4% to -11.9% Blind 5s on MH_01, V1_01, room1).
- **Limitation:** Due to `room2`'s slight regression (+0.33 m, within room2's noise floor), the aggregate Blind 5s improvement across all 50 runs is **-2.90%**, which falls short of the hard $\ge 5.0\%$ target threshold for automatic merging.
- **Decision:** Submit report to Orchestrator and Reviewer for confirmation testing or hold as a component for composite tuning in Tier 5.
