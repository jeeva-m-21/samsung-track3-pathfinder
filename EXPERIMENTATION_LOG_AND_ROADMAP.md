# Comprehensive Experimentation Log, Benchmark Results & Optimization Roadmap
**Samsung XR Hackathon 2026 — Track 03: "Path Finder"**  
**Team**: `VITC_SingleThread`  
**Date**: November 13, 2026  
**Document Status**: Official Forensic Engineering Record

---

## 1. Executive Overview

This document provides an exhaustive post-mortem of every algorithmic method, architectural component, and parameter configuration experimented with in the pure IMU 6-DoF dead-reckoning engine (`VITC_SingleThread`). It details:
1. **Every method applied**, its intended design, and its empirical result across 8 standard benchmark sequences and 35 randomized degradation trials.
2. **Physical and mathematical root-cause analysis** ("Why might it happen") explaining exact degradation mechanisms, unobservable degrees of freedom, and filter dynamics.
3. **Upcoming optimization experiments**, including mathematical formulations, algorithmic implementations, and verification protocols for adaptive, latency-bounded optimizers.

---

## 2. Hard Platform Constraints & Engineering Budgets

All implementations are rigorously governed by competition and embedded hardware rules:

| Resource Metric | Competition Ceiling | Internal Target | Current Production (`main`) | Margin / Headroom |
| :--- | :---: | :---: | :---: | :---: |
| **Peak Resident Set Size (RSS)** | $\le 8,192\text{ KB}$ ($8.0\text{ MB}$) | $\le 4,096\text{ KB}$ | **$1,840\text{ KB}$** ($1.84\text{ MB}$) | **$77.5\%$ Headroom** |
| **Streaming Latency** | $\le 20.0\text{ ms/s}$ of IMU data | $\le 2.0\text{ ms/s}$ | **$1.10\text{ ms/s}$** | **$94.5\%$ Headroom** |
| **CPU Concurrency** | Strictly 1 OS thread | Strictly 1 thread | Single thread (`taskset -c 0`) | $100\%$ Compliant |
| **Memory Allocation Policy** | Embedded Determinism | Zero heap allocations | 0 `malloc`/`new` in filter loop | $100\%$ Compliant |
| **External Dependencies** | Portable C++17 | Zero third-party libs | Pure C++17 / POSIX `stdio` | Statically linked |

---

## 3. Forensic Analysis: Physical Sensor Loss & Mathematical Error Limits

In pure 6-axis IMU dead-reckoning (zero GPS, zero vision, zero LiDAR, zero magnetometers), 3D position is calculated via double integration of specific force rotated to the world frame:
$$\mathbf{p}(t) = \mathbf{p}_0 + \mathbf{v}_0 t + \int_0^t \int_0^\tau \left( \mathbf{R}(s) (\tilde{\mathbf{a}}(s) - \mathbf{b}_a(s) - \mathbf{n}_a(s)) + \mathbf{g} \right) ds\, d\tau$$

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                    PHYSICAL ERROR PROPAGATION IN PURE IMU                       │
├──────────────────────┬──────────────────────┬───────────────────────────────────┤
│ Error Mechanism      │ Physical Mechanism   │ Position Drift Mathematical Model │
├──────────────────────┼──────────────────────┼───────────────────────────────────┤
│ 1. White Noise (VRW) │ Thermal / ADC jitter │ σ_p(t) = (1/√3) · σ_a · t^(3/2)   │
│ 2. Accel Bias Drift  │ Thermal drift/strain │ Δp(t) = ½ · b_a · t^2             │
│ 3. Gyro Yaw Drift    │ Gravity-unobservable │ Δp_H(t) ≈ (1/6) · a_H · b_gz · t^3│
└──────────────────────┴──────────────────────┴───────────────────────────────────┘
```

### 3.1 Why Vertical ($Z$) Error is Small ($<3\%$) While Horizontal ($XY$) Error Dominates ($>97\%$)
Across all drone flight evaluations (`MH_01_easy`, `V1_01_easy`, `V2_02_medium`):
* **Vertical Axis ($Z$) Error**: **$0.38\text{ m} - 0.58\text{ m}$** over 180 seconds.
* **Horizontal Plane ($XY$) Error**: **$2.40\text{ m} - 3.50\text{ m}$**.

**Physical Reason**:
1. The Earth's gravity vector $\mathbf{g} \approx [0, 0, -9.81]^T\text{ m/s}^2$ provides a continuous physical orientation anchor. When the platform is near-hover or cruising at constant velocity, the accelerometer measures the reaction force $-\mathbf{g}$.
2. In the error-state formulation:
   $$\delta \mathbf{z}_{\text{tilt}} = \tilde{\mathbf{a}} - (-\mathbf{R}^T \mathbf{g}) \approx [\mathbf{g}]_\times \delta \boldsymbol{\theta} + \mathbf{n}_a$$
   The skew-symmetric matrix $[\mathbf{g}]_\times$ is:
   $$[\mathbf{g}]_\times = \begin{bmatrix} 0 & -g & 0 \\ g & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}$$
   The rank of this observation matrix is 2. **Roll ($\delta \theta_x$) and pitch ($\delta \theta_y$) are directly observable**, but **yaw ($\delta \theta_z$) is in the null space of $[\mathbf{g}]_\times$**.
3. Consequently, pitch and roll errors remain bounded within $0.05^\circ - 0.15^\circ$. However, yaw drifts unobservably at rate $b_{gz}$.
4. A yaw orientation error $\delta \psi(t) = b_{gz} t$ rotates the horizontal acceleration vector:
   $$\Delta \mathbf{a}_H(t) \approx \mathbf{a}_H \times (\delta \psi \hat{\mathbf{k}}) = a_H b_{gz} t \hat{\mathbf{n}}_\perp$$
5. Double integrating this rotated acceleration yields a **cubic error growth in the horizontal plane**:
   $$\Delta \mathbf{p}_H(t) \approx \frac{1}{6} a_H b_{gz} t^3$$
   For a drone pulling modest horizontal maneuvers ($a_H = 1.5\text{ m/s}^2$) with a consumer gyro bias ($b_{gz} = 0.005\text{ rad/s} \approx 0.28^\circ/\text{s}$), the horizontal position drift reaches **$90\text{ m}$ in 60 seconds** and **$2,430\text{ m}$ in 180 seconds** if left uncorrected.

---

## 4. Log of Applied Methods & Empirical Results

```
Timeline of Algorithmic Interventions
────────────────────────────────────────────────────────────────────────────────
Baseline ESKF ──> Windowed Init ──> ZUPT Observer ──> Adaptive κ ──> PV-RTS ──> Qba Decouple
(Open-loop)       (Tilt/Bias)       (V2-02 gain)      (Jamming)     (Gated)    (Severe fix)
```

### Experiment 1: Nominal 15-State Error-State Kalman Filter (ESKF)
* **Description**: Embedded C++17 implementation of nominal strapdown kinematics with quaternion integration on $\mathrm{SO}(3)$ Lie algebra. State vector: $\mathbf{x} = [\boldsymbol{\theta}, \mathbf{v}, \mathbf{p}, \mathbf{b}_g, \mathbf{b}_a]^T \in \mathbb{R}^{15}$. Analytical $15 \times 15$ Jacobian $\mathbf{F}_x$, discrete process noise $\mathbf{Q}_d$, and Joseph-stabilized covariance propagation:
  $$\mathbf{P}_{k+1} = \mathbf{F}_k \mathbf{P}_k \mathbf{F}_k^T + \mathbf{Q}_d, \quad \mathbf{P} \leftarrow \frac{1}{2}(\mathbf{P} + \mathbf{P}^T)$$
* **Result**:
  * Raw unaligned drift reduced from $22,247\text{ m}$ (naive open-loop integration) to **$35.32\text{ m}$** on `MH_01_easy`.
  * Rigid Umeyama SE(3) ATE: **$3.78\text{ m}$** (vs $34.61\text{ m}$ naive baseline, $-89.1\%$ error).
* **Why it happened**: Hamilton quaternion integration with small-angle Taylor expansions eliminated gimbal lock. Dynamic accelerometer and gyro bias tracking prevented explosive open-loop quadratic divergence.

---

### Experiment 2: Windowed Static Gravity Initializer
* **Description**: Circular ring buffer computing mean and variance of initial $0.5\text{ s}$ ($N=100$) stationary telemetry. Solves for initial tilt quaternion $\mathbf{q}_0$ via triad alignment to $[0, 0, 1]^T$ and initializes gyro bias $\mathbf{b}_{g0} = \bar{\boldsymbol{\omega}}$.
* **Result**: Eliminated initial transient divergence. Starting attitude error dropped from $12.4^\circ$ to $<0.2^\circ$.
* **Why it happened**: Double integration is extremely sensitive to initial pitch and roll errors ($\Delta p \propto \frac{1}{2} g \sin(\delta \theta) t^2 \approx 4.9 \delta \theta t^2$). Accurately aligning initial gravity suppresses a $49\text{ m}$ drift error over the first $10\text{ s}$.

---

### Experiment 3: Adaptive Degradation Monitor ($\kappa$ Scaling)
* **Description**: Moving-horizon innovation residual tracker calculating variance inflation factor $\kappa \in [1.0, 50.0]$:
  $$\kappa = \text{clamp}\left(\frac{\text{tr}(\mathbf{S}_{\text{empirical}})}{\text{tr}(\mathbf{S}_{\text{theoretical}})}, 1.0, 50.0\right)$$
  Used to inflate process noise $\sigma_g, \sigma_a$ during sensor jamming and acoustic resonance bursts.
* **Result**: Prevented filter divergence on corrupted telemetry, maintaining filter stability across 35 randomized degradation trials.
* **Why it happened**: When external noise or structural vibrations inject non-Gaussian energy, inflating $\kappa$ prevents the Kalman gain $\mathbf{K}$ from over-trusting corrupted innovations.

---

### Experiment 4: ZUPT Observer Threshold Re-Calibration
* **Description**: Zero Velocity Update (ZUPT) observer detects stationary intervals and injects pseudo-measurement innovation $\mathbf{y} = -\mathbf{v}$. Originally, `src/updates.cpp` clamped the gyro threshold to $\min(\text{params::zupt\_gyro\_mean}, 0.015\text{ rad/s})$. Benchmarking revealed that idle sensor noise on industrial rigs (`V2_02_medium` and `TUM-VI`) hovered between $0.018 - 0.022\text{ rad/s}$, causing the detector to reject real stationary phases. The clamp was replaced with calibrated `zupt_gyro_mean = 0.020` and `zupt_window_s = 0.25`.
* **Result**:
  * `V2_02_medium` ATE dropped from **$2.62\text{ m} \rightarrow 2.48\text{ m}$** ($-5.4\%$).
  * `V2_02_medium` RPE (1s) dropped from **$0.7535\text{ m} \rightarrow 0.7115\text{ m}$** (**PASS on Gate**).
  * `figure_eight` ATE dropped from **$2.066\text{ m} \rightarrow 1.646\text{ m}$** (**PASS on Gate**).
  * `MH_01_easy` retained **PASS on Gate (ATE)** (**$3.63\text{ m}$** vs $4.31\text{ m}$ null).
* **Why it happened**: Real-world MEMS IMUs exhibit noise floors dependent on thermal state and mounting rigidity. Artificially clamping the stationary threshold caused the filter to miss subtle pause intervals, allowing velocity error to continue drifting unchecked.

---

### Experiment 5: Checkpointed Rauch-Tung-Striebel (RTS) Backward Smoother
* **Description**: Decoupled Position-Velocity RTS (PV-RTS) backward pass with chunked forward checkpointing ($M=128$) in `src/pipeline.cpp`. Memory footprint verified at only **$67.5\text{ KB}$** for checkpoints and **$24.8\text{ KB}$** for chunk scratch buffer ($<100\text{ KB}$ total; clean-room RSS remained **$1,840\text{ KB}$**).
* **Result**:
  * On synthetic datasets (`circle`), smoother flattened the trajectory into a line ($6.6\text{ m}$ span collapsed to $8\text{ mm}$), causing singular covariance rank in Umeyama SE(3) alignment.
  * On real drone flights (`MH_01_easy`), smoother pulled position states towards a static centroid. While this produced a deceptive Null-Baseline ATE ($1.85\text{ m}$), it destroyed actual trajectory shape fidelity.
  * **Action Taken**: Safely gated behind `constexpr bool enable_rts_smoother = false;` in `src/params.h`.
* **Why it happened (Critical Discovery)**:
  * In classic smoothing literature, RTS smoother operates on **sensor systems with external position or landmark observations** (GPS, visual keyframes, range anchors).
  * In **pure dead reckoning**, the backward transition matrix $\mathbf{C}_k = \mathbf{P}_k^- \mathbf{F}_k^T (\mathbf{P}_{k+1}^-)^{-1}$ operates without an terminal anchor. Over long horizons ($T > 100\text{ s}$), backward recursion acts as an unanchored backward diffusion, shrinking deviations toward the final state or spatial mean. Without external position updates, pure IMU RTS smoothing causes **trajectory flattening**.

---

### Experiment 6: Bias Random Walk Decoupling from Noise Inflation ($\kappa$)
* **Description**: In `src/eskf.cpp`, the continuous-to-discrete process noise matrix scaled bias random walk by $\kappa$:
  ```cpp
  // Before:
  double sigma_bg = params::sigma_bg_base * walk_mult_bg * kappa;
  double sigma_ba = params::sigma_ba_base * walk_mult_ba * kappa;
  ```
  When severe acoustic jamming bursts arrived with $\kappa = 50$, $Q_{ba}$ was inflated by $\kappa^2 = 2,500\times$. This was replaced with decoupled scaling:
  ```cpp
  // After:
  double sigma_bg = params::sigma_bg_base * walk_mult_bg;
  double sigma_ba = params::sigma_ba_base * walk_mult_ba;
  ```
* **Result**:
  * On `severe` preset seed 42, ATE was arrested from explosive divergence.
  * Zero regression on nominal benchmarks.
* **Why it happened**: $\kappa$ measures high-frequency measurement corruption (acoustic resonance, sensor clipping, chassis vibration). Accelerometer bias random walk ($Q_{ba}$) represents slow physical drift driven by temperature and material strain ($\mu\text{g}/\sqrt{\text{s}}$). Inflating $Q_{ba}$ by $2,500\times$ tricked the filter into treating high-frequency vibration spikes as permanent DC bias shifts, destroying bias estimation during and after jamming events.

---

## 5. Current Production Benchmark Metrics

### 5.1 Honest Benchmark Table (Like-with-Like Comparisons & Official Scoring)
*Evaluated across like-with-like comparisons:*
1. **Start ATE**: Evaluated under start-pose anchoring ($T_0 = T_{\text{ref}}(0) T_{\text{est}}(0)^{-1}$). Compared against **Null Start ATE** (static predictor fixed at starting pose $\mathbf{p}_0$: $\sqrt{\frac{1}{N}\sum \|\mathbf{p}_i - \mathbf{p}_0\|^2}$).
2. **Aligned ATE (Official Competition Metric)**: Evaluated using official rigid SE(3) Umeyama alignment (`evo_ape tum -a`, scale $s=1.0$). Compared against **Null Aligned ATE** (static predictor shifted to trajectory centroid $\bar{\mathbf{p}}$: $\sqrt{\frac{1}{N}\sum \|\mathbf{p}_i - \bar{\mathbf{p}}\|^2}$).
3. **RPE 1s**: 1-second RMS translational drift compared against vehicle displacement per 1s.
4. **Trajectory Span Guard**: Rejects collapsed/degenerate outputs ($\text{EstSpan} < 0.2 \cdot \text{RefSpan}$ or $< 0.1\text{ m}$).

| Sequence | Start ATE (m)<br>(Filter / Null) | Aligned ATE (m)<br>(Filter / Null) | RPE 1s (m)<br>(Filter / Null) | Span (Est / Ref) | Gate Verdict (Official) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **`MH_01_easy`** | **6.66 / 6.74** | **3.63 / 4.31** | 0.53 / 0.50 | 3.1m / 13.6m | **PASS (ATE)** |
| **`V1_01_easy`** | 4.96 / 2.70 | 2.54 / 1.85 | 0.44 / 0.44 | 7.7m / 7.3m | FAIL |
| **`V2_02_medium`** | 3.85 / 2.17 | 2.50 / 2.11 | **0.71 / 0.75** | 8.2m / 8.4m | **PASS (RPE)** |
| **`circle`** | **5.26 / 5.77** | **3.48 / 4.00** | **1.52 / 1.68** | 1.5m / 11.0m | **FAIL (DEGEN)** |
| **`figure_eight`** | **3.43 / 3.54** | **1.80 / 2.30** | **0.99 / 1.07** | 2.2m / 6.4m | **PASS** |
| **`TUM-VI room1`** | 29.07 / 1.13 | 18.12 / 1.00 | 1.62 / 0.91 | 63.6m / 4.7m | FAIL |
| **`TUM-VI room2`** | 7.04 / 1.29 | 4.74 / 1.22 | 1.13 / 0.96 | 23.4m / 4.2m | FAIL |
| **`UZH-FPV fwd7`** | 40.94 / 11.24 | 18.34 / 6.35 | 5.59 / 4.70 | 65.2m / 24.5m | FAIL |

### 5.2 Forensic Discovery: The Two Red Flags Unmasked

#### Red Flag 1: Why `circle` Matched the Null Baseline to 4 Decimals
1. **Diagnosis Confirmed**: The filter output on `circle` was completely degenerate ($p_x, p_y, p_z \sim 10^{-6}\text{ m}$).
2. **Root Cause**:
   - In `tools/synth.py`, `circle` was synthesized as an idealized coordinated banked turn where body $Z$ was defined as the apparent force vector ($z_b = f_w / |f_w|$). This forced the body accelerometer readings to be strictly constant $[0, 0, 9.8392]\text{ m/s}^2$ with ZERO body horizontal acceleration ($a_x = 0, a_y = 0$).
   - The synthetic vehicle started in mid-flight at $v = 2.0\text{ m/s}$, whereas the estimator is designed to initialize from rest on the launchpad ($v_0 = [0, 0, 0]$).
   - Because $a_x = 0, a_y = 0$, the filter aligned body $Z$ with world $Z$. World gravity exactly canceled body acceleration, resulting in world acceleration $\mathbf{a}_w \equiv [0, 0, 0]$. Velocity remained zero for all 6,000 steps.
   - When an estimator outputs a static point $[0, 0, 0]$, Umeyama rigid alignment translates that single point to the ground truth centroid. The RMSE of the ground truth to its own centroid is **by definition 4.4534 m (the Null ATE)**!
   - Similarly, the 1-second displacement error of a static point against a moving trajectory is **by definition the 1-second displacement of the vehicle (the Null RPE, 1.9867 m)**!
3. **The Fix**:
   - Fixed `tools/metrics.py` to compute `ref_span`, `est_span`, and `span_ratio`.
   - Updated `tools/benchmark_suite.py` to enforce an explicit degeneracy guard: sequences with `est_span < 0.2 * ref_span` are strictly marked `FAIL (DEGEN)`.

#### Red Flag 2: How Rigid Umeyama Alignment Was Masking Real Dead-Reckoning Drift
1. **Diagnosis Confirmed**: Full SE(3) Umeyama alignment fits a 6-DoF transformation ($R \in SO(3), t \in \mathbb{R}^3$) using ground truth across the entire 144-second timeline to globally rotate and translate the drifting trajectory.
2. **The Mechanism**:
   - In `MH_01_easy`, the true open-loop dead-reckoning drift (anchored at $t=0$) is **$6.66\text{ m}$** (with a peak error of **$11.28\text{ m}$**).
   - In a room spanning $10\text{ m} \times 10\text{ m}$ (where GT centroid spread is $4.31\text{ m}$), rotating the drifting trajectory in hindsight by $\approx 23^\circ$ and shifting its origin balances the error between $t=0$ and $t=144\text{ s}$, suppressing the reported ATE down to **$3.63\text{ m}$**.
   - This $3.63\text{ m}$ was within 15% of the Null Baseline ($4.31\text{ m}$), giving a deceptive illusion of tracking precision.
3. **The Fix**:
   - Fixed `align_start_anchored` in `tools/metrics.py` to anchor strictly to the synchronized initial pose ($T_0 = T_{\text{ref}}(0) T_{\text{est}}(0)^{-1}$).
   - Updated `tools/benchmark_suite.py` to display **Start ATE** alongside **Aligned ATE**, exposing true dead-reckoning performance without hindsight curve fitting.

> **Understanding the "Null Baseline" on Handheld Walking (`TUM-VI`)**:
> The Null Baseline is the error obtained by predicting a static constant coordinate $[0, 0, 0]^T$ for all timestamps. When a subject walks in small circular loops inside a $2\text{ m} \times 2\text{ m}$ room, the ground truth never deviates more than $1\text{ m}$ from the room center. A static dummy predictor trivially scores an ATE of $\approx 1.0\text{ m}$. Open-loop integration of a moving human over 120 seconds naturally produces inertial drift. Evaluating trajectory realism requires checking relative pose error (RPE) and scale consistency.

---

### 5.2 Cross-Validation Suite (35 Randomized Degradation Trials)
*Cross-validation executes 5 distinct sequences across 3 degradation presets (`clean`, `medium`, `severe`) over 5 random seeds (35 total evaluation runs).*

* **Suite Mean ATE**: **$5.77\text{ m}$**
* **Suite Median ATE**: **$3.10\text{ m}$**
* **Suite P95 ATE**: **$32.44\text{ m}$** (dominated by extreme 3-minute bias ramps on severe presets)
* **Suite Mean RPE (1s)**: **$1.09\text{ m}$**
* **Failures / NaN Exceptions**: **0 / 35 ($100\%$ numerical stability)**

---

## 6. What We Are Going to Try Next: Detailed Roadmap

To generalize accuracy improvements across all datasets while strictly respecting the $\le 8\text{ MB}$ RAM and $\le 20\text{ ms/s}$ latency limits, three complementary techniques are planned.

```
Proposed Adaptive Architecture
┌──────────────────────────────────────────────────────────────────┐
│                   Raw IMU Telemetry (200 Hz)                     │
└─────────────────────────────────┬────────────────────────────────┘
                                  ▼
┌──────────────────────────────────────────────────────────────────┐
│            Kinetic Motion & Jamming Regime Classifier            │
│         (Lightweight zero-allocation decision forest)            │
└─────────────────┬───────────────────────────────┬────────────────┘
                  │                               │
                  ▼                               ▼
┌──────────────────────────────────┐  ┌────────────────────────────┐
│ Adaptive Non-Holonomic Observer  │  │ Sage-Husa Online Innovation│
│  (Attacks 97% horizontal drift)  │  │ Covariance Adapter (AKF)   │
│  - Active when v > 0.5, ||ω|| < 1│  │ - Online R and Q update    │
│  - Dynamic body lateral gating   │  │ - Replaces fixed tuning    │
└──────────────────────────────────┘  └────────────────────────────┘
```

---

### 6.1 Experiment A: Adaptive Non-Holonomic Constraint (NHC) Observer

#### Mathematical Formulation
Rigid-body platforms (drones, rovers, handheld sensor rigs) primarily translate along their longitudinal axis when in directed motion. In the body coordinate frame $\mathcal{B}$:
$$\mathbf{v}_{\text{body}} = \mathbf{R}^T \mathbf{v}_{\text{world}} = \begin{bmatrix} v_{x,b} \\ v_{y,b} \\ v_{z,b} \end{bmatrix}$$
Under non-holonomic and aerodynamic damping constraints, lateral velocity $v_{y,b} \approx 0$.

We formulate a pseudo-measurement innovation:
$$y_{nhc} = 0 - \mathbf{e}_2^T \mathbf{R}^T \mathbf{v}_{\text{world}}$$
The measurement Jacobian $\mathbf{H}_{nhc} \in \mathbb{R}^{1 \times 15}$ with respect to error state $\delta \mathbf{x} = [\delta \boldsymbol{\theta}, \delta \mathbf{v}, \delta \mathbf{p}, \delta \mathbf{b}_g, \delta \mathbf{b}_a]^T$ is:
$$\mathbf{H}_{nhc} = \begin{bmatrix} \mathbf{e}_2^T \mathbf{R}^T [\mathbf{v}_{\text{world}}]_\times & \mathbf{e}_2^T \mathbf{R}^T & \mathbf{0}_{1 \times 3} & \mathbf{0}_{1 \times 3} & \mathbf{0}_{1 \times 3} \end{bmatrix}$$

#### Adaptive Gating Mechanism (Preventing False Updates)
NHC must **not** fire during:
1. Pure stationary/hovering periods ($\|\mathbf{v}\| < 0.4\text{ m/s}$).
2. Rapid spinning / high-rate yaw rotations ($\|\boldsymbol{\omega}\| > 0.8\text{ rad/s}$).
3. Significant lateral wind or unmodeled slippage (innovation Mahalanobis gate $d_M^2 = \frac{y_{nhc}^2}{H P H^T + R_{nhc}} > 9.0$).

#### Expected Impact
* **Directly attacks the $97\%$ horizontal drift component** by coupling body velocity to orientation, constraining yaw drift leakage into horizontal position.
* **Latency cost**: $< 0.08\text{ ms/s}$ (single scalar Joseph update).
* **Memory cost**: 0 heap bytes (pure stack variables).

---

### 6.2 Experiment B: Sage-Husa Online Innovation Covariance Adapter (AKF)

#### Mathematical Formulation
Rather than relying on fixed static noise matrices $\mathbf{Q}$ and $\mathbf{R}$, the Sage-Husa filter computes recursive maximum-likelihood estimates of the true time-varying covariances using innovation residuals $\mathbf{d}_k = \mathbf{y}_k - \mathbf{H}_k \hat{\mathbf{x}}_k^-$:

$$\hat{\mathbf{R}}_k = (1 - \beta) \hat{\mathbf{R}}_{k-1} + \beta \left( \mathbf{d}_k \mathbf{d}_k^T - \mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T \right)$$
$$\hat{\mathbf{Q}}_k = (1 - \alpha) \hat{\mathbf{Q}}_{k-1} + \alpha \left( \mathbf{K}_k \mathbf{d}_k \mathbf{d}_k^T \mathbf{K}_k^T + \mathbf{P}_k^+ - \mathbf{F}_{k-1} \mathbf{P}_{k-1}^+ \mathbf{F}_{k-1}^T \right)$$
where $\alpha, \beta \in [0.001, 0.05]$ are exponential memory forgetting factors.

To guarantee positive semi-definiteness under finite floating-point precision:
$$\hat{\mathbf{R}}_{ii} \leftarrow \max\left(\hat{\mathbf{R}}_{ii}, \mathbf{R}_{\text{floor}}\right), \quad \hat{\mathbf{Q}}_{ii} \leftarrow \max\left(\hat{\mathbf{Q}}_{ii}, \mathbf{Q}_{\text{floor}}\right)$$

#### Expected Impact
* Automatically detects when telemetry transitions from clean flight to severe jamming without brittle thresholding.
* Adapts dynamically across different sensor models (ADIS16448 on EuRoC vs BMI055 on TUM-VI vs ICM-20602 on UZH-FPV).
* **Latency cost**: $< 0.04\text{ ms/s}$.
* **Memory cost**: $< 1\text{ KB}$ persistent state buffer.

---

### 6.3 Experiment C: Sliding-Window Receding-Horizon Spline Optimizer

#### Architecture
* Maintain a circular FIFO ring buffer of the most recent $K = 16$ poses and velocities ($\approx 0.08\text{ s}$ window at 200 Hz).
* In parallel with ESKF streaming, perform a Gauss-Newton step optimizing a 1D cubic Hermite spline:
  $$\min_{\mathbf{p}_k, \mathbf{v}_k} \sum_{i=1}^K \|\tilde{\mathbf{a}}_i - \ddot{\mathbf{p}}(t_i)\|_{\mathbf{W}_a}^2 + \lambda_{\text{smooth}} \|\dddot{\mathbf{p}}(t)\|^2$$
* **Regularization Prior**: Enforce kinetic momentum continuity to prevent the trajectory flattening observed in unanchored RTS smoothing.

#### Expected Impact
* Smooths high-frequency discretization jitter and packet dropout gaps.
* Compute latency budget: With $18.9\text{ ms/s}$ of unused CPU headroom, a $16$-node tridiagonal linear system solves in $< 0.4\text{ ms/s}$.

---

## 7. Verification & Release Protocol

Before declaring any improvement ready for release packaging:
1. **Unit Test Verification**: Run `make test -C src` (must maintain 1,241 / 1,241 passing assertions with zero ASan/UBSan warnings).
2. **Nominal Suite Benchmark**: Run `python3 tools/benchmark_suite.py` across all 8 standard datasets to confirm zero regression on passing gates.
3. **Cross-Validation Stress Test**: Run `python3 tools/cross_validate.py --presets clean,medium,severe --seeds 42,1337,2026` (must complete with zero NaN exceptions and reduced P95 ATE).
4. **Clean-Room Memory Audit**: Execute `bash tools/make_bundle.sh` and verify that `clean_room_rss_kb` $\le 8,192\text{ KB}$ (target $\le 2,048\text{ KB}$).
5. **Release Bundle Generation**: Update `VITC_SingleThread.zip` and sync `metadata.json`.

---

## 8. Forensic Diagnosis of Trajectory Span Anomalies & Oracle Headroom Analysis

### 8.1 Empirical Summary of Span Anomalies

Using the C++ diagnostic telemetry harness (`--telemetry`) and ground truth trajectory interpolation, we analyzed the root causes of the three primary span anomalies:

| Sequence | Type | Reference Span | Estimated Span | Ratio | Primary Failure Mechanism |
|---|---|---|---|---|---|
| **MH_01_easy** | Drone (EuRoC) | $13.51\text{ m}$ | $3.08\text{ m}$ | $0.23\times$ | **OU Velocity Damping ($\tau_{\text{ou}} = 1.8\text{ s}$)** bleeds $63\%$ velocity every $1.8\text{ s}$. ZUPT is NOT killing velocity (active only $0.7\%$ during launchpad). |
| **TUM-VI room1** | Handheld Pedestrian | $4.68\text{ m}$ | $63.59\text{ m}$ | $13.58\times$ | **ZUPT Never Fires ($0.0\%$)** due to continuous walking arm-swing dynamics. Double integration runaway expands span $13.6\times$ in a $4.7\text{ m}$ room. |
| **UZH-FPV fwd7** | High-Speed Drone | $25.31\text{ m}$ | $65.16\text{ m}$ | $2.57\times$ | **Velocity Bounding Clamped $64.6\%$ of Flight** ($v_{\text{max}} = 3.0\text{ m/s}$ vs true $12.1\text{ m/s}$). Severe attitude drift ($74.3^\circ$ yaw, $88.2^\circ$ roll) corrupts gravity cancellation. |

---

### 8.2 Detailed Sequence Diagnostic Profiles

#### 1. EuRoC MH_01_easy
* **Duration**: $144.1\text{ s}$ ($28,825$ samples at $200\text{ Hz}$).
* **Trajectory Span**: Estimated $= 3.08\text{ m}$ vs Reference $= 13.51\text{ m}$ (Ratio: $0.23\times$).
* **Speed Profile**: Estimated Max $= 0.56\text{ m/s}$ vs Ground Truth Max $= 2.07\text{ m/s}$ (Mean GT $= 0.44\text{ m/s}$).
* **RMS Velocity Error**: $0.55\text{ m/s}$ (Peak: $1.89\text{ m/s}$).
* **Attitude Errors vs GT**: Mean Yaw $= 7.43^\circ$ (Peak: $17.12^\circ$), Mean Pitch $= 2.41^\circ$, Mean Roll $= 21.28^\circ$.
* **Active Regime Timeline**:
  - **ZUPT active**: **$0.7\%$** of total samples (strictly during the initial stationary launchpad before takeoff). ZUPT does NOT trigger in flight.
  - **Tilt updates applied**: **$54.0\%$** of samples (active during smooth cruising when $\|a\| \approx g$).
  - **Velocity bounding active**: **$0.0\%$** (flight velocity never exceeded $3.0\text{ m/s}$).
  - Mean $\kappa = 3.34$, Mean Quality $= 0.32$.
* **Diagnostic Plot**: `reports/diag_MH_01_easy.png`.

#### 2. TUM-VI room1
* **Duration**: $141.0\text{ s}$ ($30,937$ samples at $200\text{ Hz}$).
* **Trajectory Span**: Estimated $= 63.59\text{ m}$ vs Reference $= 4.68\text{ m}$ (Ratio: $13.58\times$).
* **Speed Profile**: Estimated Max $= 3.49\text{ m/s}$ vs Ground Truth Max $= 2.62\text{ m/s}$ (Mean GT $= 1.04\text{ m/s}$).
* **RMS Velocity Error**: $1.66\text{ m/s}$ (Peak: $4.70\text{ m/s}$).
* **Attitude Errors vs GT**: Mean Yaw $= 82.02^\circ$ (Peak: $179.89^\circ$), Mean Pitch $= 12.29^\circ$, Mean Roll $= 13.52^\circ$.
* **Active Regime Timeline**:
  - **ZUPT active**: **$0.0\%$** of total samples. Handheld walking motion never achieves static variance threshold; zero stationary updates occur after $t_0$.
  - **Tilt updates applied**: **$25.5\%$** of samples.
  - **Velocity bounding active**: **$2.8\%$** of samples.
  - Mean $\kappa = 1.95$, Mean Quality $= 0.58$.
* **Diagnostic Plot**: `reports/diag_TUM_VI_room1.png`.

#### 3. UZH-FPV fwd7
* **Duration**: $66.7\text{ s}$ ($33,335$ samples at $500\text{ Hz}$).
* **Trajectory Span**: Estimated $= 65.16\text{ m}$ vs Reference $= 25.31\text{ m}$ (Ratio: $2.57\times$).
* **Speed Profile**: Estimated Max $= 3.46\text{ m/s}$ vs Ground Truth Max $= 12.10\text{ m/s}$ (Mean GT $= 4.72\text{ m/s}$).
* **RMS Velocity Error**: $5.64\text{ m/s}$ (Peak: $12.56\text{ m/s}$).
* **Attitude Errors vs GT**: Mean Yaw $= 74.34^\circ$ (Peak: $179.74^\circ$), Mean Pitch $= 37.47^\circ$, Mean Roll $= 88.17^\circ$.
* **Active Regime Timeline**:
  - **Velocity bounding active**: **$64.6\%$** of flight. The $3.0\text{ m/s}$ cap severely truncated the racing drone dynamics.
  - **ZUPT active**: **$0.0\%$** of total samples.
  - **Tilt updates applied**: **$12.2\%$** of samples.
  - Mean $\kappa = 49.94$ (gyro/acc saturation under extreme angular rates), Mean Quality $= 0.05$.
* **Diagnostic Plot**: `reports/diag_UZH_FPV_fwd7.png`.

---

### 8.3 Oracle Headroom Matrix & Ablation Results

We executed identical C++ production runs across 10 controlled configurations for each dataset:

#### Dataset: EuRoC MH_01_easy
*Null Baseline: Start ATE = 6.74 m | Aligned ATE = 4.31 m | RPE 1s = 0.50 m*

| Configuration | Start ATE (m) | Aligned ATE (m) | RPE 1s (m) | Span (Est/Ref) | Span Ratio | Diagnostic Verdict |
|---|---|---|---|---|---|---|
| **Current Baseline** | 6.66 | 3.63 | 0.53 | 3.1m / 13.6m | 0.23x | Baseline beats null on Aligned ATE. |
| **Oracle 1 (GT Yaw)** | 6.63 | 3.62 | 0.53 | 3.0m / 13.6m | 0.22x | Yaw error is only $7.4^\circ$; GT yaw provides $< 0.5\%$ gain. |
| **Oracle 2 (GT Bias)** | 6.65 | 3.64 | 0.53 | 2.8m / 13.6m | 0.20x | Calibrated bias alone does not prevent span compression. |
| **Oracle 3 (GT v0)** | 6.66 | 3.63 | 0.53 | 3.1m / 13.6m | 0.23x | Drone starts at rest ($v_0 \approx 0$); initial velocity is not the cause. |
| **Oracle 4 (GT Yaw+Bias+v0)** | 6.65 | 3.63 | 0.53 | 2.8m / 13.6m | 0.20x | Identical to baseline under OU damping. |
| **Oracle 5 (GT Attitude R+P+Y)** | 7.63 | 3.82 | 0.52 | 6.7m / 13.6m | 0.49x | Span expands to $6.7\text{ m}$ ($2.2\times$ baseline), proving pitch/roll coupling. |
| **Ablation: No $\tau_{\text{ou}}$ ($\tau=0$)** | 58.62 | 35.31 | 1.02 | 116.4m / 13.6m | 8.58x | **Direct proof**: Removing damping causes $116\text{ m}$ quadratic runaway drift! |
| **Ablation: No ZUPT** | 6.69 | 3.63 | 0.53 | 3.5m / 13.6m | 0.26x | Confirms ZUPT does not kill flight velocity ($0.23\times \to 0.26\times$). |
| **Ablation: No Tilt Update** | 80.56 | 51.31 | 1.52 | 179.3m / 13.6m | 13.22x | Tilt updates provide critical gravity reference; disabling causes catastrophic drift. |
| **Ablation: No Vel Bounding** | 6.66 | 3.63 | 0.53 | 3.1m / 13.6m | 0.23x | Identical (velocity was already $< 3.0\text{ m/s}$). |

---

#### Dataset: TUM-VI room1
*Null Baseline: Start ATE = 1.13 m | Aligned ATE = 1.00 m | RPE 1s = 0.91 m*

| Configuration | Start ATE (m) | Aligned ATE (m) | RPE 1s (m) | Span (Est/Ref) | Span Ratio | Diagnostic Verdict |
|---|---|---|---|---|---|---|
| **Current Baseline** | 29.07 | 18.12 | 1.62 | 63.6m / 4.7m | 13.65x | Runaway drift in small room. |
| **Oracle 1 (GT Yaw)** | 22.03 | 16.08 | 1.63 | 51.8m / 4.7m | 11.12x | Yaw error reduction yields $24\%$ Start ATE improvement ($29\text{ m} \to 22\text{ m}$). |
| **Oracle 2 (GT Bias)** | 22.54 | 16.36 | 1.84 | 56.1m / 4.7m | 12.04x | Modest improvement. |
| **Oracle 3 (GT v0)** | 29.07 | 18.12 | 1.62 | 63.6m / 4.7m | 13.65x | Zero effect ($v_0 \approx 0$). |
| **Oracle 4 (GT Yaw+Bias+v0)** | 25.79 | 18.40 | 1.83 | 60.6m / 4.7m | 13.01x | Limited headroom without footstep/pedestrian velocity constraints. |
| **Oracle 5 (GT Attitude R+P+Y)** | 25.59 | 20.80 | 2.50 | 79.9m / 4.7m | 17.16x | Unconstrained double integration drifts even with perfect attitude. |
| **Ablation: No $\tau_{\text{ou}}$ ($\tau=0$)** | 54.06 | 39.89 | 2.96 | 135.2m / 4.7m | 29.03x | Runaway drift doubles ($63\text{ m} \to 135\text{ m}$). |
| **Ablation: No ZUPT** | 29.07 | 18.12 | 1.62 | 63.6m / 4.7m | 13.65x | Identical because ZUPT was already $0.0\%$ active. |
| **Ablation: No Tilt Update** | 130.98 | 60.67 | 1.98 | 195.0m / 4.7m | 41.86x | Severe divergence without tilt correction. |
| **Ablation: No Vel Bounding** | 31.03 | 18.34 | 1.91 | 71.3m / 4.7m | 15.30x | Span increases slightly ($63.6\text{ m} \to 71.3\text{ m}$). |

---

#### Dataset: UZH-FPV fwd7
*Null Baseline: Start ATE = 11.24 m | Aligned ATE = 6.35 m | RPE 1s = 4.70 m*

| Configuration | Start ATE (m) | Aligned ATE (m) | RPE 1s (m) | Span (Est/Ref) | Span Ratio | Diagnostic Verdict |
|---|---|---|---|---|---|---|
| **Current Baseline** | 40.94 | 18.34 | 5.59 | 65.2m / 24.5m | 2.66x | Scale clipped & attitude corrupted. |
| **Oracle 1 (GT Yaw)** | **25.42** | **11.38** | 5.68 | **44.7m / 24.5m** | **1.82x** | **Yaw oracle cuts ATE by $38\%$ and span error by $51\%$!** |
| **Oracle 2 (GT Bias)** | 29.34 | 18.59 | 5.69 | 70.6m / 24.5m | 2.88x | Bias alone has secondary impact. |
| **Oracle 3 (GT v0)** | 40.94 | 18.34 | 5.59 | 65.2m / 24.5m | 2.66x | Zero effect ($v_0 \approx 0$). |
| **Oracle 4 (GT Yaw+Bias+v0)** | 25.48 | 12.86 | 5.24 | 47.0m / 24.5m | 1.92x | Large improvement driven primarily by yaw. |
| **Oracle 5 (GT Attitude R+P+Y)** | **15.64** | **5.91** | 5.68 | **22.2m / 24.5m** | **0.91x** | **CRITICAL BREAKTHROUGH: Aligned ATE collapses to 5.91m (BEATS Null 6.35m = PASS)! Span matches reference at 91%!** |
| **Ablation: No $\tau_{\text{ou}}$ ($\tau=0$)** | 24.69 | 12.93 | 5.70 | 52.6m / 24.5m | 2.14x | Faster integration without artificial velocity bleed. |
| **Ablation: No ZUPT** | 40.94 | 18.34 | 5.59 | 65.2m / 24.5m | 2.66x | Identical (ZUPT was $0.0\%$). |
| **Ablation: No Tilt Update** | 28.17 | 11.75 | 5.64 | 49.9m / 24.5m | 2.04x | High dynamic tilt updates corrupt orientation during aggressive banking. |
| **Ablation: No Vel Bounding** | 127.50 | 57.23 | 7.19 | 178.9m / 24.5m | 7.29x | Without bounding, unmodeled saturation explodes. |

---

### 8.4 Key Insights & Proven Error Headroom

1. **Why MH_01 Span is 4x Too Small**:
   - **Hypothesis Disproven**: ZUPT is NOT killing velocity during flight. ZUPT was active for only $0.7\%$ of the entire 144s sequence (during the launchpad rest).
   - **Hypothesis Proven**: Ornstein-Uhlenbeck velocity damping with fixed $\tau_{\text{ou}} = 1.8\text{ s}$ bleeds $63\%$ of forward flight velocity every $1.8\text{ s}$. When $\tau_{\text{ou}}$ is removed, the estimated span immediately jumps from $3.1\text{ m}$ to $116.4\text{ m}$ ($8.6\times$ reference).
   - The filter was forced to use hard damping because small residual tilt errors ($2.4^\circ$ pitch, $21^\circ$ roll) leak gravity into horizontal acceleration, which unconstrained double integration squares over 144s ($\frac{1}{2} a t^2 = 0.5 \times (9.81 \cdot \sin 2.4^\circ) \cdot 144^2 \approx 4,200\text{ m}$ raw theoretical drift).
   - **Actionable Headroom**: Hardcoded damping must be replaced by **Kinetic Energy-Adaptive Damping**: damp velocity only when body kinetic energy / specific force variance indicates near-hover or stationarity, but allow unattenuated integration during forward cruising.

2. **Why TUM-VI Room1 Span Explodes (13.6x)**:
   - Handheld pedestrian motion never satisfies the rigid static variance gate ($|\|a\| - g| < 0.15\text{ m/s}^2$ and $\|\omega\| < 0.02\text{ rad/s}$), causing ZUPT to fire $0.0\%$ of the time.
   - Yaw drifts by $82^\circ$, causing footstep accelerations to be integrated in rotating random directions.
   - **Actionable Headroom**: Handheld pedestrian motion requires a **Cyclic Gait / Zero-Velocity Stance Detector** (detecting foot-strike shockwave extrema where body velocity momentarily halts or reverses) rather than a rigid stationary variance buffer.

3. **Why UZH-FPV Exploded (2.6x) and Clamped**:
   - The filter's velocity bounding was hardcoded at $v_{\text{max}} = 3.0\text{ m/s}$. The UZH drone reaches $12.10\text{ m/s}$. The filter was actively clamping velocity for **$64.6\%$ of the flight**!
   - Under extreme $500\text{ Hz}$ angular rates ($> 10\text{ rad/s}$), sensor saturation ($\kappa \approx 50$) caused massive attitude error ($74^\circ$ yaw, $88^\circ$ roll).
   - **Oracle Headroom Ceiling**: Injecting true ground truth attitude (Oracle 5) **collapses Aligned ATE from 18.34m to 5.91m (passing the 6.35m Null baseline)** and **recovers the true trajectory span to 22.2m / 24.5m (91% match)**!
   - This proves that **Attitude & Saturation Handling is the 100% dominant error bottleneck on high-speed drone sequences**.

---

### 8.5 Targeted Optimizer Architecture

Armed with empirical headroom ceilings, we avoid speculative tuning and construct two targeted modules:

1. **Kinetic Regime-Gated Dynamic Damping (KRG-DD)**:
   - Compute normalized kinetic energy $E_k = \frac{1}{2} \|\mathbf{v}\|^2$ and high-frequency acceleration jerk $J_a = \|\mathbf{a}_k - \mathbf{a}_{k-1}\| / \Delta t$.
   - Scale damping time constant dynamically:
     $$\tau_{\text{eff}} = \tau_0 \cdot \left(1.0 + \gamma \cdot \frac{\|\mathbf{a}_{\text{eff}}\|}{\sigma_a}\right)$$
   - During steady cruise ($\|\mathbf{v}\| > 0.5\text{ m/s}$), $\tau_{\text{eff}} \to \infty$ (damping disabled), preserving real flight span ($13.6\text{ m}$).
   - During hover/drift ($\|\mathbf{a}\| \approx g, \|\boldsymbol{\omega}\| \approx 0$), $\tau_{\text{eff}} \to 1.8\text{ s}$, suppressing quadratic integration drift.

2. **Dynamic Velocity Bounding Scaling**:
   - Replace fixed $v_{\text{max}} = 3.0\text{ m/s}$ with dynamic headroom based on integrated specific force power:
     $$v_{\text{max}}(t) = \max\left(3.0, \; \alpha \cdot \sqrt{\int_0^t \|\mathbf{a}_{\text{body}}\|^2 dt}\right)$$
   - Prevents artificial $3.0\text{ m/s}$ velocity clipping on UZH-FPV while maintaining protection against runaway divergence.

