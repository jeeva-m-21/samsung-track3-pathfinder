# Samsung Hackathon — Track 3: Path Finder
## Presentation Deck & Pitch Script

**Team:** `VITC_SingleThread`  
**Track:** Track 3: Path Finder (IMU-Only Dead-Reckoning under Sensor Degradation)  
**System:** High-Performance, Zero-Dependency 15-State Error-State Kalman Filter (ESKF)

---

## Slide 1: Title & Overview
- **Title:** Path Finder: Robust IMU-Only Dead-Reckoning Under Heavy Degradation
- **Team Name:** `VITC_SingleThread`
- **Core Problem:** Estimate full 3D trajectory ($x, y, z, q$) using **pure 6-axis IMU only** (zero GPS, zero cameras, zero LiDAR, zero neural networks) under severe sensor degradation (spikes, bias walk, noise escalation, dropouts, and blackout cutoffs).
- **Key Breakthrough:** Achieving **`2.80 m` ATE** and **`4.88 m` Blind-5s error** across 50 multi-seed benchmarks in pure Python standard library at **14.5 ms/s** single-thread latency.

---

## Slide 2: The Core Technical Challenge
- **Double Integration Divergence:** In pure dead-reckoning, position is $\iint (a - b_a) dt^2$. An uncalibrated accelerometer bias of just $0.05\text{ m/s}^2$ integrates to $> 225\text{ meters}$ of position drift within 100 seconds.
- **Sensor Degradation Modes:**
  1. *Spurious Impulses*: Mechanical shock and synthetic glitch spikes.
  2. *Sensor Freeze & Missing Data*: 0.5s to 2.0s communication blackouts.
  3. *Rotational Tilt Corruption*: Turning creates lateral centripetal acceleration ($\boldsymbol{\omega} \times \mathbf{v}$) that corrupts gravity tilt alignment.
  4. *Terminal Blackout Cutoff*: Complete sensor loss in the final 5 seconds.

---

## Slide 3: System Architecture & Mathematical Foundations
- **15-State Manifold Formulation:**
  $$\mathbf{x} = [\mathbf{q}, \mathbf{v}, \mathbf{p}, \mathbf{b}_g, \mathbf{b}_a]^T \in \text{SO}(3) \times \mathbb{R}^{12}$$
- **Midpoint Strapdown Kinematics:** 2nd-order Runge-Kutta orientation integration with gravity removal.
- **Centripetal-Compensated Tilt Alignment:**
  $$\mathbf{a}_{\text{grav}} = (\mathbf{a}_{\text{meas}} - \mathbf{b}_a) - \boldsymbol{\omega} \times (R^T \hat{\mathbf{v}})$$
  *Analytically decouples turning acceleration from gravity reaction force, dropping pedestrian tracking error by -15.7%.*
- **Noise-Adaptive Wiener Process Noise ($Q$):** Scales filter covariance proportionally to real-time high-frequency vibration and estimated sensor noise.

---

## Slide 4: Multi-Domain Kinematic Priors
- **Micro Aerial Vehicles (Drones - `MH_01`, `V1_01`, `V2_02`):**
  - Aerodynamic quadrotor rotor drag damping ($k_d$).
  - Soft vertical flight velocity prior ($\sigma_{vz} = 1.20\text{ m/s}$) preventing vertical divergence during extended maneuvers.
  - Symmetrically capped gyro-derivative monitor ($n_{\text{gyr}}$) preventing agile flips from falsely triggering noise degradation.
- **Pedestrian & Handheld (`room1`, `room2`):**
  - Dual-axis non-holonomic velocity constraints (lateral and vertical velocity damping).
  - Gait-adaptive Weinberg stride bounding ($v_{\text{fwd}} \le 1.15 \cdot k \cdot (\Delta a_{pp})^{0.25}$) preventing forward runaway integration under accelerometer bias drift.

---

## Slide 5: Comprehensive Benchmark Results (50 Runs)

| Sequence | Metric | Baseline | VITC_SingleThread | Improvement |
| :--- | :--- | :---: | :---: | :---: |
| **`MH_01_easy`** (Drone) | ATE RMSE | 4.31 m | **3.57 m** | **-17.2%** |
| **`V1_01_easy`** (Drone) | ATE RMSE | 2.45 m | **1.77 m** | **-27.8%** |
| **`V2_02_medium`** (Agile) | Blind-5s RMSE | 31.50 m | **3.49 m** | **-88.9%** 🏆 |
| **`room1`** (Pedestrian) | ATE RMSE | 5.53 m | **4.43 m** | **-19.9%** |
| **`room2`** (Pedestrian) | Blind-5s RMSE | 5.98 m | **4.00 m** | **-33.1%** |
| **OVERALL (50 Runs)** | **ATE RMSE** | **5.79 m** | **`2.80 m`** | **-51.6%** 🏆 |
| **OVERALL (50 Runs)** | **Final 5s Blind** | **18.06 m** | **`4.88 m`** | **-73.0%** 🏆 |
| **OVERALL (50 Runs)** | **2s Gap Blackout** | **15.65 m** | **`5.00 m`** | **-68.1%** 🏆 |

---

## Slide 6: Resource Footprint & Engineering Rigor
- **Single-Core Latency:** **`14.54 ms` per 1.0 s of IMU** ($\le 20.0\text{ ms/s}$ competition limit, $>68\times$ real-time).
- **Peak Process Memory:** **`23.25 MB`** RSS ($\le 64.0\text{ MB}$ competition ceiling, $>63\%$ headroom).
- **Zero Third-Party Dependencies:** 100% Pure Python standard library (`math`, `struct`, `array`, `zipfile`, `sys`). Runs without `pip`, internet access, or external packages.
- **Clean-Room Test Verification:** 219/219 unit tests passing, 57/57 fuzz tests passing. Bit-for-bit verified package layout.

---

## Slide 7: Conclusion & Summary
- **Robustness Across Domains:** Proven out-of-distribution performance across both aerial drones and pedestrian walking telemetry.
- **Submission Ready:** Fully packaged standalone archive [VITC_SingleThread.zip](file:///home/jeeva/projects/samsung_hackathon/VITC_SingleThread.zip) verified against open-source `evo` evaluation suite.
