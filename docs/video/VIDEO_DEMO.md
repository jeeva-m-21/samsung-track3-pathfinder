# Video Demonstration & Walkthrough Guide

**Samsung Hackathon — Track 3: Path Finder (IMU-Only Dead-Reckoning)**  
**Team:** `VITC_SingleThread`

---

## 1. Video Overview

- **Title:** VITC_SingleThread: Robust IMU Dead-Reckoning Under Severe Sensor Degradation
- **Duration:** 2 Minutes 45 Seconds
- **Video Link:** `https://youtu.be/placeholder-vitc-singlethread-demo` *(or uploaded MP4 in presentation portal)*

---

## 2. Video Storyboard & Script Breakdown

### [0:00 - 0:30] Introduction & Problem Statement
- **Speaker:** "Welcome to our presentation for Samsung Hackathon Track 3: Path Finder. Our team is VITC_SingleThread."
- **Visual:** Display competition problem slide: 6-axis IMU only, zero vision, zero GPS, severe noise, dropouts, and blackouts.
- **Key Point:** Highlight why dead-reckoning diverges quadratically ($\frac{1}{2} a t^2$) and why standard filters blow out by hundreds of meters.

### [0:30 - 1:15] Technological Architecture
- **Speaker:** "To conquer this, we built an embedded 15-state Error-State Kalman Filter featuring two core breakthroughs."
- **Visual:** Screen capture showing filter architecture diagram:
  1. *Centripetal-Compensated Tilt Alignment*: Subtraction of $\boldsymbol{\omega} \times \mathbf{v}_B$ before gravity vector normalization.
  2. *Dual-Domain Priors*: Aerodynamic rotor drag damping for drones + gait-adaptive Weinberg stride bounding for pedestrians.
  3. *Noise-Adaptive Process Noise*: Real-time scaling of covariance based on high-frequency noise estimation.

### [1:15 - 2:05] Live Terminal Execution & Zero-Dependency Demo
- **Speaker:** "Our pipeline is written entirely in pure Python standard library with zero external dependencies."
- **Visual:** Live terminal recording running:
  ```bash
  # 1. Inspect bare environment (zero pip packages)
  python3 -m pip list || echo "Zero external packages installed"

  # 2. Run single-threaded execution on degraded input
  /usr/bin/time -v taskset -c 0 python3 VITC_SingleThread/main.py sample_input_degraded.zip estimated_trajectory.txt

  # 3. Verify evo evaluation
  evo_ape tum groundtruth.txt estimated_trajectory.txt -a
  ```
- **Showcase:** Execution finishes in $< 1.7\text{ seconds}$ with only $23.2\text{ MB}$ peak RSS.

### [2:05 - 2:45] 50-Run Benchmark Results & Conclusion
- **Visual:** Display summary table of 50 multi-seed runs:
  - Overall ATE: **`2.80 m`** (-51.6% reduction vs baseline)
  - Blind-5s Cutoff: **`4.88 m`** (-73.0% reduction)
  - 2s Gap Blackout: **`5.00 m`** (-68.1% reduction)
  - Single-Core Latency: **`14.5 ms/s`** (well below $20\text{ ms/s}$ limit).
- **Speaker:** "With zero heap churn, zero third-party dependencies, and verified multi-seed generalization, VITC_SingleThread delivers robust, production-grade inertial dead-reckoning."
