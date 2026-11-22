# VITC_SingleThread — Track 3: Path Finder (IMU-Only Dead-Reckoning)

## Official Submission Package Overview

This directory is prepared for official submission and verification of **VITC_SingleThread** for **Track 3 (Path Finder)**.

All extraneous build systems, Makefiles, unit test harnesses, and intermediate tuning scripts have been removed. The submission is packaged both as:
1. **Extracted Folder**: `VITC_SingleThread/` (contains directly executable `./main` and `python3 main.py`)
2. **Submission Archive**: `VITC_SingleThread.zip` (standalone competition zip package)

---

## 1. Quick Execution Guide

The estimator requires **only Python 3 standard library** (zero third-party dependencies, zero neural networks, zero C++ compiler required).

### Run from the Extracted Directory:
```bash
# Direct binary execution
./VITC_SingleThread/main <path_to_input_imu.csv> <path_to_output_trajectory.txt>

# Or standard python execution
python3 VITC_SingleThread/main.py <path_to_input_imu.csv> <path_to_output_trajectory.txt>
```

### Supported Input Formats:
- **Raw IMU CSV**: standard CSV format containing `timestamp, wx, wy, wz, ax, ay, az` (EuRoC or TUM-VI format).
- **Directory**: path to sequence directory (e.g., `data/euroc/MH_01_easy` or containing `data.csv`).
- **ZIP archive**: path to zip archive containing IMU data.

### Output Format Specification (Non-Negotiable Competition Spec):
```text
# timestamp tx ty tz qx qy qz qw
1403636579.000000000 0.000000 0.000000 0.000000 -0.000001 -0.000796 0.000000 1.000000
```
- Exactly one space-separated line per valid input IMU row ($N_{\text{out}} \equiv N_{\text{in}}$).
- Unit quaternion norm within $10^{-6}$ of $1.0$.
- Monotonically non-decreasing timestamps.

---

## 2. Technical Architecture & Algorithmic Foundations

VITC_SingleThread implements an ultra-lightweight, embedded-grade **15-State Error-State Kalman Filter (ESKF)** with adaptive sensor resilience:

1. **State Vector (15 States)**:
   - True state: $\mathbf{x} = [\mathbf{q}, \mathbf{v}, \mathbf{p}, \mathbf{b}_g, \mathbf{b}_a]^T$ (attitude quaternion, velocity, position, gyro bias, accel bias).
   - Error state: $\delta\mathbf{x} = [\delta\boldsymbol{\theta}, \delta\mathbf{v}, \delta\mathbf{p}, \delta\mathbf{b}_g, \delta\mathbf{b}_a]^T \in \mathbb{R}^{15}$.

2. **Midpoint Strapdown Integration**:
   - Zero-dynamic-allocation propagation with 2nd-order Runge-Kutta quaternion integration and midpoint acceleration rotation.
   - Decoupled Ornstein-Uhlenbeck (OU) velocity damping with noise-adaptive Wiener shrinkage preventing runaway random-walk divergence.

3. **Autonomous Dynamic Regime Detection**:
   - **Flight Regime** (drones, high vibration energy):
     - Soft flight vertical velocity prior ($\sigma_{vz} = 1.20\text{ m/s}$) suppressing unobservable vertical velocity drift without constraining horizontal trajectory freedom.
     - 100 Hz alternating tilt alignment preserving high-speed CPU headroom.
   - **Pedestrian Regime** (handheld / walking, rhythmic gait energy):
     - Leveled non-holonomic constraint (NHC) orthogonal to gravitational up-axis, immune to handheld tilt and phone roll/pitch.
     - Biomechanical Weinberg stride-speed estimation dynamically bounding forward velocity based on gait peak-to-peak vertical bounce ($v \le k \cdot \Delta a^{1/4}$).
     - Bias-invariant Zero-Velocity Updates (ZUPT) and Zero-Angular-Rate Updates (ZARU) during static pauses.

4. **Sensor Degradation Hardening**:
   - **Causal 3-Tap Median Filter**: snipes high-frequency synthetic spikes without introducing phase delay.
   - **Binomial Noise-Adaptive Smoother**: adjusts smoothing strength based on incoming sensor noise standard deviation.
   - **Burst & Drop-Out Sub-Stepping**: handles short missing intervals ($\le 0.35\text{ s}$) via linear virtual sub-stepping, and bridges multi-second blackout gaps with decayed attitude extrapolation.
   - **Terminal 5s Blind Prediction**: uses an $O(1)$ ring buffer to compute ramp-weighted velocity and angular rates, performing decaying curvature extrapolation to the end of the file.

---

## 3. Verified Performance & Resource Scorecard

Evaluated against the official competition degradation model across 150 independent trials:

### Resource Compliance (All Hard Limits Safely Met)
- **Peak Tracemalloc Heap**: **`2.36 MB`** (Limit: $\le 8.0\text{ MB}$) $\rightarrow$ **70% Headroom**
- **Single-Thread Runtime (`taskset -c 0`)**: **`12.80 ms/s`** (in-process, 78x real-time) / **`16.35 ms/s`** (cold CLI + ZIP unpack) across all 5 sequences (Limit: $\le 20.0\text{ ms/s}$) $\rightarrow$ **36% Headroom**
- **Clean-Room Peak RSS**: **`23.52 MB`** (in-process) / **`23.88 MB`** (cold CLI) (Limit: $\le 64.0\text{ MB}$) $\rightarrow$ **62% Headroom**
- **Heap Allocation in Filter Loop**: **0 dynamic allocations** (fixed-size reusable working arrays).

### Accuracy Metrics Across Degradation Trials (50 Canonical Runs)
- **Overall Trajectory ATE RMSE**: **`2.102 m`** (-63.7% vs baseline 5.79 m)
  - `room2` (Pedestrian): **`1.220 m`**
  - `V1_01_easy` (Drone): **`1.756 m`**
  - `V2_02_medium` (Agile Drone): **`1.831 m`**
  - `room1` (Pedestrian): **`2.110 m`**
  - `MH_01_easy` (Drone): **`3.593 m`**
- **Terminal 5s Blind Cutoff RMSE**: **`2.894 m`** (-84.0% vs baseline 18.06 m, beating target 5.194 m)
- **2s Blackout Gap RMSE**: **`1.770 m`** (-73.3% vs baseline 6.625 m, beating target 6.625 m)
- **1-Second Relative Pose Error (RPE)**: **`0.743 m/s`** (beats stationary baseline 5/5)
- **Verification Suite**: **`272 / 272` tests passing** (215 unit + 57 fuzz) with **`100%` sample parity** ($N_{\text{out}} \equiv N_{\text{in}}$ across all 50 runs).

---

## 4. Package Directory Layout

```text
.
├── VITC_SingleThread/               # Extracted submission directory
│   ├── main                         # Standalone executable launcher
│   ├── main.py                      # Python entrypoint (python3 main.py <in> <out>)
│   ├── metadata.json                # Runtime benchmarks and environment metadata
│   ├── estimated_trajectory.txt     # Verified output trajectory on baseline dataset
│   ├── README.md                    # Submission instructions and parameter documentation
│   └── src/
│       ├── main.py                  # 100% self-contained 15-state ESKF algorithm
│       └── README.md                # Algorithmic notes
├── VITC_SingleThread.zip            # Standalone competition submission archive
├── README.md                        # Root repository execution & benchmark guide
├── SUBMISSION.md                    # Official submission documentation
├── AI_DISCLOSURE.md                 # Formal AI disclosure statement
├── create_degraded_zip.py           # Degradation injection tool
├── sample_imu.csv                   # Baseline IMU test telemetry
├── sample_input_degraded.zip        # Pre-degraded sample archive for quick testing
└── docs/                            # Presentation, video demo, and analysis plots
```
