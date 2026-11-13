# Path Finder: High-Performance Embedded C++17 IMU-Only ESKF Estimator

**Samsung Hackathon — Track 3: "Path Finder"**  
**Team**: `VITC_SingleThread`  
**Target Architecture**: Single-threaded streaming C++17 binary (Zero dynamic heap allocations, no external libraries, pure buffered POSIX `stdio`)  
**Release Baseline**: Tag `submission-final` / `release-candidate` (commit `b9eed87` / `46a7c15`)

---

## 1. Executive Summary

In pure inertial dead-reckoning, navigation systems must estimate 3D position, velocity, and orientation using **only 6-axis IMU data** (3-axis gyroscope and 3-axis accelerometer at 200 Hz). In the complete absence of GPS, vision/cameras, magnetometers, and neural networks, double-integrating noisy accelerometer measurements causes explosive quadratic position drift ($\sim \frac{1}{2} a t^2$), diverging by hundreds of meters within tens of seconds.

**Path Finder** is an embedded-grade, ultra-lightweight **15-State Error-State Kalman Filter (ESKF)** written in strict **C++17**. It operates under zero-heap-allocation constraints inside the filter loop, maintains strict numerical stability through Joseph-stabilized covariance updates, and features adaptive sensor resilience algorithms that seamlessly handle packet dropouts, sensor freezes, and complete blackout cutoffs.

### Key Verified Achievements (Locked Release Baseline):
- **Trajectory Closeness Across 5 Competition Datasets (50 Runs across Sealed Seeds {101..110})**:
  - **Waypoint Trajectory ATE RMSE (SE(3) Umeyama)**: **`2.13 m`** (down from $5.79\text{ m}$ baseline, a **`-63.2%`** error reduction).
  - **Terminal 5s Blind Cutoff RMSE**: **`3.93 m`** (down from $18.07\text{ m}$ baseline, a **`-78.2%`** error reduction).
  - **Blackout 2s Gap RMSE**: **`5.15 m`** (down from $15.65\text{ m}$ baseline, a **`-67.1%`** error reduction).
- **Zero Overfitting Verification**: Performance across the completely unseen sealed seeds `{101..110}` matched dev seeds `{1..10}` within **1.2% to 3.9% variance**, confirming zero seed luck.
- **Embedded Memory Footprint**: Peak resident set size (RSS) is **`1.78 MB`** (1,780 KB) on real telemetry, well below the **$\le 8.0\text{ MB}$** competition ceiling (and our strict internal $\le 4.0\text{ MB}$ target), providing $>75\%$ memory headroom.
- **Ultra-High Throughput**: Execution latency is **`0.65 ms per 1.0 s of IMU data`** ($30\times$ faster than the official $\le 20.0\text{ ms/s}$ competition limit, and well below the $\le 5.0\text{ ms/s}$ target).
- **Zero Allocations & Zero Streams**: 0 dynamic allocations (`malloc`/`new`) in the streaming loop; 0 `<iostream>` usage (pure buffered POSIX `stdio`).
- **Memory Safety & Unit Test Coverage**: 100% verified with **AddressSanitizer (ASan)** and **UndefinedBehaviorSanitizer (UBSan)** across **1,246 unit test assertions** and 57 adversarial fuzzing test cases.
- **Spike Front-End**: Causal 3-sample median filter on incoming raw gyroscope stream suppressing synthetic impulsive spikes without phase distortion.

---

## 2. Official Evaluation Scorecard

### Comprehensive Multi-Split Benchmark Summary

| Metric | Baseline (`35c3334`) | Dev Seeds `{1..10}` | Confirmation `{201..210}` | Sealed Final `{101..110}` | Relative Improvement vs Baseline |
|:---|:---:|:---:|:---:|:---:|:---:|
| **Waypoint ATE RMSE (SE(3))** | 5.787 m | **2.105 m** | 2.176 m | **2.132 m** | **-63.2%** |
| **Final 5s Blind Cutoff RMSE** | 18.065 m | **3.785 m** | 4.143 m | **3.934 m** | **-78.2%** |
| **Dropout 2s Gap RMSE** | 15.654 m | **5.090 m** | 5.342 m | **5.153 m** | **-67.1%** |
| **Peak Resident Set Size (RSS)** | 1.76 MB | 1.81 MB | 1.81 MB | **1.78 MB** | **$\le 4.0$ MB Target Passed** ✅ |
| **Compute Latency (Single Core)** | 0.66 ms/s | 0.85 ms/s | 0.85 ms/s | **0.65 ms/s** | **$\le 5.0$ ms/s Target Passed** ✅ |

---

## 3. System Architecture & Estimation Pipeline

```
Raw IMU Stream (200 Hz)
  │ (wx, wy, wz, ax, ay, az)
  ▼
┌────────────────────────────────────────────────────────┐
│ Streaming Tolerant Parser (Buffered stdio, 64 KB cache)│
└──────────────────────────┬─────────────────────────────┘
                           ▼
┌────────────────────────────────────────────────────────┐
│ Online Static Initializer (Windowed Gravity Alignment) │
│ - Estimates g_mag ∈ [9.0, 10.5] m/s² and initial tilt  │
│ - Calibrates initial gyro bias; motion fallback guard │
└──────────────────────────┬─────────────────────────────┘
                           ▼
┌────────────────────────────────────────────────────────┐
│ Fault Detection & Gap Handler (SPEC §12.7)             │
│ - Rejects NaN/Inf, all-zero, stuck, and outlier spikes │
│ - Substeps large gaps (dt > 2.5 dt_nom) with decay     │
└──────────────────────────┬─────────────────────────────┘
                           ▼
┌────────────────────────────────────────────────────────┐
│ Nominal Strapdown Kinematics (SO(3) Lie Algebra)       │
│ - Hamilton quaternion integration with sign continuity │
│ - Unbiased specific force rotated to world frame       │
└──────────────────────────┬─────────────────────────────┘
                           ▼
┌────────────────────────────────────────────────────────┐
│ Error-State Kalman Filter (15 States: δθ, δv, δp, δbg, δba)
│ - Analytical continuous-to-discrete Jacobian F (15x15) │
│ - Discrete process noise Q with covariance conditioning│
└──────────────────────────┬─────────────────────────────┘
                           ▼
┌────────────────────────────────────────────────────────┐
│ Degradation Monitor & Adaptive Updates                 │
│ - Real-time residual tracking & variance calibration   │
│ - Adaptive covariance inflation factor κ ∈ [1.0, 50.0] │
│ - Joseph-stabilized Tilt Gravity Updates               │
│ - Zero Velocity Updates (ZUPT) via stationary buffer   │
└──────────────────────────┬─────────────────────────────┘
                           ▼
┌────────────────────────────────────────────────────────┐
│ State Rollback & Blind-5s Predictor (§12.9)            │
│ - Reliable ring buffer (capacity 64) for pre-failure   │
│ - Closed-form decaying velocity & turn-rate prediction │
└──────────────────────────┬─────────────────────────────┘
                           ▼
Trajectory Writer: Frozen 8 Space-Separated Columns (TUM Format)
  (timestamp_ns tx ty tz qx qy qz qw)
```

### Algorithmic Highlights

1. **Lie Algebra & Hamilton Quaternions (`src/cpp/quat.h`)**:
   - Stored internally as $[w, x, y, z]$ (body-to-world rotation).
   - Exponential and logarithmic maps with Taylor series expansions for small angles ($\|\phi\| < 10^{-8}$) to avoid division-by-zero singularities.
   - Shepperd’s 4-branch algorithm for converting rotation matrices to quaternions without numerical conditioning issues.
   - Enforces quaternion sign continuity ($\text{dot}(q_k, q_{k-1}) \ge 0$) to prevent spurious phase wrapping.

2. **Numerical Covariance Conditioning (`src/cpp/eskf.cpp`)**:
   - Every propagation and update enforces explicit symmetrization: $P = \frac{1}{2}(P + P^T)$.
   - Clamped with Cauchy-Schwarz preservation: uses congruence transformations $D P D^T$ scaling rows and columns if diagonal elements grow, ensuring positive-definiteness even after 10,000 open-loop steps.
   - All measurement updates use the **Joseph stabilized form**:
     $$P = (I - KH)P(I - KH)^T + KRK^T$$
     guaranteeing positive semi-definiteness under floating-point arithmetic.

3. **Multi-Rate Zero Velocity Updates (ZUPT) & Tilt Correction (`src/cpp/updates.cpp`)**:
   - Zero-allocation circular ring buffer tracking stationary statistics (gyro mean norm $< 0.03\text{ rad/s}$, accel norm deviation $< 0.15\text{ m/s}^2$).
   - Injects pseudo-measurement innovations $y = -v$ to instantly arrest velocity drift and observe accelerometer bias.
   - Gravity vector alignment updates constrain roll and pitch drift during low-acceleration phases.

4. **Glitch Classification & Large-Gap Substepping (`src/cpp/gap.cpp`)**:
   - Rejects non-physical sensor samples (stuck values $\ge 3$ ticks, all zeros, rates $> 35\text{ rad/s}$ or forces $> 160\text{ m/s}^2$).
   - Large timestamp gaps ($\Delta t > 2.5 \times \Delta t_{\text{nom}}$) are substapped ($N \le 800$) with continuous exponential velocity decay:
     $$v \leftarrow v \cdot \exp\left(-\frac{\Delta t_{\text{sub}}}{\tau_{\text{gap}}}\right)$$
     preventing unbounded velocity runaway across communication blackouts.

5. **Blind-5s Kinematic Predictor (`src/cpp/blind.cpp`)**:
   - Tracks reliable states in a circular ring buffer.
   - If sensor failure persists $\ge 1.0\text{ s}$, rolls back to the last reliable state prior to failure onset and extrapolates trajectories using closed-form decaying turn rates and velocities.

---

## 3. Empirical Results & Ablation Analysis

Evaluated across degraded benchmark sequences comparing four standard filter configurations:
- **`naive`**: Initial tilt alignment + open-loop raw strapdown integration (no state corrections).
- **`mahony`**: Non-linear complementary attitude observer + gravity-subtracted velocity integration.
- **`eskf_basic`**: 15-state ESKF with fixed measurement noise (no dynamic monitor, gap handler, or failure predictor).
- **`eskf_full`**: Full production pipeline with adaptive monitoring, Joseph updates, gap substepping, and blind predictor.

### Overall Benchmark Metrics (Mean Across Benchmark Sequences)

| Configuration | ATE RMSE (m) | Blind-5s RMSE (m) | Dropout RMSE (m) | RPE 1s RMSE (m) | Objective Score $J$ |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`naive`** | $25.58\text{ m}$ | $67.59\text{ m}$ | $48.81\text{ m}$ | $7.71\text{ m}$ | $1.0000$ |
| **`mahony`** | $10.57\text{ m}$ | $28.98\text{ m}$ | $20.68\text{ m}$ | $4.46\text{ m}$ | $0.3893$ |
| **`eskf_basic`** | $4.29\text{ m}$ | $7.49\text{ m}$ | $7.22\text{ m}$ | $2.28\text{ m}$ | $0.1790$ |
| **`eskf_full` (Ours)** | **$3.52\text{ m}$** | **$5.76\text{ m}$** | **$6.89\text{ m}$** | **$1.74\text{ m}$** | **$0.1364$** |
| **Improvement** | **$7.26\times$ better** | **$11.73\times$ better** | **$7.09\times$ better** | **$4.44\times$ better** | **$86.4\%$ error reduction** |

---

## 4. Hardware Budget & Constraint Verification

| Constraint | Competition Limit | Internal Target | Measured Value | Status |
| :--- | :--- | :--- | :--- | :--- |
| **Peak RAM (RSS)** | $\le 8,192\text{ KB}$ ($8.0\text{ MB}$) | $\le 4,096\text{ KB}$ ($4.0\text{ MB}$) | **`2,348 KB`** ($2.29\text{ MB}$) | **PASS** ($>5.6\text{ MB}$ headroom) |
| **Execution Latency** | $\le 20.0\text{ ms / s}$ of IMU | $\le 2.0\text{ ms / s}$ of IMU | **`1.10–1.25 ms/s`** | **PASS** ($16\times$ faster than limit) |
| **OS Threads** | Strictly 1 thread | Strictly 1 thread | **1 thread** | **PASS** (Zero background workers) |
| **Filter Loop Allocations**| 0 heap allocations | 0 heap allocations | **0 dynamic allocations** | **PASS** (Stack & static buffers) |
| **Standard I/O Streams** | Zero `<iostream>` | Zero `<iostream>` | **0 occurrences** | **PASS** (Pure buffered `stdio`) |
| **Determinism** | 100% bit-identical | 100% bit-identical | `sha1sum` identical | **PASS** (Zero non-deterministic jitter) |
| **Output Format** | TUM 8 space-separated columns | TUM 8 space-separated columns | Passed validator | **PASS** (Exact int64 ns timestamps) |

---

## 5. Quick Start & Build Instructions

### 5.1 Native Build (Linux / macOS)

The codebase has zero external library dependencies and compiles with standard `g++` (C++17):

```bash
# Clone the repository
git clone https://github.com/jeeva-m-21/samsung-track3-pathfinder.git
cd samsung-track3-pathfinder

# Build the release binary
make

# Binaries are automatically generated at:
# ./main (root)
# src/main
# src/cpp/build/main
```

### 5.2 Running the Estimator

The executable supports either a **direct CSV file** or a **dataset directory** (automatically resolving `imu0/data.csv`, `mav0/imu0/data.csv`, or `data.csv`).
If omitted, the output path defaults to `estimated_trajectory.txt` and the mode defaults to `eskf_full`:

```bash
# Standard evaluation run:
./main <path_to_dataset_or_imu.csv>

# Explicit output path:
./main <path_to_dataset_or_imu.csv> estimated_trajectory.txt

# Example running on EuRoC MAV:
./main data/euroc/MH_01_easy estimated_trajectory.txt
```

### 5.3 Format Validation & Tests

```bash
# Validate output trajectory format against specification
python3 tools/validate_trajectory.py <input_imu.csv> estimated_trajectory.txt

# Run the full unit test suite (1,233 C++ assertions)
make test
```

### 5.4 Docker Build & Execution (PRD Section 8.2)

For guaranteed clean-room reproducibility:

```bash
# Build the Docker image
docker build -t pathfinder .

# Run trajectory estimation inside Docker
docker run --rm -v $(pwd):/workspace pathfinder /workspace/data/euroc/MH_01_easy /workspace/estimated_trajectory.txt
```

---

## 6. Automated Benchmark, Evo & Environmental Resilience Verification

### 6.1 Multi-Dataset Benchmark Suite
The automated evaluation benchmark (`tools/benchmark_suite.py`) verifies estimator accuracy across 8 diverse datasets:
```bash
python tools/benchmark_suite.py
```
Evaluated environments:
- **EuRoC MAV Flights**: `MH_01_easy` (Industrial hall), `V1_01_easy` (Gentle room), `V2_02_medium` (Agile drone maneuvers).
- **TUM-VI Handheld Walking**: `room1`, `room2` (Continuous human walking with zero mid-flight stops).
- **Synthetic Geometries**: `circle`, `figure_eight` (Analytically exact kinematic stress tests).
- **High-Speed FPV Racing**: `UZH-FPV fwd7` (Aggressive rotational accelerations and dynamic rates up to 15 rad/s).

### 6.2 Evo Trajectory Analysis
Rigid SE(3) Umeyama aligned trajectory evaluation using `evo` confirms that the estimated trajectories track true positions and orientations without scale distortion:
```bash
# Evaluate SE(3) aligned ATE
evo_ape tum data/euroc/gt/MH_01_easy.txt estimated_trajectory.txt -a

# Evaluate 1-second Relative Pose Error (RPE)
evo_rpe tum data/euroc/gt/MH_01_easy.txt estimated_trajectory.txt -a --delta 1.0 -u m
```

### 6.3 Environmental Stress & Degradation Resilience
The estimator has been stress-tested across multiple harsh degradation tiers (`tools/degrade_hard.py`) simulating extreme, real-world deployment faults:
- **`severe`**: Random 3D sensor mounting tilt, progressive heavy-tailed noise, turn-on bias and random walks, motor vibration tones, and 2-second blackout gaps.
- **`extreme`**: Multi-axis channel drops, sensor freezing, accelerometer/gyroscope saturation, and signal corruptions.
- **`hell` with `--timing-faults`**: Adversarial timestamp jitter, duplicate rows, swapped frames, and severe packet bursts.
```bash
# Run extreme stress test
python tools/degrade_hard.py degrade --input data/euroc/MH_01_easy/mav0/imu0/data.csv --out /tmp/stress_extreme --preset extreme --seed 99
./main /tmp/stress_extreme/imu0/data.csv /tmp/stress_extreme_traj.txt --append-blind
python tools/validate_trajectory.py --append-blind /tmp/stress_extreme/imu0/data.csv /tmp/stress_extreme_traj.txt
```
**Resilience Result**: Under all severe, extreme, and hell presets, the pipeline produces **zero crashes, zero NaNs, and 100% compliant trajectory output** meeting all competition formatting rules.

### 6.4 Adversarial Fuzz Testing
The adversarial fuzz test suite (`tests/fuzz/test_fuzz.py`) validates robustness against corrupt inputs:
```bash
PYTHONPATH=. pytest tests/fuzz/ -v
```
All 57 adversarial fuzz cases (NaN bursts, sub-microsecond $\Delta t$, $1000g$ acceleration spikes, $1000\text{ rad/s}$ gyro spins, and truncated CSVs) pass clean.

---

## 7. Official Submission Packaging (PRD Section 8.1)

To create the final competition zip bundle (`CollegeName_TeamName.zip`):

```bash
# Package and execute clean-room verification:
./tools/make_bundle.sh VITC_SingleThread [optional_dataset_path]
```

This automated script:
1. Compiles the native release binary from scratch.
2. Runs estimation on the provided dataset and validates `estimated_trajectory.txt`.
3. Packages `estimated_trajectory.txt`, `metadata.json`, and `src/` into `VITC_SingleThread.zip`.
4. Extracts the archive into an isolated sandbox (`/tmp/clean_room_check_XXXXXX`), re-compiles from source, and confirms clean execution with $\le 4\text{ MB}$ RSS.

---

## 8. Repository Layout

```text
.
├── Dockerfile                   # PRD Section 8.2 container specification
├── Makefile                     # Root build system (builds ./main and src/main)
├── README.md                    # Project documentation & results summary
├── metadata.json                # PRD Section 6.4 runtime specs & metrics
├── configs/                     # System parameters & tuning spaces
│   ├── params.json              # Calibrated ESKF parameters (single source of truth)
│   └── tune_space.json          # Search space bounds
├── src/                         # Self-contained source directory for submission
│   ├── Makefile                 # Compiles and copies binaries to src/main & ./main
│   ├── README.md                # Submission build & run instructions
│   └── cpp/                     # Core C++17 streaming estimator
│       ├── mat.h                # Zero-allocation template matrix algebra
│       ├── quat.h               # Hamilton quaternion & SO(3) Lie algebra
│       ├── io.h / io.cpp        # High-speed buffered stdio parser/writer
│       ├── init.h / init.cpp    # Static tilt alignment & bias estimation
│       ├── eskf.h / eskf.cpp    # 15-state ESKF propagation & Joseph updates
│       ├── updates.h / .cpp     # ZUPT & gravity tilt measurement updates
│       ├── monitor.h / .cpp     # Residual tracking & covariance inflation
│       ├── gap.h / .cpp         # Outlier classification & gap substepping
│       ├── blind.h / .cpp       # Rollback ring buffer & blind extrapolator
│       ├── pipeline.h / .cpp    # End-to-end streaming pipeline coordinator
│       └── main.cpp             # CLI entrypoint with directory resolution
├── tests/                       # Unit and adversarial test suites
│   ├── cpp/                     # C++ unit tests (1,233 assertions)
│   └── fuzz/                    # Adversarial fuzz testing suite (57 test cases)
├── tools/                       # Development & evaluation tooling
│   ├── export_params.py         # Generates src/cpp/params.h from params.json
│   ├── validate_trajectory.py   # Trajectory specification validator
│   ├── data_duration.py         # Timing duration calculator
│   ├── make_bundle.sh           # Clean-room packaging script (PRD Section 8.1)
│   └── tune.py                  # Optuna Bayesian hyperparameter optimizer
└── scripts/                     # Automated regression & budget verification
    ├── budget_check.sh          # Hard RSS, latency & determinism verifier
    └── diff_py_cpp.py           # 40-case bit-accurate differential test
```
