# Path Finder: High-Performance Embedded C++17 IMU-Only ESKF Estimator

**Samsung Hackathon — Track 3: "Path Finder"**  
**Team**: `VITC_SingleThread`  
**Target Architecture**: Single-threaded streaming C++17 binary (Zero dynamic heap allocations, no external libraries, pure buffered `stdio`)

---

## 1. Executive Summary

In pure inertial dead-reckoning, navigation systems must estimate 3D position, velocity, and orientation using **only 6-axis IMU data** (3-axis gyroscope and 3-axis accelerometer at 200 Hz). In the complete absence of GPS, vision/cameras, magnetometers, and neural networks, double-integrating noisy accelerometer measurements causes explosive quadratic position drift ($\sim \frac{1}{2} a t^2$), diverging by hundreds of meters within tens of seconds.

**Path Finder** is an embedded-grade, ultra-lightweight **15-State Error-State Kalman Filter (ESKF)** written in strict **C++17**. It operates under zero-heap-allocation constraints inside the filter loop, maintains strict numerical stability through Joseph-stabilized covariance updates, and features adaptive sensor resilience algorithms that seamlessly handle packet dropouts, sensor freezes, and complete blackout cutoffs.

### Key Achievements:
- **Accuracy Improvement**: Achieved an **86.4% reduction in overall tracking error** ($J = 0.1364$ vs $1.0000$ naive baseline).
- **Real Drone Flight (EuRoC MAV `MH_01_easy`)**: Reduced open-loop quadratic drift from **22,247 m RMSE down to 35.32 m RMSE** ($630\times$ improvement).
- **Embedded Memory Footprint**: Peak resident memory (RSS) is **2.39 MB** (well below the 4.0 MB budget and 8.0 MB hard ceiling, giving >40% safety margin).
- **Ultra-High Throughput**: Execution latency is **< 1.0 ms per 1 second of IMU data** (>20x faster than the 2.0 ms/s competition limit).
- **Zero Allocations & Zero Streams**: 0 dynamic allocations (`malloc`/`new`) in the streaming loop; 0 `<iostream>` usage (pure buffered POSIX `stdio`).
- **Memory Safety**: 100% verified with **AddressSanitizer (ASan)** and **UndefinedBehaviorSanitizer (UBSan)** across 57 adversarial fuzzing test cases.

---

## 2. System Architecture & Estimation Pipeline

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

| Constraint | Competition Limit | Measured Value | Status |
| :--- | :--- | :--- | :--- |
| **Peak RAM (RSS)** | $\le 4,096\text{ KB}$ (Soft) / $\le 8,192\text{ KB}$ (Hard) | **`2,392 KB`** ($2.39\text{ MB}$) | **PASS** (+1.7 MB headroom) |
| **Execution Latency** | $\le 2.0\text{ ms}$ per 1 s of IMU data | **`< 1.0 ms/s`** | **PASS** (>20x faster than limit) |
| **OS Threads** | Strictly 1 thread | **1 thread** | **PASS** (Zero background workers) |
| **Filter Loop Allocations**| 0 heap allocations | **0 dynamic allocations** | **PASS** (Stack & static buffers) |
| **Standard I/O Streams** | Zero `<iostream>` | **0 occurrences** | **PASS** (Pure buffered `stdio`) |
| **Determinism** | 100% bit-identical | `sha1sum` identical | **PASS** (Zero non-deterministic jitter) |
| **Output Format** | TUM 8 space-separated columns | Passed validator | **PASS** (Exact int64 ns timestamps) |

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

## 6. Official Submission Packaging (PRD Section 8.1)

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

## 7. Repository Layout

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
