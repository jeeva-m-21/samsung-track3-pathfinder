# Path Finder: Embedded C++17 IMU-Only ESKF Estimator (Track 3)

High-performance, zero-allocation, single-threaded streaming Error-State Kalman Filter (ESKF) for IMU-only dead-reckoning.

## 1. Quick Start & Build

The production pipeline is written entirely in **C++17** and compiled using standard `g++` or `clang++`. It has zero external dependencies (no OpenCV, Eigen, ROS, or Boost).

```bash
# Build the production release binary
make

# Output binary located at:
./src/cpp/build/main
```

## 2. Running Trajectory Estimation

The binary accepts either a **direct CSV file** or a **dataset folder** containing `imu0/data.csv` (or `mav0/imu0/data.csv`).
If omitted, the output path defaults to `estimated_trajectory.txt`, and the filter automatically runs the production `eskf_full` pipeline.

```bash
# Standard evaluation run (defaults to eskf_full and outputs to estimated_trajectory.txt):
./src/cpp/build/main <path_to_dataset_or_imu.csv>

# Explicit output path:
./src/cpp/build/main <path_to_dataset_or_imu.csv> estimated_trajectory.txt

# Ablation modes (optional):
./src/cpp/build/main <input> estimated_trajectory.txt --ablation eskf_full
./src/cpp/build/main <input> estimated_trajectory.txt --ablation eskf_basic
./src/cpp/build/main <input> estimated_trajectory.txt --ablation mahony
./src/cpp/build/main <input> estimated_trajectory.txt --ablation naive
```

> **Note on `--append-blind`**: By default, the binary outputs **strictly 1 pose per input IMU row** (1-to-1 frame synchronization). Do NOT pass `--append-blind` unless the input sequence is explicitly truncated 5 seconds early.

## 3. Format Verification & Tests

```bash
# Run unit tests (1,233 C++ checks)
make test

# Validate trajectory format (TUM 8 space-separated columns, int64 ns timestamps)
python3 tools/validate_trajectory.py <input.csv> estimated_trajectory.txt

# Run full gate checks
PATH=".venv/bin:$PATH" PYTHONPATH=. bash scripts/gate_g5.sh
```

## 4. Hardware Budget Specifications

- **Peak Resident Memory (RSS)**: `2.39 MB` (Hard limit: `4.0 MB` / `8.0 MB`).
- **Processing Latency**: `< 1.0 ms` per 1 second of IMU data (Hard limit: `2.0 ms/s`).
- **Concurrency**: Strictly 1 thread (single-threaded, deterministic).
- **Allocations**: Zero dynamic heap allocations in the main filter loop.
