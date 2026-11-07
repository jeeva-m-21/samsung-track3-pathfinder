# Source Code & Build Instructions (Track 3: Path Finder)

Team: **VITC_SingleThread**

This folder contains the complete, self-contained source code for our IMU-only Error-State Kalman Filter (ESKF) estimator.

## 1. Requirements

- C++17 compiler (`g++` >= 9.0 or `clang++` >= 10.0)
- `make`
- No external libraries required (zero dependencies on Eigen, OpenCV, ROS, or Boost).

## 2. Compilation

From within this `src/` directory (or from the project root):

```bash
make
```

This compiles the optimized streaming binary:
- `src/cpp/build/main`
- Automatically copies it to `src/main` and `./main` (root) for maximum compatibility with automated grading harnesses.

To clean:
```bash
make clean
```

To run unit tests:
```bash
make test
```

## 3. Execution

```bash
# Basic usage (defaults to eskf_full and outputs to estimated_trajectory.txt):
./main <path_to_imu_csv_or_folder>

# Explicit output path:
./main <path_to_imu_csv_or_folder> estimated_trajectory.txt
```

### Supported Input Types:
1. **Direct CSV file**: `./main path/to/imu0/data.csv estimated_trajectory.txt`
2. **Dataset folder**: `./main path/to/dataset/` (automatically locates `imu0/data.csv`, `mav0/imu0/data.csv`, or `data.csv`).

### Ablation Modes (optional):
```bash
./main <input> estimated_trajectory.txt --ablation eskf_full   # Default: production filter
./main <input> estimated_trajectory.txt --ablation eskf_basic  # Fixed noise, no monitor/gap/blind
./main <input> estimated_trajectory.txt --ablation mahony      # Mahony attitude observer
./main <input> estimated_trajectory.txt --ablation naive       # Raw strapdown baseline
```

> **Row-count synchronization**: By default, the binary outputs **strictly 1 pose per input IMU frame**. `--append-blind` is only used if the input CSV is truncated 5 seconds early.

## 4. Hardware Constraints

- **Peak RSS**: 2.39 MB (well below 4 MB budget, hard ceiling 8 MB).
- **Latency**: < 1.0 ms per 1 s IMU data (well below 2.0 ms/s budget).
- **Concurrency**: Strictly 1 OS thread (single-threaded).
- **Memory safety**: Zero dynamic heap allocations in filter loop.
