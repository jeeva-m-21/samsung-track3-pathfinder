# Python SDK & Programmatic Interface (Track 3)

**Team:** `VITC_SingleThread`  
**Module:** `src.main`

---

## 1. Overview

While `src/main.py` is configured as a standalone command-line executable, it is also structured as a reusable Python library (SDK) that can be imported directly into other navigation or simulation workflows using only the Python standard library.

---

## 2. API Reference

### Function: `run_pipeline`

Processes an entire IMU dataset file or archive and streams the estimated poses to disk in TUM format.

```python
from src.main import run_pipeline

run_pipeline(
    input_path="path/to/dataset.zip",         # Supports .zip, .csv, or directory
    output_path="estimated_trajectory.txt",   # TUM format output file
    params=None,                              # Optional dictionary overriding DEFAULT_PARAMS
    ablation=None                             # Optional ablation mode string ("eskf_full", "eskf_basic")
)
```

**Parameters:**
- `input_path` (`str` or `Path`): Path to input dataset.
  - If `.zip`: Automatically extracts and reads `imu0/data.csv` or `mav0/imu0/data.csv`.
  - If `.csv`: Reads raw IMU telemetry rows directly.
  - If directory: Locates and reads the contained IMU telemetry file.
- `output_path` (`str` or `Path`): Destination path for the 8-column TUM trajectory file.
- `params` (`dict`, optional): Custom hyperparameter dictionary to override defaults.
- `ablation` (`str`, optional): Select ablation profile (`"eskf_full"` or `"eskf_basic"`).

---

### Class: `SparseESKF`

Direct step-by-step filter interface for real-time or streaming integration.

```python
from src.main import SparseESKF, DEFAULT_PARAMS

# 1. Instantiate filter with configuration
filter_instance = SparseESKF(DEFAULT_PARAMS)

# 2. Initialize with initial stationary or startup buffer
# ws: list of gyro vectors [(wx, wy, wz), ...]
# acc: list of accel vectors [(ax, ay, az), ...]
filter_instance.initialize(ws_buffer, acc_buffer, t0_s=0.0)

# 3. Propagate each step
# dt: time step in seconds
# ax, ay, az: accelerometer measurement (m/s^2)
# gx, gy, gz: gyroscope measurement (rad/s)
# noise_mult: estimated noise multiplier from front-end monitor
# t_el: elapsed mission time in seconds
filter_instance.step(dt, ax, ay, az, gx, gy, gz, noise_mult=1.0, t_el=t_now)

# 4. Access current state
position = filter_instance.p   # [px, py, pz] in meters
velocity = filter_instance.v   # [vx, vy, vz] in m/s
quaternion = filter_instance.q # (qw, qx, qy, qz) Hamilton unit quaternion
```

---

## 3. CLI Interface

The SDK can be executed directly from the terminal:

```bash
# Basic usage:
python3 src/main.py input_dataset.zip [output_trajectory.txt]

# Using root executable alias:
./main input_dataset.zip [output_trajectory.txt]
```
