# Source Code & Execution Instructions (Track 3: Path Finder)

Team: **VITC_SingleThread**  
Track: **Track 3: Path Finder (IMU-Only Dead-Reckoning)**

---

## 1. Requirements & Dependencies

- **Language:** Python 3.10+
- **Dependencies:** **NONE** (100% Pure Python Standard Library: `math`, `struct`, `array`, `zipfile`, `sys`, `os`, `gc`)
- **Compilation:** None required (interpreted pure Python).
- **Environment:** Runs in any minimal Python 3 environment without `pip` or third-party packages.

---

## 2. Execution

The entry point is `src/main.py` (also available at root as `main.py` and executable `main`):

```bash
# Basic usage (defaults to writing estimated_trajectory.txt in current directory):
python3 src/main.py <path_to_input>

# Explicit output file path:
python3 src/main.py <path_to_input> estimated_trajectory.txt
```

### Supported Input Types:
1. **Zipped EuRoC Archive (.zip):** Zipped folder containing `imu0/data.csv` (or `mav0/imu0/data.csv`):
   ```bash
   python3 src/main.py dataset.zip estimated_trajectory.txt
   ```
2. **Direct CSV File (.csv):**
   ```bash
   python3 src/main.py imu0/data.csv estimated_trajectory.txt
   ```
3. **Dataset Directory:** Directory containing `imu0/data.csv` or `data.csv`:
   ```bash
   python3 src/main.py path/to/dataset/ estimated_trajectory.txt
   ```

---

## 3. Output Format: TUM Trajectory

The output file `estimated_trajectory.txt` follows the standard TUM format required by `evo`:
- Space-separated, 8 columns per row:
  `timestamp tx ty tz qx qy qz qw`
- **Timestamp:** Floating-point seconds since Unix epoch (`s = ns * 10^-9`).
- **Translation:** `tx ty tz` in meters (world frame).
- **Orientation:** `qx qy qz qw` normalized unit quaternion (Hamilton convention).
- **Parity:** Exactly one pose row per input IMU sample ($N_{\text{out}} \equiv N_{\text{in}}$).

---

## 4. Evaluated Performance (safe-2102 Production Lock)

- **Overall 50-Run ATE (RMSE):** `2.102 m` (room2: `1.220 m`, V1_01: `1.756 m`, V2_02: `1.831 m`, room1: `2.110 m`, MH_01: `3.593 m`)
- **Final 5s Blind RMSE (SE(3) aligned):** `2.894 m` (vs baseline 18.06 m, -84.0%)
- **Dropout 2s Gap RMSE (SE(3) aligned):** `1.770 m` (vs baseline 6.63 m, -73.3%)
- **Relative Pose Error (1s window):** `0.743 m/s` (beats stationary baseline across 5/5 sequences)
- **Single-Thread Compute Runtime:** `12.80 ms/s` in-process (78x real-time) / `16.35 ms/s` cold CLI with ZIP unpack (Limit: $\le 20.0$ ms/s)
- **Peak Process RSS:** `23.52 MB` in-process / `23.88 MB` cold CLI (Limit: $\le 64.0$ MB)
- **Peak Tracemalloc Heap:** `2.36 MB` (Limit: $\le 8.0$ MB)
- **Verification Suite:** `272/272` tests passing (215 unit + 57 fuzz), `50/50` runs (100%) strict sample parity ($N_{\text{out}} \equiv N_{\text{in}}$)
