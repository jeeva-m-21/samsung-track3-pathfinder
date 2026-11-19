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

## 4. Evaluated Performance

- **Held-Out LOOCV ATE (RMSE):** `2.4917 m` (vs `2.4530 m` Clean Baseline)
- **Held-Out LOOCV Blind-5s (RMSE):** `4.2377 m`
- **Held-Out LOOCV Gap-2s (RMSE):** `4.7414 m`
- **Single-Thread Compute Runtime:** `9.23 – 11.96 ms` per 1.0 s of IMU data (Target: $\le 12.0$ ms/s; Limit: $\le 20.0$ ms/s)
- **Peak Process RSS:** `21.42 MB` (Target: $\le 25.0$ MB; Limit: $\le 64.0$ MB)
- **Peak Tracemalloc Heap:** `2.14 MB`
