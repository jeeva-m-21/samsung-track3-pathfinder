# Decisions and Assumptions Log

Append-only log of non-obvious engineering decisions, assumptions, and tool findings.

## Format
- **Date**: YYYY-MM-DD
- **Packet**: WP-xx
- **Decision / Assumption**: Description
- **Rationale**: Justification or reference URL

## 2026-11-06: Memory Footprint Probe
- **Packet**: WP-01
- **Findings**:
  - Minimal dynamic binary (`empty_dyn`): Peak RSS = 1200 kbytes (~1.17 MB).
  - Minimal static binary (`empty_static`): Peak RSS = 688 kbytes (~0.67 MB).
- **Decision**:
  - Prefer static linking (`-static`) for the C++ binary when possible to minimize startup RSS and eliminate dynamic linker overhead, staying well below the 4 MB internal limit and 8 MB hard budget.
  - Python toolchain verified: `.venv` contains `numpy 2.5.4`, `scipy 1.18.1`, `pytest 9.2.0`, `evo 1.38.1`, `optuna 5.0.0`.

## 2026-11-06: Dataset Acquisition & Robustness
- **Packet**: WP-01 prep / Data setup
- **Action**: Acquired TUM-VI `room1` and `room2` sequences (IMU CSV and mocap ground truth) via targeted HTTP byte-range requests directly from `https://cdn2.vision.in.tum.de/tumvi/exported/euroc/512_16/`, avoiding ~3.4 GB of camera images. Acquired EuRoC ground truth trajectories for `MH_01_easy`, `V1_01_easy`, `V2_02_medium` from OpenVINS benchmark mirrors.
- **ETH ASL Server Status**: `robotics.ethz.ch` (`129.132.38.186:80`) timed out / blocked at firewall; `research-collection.ethz.ch` returns HTTP 429 / Access Restricted to automated IP ranges.
- **Robustness Check**:
  - `data/tumvi/room1/mav0/imu0/data.csv`: 30,943 rows, 199.37 Hz, 0 NaN/Inf, 0 gaps, mean accel norm 10.044 m/s^2, mean gyro 1.33 rad/s.
  - `data/tumvi/room2/mav0/imu0/data.csv`: 31,614 rows, 199.40 Hz, 0 NaN/Inf, 0 gaps, mean accel norm 9.870 m/s^2, mean gyro 0.98 rad/s.

## 2026-11-07: WP-28 Allowlist Scope Extension
- **Packet**: WP-28
- **Decision**: Widen WP-28 allowlist to include `tools/run_eval.py` alongside `src/ref/pipeline.py`, `src/ref/__main__.py`, and `results/ablation_py.md`.
- **Rationale**: `scripts/gate_g2.sh` executes `python3 tools/run_eval.py --eval-py-ref` to run comparative evaluations of `eskf_full`, `eskf_basic`, `mahony`, and `naive` and write the ablation table to `results/ablation_py.md`.

