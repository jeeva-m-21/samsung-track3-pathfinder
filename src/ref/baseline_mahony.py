"""Mahony Attitude Filter + Dead-Reckoning Baseline (SPEC §11.5).

Uses a non-linear complementary attitude filter (Mahony et al.) to correct
roll and pitch drift using accelerometer-derived gravity direction,
then performs gravity-compensated double integration.
"""

import argparse
import sys
from pathlib import Path
from typing import List, Tuple, Union
import numpy as np

from tools.synth import quat_exp, quat_mult, quat_to_rot_matrix
from tools.validate_trajectory import resolve_imu_csv
from src.ref.baseline_naive import compute_initial_tilt


def run_mahony_baseline(
    imu_input: Union[str, Path],
    traj_output: Union[str, Path],
    kp: float = 1.0,
    ki: float = 0.05,
    append_blind: bool = False,
) -> None:
    """Run Mahony attitude filter + dead-reckoning baseline on an IMU dataset."""
    csv_path = resolve_imu_csv(imu_input)
    out_path = Path(traj_output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # 1. Parse IMU CSV
    t_list: List[int] = []
    omega_list: List[List[float]] = []
    acc_list: List[List[float]] = []

    with open(csv_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 7:
                continue
            try:
                t_val = int(parts[0])
                wx = float(parts[1])
                wy = float(parts[2])
                wz = float(parts[3])
                ax = float(parts[4])
                ay = float(parts[5])
                az = float(parts[6])
                t_list.append(t_val)
                omega_list.append([wx, wy, wz])
                acc_list.append([ax, ay, az])
            except ValueError:
                continue

    n = len(t_list)
    if n == 0:
        raise ValueError(f"No valid IMU samples found in {csv_path}")

    t_arr = np.array(t_list, dtype=np.int64)
    omega_arr = np.array(omega_list, dtype=np.float64)
    acc_arr = np.array(acc_list, dtype=np.float64)

    # 2. Initialisation on first 1.0s
    t0_ns = t_arr[0]
    init_mask = (t_arr - t0_ns) <= 1_000_000_000
    if np.sum(init_mask) < 2:
        init_mask[: min(n, 20)] = True

    a_init_mean = np.mean(acc_arr[init_mask], axis=0)
    q0, g_mag = compute_initial_tilt(a_init_mean)
    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)

    # 3. State propagation loop
    p = np.zeros(3, dtype=np.float64)
    v = np.zeros(3, dtype=np.float64)
    q = q0.copy()
    e_int = np.zeros(3, dtype=np.float64)

    with open(out_path, "w", encoding="utf-8") as f_out:
        f_out.write("# timestamp tx ty tz qx qy qz qw\n")

        for k in range(n):
            t_curr = t_arr[k]
            if k == 0:
                dt = 0.0
            else:
                dt = (t_curr - t_arr[k - 1]) * 1e-9

            if dt > 0.0 and dt < 10.0:
                w_k = omega_arr[k]
                a_k = acc_arr[k]

                # Mahony attitude correction:
                # Direction of gravity in body frame: v_b = R(q)^T * [0, 0, 1]
                R_mat = quat_to_rot_matrix(q)
                v_b = R_mat[2, :]  # third row of R = third column of R^T

                a_norm = np.linalg.norm(a_k)
                if a_norm > 1e-3 and np.abs(a_norm - g_mag) < 3.0:
                    a_b = a_k / a_norm
                    # Error is cross product of measured accel and expected gravity in body frame
                    e = np.cross(a_b, v_b)
                else:
                    e = np.zeros(3, dtype=np.float64)

                e_int += e * dt
                w_corr = w_k + kp * e + ki * e_int

                # Update orientation
                delta_q = quat_exp(w_corr * dt)
                q_new = quat_mult(q, delta_q)
                q_new = q_new / np.linalg.norm(q_new)
                if np.dot(q_new, q) < 0.0:
                    q_new = -q_new

                # Update position and velocity
                a_w = R_mat @ a_k + g_w
                p = p + v * dt + 0.5 * a_w * (dt**2)
                v = v + a_w * dt
                q = q_new

            sec = t_curr // 1_000_000_000
            nsec = t_curr % 1_000_000_000
            t_str = f"{sec}.{nsec:09d}"

            w_val, x_val, y_val, z_val = q
            f_out.write(
                f"{t_str} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {x_val:.9f} {y_val:.9f} {z_val:.9f} {w_val:.9f}\n"
            )

        # 4. Handle append-blind if requested
        if append_blind and n > 1:
            median_dt = float(np.median(np.diff(t_arr))) * 1e-9
            blind_steps = int(round(5.0 / median_dt))
            t_last = t_arr[-1]
            dt_ns_step = int(round(median_dt * 1e9))

            for step in range(1, blind_steps + 1):
                t_extrap = t_last + step * dt_ns_step
                p_extrap = p + v * (step * median_dt)
                sec = t_extrap // 1_000_000_000
                nsec = t_extrap % 1_000_000_000
                t_str = f"{sec}.{nsec:09d}"
                w_val, x_val, y_val, z_val = q
                f_out.write(
                    f"{t_str} {p_extrap[0]:.9f} {p_extrap[1]:.9f} {p_extrap[2]:.9f} {x_val:.9f} {y_val:.9f} {z_val:.9f} {w_val:.9f}\n"
                )


def main() -> None:
    parser = argparse.ArgumentParser(description="Run Mahony attitude filter baseline on IMU data.")
    parser.add_argument("imu_input", type=str, help="Path to imu0/data.csv or dataset folder")
    parser.add_argument("traj_output", type=str, help="Path to write estimated_trajectory.txt")
    parser.add_argument("--kp", type=float, default=1.0, help="Proportional gain")
    parser.add_argument("--ki", type=float, default=0.05, help="Integral gain")
    parser.add_argument("--append-blind", action="store_true", help="Extrapolate 5.0 s of poses after last row")

    args = parser.parse_args()
    run_mahony_baseline(args.imu_input, args.traj_output, kp=args.kp, ki=args.ki, append_blind=args.append_blind)


if __name__ == "__main__":
    main()
