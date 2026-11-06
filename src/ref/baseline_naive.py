"""Naive Dead-Reckoning Baseline (SPEC §11.5, §12.2, §12.3).

Initialises tilt from the first 1.0 s of IMU samples (assuming static),
then integrates raw specific force and gyro without bias updates or corrections.
Exhibits quadratic position drift ~ 0.5 * a_bias * t^2.
"""

import argparse
import sys
from pathlib import Path
from typing import List, Tuple, Union
import numpy as np

from tools.synth import quat_exp, quat_mult, quat_to_rot_matrix
from tools.validate_trajectory import resolve_imu_csv


def compute_initial_tilt(a_mean: np.ndarray) -> Tuple[np.ndarray, float]:
    """Compute initial unit quaternion [w, x, y, z] aligning mean specific force to world +z (SPEC §12.2).

    R(q) * (a_mean / |a_mean|) = [0, 0, 1]^T.
    Returns:
      q0: (4,) unit quaternion [w, x, y, z]
      g_mag: float clamped to [9.0, 10.5]
    """
    g_raw = float(np.linalg.norm(a_mean))
    g_mag = float(np.clip(g_raw, 9.0, 10.5))

    v = a_mean / (g_raw if g_raw > 1e-6 else 1.0)  # direction of specific force in body
    u = np.array([0.0, 0.0, 1.0], dtype=np.float64)  # target in world

    # Dot product
    cos_theta = float(np.dot(v, u))

    if cos_theta > 0.999999:
        q0 = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    elif cos_theta < -0.999999:
        # 180 degree rotation around x axis
        q0 = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)
    else:
        # Rotation from body to world that takes v to u:
        # Axis of rotation in world or body
        axis = np.cross(v, u)
        axis_norm = np.linalg.norm(axis)
        axis = axis / axis_norm
        angle = np.arccos(np.clip(cos_theta, -1.0, 1.0))
        q0 = np.array(
            [
                np.cos(0.5 * angle),
                axis[0] * np.sin(0.5 * angle),
                axis[1] * np.sin(0.5 * angle),
                axis[2] * np.sin(0.5 * angle),
            ],
            dtype=np.float64,
        )

    q0 = q0 / np.linalg.norm(q0)
    return q0, g_mag


def run_naive_baseline(
    imu_input: Union[str, Path],
    traj_output: Union[str, Path],
    append_blind: bool = False,
) -> None:
    """Run naive strapdown baseline on an IMU dataset."""
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

    # 2. Initialisation on first 1.0s (or all if < 1.0s)
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

    with open(out_path, "w", encoding="utf-8") as f_out:
        f_out.write("# timestamp tx ty tz qx qy qz qw\n")

        for k in range(n):
            t_curr = t_arr[k]
            if k == 0:
                dt = 0.0
            else:
                dt = (t_curr - t_arr[k - 1]) * 1e-9

            if dt > 0.0 and dt < 10.0:  # Valid time increment
                w_k = omega_arr[k]
                a_k = acc_arr[k]

                # Update orientation
                delta_q = quat_exp(w_k * dt)
                q_new = quat_mult(q, delta_q)
                q_new = q_new / np.linalg.norm(q_new)
                if np.dot(q_new, q) < 0.0:
                    q_new = -q_new

                # Update position & velocity using q_old
                R_old = quat_to_rot_matrix(q)
                a_w = R_old @ a_k + g_w

                p = p + v * dt + 0.5 * a_w * (dt**2)
                v = v + a_w * dt
                q = q_new

            # Format timestamp from int64 nanoseconds
            sec = t_curr // 1_000_000_000
            nsec = t_curr % 1_000_000_000
            t_str = f"{sec}.{nsec:09d}"

            # q is [w, x, y, z], output format requires tx ty tz qx qy qz qw
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
    parser = argparse.ArgumentParser(description="Run naive dead-reckoning baseline on IMU data.")
    parser.add_argument("imu_input", type=str, help="Path to imu0/data.csv or dataset folder")
    parser.add_argument("traj_output", type=str, help="Path to write estimated_trajectory.txt")
    parser.add_argument("--append-blind", action="store_true", help="Extrapolate 5.0 s of poses after last row")

    args = parser.parse_args()
    run_naive_baseline(args.imu_input, args.traj_output, append_blind=args.append_blind)


if __name__ == "__main__":
    main()
