"""Controlled IMU dataset degradation and dropout injector.

Applies:
- Gravity misalignment rotation
- Progressive noise growth
- Progressive bias drift (random walk)
- Frame dropouts with 4 encodings: missing rows, NaN, zeros, repeated values
- IMU failure point at sequence tail for blind dead-reckoning evaluation

Outputs:
- Degraded imu0/data.csv
- Ground truth trajectory (TUM format)
- degrade_info.json sidecar (for segment metrics)
"""

import argparse
import json
import shutil
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union
import numpy as np

from tools.synth import quat_exp, quat_to_rot_matrix


def load_degrade_config(config_path: Union[str, Path]) -> Dict[str, Any]:
    """Load JSON degradation configuration."""
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


def read_euroc_imu_csv(csv_path: Union[str, Path]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read EuRoC IMU CSV file.

    Returns:
      t_ns: (N,) int64 nanoseconds
      omega: (N, 3) float64 rad/s
      acc: (N, 3) float64 m/s^2
    """
    t_list = []
    omega_list = []
    acc_list = []

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

    return (
        np.array(t_list, dtype=np.int64),
        np.array(omega_list, dtype=np.float64),
        np.array(acc_list, dtype=np.float64),
    )


def apply_degradations(
    t_ns: np.ndarray,
    omega: np.ndarray,
    acc: np.ndarray,
    config: Dict[str, Any],
    seed: int = 42,
) -> Dict[str, Any]:
    """Apply configured degradations deterministically using seed."""
    rng = np.random.RandomState(seed)
    n = len(t_ns)
    if n == 0:
        raise ValueError("Cannot apply degradations to empty data")

    t0_ns = t_ns[0]
    tend_ns = t_ns[-1]
    t_s = (t_ns - t0_ns) * 1e-9
    total_dur_s = t_s[-1]

    deg_omega = omega.copy()
    deg_acc = acc.copy()

    # 1. Unknown gravity/mounting misalignment rotation
    if config.get("gravity_rotation", False):
        # Sample random rotation angle up to ~15 degrees
        phi_mis = rng.normal(0.0, 0.1, size=3)
        q_mis = quat_exp(phi_mis)
        R_mis = quat_to_rot_matrix(q_mis)  # misaligned to nominal
        R_mis_T = np.swapaxes(R_mis, -1, -2)
        deg_omega = (R_mis_T @ deg_omega[..., np.newaxis]).squeeze(-1)
        deg_acc = (R_mis_T @ deg_acc[..., np.newaxis]).squeeze(-1)

    # 2. Progressive noise ramp
    noise_rate = float(config.get("noise_growth_rate", 0.0))
    if noise_rate > 0.0:
        sigma_g_base = 1.7e-4
        sigma_a_base = 2.0e-3
        g_ramp = sigma_g_base * (1.0 + noise_rate * t_s)[:, np.newaxis]
        a_ramp = sigma_a_base * (1.0 + noise_rate * t_s)[:, np.newaxis]
        deg_omega += rng.normal(0.0, 1.0, size=deg_omega.shape) * g_ramp
        deg_acc += rng.normal(0.0, 1.0, size=deg_acc.shape) * a_ramp

    # 3. Progressive bias random walk
    bias_walk_rate = float(config.get("bias_drift_walk", 0.0))
    if bias_walk_rate > 0.0:
        sigma_bg_walk = 1.9e-5 * np.sqrt(bias_walk_rate)
        sigma_ba_walk = 3.0e-3 * np.sqrt(bias_walk_rate)
        # Compute dt per step
        dt_arr = np.diff(t_s, prepend=t_s[0])
        dt_arr[0] = dt_arr[1] if len(dt_arr) > 1 else 0.005
        dt_sqrt = np.sqrt(np.maximum(dt_arr, 0.0))[:, np.newaxis]

        bg_steps = rng.normal(0.0, sigma_bg_walk, size=deg_omega.shape) * dt_sqrt
        ba_steps = rng.normal(0.0, sigma_ba_walk, size=deg_acc.shape) * dt_sqrt
        deg_omega += np.cumsum(bg_steps, axis=0)
        deg_acc += np.cumsum(ba_steps, axis=0)

    # 4. Dropouts
    dropouts = config.get("dropouts", [])
    valid_mask = np.ones(n, dtype=bool)
    dropout_sidecars = []

    for d in dropouts:
        start_s = float(d["start_s"])
        dur_s = float(d["duration_s"])
        end_s = start_s + dur_s
        encoding = d.get("encoding", "missing").lower()

        w_mask = (t_s >= start_s) & (t_s < end_s)
        if not np.any(w_mask):
            continue

        indices = np.where(w_mask)[0]
        start_idx = indices[0]
        dropout_sidecars.append(
            {
                "start_s": start_s,
                "duration_s": dur_s,
                "end_s": end_s,
                "encoding": encoding,
                "start_ns": int(t_ns[indices[0]]),
                "end_ns": int(t_ns[indices[-1]]),
            }
        )

        if encoding == "missing":
            valid_mask[w_mask] = False
        elif encoding == "nan":
            deg_omega[w_mask] = np.nan
            deg_acc[w_mask] = np.nan
        elif encoding == "zeros":
            deg_omega[w_mask] = 0.0
            deg_acc[w_mask] = 0.0
        elif encoding == "repeated":
            rep_idx = max(0, start_idx - 1)
            deg_omega[w_mask] = deg_omega[rep_idx]
            deg_acc[w_mask] = deg_acc[rep_idx]
        else:
            raise ValueError(f"Unknown dropout encoding {encoding}")

    # 5. Failure at end
    failure_at_end_s = float(config.get("failure_at_end_s", 0.0))
    if failure_at_end_s > 0.0:
        failure_time_s = max(0.0, total_dur_s - failure_at_end_s)
        fail_mask = t_s >= failure_time_s
        valid_mask[fail_mask] = False
        failure_t_ns = int(t0_ns + round(failure_time_s * 1e9))
    else:
        failure_time_s = total_dur_s
        failure_t_ns = int(tend_ns)

    # Apply valid mask (removes missing rows and failed-tail rows)
    out_t_ns = t_ns[valid_mask]
    out_omega = deg_omega[valid_mask]
    out_acc = deg_acc[valid_mask]

    # Checkpoint times every 10s up to failure time
    checkpoints_s = [round(float(s), 2) for s in np.arange(10.0, failure_time_s, 10.0)]

    sidecar = {
        "seed": seed,
        "duration_s": total_dur_s,
        "t0_ns": int(t0_ns),
        "tend_ns": int(tend_ns),
        "failure_time_s": failure_time_s,
        "failure_t_ns": failure_t_ns,
        "dropout_windows": dropout_sidecars,
        "checkpoints_s": checkpoints_s,
    }

    return {
        "t_ns": out_t_ns,
        "omega": out_omega,
        "acc": out_acc,
        "sidecar": sidecar,
    }


def write_degraded_dataset(
    imu_in: Union[str, Path],
    gt_in: Optional[Union[str, Path]],
    out_dir: Union[str, Path],
    config_path: Optional[Union[str, Path]] = None,
    config_dict: Optional[Dict[str, Any]] = None,
    seed: int = 42,
) -> None:
    """Create a complete degraded dataset directory with imu0/data.csv, groundtruth.txt, and sidecar."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if config_dict is not None:
        cfg = config_dict
    elif config_path is not None:
        cfg = load_degrade_config(config_path)
    else:
        raise ValueError("Must provide config_path or config_dict")

    t_ns, omega, acc = read_euroc_imu_csv(imu_in)
    res = apply_degradations(t_ns, omega, acc, cfg, seed=seed)

    # 1. Write degraded IMU CSV
    imu_out_path = out_dir / "imu0" / "data.csv"
    imu_out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(imu_out_path, "w", encoding="utf-8") as f:
        f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        out_t = res["t_ns"]
        out_w = res["omega"]
        out_a = res["acc"]
        for i in range(len(out_t)):
            wx, wy, wz = out_w[i]
            ax, ay, az = out_a[i]
            # Handle NaN formatting cleanly
            wx_str = "nan" if np.isnan(wx) else f"{wx:.10f}"
            wy_str = "nan" if np.isnan(wy) else f"{wy:.10f}"
            wz_str = "nan" if np.isnan(wz) else f"{wz:.10f}"
            ax_str = "nan" if np.isnan(ax) else f"{ax:.10f}"
            ay_str = "nan" if np.isnan(ay) else f"{ay:.10f}"
            az_str = "nan" if np.isnan(az) else f"{az:.10f}"
            f.write(f"{int(out_t[i])},{wx_str},{wy_str},{wz_str},{ax_str},{ay_str},{az_str}\n")

    # 2. Copy or write ground truth unchanged (evaluator compares against full truth)
    if gt_in is not None and Path(gt_in).exists():
        gt_out_path = out_dir / "groundtruth.txt"
        shutil.copyfile(gt_in, gt_out_path)

    # 3. Write sidecar JSON
    sidecar_out_path = out_dir / "degrade_info.json"
    with open(sidecar_out_path, "w", encoding="utf-8") as f:
        json.dump(res["sidecar"], f, indent=2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Inject controlled degradations into an IMU dataset.")
    parser.add_argument("--input-imu", type=str, required=True, help="Input EuRoC imu0/data.csv")
    parser.add_argument("--input-gt", type=str, default=None, help="Input groundtruth.txt (TUM format)")
    parser.add_argument("--config", type=str, default="configs/degrade_default.json", help="Degradation config JSON")
    parser.add_argument("--output-dir", type=str, required=True, help="Output dataset directory")
    parser.add_argument("--seed", type=int, default=42, help="RNG seed for deterministic degradation")

    args = parser.parse_args()
    write_degraded_dataset(
        imu_in=args.input_imu,
        gt_in=args.input_gt,
        out_dir=args.output_dir,
        config_path=args.config,
        seed=args.seed,
    )
    print(f"Degraded dataset written to {args.output_dir}")


if __name__ == "__main__":
    main()
