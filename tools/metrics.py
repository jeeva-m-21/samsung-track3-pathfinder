"""Evaluation metrics and segment scorer wrapping evo and custom segment criteria.

Implements SPEC §6.5 & PLAN.md §11:
- ATE translation RMSE (aligned via SE(3) and unaligned)
- RPE 1-second translation RMSE
- Start-anchored trajectory alignment and RMSE
- Checkpoint errors at designated timepoints
- Blind-5s RMSE and final point error
- Dropout recovery RMSE
- Geodesic rotation error (degrees)
- Scalar tuning objective J (normalized by naive baseline)
"""

import argparse
import copy
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union
import numpy as np

from evo.core import metrics, trajectory, sync
from evo.tools import file_interface

from tools.synth import quat_mult, quat_to_rot_matrix


def load_tum_trajectory(file_path: Union[str, Path]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load TUM format trajectory.

    Returns:
      t_ns: (N,) int64 timestamps (ns)
      p: (N, 3) positions (m)
      q: (N, 4) unit Hamilton quaternions [w, x, y, z]
    """
    path = Path(file_path)
    t_list: List[int] = []
    p_list: List[List[float]] = []
    q_list: List[List[float]] = []

    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                # Timestamp: float seconds string -> int64 ns
                ts_str = parts[0]
                if "." in ts_str:
                    sec_str, nsec_str = ts_str.split(".", 1)
                    sec_val = int(sec_str)
                    nsec_padded = (nsec_str + "000000000")[:9]
                    t_ns = sec_val * 1_000_000_000 + int(nsec_padded)
                else:
                    t_ns = int(ts_str)

                tx = float(parts[1])
                ty = float(parts[2])
                tz = float(parts[3])

                # TUM stores qx, qy, qz, qw
                qx = float(parts[4])
                qy = float(parts[5])
                qz = float(parts[6])
                qw = float(parts[7])

                t_list.append(t_ns)
                p_list.append([tx, ty, tz])
                # Internal convention: [w, x, y, z]
                q_list.append([qw, qx, qy, qz])
            except ValueError:
                continue

    t_arr = np.array(t_list, dtype=np.int64)
    p_arr = np.array(p_list, dtype=np.float64)
    q_arr = np.array(q_list, dtype=np.float64)
    # Ensure unit norm
    if len(q_arr) > 0:
        q_norm = np.linalg.norm(q_arr, axis=1, keepdims=True)
        q_norm = np.where(q_norm < 1e-12, 1.0, q_norm)
        q_arr = q_arr / q_norm

    return t_arr, p_arr, q_arr


def geodesic_distance_deg(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Compute shortest geodesic angular distance between unit quaternions in degrees.

    q1, q2: (N, 4) in [w, x, y, z]
    """
    # Dot product: rho = |w1*w2 + x1*x2 + y1*y2 + z1*z2|
    dot = np.abs(np.sum(q1 * q2, axis=-1))
    dot = np.clip(dot, 0.0, 1.0)
    angle_rad = 2.0 * np.arccos(dot)
    return np.degrees(angle_rad)


def align_start_anchored(
    p_est: np.ndarray,
    q_est: np.ndarray,
    p_ref0: np.ndarray,
    q_ref0: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Transform estimated trajectory so its initial pose matches reference initial pose."""
    p_est0 = p_est[0]
    q_est0 = q_est[0]

    # q_rel = q_est0^* (x) q_est(t)
    q_est0_conj = np.array([q_est0[0], -q_est0[1], -q_est0[2], -q_est0[3]], dtype=np.float64)
    q_rel = quat_mult(q_est0_conj[np.newaxis, :], q_est)

    # p_rel = p_est(t) - p_est0
    p_rel = p_est - p_est0[np.newaxis, :]

    # In ref initial frame:
    R_ref0 = quat_to_rot_matrix(q_ref0)
    p_aligned = p_ref0[np.newaxis, :] + (R_ref0 @ p_rel[..., np.newaxis]).squeeze(-1)
    q_aligned = quat_mult(q_ref0[np.newaxis, :], q_rel)
    q_aligned = q_aligned / np.linalg.norm(q_aligned, axis=-1, keepdims=True)

    return p_aligned, q_aligned


def compute_trajectory_metrics(
    ref_path: Union[str, Path],
    est_path: Union[str, Path],
    sidecar_path: Optional[Union[str, Path]] = None,
) -> Dict[str, float]:
    """Compute all evaluation and segment metrics for an estimated trajectory against reference."""
    ref_p = Path(ref_path)
    est_p = Path(est_path)

    # 1. Load via evo file interface
    t_ref_evo = file_interface.read_tum_trajectory_file(str(ref_p))
    t_est_evo = file_interface.read_tum_trajectory_file(str(est_p))

    # Synchronize timestamps (allow up to 10 ms max difference)
    t_ref_sync, t_est_sync = sync.associate_trajectories(t_ref_evo, t_est_evo, max_diff=0.01)

    # 2. APE Unaligned
    ape_unaligned = metrics.APE(metrics.PoseRelation.translation_part)
    ape_unaligned.process_data((t_ref_sync, t_est_sync))
    ate_rmse_unaligned = float(ape_unaligned.get_statistic(metrics.StatisticsType.rmse))

    # 3. APE Aligned (SE(3) Umeyama alignment, no scale correction)
    t_est_aligned = copy.deepcopy(t_est_sync)
    t_est_aligned.align(t_ref_sync, correct_scale=False)

    ape_aligned = metrics.APE(metrics.PoseRelation.translation_part)
    ape_aligned.process_data((t_ref_sync, t_est_aligned))
    ate_rmse_aligned = float(ape_aligned.get_statistic(metrics.StatisticsType.rmse))

    # 4. RPE 1-second Translation RMSE
    dt = float(np.median(np.diff(t_ref_sync.timestamps))) if len(t_ref_sync.timestamps) > 1 else 0.005
    frames_1s = max(1, int(round(1.0 / dt)))
    rpe_trans_1s = 0.0
    try:
        rpe_metric = metrics.RPE(
            metrics.PoseRelation.translation_part,
            delta=frames_1s,
            delta_unit=metrics.Unit.frames,
            all_pairs=False,
        )
        rpe_metric.process_data((t_ref_sync, t_est_aligned))
        rpe_trans_1s = float(rpe_metric.get_statistic(metrics.StatisticsType.rmse))
    except Exception:
        rpe_trans_1s = 0.0

    # 5. Start-anchored trajectory error
    t_ns_ref, p_ref, q_ref = load_tum_trajectory(ref_p)
    t_ns_est, p_est, q_est = load_tum_trajectory(est_p)

    # Associate indices
    t_ref_s = (t_ns_ref - t_ns_ref[0]) * 1e-9
    t_est_s = (t_ns_est - t_ns_ref[0]) * 1e-9

    p_est_anchored, q_est_anchored = align_start_anchored(p_est, q_est, p_ref[0], q_ref[0])
    # Compute error on synchronized prefix
    min_len = min(len(p_ref), len(p_est_anchored))
    pos_err_anchored = np.linalg.norm(p_est_anchored[:min_len] - p_ref[:min_len], axis=1)
    start_anchored_rmse = float(np.sqrt(np.mean(pos_err_anchored**2)))

    # 6. Rotation Error (geodesic distance in degrees)
    rot_err_arr = geodesic_distance_deg(q_est_anchored[:min_len], q_ref[:min_len])
    rot_err_deg = float(np.mean(rot_err_arr))

    # 7. Segment metrics from sidecar (if available)
    checkpoint_err = float(ate_rmse_aligned)
    blind5_rmse = 0.0
    blind5_final_err = 0.0
    dropout_rmse = float(ate_rmse_aligned)

    if sidecar_path is not None and Path(sidecar_path).exists():
        with open(sidecar_path, "r", encoding="utf-8") as f:
            sidecar = json.load(f)

        # Checkpoints
        checkpoints_s = sidecar.get("checkpoints_s", [])
        if checkpoints_s:
            chk_errs = []
            for chk_s in checkpoints_s:
                idx = np.argmin(np.abs(t_ref_s[:min_len] - chk_s))
                chk_errs.append(pos_err_anchored[idx])
            checkpoint_err = float(np.mean(chk_errs))

        # Blind 5s
        failure_time_s = sidecar.get("failure_time_s")
        if failure_time_s is not None:
            blind_mask = (t_ref_s[:min_len] >= failure_time_s) & (t_ref_s[:min_len] <= failure_time_s + 5.0)
            if np.any(blind_mask):
                blind5_rmse = float(np.sqrt(np.mean(pos_err_anchored[blind_mask] ** 2)))
                blind5_final_err = float(pos_err_anchored[blind_mask][-1])
            else:
                blind5_rmse = float(pos_err_anchored[-1])
                blind5_final_err = float(pos_err_anchored[-1])

        # Dropouts
        dropouts = sidecar.get("dropout_windows", [])
        if dropouts:
            drop_errs = []
            for d in dropouts:
                d_start = float(d["start_s"])
                d_end = float(d["end_s"]) + 5.0  # inside dropout and 5s after
                d_mask = (t_ref_s[:min_len] >= d_start) & (t_ref_s[:min_len] <= d_end)
                if np.any(d_mask):
                    drop_errs.extend(pos_err_anchored[d_mask].tolist())
            if drop_errs:
                dropout_rmse = float(np.sqrt(np.mean(np.array(drop_errs) ** 2)))

    return {
        "ate_rmse_aligned": ate_rmse_aligned,
        "ate_rmse_unaligned": ate_rmse_unaligned,
        "rpe_trans_1s": rpe_trans_1s,
        "start_anchored_rmse": start_anchored_rmse,
        "checkpoint_err": checkpoint_err,
        "blind5_rmse": blind5_rmse,
        "blind5_final_err": blind5_final_err,
        "dropout_rmse": dropout_rmse,
        "rot_err_deg": rot_err_deg,
    }


def compute_objective_J(metrics_dict: Dict[str, float], naive_metrics_dict: Dict[str, float]) -> float:
    """Compute scalar tuning objective J normalized by naive baseline (PLAN.md §11.4).

    J = 0.30 * norm(checkpoint_err) + 0.20 * norm(blind5_rmse) + 0.10 * norm(dropout_rmse)
      + 0.25 * norm(ate_rmse_aligned) + 0.15 * norm(rpe_trans_1s)
    """
    weights = {
        "checkpoint_err": 0.30,
        "blind5_rmse": 0.20,
        "dropout_rmse": 0.10,
        "ate_rmse_aligned": 0.25,
        "rpe_trans_1s": 0.15,
    }

    total_J = 0.0
    for key, w in weights.items():
        val = metrics_dict.get(key, 0.0)
        naive_val = naive_metrics_dict.get(key, 1.0)
        norm_val = val / max(naive_val, 1e-6)
        total_J += w * norm_val

    return float(total_J)


def main() -> None:
    parser = argparse.ArgumentParser(description="Compute trajectory evaluation metrics against ground truth.")
    parser.add_argument("ref_file", type=str, help="Reference ground truth trajectory (TUM format)")
    parser.add_argument("est_file", type=str, help="Estimated trajectory (TUM format)")
    parser.add_argument("--sidecar", type=str, default=None, help="Path to degrade_info.json sidecar")
    parser.add_argument("--naive-metrics", type=str, default=None, help="Path to JSON metrics of naive baseline for J")
    parser.add_argument("--json-out", type=str, default=None, help="Optional output path to write JSON metrics")

    args = parser.parse_args()

    results = compute_trajectory_metrics(args.ref_file, args.est_file, sidecar_path=args.sidecar)

    if args.naive_metrics and Path(args.naive_metrics).exists():
        with open(args.naive_metrics, "r", encoding="utf-8") as f:
            naive_dict = json.load(f)
        results["scalar_objective_J"] = compute_objective_J(results, naive_dict)

    if args.json_out:
        out_p = Path(args.json_out)
        out_p.parent.mkdir(parents=True, exist_ok=True)
        with open(out_p, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2)

    print("Trajectory Evaluation Metrics:")
    for k, v in results.items():
        print(f"  {k:22s}: {v:.6f}")


if __name__ == "__main__":
    main()
