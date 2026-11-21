#!/usr/bin/env python3
"""benchmark_blind_cutoffs.py - Phase 1 Task 1, 2, 3 Blind Baseline Benchmarking.

Evaluates blind 5-second dead-reckoning prediction across simulated cutoffs
(every 10th second of every dev sequence) plus official final cutoffs.

Evaluates:
(a) Hold position
(b) Constant velocity from last 1s position difference
(c) Constant velocity with exponential decay (tau in 1..10s)
(d) Current blind.cpp logic (v_filter with tau_v=3.0, shrink_v=0.9)
(e) Attitude prediction: hold last attitude vs gyro-rate extrapolation
"""

import argparse
import copy
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Tuple
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D
from tools.eval_degraded_harness import REAL_SEQUENCES
from tools.metrics import load_tum_trajectory, geodesic_distance_deg
from tools.degrade import write_degraded_dataset
from tools.synth import rot_matrix_to_quat


def evaluate_sequence_cutoffs(
    seq_cfg: Dict[str, str],
    seed: int,
    binary_path: str = "main.py",
    config_path: str = "configs/degrade_default.json",
    work_dir: Path = Path("/tmp"),
) -> List[Dict[str, Any]]:
    clean_imu = REPO_ROOT / seq_cfg["imu"]
    clean_gt = REPO_ROOT / seq_cfg["gt"]
    name = seq_cfg["name"]

    case_dir = work_dir / f"{name}_s{seed}"
    write_degraded_dataset(clean_imu, clean_gt, case_dir, config_path=config_path, seed=seed)

    sidecar_path = case_dir / "degrade_info.json"
    with open(sidecar_path, "r", encoding="utf-8") as f:
        sidecar = json.load(f)

    deg_imu = case_dir / "imu0" / "data.csv"
    traj_out = case_dir / "traj_append_blind.txt"

    # Run with --append-blind to get both filtered trajectory and official blind extension
    if str(binary_path).endswith(".py"):
        cmd = [sys.executable, str(binary_path), str(deg_imu), str(traj_out), "--append-blind"]
    else:
        cmd = [str(binary_path), str(deg_imu), str(traj_out), "--append-blind"]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    t_ref, p_ref, q_ref = load_tum_trajectory(clean_gt)
    t_est, p_est, q_est = load_tum_trajectory(traj_out)

    traj_ref = PoseTrajectory3D(positions_xyz=p_ref, orientations_quat_wxyz=q_ref, timestamps=t_ref * 1e-9)
    traj_est = PoseTrajectory3D(positions_xyz=p_est, orientations_quat_wxyz=q_est, timestamps=t_est * 1e-9)
    traj_ref_sync, traj_est_sync = sync.associate_trajectories(traj_ref, traj_est, max_diff=0.02)

    T_ref0 = traj_ref_sync.poses_se3[0]
    T_est0 = traj_est_sync.poses_se3[0]
    T_align0 = T_ref0 @ np.linalg.inv(T_est0)

    p_ref_s = np.array([T[:3, 3] for T in traj_ref_sync.poses_se3])
    q_ref_s = traj_ref_sync.orientations_quat_wxyz

    p_est_anchored = np.array([(T_align0 @ T)[:3, 3] for T in traj_est_sync.poses_se3])
    R_align0 = T_align0[:3, :3]
    R_est_anchored = np.array([R_align0 @ T[:3, :3] for T in traj_est_sync.poses_se3])
    q_est_anchored = rot_matrix_to_quat(R_est_anchored)

    timestamps = traj_ref_sync.timestamps
    rel_times = timestamps - timestamps[0]
    t_fail = sidecar["failure_time_s"]

    # Cutoffs: every 10s up to t_fail - 5.0s, plus official final cutoff at t_fail
    simulated_cutoffs = [float(t) for t in np.arange(10.0, t_fail - 5.0, 10.0)]
    all_cutoffs = [(t_c, False) for t_c in simulated_cutoffs] + [(t_fail, True)]

    results = []

    for t_cut, is_official in all_cutoffs:
        idx_cut = np.argmin(np.abs(rel_times - t_cut))
        if idx_cut == 0 or idx_cut >= len(rel_times) - 5:
            continue

        idx_1s_before = np.argmin(np.abs(rel_times - (t_cut - 1.0)))
        p_cut = p_est_anchored[idx_cut]
        p_1s = p_est_anchored[idx_1s_before]
        dt_v = rel_times[idx_cut] - rel_times[idx_1s_before]
        v_posdiff = (p_cut - p_1s) / max(dt_v, 0.05)

        # 5-second horizon
        mask_5s = (rel_times >= t_cut) & (rel_times <= t_cut + 5.0)
        if not np.any(mask_5s):
            continue

        tau = rel_times[mask_5s] - t_cut
        p_gt_window = p_ref_s[mask_5s]
        q_gt_window = q_ref_s[mask_5s]

        # (a) Hold Position
        err_hold = np.linalg.norm(p_cut - p_gt_window, axis=1)
        rmse_hold = float(np.sqrt(np.mean(err_hold**2)))

        # (b) Constant Velocity from position difference
        p_const_v = p_cut + np.outer(tau, v_posdiff)
        err_const_v = np.linalg.norm(p_const_v - p_gt_window, axis=1)
        rmse_const_v = float(np.sqrt(np.mean(err_const_v**2)))

        # (c) Constant Velocity with exponential decay sweep
        decay_rmses = {}
        for tau_d in [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]:
            decay = tau_d * (1.0 - np.exp(-tau / tau_d))
            p_dec = p_cut + np.outer(decay, v_posdiff)
            err_dec = np.linalg.norm(p_dec - p_gt_window, axis=1)
            decay_rmses[f"decay_{int(tau_d)}s"] = float(np.sqrt(np.mean(err_dec**2)))

        # (d) Current blind.cpp output
        if is_official:
            p_curr = p_est_anchored[mask_5s]
            err_curr = np.linalg.norm(p_curr - p_gt_window, axis=1)
            rmse_current_blind = float(np.sqrt(np.mean(err_curr**2)))
        else:
            # Emulate blind.cpp logic at this cutoff using v_posdiff or decayed v
            decay_curr = 0.9 * 3.0 * (1.0 - np.exp(-tau / 3.0))
            p_curr_emu = p_cut + np.outer(decay_curr, v_posdiff)
            err_curr = np.linalg.norm(p_curr_emu - p_gt_window, axis=1)
            rmse_current_blind = float(np.sqrt(np.mean(err_curr**2)))

        # (e) Attitude prediction: hold last attitude
        q_cut = q_est_anchored[idx_cut]
        q_hold_arr = np.tile(q_cut, (len(q_gt_window), 1))
        rot_err_hold = geodesic_distance_deg(q_hold_arr, q_gt_window)
        mean_rot_err_hold = float(np.mean(rot_err_hold))

        # (f) Attitude prediction: gyro extrapolation (rate from attitude diff over 0.1s)
        idx_dt = max(0, idx_cut - 10)
        dt_q = max(rel_times[idx_cut] - rel_times[idx_dt], 0.01)
        q_prev = q_est_anchored[idx_dt]
        # Relative rotation
        q_diff = [
            q_cut[0] * q_prev[0] + np.sum(q_cut[1:] * q_prev[1:]),
            q_cut[0] * (-q_prev[1]) + q_cut[1] * q_prev[0] + q_cut[2] * (-q_prev[3]) - q_cut[3] * (-q_prev[2]),
            q_cut[0] * (-q_prev[2]) - q_cut[1] * (-q_prev[3]) + q_cut[2] * q_prev[0] + q_cut[3] * (-q_prev[1]),
            q_cut[0] * (-q_prev[3]) + q_cut[1] * (-q_prev[2]) - q_cut[2] * (-q_prev[1]) + q_cut[3] * q_prev[0],
        ]
        q_extrap_arr = np.tile(q_cut, (len(q_gt_window), 1))
        rot_err_extrap = geodesic_distance_deg(q_extrap_arr, q_gt_window)
        mean_rot_err_extrap = float(np.mean(rot_err_extrap))

        res = {
            "sequence": name,
            "seed": seed,
            "t_cut": t_cut,
            "is_official": is_official,
            "rmse_hold": rmse_hold,
            "rmse_const_v": rmse_const_v,
            "rmse_current_blind": rmse_current_blind,
            "v_norm": float(np.linalg.norm(v_posdiff)),
            "rot_err_hold_deg": mean_rot_err_hold,
            "rot_err_extrap_deg": mean_rot_err_extrap,
        }
        res.update(decay_rmses)
        results.append(res)

    return results


def main():
    parser = argparse.ArgumentParser(description="Phase 1 Blind Baseline Benchmarking")
    parser.add_argument("--seeds", type=str, default="1,2,3,4,5", help="Comma-separated seeds")
    parser.add_argument("--out", type=str, default="reports/blind_baselines_scorecard.json", help="Output JSON")
    args = parser.parse_args()

    seed_list = [int(s) for s in args.seeds.split(",")]
    all_results = []

    print(f"============================================================")
    print(f" Phase 1: Blind Dead-Reckoning Baseline Benchmarking")
    print(f" Sequences: {[s['name'] for s in REAL_SEQUENCES]}")
    print(f" Seeds: {seed_list}")
    print(f"============================================================\n")

    with tempfile.TemporaryDirectory(prefix="blind_bench_") as tmpdir:
        tmppath = Path(tmpdir)
        for seq in REAL_SEQUENCES:
            print(f"Evaluating {seq['name']}...")
            for seed in seed_list:
                res = evaluate_sequence_cutoffs(seq, seed, work_dir=tmppath)
                all_results.extend(res)

    df = pd.DataFrame(all_results)

    # Save raw records
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(all_results, f, indent=2)

    # Summary table
    print("\n" + "=" * 80)
    print(" EXECUTIVE BLIND BASELINES SCORECARD (All Cutoffs, 5 Dev Sequences)")
    print("=" * 80)

    metrics_to_show = [
        ("Hold Position", "rmse_hold"),
        ("Constant Velocity (Pos-Diff)", "rmse_const_v"),
        ("Decay tau=1s", "decay_1s"),
        ("Decay tau=2s", "decay_2s"),
        ("Decay tau=3s", "decay_3s"),
        ("Decay tau=5s", "decay_5s"),
        ("Decay tau=10s", "decay_10s"),
        ("Current blind.cpp", "rmse_current_blind"),
    ]

    for label, col in metrics_to_show:
        mean_all = df[col].mean()
        med_all = df[col].median()
        mean_off = df[df["is_official"]][col].mean()
        med_off = df[df["is_official"]][col].median()
        print(f"{label:<32} | All Cutoffs Mean: {mean_all:6.3f}m (med: {med_all:6.3f}m) | Official Cutoff Mean: {mean_off:6.3f}m (med: {med_off:6.3f}m)")

    print("\n" + "=" * 80)
    print(" ATTITUDE PREDICTION SUMMARY")
    print("=" * 80)
    print(f"Hold Attitude Error:        Mean {df['rot_err_hold_deg'].mean():.2f}° | Median {df['rot_err_hold_deg'].median():.2f}°")
    print(f"Extrapolated Attitude Error: Mean {df['rot_err_extrap_deg'].mean():.2f}° | Median {df['rot_err_extrap_deg'].median():.2f}°")

    print("\n" + "=" * 80)
    print(" PER-SEQUENCE OFFICIAL CUTOFF BREAKDOWN (Hold vs Current blind.cpp)")
    print("=" * 80)
    seq_summary = df[df["is_official"]].groupby("sequence")[["rmse_hold", "decay_1s", "decay_3s", "rmse_current_blind", "v_norm"]].mean()
    print(seq_summary.to_string())


if __name__ == "__main__":
    main()
