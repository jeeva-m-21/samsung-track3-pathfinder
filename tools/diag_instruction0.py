#!/usr/bin/env python3
"""Instruction 0 Diagnostic Script:
1. Computes evo_ape tum (-a) and evo_rpe tum (-a, 1s delta) for:
   - All 8 clean datasets
   - Degraded Dev Seed 1 (5 sequences)
2. Detailed gap breakdown for MH_01_easy vs room1 on Seed 1 and Seed 301.
3. Fresh seeds 301-310 exact medians.
"""

import sys
import os
import json
import copy
import subprocess
import tempfile
import numpy as np
import pandas as pd
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from tools.benchmark_suite import DATASETS
from tools.eval_degraded_harness import REAL_SEQUENCES
from tools.degrade import write_degraded_dataset
from tools.metrics import compute_trajectory_metrics
from evo.core import metrics, sync
from evo.tools import file_interface

MAIN_BIN = REPO_ROOT / "main"
CONFIG_PATH = REPO_ROOT / "configs/degrade_default.json"


def evaluate_evo(ref_path: Path, est_path: Path):
    t_ref = file_interface.read_tum_trajectory_file(str(ref_path))
    t_est = file_interface.read_tum_trajectory_file(str(est_path))
    t_ref_sync, t_est_sync = sync.associate_trajectories(t_ref, t_est, max_diff=0.01)

    # APE with Umeyama SE(3) alignment
    t_est_aligned = copy.deepcopy(t_est_sync)
    t_est_aligned.align(t_ref_sync, correct_scale=False)

    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((t_ref_sync, t_est_aligned))
    ape_rmse = float(ape_metric.get_statistic(metrics.StatisticsType.rmse))

    dt = float(np.median(np.diff(t_ref_sync.timestamps))) if len(t_ref_sync.timestamps) > 1 else 0.005
    frames_1s = max(1, int(round(1.0 / dt)))

    # RPE trans (1s)
    rpe_trans_metric = metrics.RPE(
        metrics.PoseRelation.translation_part,
        delta=frames_1s,
        delta_unit=metrics.Unit.frames,
        all_pairs=False,
    )
    rpe_trans_metric.process_data((t_ref_sync, t_est_aligned))
    rpe_trans_rmse = float(rpe_trans_metric.get_statistic(metrics.StatisticsType.rmse))

    # RPE rot (1s in deg/s)
    rpe_rot_metric = metrics.RPE(
        metrics.PoseRelation.rotation_angle_deg,
        delta=frames_1s,
        delta_unit=metrics.Unit.frames,
        all_pairs=False,
    )
    rpe_rot_metric.process_data((t_ref_sync, t_est_aligned))
    rpe_rot_rmse = float(rpe_rot_metric.get_statistic(metrics.StatisticsType.rmse))

    return ape_rmse, rpe_trans_rmse, rpe_rot_rmse


def run_clean_suite():
    print("=" * 80)
    print("PART 1A: CLEAN BENCHMARK SUITE (8 Datasets)")
    print("=" * 80)
    rows = []
    with tempfile.TemporaryDirectory() as tmp_dir:
        for name, imu_path, gt_path in DATASETS:
            imu_file = REPO_ROOT / imu_path
            gt_file = REPO_ROOT / gt_path
            out_traj = Path(tmp_dir) / f"{name}_clean.txt"

            res = subprocess.run([str(MAIN_BIN), str(imu_file), str(out_traj)],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if res.returncode != 0:
                print(f"FAILED on {name}: {res.stderr}")
                continue

            ape_rmse, rpe_trans, rpe_rot = evaluate_evo(gt_file, out_traj)
            rows.append({
                "Dataset": name,
                "APE RMSE (m)": ape_rmse,
                "RPE trans 1s (m)": rpe_trans,
                "RPE rot 1s (deg/s)": rpe_rot,
            })
            print(f"  {name:<15}: APE = {ape_rmse:.4f} m | RPE_trans = {rpe_trans:.4f} m | RPE_rot = {rpe_rot:.4f} deg/s")

    df = pd.DataFrame(rows)
    return df


def run_degraded_dev_seed1():
    print("\n" + "=" * 80)
    print("PART 1B: DEGRADED DEV SEED 1 (5 Sequences)")
    print("=" * 80)
    rows = []
    with tempfile.TemporaryDirectory() as tmp_dir:
        for seq in REAL_SEQUENCES:
            name = seq["name"]
            imu_in = REPO_ROOT / seq["imu"]
            gt_in = REPO_ROOT / seq["gt"]

            deg_dir = Path(tmp_dir) / f"{name}_s1"
            write_degraded_dataset(
                imu_in=imu_in,
                gt_in=gt_in,
                out_dir=deg_dir,
                config_path=CONFIG_PATH,
                seed=1,
            )
            deg_imu = deg_dir / "imu0" / "data.csv"
            deg_gt = deg_dir / "groundtruth.txt"
            sidecar = deg_dir / "degrade_info.json"
            out_traj = deg_dir / "est_traj.txt"

            res = subprocess.run([str(MAIN_BIN), str(deg_imu), str(out_traj), "--append-blind"],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if res.returncode != 0:
                print(f"FAILED on degraded {name}: {res.stderr}")
                continue

            ape_rmse, rpe_trans, rpe_rot = evaluate_evo(deg_gt, out_traj)
            custom_metrics = compute_trajectory_metrics(deg_gt, out_traj, sidecar)
            blind5 = custom_metrics["blind5_rmse"]
            gap2s = custom_metrics["gap_2s_rmse"]

            rows.append({
                "Sequence": name,
                "APE RMSE (m)": ape_rmse,
                "RPE trans 1s (m)": rpe_trans,
                "RPE rot 1s (deg/s)": rpe_rot,
                "Blind 5s RMSE (m)": blind5,
                "Gap 2s RMSE (m)": gap2s,
            })
            print(f"  {name:<15}: APE = {ape_rmse:.4f} m | RPE_trans = {rpe_trans:.4f} m | RPE_rot = {rpe_rot:.4f} deg/s | Blind5 = {blind5:.4f} m | Gap2 = {gap2s:.4f} m")

    df = pd.DataFrame(rows)
    return df


def inspect_gap_behavior(seed: int):
    print("\n" + "=" * 80)
    print(f"PART 2: DETAILED GAP ANALYSIS (Seed {seed})")
    print("=" * 80)
    for seq_name in ["MH_01_easy", "room1"]:
        seq_cfg = [s for s in REAL_SEQUENCES if s["name"] == seq_name][0]
        with tempfile.TemporaryDirectory() as tmp_dir:
            deg_dir = Path(tmp_dir) / f"{seq_name}_s{seed}"
            write_degraded_dataset(
                imu_in=REPO_ROOT / seq_cfg["imu"],
                gt_in=REPO_ROOT / seq_cfg["gt"],
                out_dir=deg_dir,
                config_path=CONFIG_PATH,
                seed=seed,
            )
            deg_imu = deg_dir / "imu0" / "data.csv"
            deg_gt = deg_dir / "groundtruth.txt"
            sidecar = deg_dir / "degrade_info.json"
            out_traj = deg_dir / "est_traj.txt"

            with open(sidecar) as f:
                sc = json.load(f)

            # Inspect gap info
            gaps = sc.get("events", {}).get("dropouts", [])
            print(f"\n[{seq_name} Seed {seed}] Dropout events in sidecar:")
            for g in gaps:
                print(f"   t = {g.get('start_sec', g.get('t_start'))}s to {g.get('end_sec', g.get('t_end'))}s, duration: {g.get('duration_sec', g.get('duration'))}s, type: {g.get('type')}")

            # Run estimator
            subprocess.run([str(MAIN_BIN), str(deg_imu), str(out_traj), "--append-blind"], check=True)

            # Load IMU timestamps & GT vs Est
            df_imu = pd.read_csv(deg_imu)
            imu_t = df_imu.iloc[:, 0].values * 1e-9  # seconds
            t_start = imu_t[0]
            imu_rel_t = imu_t - t_start

            # Find invalid / gap rows in degraded IMU
            is_nan = df_imu.iloc[:, 1:].isna().any(axis=1).values
            is_zero = (df_imu.iloc[:, 1:] == 0.0).all(axis=1).values
            dt_diff = np.diff(imu_t, prepend=imu_t[0])
            is_time_jump = dt_diff > 0.05

            gap_rows = np.where(is_nan | is_zero | is_time_jump)[0]
            if len(gap_rows) > 0:
                print(f"   Degraded IMU gap rows detected: {len(gap_rows)} rows")
                print(f"   First gap row at relative t = {imu_rel_t[gap_rows[0]]:.3f}s, last at relative t = {imu_rel_t[gap_rows[-1]]:.3f}s")
                # Group contiguous chunks
                chunks = np.split(gap_rows, np.where(np.diff(gap_rows) > 1)[0] + 1)
                for i, c in enumerate(chunks):
                    print(f"     Chunk {i+1}: rows {c[0]}..{c[-1]} | rel t = {imu_rel_t[c[0]]:.3f}s .. {imu_rel_t[c[-1]]:.3f}s (dur = {imu_rel_t[c[-1]] - imu_rel_t[c[0]]:.3f}s)")

            # Frame by frame analysis around t = 24.90 .. 27.10s
            t_est = file_interface.read_tum_trajectory_file(str(out_traj))
            t_ref = file_interface.read_tum_trajectory_file(str(deg_gt))
            t_ref_sync, t_est_sync = sync.associate_trajectories(t_ref, t_est, max_diff=0.01)

            t_est_aligned = copy.deepcopy(t_est_sync)
            t_est_aligned.align(t_ref_sync, correct_scale=False)

            timestamps = t_ref_sync.timestamps - t_ref_sync.timestamps[0]
            p_ref = np.array([T[:3, 3] for T in t_ref_sync.poses_se3])
            p_est = np.array([T[:3, 3] for T in t_est_aligned.poses_se3])
            pos_err = np.linalg.norm(p_est - p_ref, axis=1)

            # Yaw angle extraction from rotation matrices
            R_ref = np.array([T[:3, :3] for T in t_ref_sync.poses_se3])
            R_est = np.array([T[:3, :3] for T in t_est_aligned.poses_se3])
            # Relative rotation R_err = R_ref^T * R_est
            R_err = np.einsum('nij,njk->nik', np.swapaxes(R_ref, 1, 2), R_est)
            # Yaw error (rotation around z)
            yaw_err_deg = np.degrees(np.arctan2(R_err[:, 1, 0], R_err[:, 0, 0]))

            # Find window 24.90 .. 27.10
            mask_window = (timestamps >= 24.90) & (timestamps <= 27.10)
            idx_window = np.where(mask_window)[0]
            if len(idx_window) > 0:
                print(f"\n   Frame-by-frame error in window [24.90s .. 27.10s]:")
                print(f"   {'t (rel s)':<10} {'Pos Err (m)':<14} {'Yaw Err (deg)':<14} {'Ref Vel Speed (m/s)':<20}")
                stride = max(1, len(idx_window) // 10)
                for idx in idx_window[::stride]:
                    t_val = timestamps[idx]
                    p_err = pos_err[idx]
                    y_err = yaw_err_deg[idx]
                    # approximate ref velocity
                    if idx > 0:
                        dt_ref = timestamps[idx] - timestamps[idx-1]
                        v_speed = np.linalg.norm(p_ref[idx] - p_ref[idx-1]) / dt_ref if dt_ref > 0 else 0.0
                    else:
                        v_speed = 0.0
                    print(f"   {t_val:<10.3f} {p_err:<14.4f} {y_err:<14.2f} {v_speed:<20.3f}")


if __name__ == "__main__":
    run_clean_suite()
    run_degraded_dev_seed1()
    inspect_gap_behavior(seed=1)
    inspect_gap_behavior(seed=301)
