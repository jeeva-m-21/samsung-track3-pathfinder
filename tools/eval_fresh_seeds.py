#!/usr/bin/env python3
"""Evaluate best-011 on fresh untouched seeds {301..310} across all 5 competition sequences.
Runs in parallel with ProcessPoolExecutor and outputs JSON + Markdown scorecard.
"""

import sys
import os
import json
import time
import subprocess
import tempfile
import numpy as np
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from tools.eval_degraded_harness import REAL_SEQUENCES
from tools.degrade import write_degraded_dataset
from tools.metrics import compute_trajectory_metrics

CANDIDATE_BIN = REPO_ROOT / "main"
CONFIG_PATH = REPO_ROOT / "configs/degrade_default.json"
FRESH_SEEDS = list(range(301, 311))


def run_single_case(args):
    seq_cfg, seed, tmp_root_str = args
    tmp_root = Path(tmp_root_str)
    seq_name = seq_cfg["name"]
    case_dir = tmp_root / f"{seq_name}_s{seed}"
    case_dir.mkdir(parents=True, exist_ok=True)

    clean_imu = REPO_ROOT / seq_cfg["imu"]
    clean_gt = REPO_ROOT / seq_cfg["gt"]

    # 1. Degrade dataset
    deg_dir = case_dir / "deg"
    deg_imu = deg_dir / "imu0" / "data.csv"
    deg_gt = deg_dir / "groundtruth.txt"
    sidecar = deg_dir / "degrade_info.json"
    write_degraded_dataset(
        imu_in=clean_imu,
        gt_in=clean_gt,
        out_dir=deg_dir,
        config_path=CONFIG_PATH,
        seed=seed,
    )

    # 2. Run Candidate (best-011)
    cand_traj = case_dir / "cand_traj.txt"
    cmd_cand = [str(CANDIDATE_BIN), str(deg_imu), str(cand_traj), "--append-blind"]
    res_cand = subprocess.run(cmd_cand, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res_cand.returncode != 0:
        raise RuntimeError(f"Candidate failed on {seq_name} seed {seed}: {res_cand.stderr}")

    metrics_cand = compute_trajectory_metrics(
        ref_path=deg_gt,
        est_path=cand_traj,
        sidecar_path=sidecar,
    )

    return {
        "sequence": seq_name,
        "seed": seed,
        "metrics": metrics_cand,
    }


def main():
    print(f"Running Fresh Seeds {FRESH_SEEDS[0]}..{FRESH_SEEDS[-1]} (50 runs) on best-011...")
    start_time = time.time()

    with tempfile.TemporaryDirectory() as tmp_dir:
        tasks = []
        for seq_cfg in REAL_SEQUENCES:
            for seed in FRESH_SEEDS:
                tasks.append((seq_cfg, seed, tmp_dir))

        with ProcessPoolExecutor(max_workers=min(os.cpu_count() or 4, 16)) as executor:
            results = list(executor.map(run_single_case, tasks))

    elapsed = time.time() - start_time
    print(f"Completed 50 runs in {elapsed:.1f} s.")

    ate_list = [r["metrics"]["ate_rmse_aligned"] for r in results]
    blind_list = [r["metrics"]["blind5_rmse"] for r in results]
    gap_list = [r["metrics"]["gap_2s_rmse"] for r in results]
    att_list = [r["metrics"]["rot_err_deg"] for r in results]

    seq_names = [s["name"] for s in REAL_SEQUENCES]
    per_seq = {}
    for name in seq_names:
        seq_res = [r for r in results if r["sequence"] == name]
        per_seq[name] = {
            "ate_rmse_aligned": float(np.mean([r["metrics"]["ate_rmse_aligned"] for r in seq_res])),
            "blind5_rmse": float(np.mean([r["metrics"]["blind5_rmse"] for r in seq_res])),
            "gap_2s_rmse": float(np.mean([r["metrics"]["gap_2s_rmse"] for r in seq_res])),
            "rot_err_deg": float(np.mean([r["metrics"]["rot_err_deg"] for r in seq_res])),
        }

    summary = {
        "overall": {
            "ate_rmse_mean": float(np.mean(ate_list)),
            "ate_rmse_median": float(np.median(ate_list)),
            "ate_rmse_worst": float(np.max(ate_list)),
            "blind_5s_mean": float(np.mean(blind_list)),
            "blind_5s_median": float(np.median(blind_list)),
            "blind_5s_worst": float(np.max(blind_list)),
            "gap_2s_mean": float(np.mean(gap_list)),
            "gap_2s_median": float(np.median(gap_list)),
            "gap_2s_worst": float(np.max(gap_list)),
            "attitude_error_mean": float(np.mean(att_list)),
        },
        "per_sequence": per_seq,
        "runs": results,
    }

    out_json = REPO_ROOT / "reports/fresh_seeds_301_310.json"
    with open(out_json, "w") as f:
        json.dump(summary, f, indent=2)

    print("\n" + "=" * 60)
    print("FRESH SEEDS {301..310} SCORECARD (best-011)")
    print("=" * 60)
    print(f"Blind 5s RMSE:       {summary['overall']['blind_5s_mean']:.4f} m (median: {summary['overall']['blind_5s_median']:.4f} m)")
    print(f"Waypoint ATE RMSE:  {summary['overall']['ate_rmse_mean']:.4f} m (median: {summary['overall']['ate_rmse_median']:.4f} m)")
    print(f"Dropout Gap 2s RMSE: {summary['overall']['gap_2s_mean']:.4f} m (median: {summary['overall']['gap_2s_median']:.4f} m)")
    print(f"Attitude Error:      {summary['overall']['attitude_error_mean']:.2f} deg")
    print("=" * 60)
    print("\nPer-Sequence Breakdown:")
    for name, s in per_seq.items():
        print(f"  {name:<15}: ATE={s['ate_rmse_aligned']:.3f}m, Blind={s['blind5_rmse']:.3f}m, Gap={s['gap_2s_rmse']:.3f}m")
    print("=" * 60)


if __name__ == "__main__":
    main()
