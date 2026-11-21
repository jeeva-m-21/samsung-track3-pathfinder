#!/usr/bin/env python3
"""Worker evaluation script for EXP-E74 (True SE_2(3) Right-Invariant InEKF).
Evaluates parent tag baseline (/tmp/main_best011) vs candidate (./main)
across Dev seeds (1..10) and Confirmation seeds (201..210) in parallel.
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

BASELINE_BIN = Path("/tmp/main_best011")
CANDIDATE_BIN = REPO_ROOT / "main"
CONFIG_PATH = REPO_ROOT / "configs/degrade_default.json"

DEV_SEEDS = list(range(1, 11))
CONF_SEEDS = list(range(201, 211))


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

    # 2. Run Baseline
    base_traj = case_dir / "base_traj.txt"
    cmd_base = [str(BASELINE_BIN), str(deg_imu), str(base_traj), "--append-blind"]
    res_base = subprocess.run(cmd_base, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res_base.returncode != 0:
        raise RuntimeError(f"Baseline failed on {seq_name} seed {seed}: {res_base.stderr}")
    base_metrics = compute_trajectory_metrics(deg_gt, base_traj, sidecar_path=sidecar)

    # 3. Run Candidate
    cand_traj = case_dir / "cand_traj.txt"
    cmd_cand = [str(CANDIDATE_BIN), str(deg_imu), str(cand_traj), "--append-blind"]
    res_cand = subprocess.run(cmd_cand, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res_cand.returncode != 0:
        raise RuntimeError(f"Candidate failed on {seq_name} seed {seed}: {res_cand.stderr}")
    cand_metrics = compute_trajectory_metrics(deg_gt, cand_traj, sidecar_path=sidecar)

    # Clean up trajectories and degraded data to save disk space
    try:
        base_traj.unlink(missing_ok=True)
        cand_traj.unlink(missing_ok=True)
        import shutil
        shutil.rmtree(deg_dir, ignore_errors=True)
    except Exception:
        pass

    return {
        "sequence": seq_name,
        "seed": seed,
        "base": base_metrics,
        "cand": cand_metrics,
    }


def bootstrap_ci(deltas, n_boot=10000, alpha=0.05, seed=42):
    rng = np.random.RandomState(seed)
    n = len(deltas)
    boot_means = np.mean(rng.choice(deltas, size=(n_boot, n), replace=True), axis=1)
    low = np.percentile(boot_means, 100 * (alpha / 2))
    high = np.percentile(boot_means, 100 * (1 - alpha / 2))
    return float(low), float(high)


def summarize_results(records):
    metrics = ["blind5_rmse", "ate_rmse_aligned", "gap_2s_rmse", "rot_err_deg"]
    summary = {}
    for m in metrics:
        b_vals = np.array([r["base"][m] for r in records])
        c_vals = np.array([r["cand"][m] for r in records])
        diffs = c_vals - b_vals
        mean_b = float(np.mean(b_vals))
        mean_c = float(np.mean(c_vals))
        delta = float(np.mean(diffs))
        rel_pct = (delta / mean_b * 100.0) if mean_b > 1e-9 else 0.0
        ci_low, ci_high = bootstrap_ci(diffs)
        summary[m] = {
            "mean_base": mean_b,
            "mean_cand": mean_c,
            "delta": delta,
            "rel_pct": rel_pct,
            "ci_95": [ci_low, ci_high],
        }

    # Per-sequence breakdown for blind5 and ate
    seq_breakdown = {}
    seq_names = sorted(list(set(r["sequence"] for r in records)))
    for seq in seq_names:
        seq_records = [r for r in records if r["sequence"] == seq]
        seq_breakdown[seq] = {}
        for m in metrics:
            b_vals = np.array([r["base"][m] for r in seq_records])
            c_vals = np.array([r["cand"][m] for r in seq_records])
            diffs = c_vals - b_vals
            mean_b = float(np.mean(b_vals))
            mean_c = float(np.mean(c_vals))
            delta = float(np.mean(diffs))
            rel_pct = (delta / mean_b * 100.0) if mean_b > 1e-9 else 0.0
            seq_breakdown[seq][m] = {
                "base": mean_b,
                "cand": mean_c,
                "delta": delta,
                "rel_pct": rel_pct,
            }

    return {"suite": summary, "per_sequence": seq_breakdown}


def main():
    print(f"Starting Worker Evaluation for EXP-E74...")
    print(f"Baseline:  {BASELINE_BIN}")
    print(f"Candidate: {CANDIDATE_BIN}")

    t_start = time.time()
    tmp_root = tempfile.mkdtemp(prefix="eval_e74_")

    try:
        # 1. Dev seeds (1..10)
        dev_args = [(seq, seed, tmp_root) for seq in REAL_SEQUENCES for seed in DEV_SEEDS]
        print(f"\nRunning Dev Set (50 cases across seeds 1..10)...")
        with ProcessPoolExecutor(max_workers=8) as pool:
            dev_records = list(pool.map(run_single_case, dev_args))
        dev_summary = summarize_results(dev_records)

        # 2. Confirmation seeds (201..210)
        conf_args = [(seq, seed, tmp_root) for seq in REAL_SEQUENCES for seed in CONF_SEEDS]
        print(f"\nRunning Confirmation Set (50 cases across seeds 201..210)...")
        with ProcessPoolExecutor(max_workers=8) as pool:
            conf_records = list(pool.map(run_single_case, conf_args))
        conf_summary = summarize_results(conf_records)

        output_data = {
            "dev": dev_summary,
            "confirmation": conf_summary,
            "dev_records": dev_records,
            "conf_records": conf_records,
        }

        out_path = REPO_ROOT / "reports" / "eval_worker_e74.json"
        with open(out_path, "w") as f:
            json.dump(output_data, f, indent=2)
        print(f"\nResults successfully written to {out_path}")

        # Print Scorecard
        print("\n" + "=" * 80)
        print("SCORECARD: Baseline best-011 vs Candidate E74 (Dev Seeds 1..10)")
        print("=" * 80)
        s_dev = dev_summary["suite"]
        for m, label in [("blind5_rmse", "Blind 5s RMSE"),
                         ("ate_rmse_aligned", "Waypoint ATE RMSE"),
                         ("gap_2s_rmse", "Dropout Gap 2s RMSE"),
                         ("rot_err_deg", "Attitude Error")]:
            stat = s_dev[m]
            print(f"{label:22s}: Base={stat['mean_base']:.4f} -> Cand={stat['mean_cand']:.4f} "
                  f"| Delta={stat['delta']:+.4f} ({stat['rel_pct']:+.2f}%) "
                  f"| 95% CI=[{stat['ci_95'][0]:+.4f}, {stat['ci_95'][1]:+.4f}]")

        print("\nPer-Sequence Breakdown (Dev Set):")
        for seq, m_data in dev_summary["per_sequence"].items():
            b5 = m_data["blind5_rmse"]
            ate = m_data["ate_rmse_aligned"]
            gap = m_data["gap_2s_rmse"]
            print(f"  {seq:15s} | Blind5: {b5['base']:.3f}->{b5['cand']:.3f}m ({b5['rel_pct']:+6.2f}%) "
                  f"| ATE: {ate['base']:.3f}->{ate['cand']:.3f}m ({ate['rel_pct']:+6.2f}%) "
                  f"| Gap: {gap['base']:.3f}->{gap['cand']:.3f}m ({gap['rel_pct']:+6.2f}%)")

        print("\n" + "=" * 80)
        print("CONFIRMATION SET: Seeds 201..210 (50 Runs)")
        print("=" * 80)
        s_conf = conf_summary["suite"]
        for m, label in [("blind5_rmse", "Blind 5s RMSE"),
                         ("ate_rmse_aligned", "Waypoint ATE RMSE"),
                         ("gap_2s_rmse", "Dropout Gap 2s RMSE"),
                         ("rot_err_deg", "Attitude Error")]:
            stat = s_conf[m]
            print(f"{label:22s}: Base={stat['mean_base']:.4f} -> Cand={stat['mean_cand']:.4f} "
                  f"| Delta={stat['delta']:+.4f} ({stat['rel_pct']:+.2f}%) "
                  f"| 95% CI=[{stat['ci_95'][0]:+.4f}, {stat['ci_95'][1]:+.4f}]")

        print("\nPer-Sequence Breakdown (Confirmation Set):")
        for seq, m_data in conf_summary["per_sequence"].items():
            b5 = m_data["blind5_rmse"]
            ate = m_data["ate_rmse_aligned"]
            gap = m_data["gap_2s_rmse"]
            print(f"  {seq:15s} | Blind5: {b5['base']:.3f}->{b5['cand']:.3f}m ({b5['rel_pct']:+6.2f}%) "
                  f"| ATE: {ate['base']:.3f}->{ate['cand']:.3f}m ({ate['rel_pct']:+6.2f}%) "
                  f"| Gap: {gap['base']:.3f}->{gap['cand']:.3f}m ({gap['rel_pct']:+6.2f}%)")

        print(f"\nEvaluation completed in {time.time() - t_start:.1f}s.")

    finally:
        import shutil
        shutil.rmtree(tmp_root, ignore_errors=True)


if __name__ == "__main__":
    main()
