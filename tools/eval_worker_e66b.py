#!/usr/bin/env python3
"""Worker evaluation script for EXP-E67 (Isolated gap_omega_decay only).
Evaluates baseline (/tmp/main_best008) vs candidate (./main) on dev seeds (1..10)
using parallel workers and outputs a summary.
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

BASELINE_BIN = Path("/tmp/main_best008")
CANDIDATE_BIN = REPO_ROOT / "main"
CONFIG_PATH = REPO_ROOT / "configs/degrade_default.json"

DEV_SEEDS = list(range(1, 11))


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
        b_mean = float(np.mean(b_vals))
        c_mean = float(np.mean(c_vals))
        d_mean = float(np.mean(diffs))
        rel_pct = float(d_mean / b_mean * 100.0) if abs(b_mean) > 1e-6 else 0.0
        ci_low, ci_high = bootstrap_ci(diffs)
        summary[m] = {
            "base": b_mean,
            "cand": c_mean,
            "diff": d_mean,
            "rel": rel_pct,
            "ci": [ci_low, ci_high],
            "base_std": float(np.std(b_vals)),
            "cand_std": float(np.std(c_vals)),
        }

    seq_summary = {}
    for seq_cfg in REAL_SEQUENCES:
        sname = seq_cfg["name"]
        seq_recs = [r for r in records if r["sequence"] == sname]
        seq_res = {}
        for m in metrics:
            b_vals = np.array([r["base"][m] for r in seq_recs])
            c_vals = np.array([r["cand"][m] for r in seq_recs])
            diffs = c_vals - b_vals
            b_mean = float(np.mean(b_vals))
            c_mean = float(np.mean(c_vals))
            d_mean = float(np.mean(diffs))
            rel_pct = float(d_mean / b_mean * 100.0) if abs(b_mean) > 1e-6 else 0.0
            seq_res[m] = {
                "base": b_mean,
                "cand": c_mean,
                "diff": d_mean,
                "rel": rel_pct,
            }
        seq_summary[sname] = seq_res

    return summary, seq_summary


def main():
    print("Starting Worker Dev Evaluation for EXP-E67 (Isolated gap_omega_decay only)...")
    print(f"Baseline binary: {BASELINE_BIN}")
    print(f"Candidate binary: {CANDIDATE_BIN}")

    tmp_dir = tempfile.mkdtemp(prefix="worker_e67_iso_")
    try:
        dev_args = [
            (seq_cfg, seed, tmp_dir)
            for seq_cfg in REAL_SEQUENCES
            for seed in DEV_SEEDS
        ]
        t0 = time.time()
        with ProcessPoolExecutor(max_workers=8) as pool:
            dev_records = list(pool.map(run_single_case, dev_args))
        print(f"Dev set completed in {time.time() - t0:.1f}s")
        dev_summary, dev_seq_summary = summarize_results(dev_records)

        print("\n" + "="*80)
        print("DEV SUMMARY (SEEDS 1..10):")
        print("="*80)
        for m, v in dev_summary.items():
            print(f"{m:20s}: Base={v['base']:.4f}  Cand={v['cand']:.4f}  Diff={v['diff']:+.4f} ({v['rel']:+.2f}%)  95% CI=[{v['ci'][0]:+.4f}, {v['ci'][1]:+.4f}]")

        print("\nPER-SEQUENCE BREAKDOWN (DEV):")
        print(f"{'Sequence':15s} | {'Blind5 Base':11s} -> {'Cand':7s} ({'Rel%':7s}) | {'Gap Base':9s} -> {'Cand':7s} ({'Rel%':7s}) | {'ATE Base':9s} -> {'Cand':7s} ({'Rel%':7s})")
        print("-" * 90)
        for sname, sres in dev_seq_summary.items():
            b_bl = sres["blind5_rmse"]
            b_gap = sres["gap_2s_rmse"]
            b_ate = sres["ate_rmse_aligned"]
            print(f"{sname:15s} | {b_bl['base']:6.4f} -> {b_bl['cand']:6.4f} ({b_bl['rel']:+6.2f}%) | {b_gap['base']:6.4f} -> {b_gap['cand']:6.4f} ({b_gap['rel']:+6.2f}%) | {b_ate['base']:6.4f} -> {b_ate['cand']:6.4f} ({b_ate['rel']:+6.2f}%)")

        out_path = REPO_ROOT / "reports/runs/E67/results_isolated.json"
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w") as f:
            json.dump({"summary": dev_summary, "sequences": dev_seq_summary}, f, indent=2)
        print(f"\nSaved results to {out_path}")

    finally:
        import shutil
        shutil.rmtree(tmp_dir, ignore_errors=True)


if __name__ == "__main__":
    main()
