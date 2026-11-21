#!/usr/bin/env python3
"""Independent Reviewer evaluation script for EXP-E66b.
Evaluates baseline vs candidate on dev seeds (1..10) and confirmation seeds (201..210)
using parallel workers and outputs a JSON report.
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

BASELINE_BIN = REPO_ROOT / "main"
CANDIDATE_BIN = Path("/home/jeeva/projects/samsung_hackathon_wt/E66/main")
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
    print("Starting Independent Reviewer Evaluation for EXP-E66b...")
    print(f"Baseline binary: {BASELINE_BIN}")
    print(f"Candidate binary: {CANDIDATE_BIN}")

    tmp_dir = tempfile.mkdtemp(prefix="reviewer_e66b_")
    try:
        # Dev set evaluation
        print("\n=== Running Dev Set (Seeds 1..10, 50 paired runs) ===")
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

        # Confirmation set evaluation
        print("\n=== Running Confirmation Set (Seeds 201..210, 50 paired runs) ===")
        conf_args = [
            (seq_cfg, seed, tmp_dir)
            for seq_cfg in REAL_SEQUENCES
            for seed in CONF_SEEDS
        ]
        t0 = time.time()
        with ProcessPoolExecutor(max_workers=8) as pool:
            conf_records = list(pool.map(run_single_case, conf_args))
        print(f"Confirmation set completed in {time.time() - t0:.1f}s")
        conf_summary, conf_seq_summary = summarize_results(conf_records)

        out_data = {
            "dev": {
                "summary": dev_summary,
                "seq_summary": dev_seq_summary,
            },
            "confirmation": {
                "summary": conf_summary,
                "seq_summary": conf_seq_summary,
            },
        }

        out_file = REPO_ROOT / "reports/reviewer_e66b_verification.json"
        with open(out_file, "w") as f:
            json.dump(out_data, f, indent=2)
        print(f"\nSaved reviewer verification results to {out_file}")

        # Print formatted scorecard
        print("\n" + "="*80)
        print("DEV SET SCORECARD (Seeds 1..10)")
        print("="*80)
        for m, v in dev_summary.items():
            print(f"{m:<18}: base={v['base']:7.4f}, cand={v['cand']:7.4f}, diff={v['diff']:+7.4f} ({v['rel']:+6.2f}%), 95% CI=[{v['ci'][0]:+7.4f}, {v['ci'][1]:+7.4f}]")

        print("\n" + "-"*80)
        print("DEV PER-SEQUENCE BREAKDOWN (Blind 5s RMSE)")
        print("-"*80)
        for sname, sdata in dev_seq_summary.items():
            b = sdata["blind5_rmse"]
            print(f"{sname:<15}: base={b['base']:7.4f}, cand={b['cand']:7.4f}, diff={b['diff']:+7.4f} ({b['rel']:+6.2f}%)")

        print("\n" + "="*80)
        print("CONFIRMATION SET SCORECARD (Seeds 201..210)")
        print("="*80)
        for m, v in conf_summary.items():
            print(f"{m:<18}: base={v['base']:7.4f}, cand={v['cand']:7.4f}, diff={v['diff']:+7.4f} ({v['rel']:+6.2f}%), 95% CI=[{v['ci'][0]:+7.4f}, {v['ci'][1]:+7.4f}]")

        print("\n" + "-"*80)
        print("CONFIRMATION PER-SEQUENCE BREAKDOWN (Blind 5s RMSE)")
        print("-"*80)
        for sname, sdata in conf_seq_summary.items():
            b = sdata["blind5_rmse"]
            print(f"{sname:<15}: base={b['base']:7.4f}, cand={b['cand']:7.4f}, diff={b['diff']:+7.4f} ({b['rel']:+6.2f}%)")

    finally:
        import shutil
        shutil.rmtree(tmp_dir, ignore_errors=True)


if __name__ == "__main__":
    main()
