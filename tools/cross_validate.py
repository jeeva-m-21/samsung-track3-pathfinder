#!/usr/bin/env python3
"""
cross_validate.py - Rigorous cross-validation harness with randomized degradations
for Samsung Track 3 IMU-only dead reckoning.

Evaluates candidate binaries across nominal and randomized degraded conditions:
- Presets: clean, mild, medium, severe
- Multiple pseudo-random seeds per condition
- Computes per-trial ATE RMSE (SE(3) aligned) and RPE 1s
- Reports Mean, Median, 95th percentile (P95), and Worst-Case metrics
- Supports feature ablation comparisons via CLI flags
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Tuple
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from tools.metrics import compute_trajectory_metrics

DATASET_CONFIGS = [
    {
        "name": "MH_01_easy",
        "imu": "data/euroc/MH_01_easy/mav0/imu0/data.csv",
        "gt": "data/euroc/gt/MH_01_easy.txt",
        "type": "drone"
    },
    {
        "name": "V1_01_easy",
        "imu": "data/euroc/V1_01_easy/mav0/imu0/data.csv",
        "gt": "data/euroc/gt/V1_01_easy.txt",
        "type": "drone"
    },
    {
        "name": "V2_02_medium",
        "imu": "data/euroc/V2_02_medium/mav0/imu0/data.csv",
        "gt": "data/euroc/gt/V2_02_medium.txt",
        "type": "drone"
    },
    {
        "name": "circle",
        "imu": "data/synth/circle.csv",
        "gt": "data/synth/circle_gt.txt",
        "type": "synthetic"
    },
    {
        "name": "figure_eight",
        "imu": "data/synth/figure_eight.csv",
        "gt": "data/synth/figure_eight_gt.txt",
        "type": "synthetic"
    }
]

def generate_degraded_imu(clean_imu: Path, clean_gt: Path, preset: str, seed: int, out_dir: Path) -> Tuple[Path, Path]:
    """Generates degraded IMU dataset and adjusted ground truth."""
    if preset == "clean":
        return clean_imu, clean_gt

    out_dir.mkdir(parents=True, exist_ok=True)
    cmd_deg = [
        sys.executable,
        str(REPO_ROOT / "tools" / "degrade_hard.py"),
        "degrade",
        "--input", str(clean_imu),
        "--out", str(out_dir),
        "--preset", preset,
        "--seed", str(seed)
    ]
    subprocess.run(cmd_deg, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    manifest = out_dir / "degradation_manifest.json"
    adj_gt = out_dir / "gt_adj.txt"
    cmd_adj = [
        sys.executable,
        str(REPO_ROOT / "tools" / "degrade_hard.py"),
        "adjust-gt",
        "--gt", str(clean_gt),
        "--manifest", str(manifest),
        "--out", str(adj_gt)
    ]
    subprocess.run(cmd_adj, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    deg_imu = out_dir / "imu0" / "data.csv"
    return deg_imu, adj_gt

def run_evaluation(binary: Path, extra_args: List[str], presets: List[str], seeds: List[int]) -> Dict:
    """Runs cross-validation across all datasets, presets, and seeds."""
    all_results = []
    per_seq_results = {cfg["name"]: [] for cfg in DATASET_CONFIGS}

    with tempfile.TemporaryDirectory(prefix="cv_eval_") as tmpdir:
        tmp_path = Path(tmpdir)

        total_runs = len(DATASET_CONFIGS) * (1 if "clean" in presets else 0) + \
                     len(DATASET_CONFIGS) * len([p for p in presets if p != "clean"]) * len(seeds)
        print(f"Starting Cross-Validation: {len(DATASET_CONFIGS)} sequences, {len(presets)} presets, {len(seeds)} seeds (Total runs: {total_runs})")

        run_idx = 0
        for cfg in DATASET_CONFIGS:
            name = cfg["name"]
            clean_imu = REPO_ROOT / cfg["imu"]
            clean_gt = REPO_ROOT / cfg["gt"]

            for preset in presets:
                preset_seeds = [0] if preset == "clean" else seeds
                for seed in preset_seeds:
                    run_idx += 1
                    case_id = f"{name}_{preset}_s{seed}"
                    case_dir = tmp_path / case_id

                    imu_path, gt_path = generate_degraded_imu(clean_imu, clean_gt, preset, seed, case_dir)
                    out_traj = case_dir / "est_traj.txt"

                    if str(binary).endswith(".py"):
                        cmd = [sys.executable, str(binary), str(imu_path), str(out_traj)] + extra_args
                    else:
                        cmd = [str(binary), str(imu_path), str(out_traj)] + extra_args
                    res = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
                    if res.returncode != 0:
                        print(f"[{run_idx}/{total_runs}] ERROR on {case_id}: {res.stderr.strip()}")
                        continue

                    metrics = compute_trajectory_metrics(str(gt_path), str(out_traj))
                    ate = metrics["ate_rmse_aligned"]
                    rpe = metrics["rpe_trans_1s"]

                    record = {
                        "sequence": name,
                        "preset": preset,
                        "seed": seed,
                        "ate": ate,
                        "rpe": rpe
                    }
                    all_results.append(record)
                    per_seq_results[name].append(record)

    return {"all": all_results, "by_seq": per_seq_results}

def print_summary_table(eval_data: Dict, title: str = "Evaluation Summary"):
    """Prints a structured Markdown summary table with Mean, Median, P95, Worst."""
    print(f"\n### {title}")
    print("| Sequence / Group | Count | Mean ATE (m) | Median ATE (m) | P95 ATE (m) | Worst ATE (m) | Mean RPE 1s (m) |")
    print("|:---|:---:|:---:|:---:|:---:|:---:|:---:|")

    for seq_name, records in eval_data["by_seq"].items():
        if not records:
            continue
        ates = np.array([r["ate"] for r in records])
        rpes = np.array([r["rpe"] for r in records])
        mean_ate = np.mean(ates)
        med_ate = np.median(ates)
        p95_ate = np.percentile(ates, 95)
        worst_ate = np.max(ates)
        mean_rpe = np.mean(rpes)

        print(f"| **{seq_name}** | {len(records)} | {mean_ate:.4f} | {med_ate:.4f} | {p95_ate:.4f} | {worst_ate:.4f} | {mean_rpe:.4f} |")

    # Global summary
    all_records = eval_data["all"]
    if all_records:
        ates = np.array([r["ate"] for r in all_records])
        rpes = np.array([r["rpe"] for r in all_records])
        print("|" + "="*18 + "|:---:|:---:|:---:|:---:|:---:|:---:|")
        print(f"| **OVERALL SUITE** | {len(all_records)} | **{np.mean(ates):.4f}** | **{np.median(ates):.4f}** | **{np.percentile(ates, 95):.4f}** | **{np.max(ates):.4f}** | **{np.mean(rpes):.4f}** |")

def main():
    parser = argparse.ArgumentParser(description="Cross-validation with randomized degradations")
    parser.add_argument("--binary", default="main.py", help="Path to executable or main.py")
    parser.add_argument("--presets", default="clean,medium,severe", help="Comma-separated degradation presets")
    parser.add_argument("--seeds", default="42,101,777", help="Comma-separated random seeds")
    parser.add_argument("--ablation", default="eskf_full", help="Pipeline ablation mode")
    parser.add_argument("--output-json", help="Optional path to write detailed JSON results")
    args = parser.parse_args()

    binary_path = REPO_ROOT / args.binary
    if not binary_path.exists():
        subprocess.run(["make", "-C", str(REPO_ROOT / "src")], check=True)
        binary_path = REPO_ROOT / "main"

    presets = [p.strip() for p in args.presets.split(",") if p.strip()]
    seeds = [int(s.strip()) for s in args.seeds.split(",") if s.strip()]

    extra_args = ["--ablation", args.ablation]

    eval_data = run_evaluation(binary_path, extra_args, presets, seeds)
    print_summary_table(eval_data, title=f"Cross-Validation Summary (Ablation: {args.ablation})")

    if args.output_json:
        with open(args.output_json, "w") as f:
            json.dump(eval_data, f, indent=2)
        print(f"Detailed JSON written to {args.output_json}")

if __name__ == "__main__":
    main()
