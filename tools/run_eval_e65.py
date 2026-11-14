#!/usr/bin/env python3
"""Evaluation and ablation runner for EXP-E64: Regime-Blended Curvature Extrapolation.

Runs candidate vs baseline across dev seeds 1..10 on the 5 dev sequences.
Computes paired bootstrap confidence intervals (10,000 resamples),
per-sequence breakdown, format validation, and resource budgets.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from tools.eval_degraded_harness import DegradedHarness, REAL_SEQUENCES
from tools.metrics import compute_trajectory_metrics
from tools.validate_trajectory import validate_trajectory


def bootstrap_ci(diffs: np.ndarray, num_samples: int = 10000, alpha: float = 0.05) -> Tuple[float, float]:
    """Compute 95% bootstrap confidence interval of the mean difference."""
    n = len(diffs)
    if n == 0:
        return 0.0, 0.0
    indices = np.random.randint(0, n, size=(num_samples, n))
    means = np.mean(diffs[indices], axis=1)
    lower = float(np.percentile(means, 100.0 * (alpha / 2.0)))
    upper = float(np.percentile(means, 100.0 * (1.0 - alpha / 2.0)))
    return lower, upper


def measure_budget(binary_path: Path, test_imu: Path, out_traj: Path) -> Tuple[float, float]:
    """Measure peak RSS (MB) and compute latency (ms/s IMU) using taskset -c 0 /usr/bin/time -v."""
    cmd = [
        "taskset", "-c", "0",
        "/usr/bin/time", "-v",
        str(binary_path),
        str(test_imu),
        str(out_traj),
        "--append-blind"
    ]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    peak_rss_kb = 0
    elapsed_sec = 0.0
    for line in proc.stderr.splitlines():
        if "Maximum resident set size" in line:
            parts = line.split(":")
            if len(parts) >= 2:
                peak_rss_kb = int(parts[1].strip())
        elif "User time (seconds)" in line:
            parts = line.split(":")
            if len(parts) >= 2:
                elapsed_sec += float(parts[1].strip())
        elif "System time (seconds)" in line:
            parts = line.split(":")
            if len(parts) >= 2:
                elapsed_sec += float(parts[1].strip())

    # Count IMU rows to get IMU duration
    imu_lines = 0
    with open(test_imu, "r") as f:
        for _ in f:
            imu_lines += 1
    imu_duration_s = max(1.0, (imu_lines - 1) * 0.005) # approx 200 Hz
    ms_per_s = (elapsed_sec * 1000.0) / imu_duration_s
    rss_mb = peak_rss_kb / 1024.0
    return rss_mb, ms_per_s


def run_evaluation(cand_bin: Path, base_bin: Path, seeds: List[int], label: str) -> Dict[str, Any]:
    np.random.seed(42)
    harness_cand = DegradedHarness(binary_path=str(cand_bin), verbose=False)
    harness_base = DegradedHarness(binary_path=str(base_bin), verbose=False)

    print(f"\n{'='*70}")
    print(f" EVALUATION: {label}")
    print(f" Candidate: {cand_bin.name}")
    print(f" Baseline:  {base_bin.name}")
    print(f" Seeds:     {seeds} ({len(seeds)} seeds x {len(REAL_SEQUENCES)} sequences = {len(seeds)*len(REAL_SEQUENCES)} runs)")
    print(f"{'='*70}")

    records = []
    total = len(REAL_SEQUENCES) * len(seeds)
    idx = 0

    tmp_base = REPO_ROOT / ".tmp_eval"
    tmp_base.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="e64_eval_", dir=tmp_base) as tmpdir:
        work_path = Path(tmpdir)
        for seq in REAL_SEQUENCES:
            seq_name = seq["name"]
            for seed in seeds:
                idx += 1
                case_id = f"{seq_name}_s{seed}"
                case_dir = work_path / case_id
                case_dir.mkdir(parents=True, exist_ok=True)

                deg_imu, deg_gt, sidecar = harness_cand.prepare_degraded_dataset(
                    seq, seed, case_dir / "deg"
                )

                # Run candidate
                cand_traj = case_dir / "cand_traj.txt"
                t0 = time.perf_counter()
                harness_cand.run_estimator(deg_imu, cand_traj)
                cand_time = time.perf_counter() - t0
                cand_m = compute_trajectory_metrics(deg_gt, cand_traj, sidecar_path=sidecar)

                # Validate trajectory format on candidate
                is_valid, val_errs = validate_trajectory(deg_imu, cand_traj, append_blind=True)
                if not is_valid:
                    print(f"[ERROR] Format validation failed for {case_id}: {val_errs}")

                # Run baseline
                base_traj = case_dir / "base_traj.txt"
                t0 = time.perf_counter()
                harness_base.run_estimator(deg_imu, base_traj)
                base_time = time.perf_counter() - t0
                base_m = compute_trajectory_metrics(deg_gt, base_traj, sidecar_path=sidecar)

                rec = {
                    "case_id": case_id,
                    "sequence": seq_name,
                    "seed": seed,
                    "cand_ate": float(cand_m["ate_rmse_aligned"]),
                    "base_ate": float(base_m["ate_rmse_aligned"]),
                    "cand_blind5": float(cand_m["blind5_rmse"]),
                    "base_blind5": float(base_m["blind5_rmse"]),
                    "cand_gap2": float(cand_m["gap_2s_rmse"]),
                    "base_gap2": float(base_m["gap_2s_rmse"]),
                    "cand_rot": float(cand_m["rot_err_deg"]),
                    "base_rot": float(base_m["rot_err_deg"]),
                    "cand_time": cand_time,
                    "base_time": base_time,
                }
                records.append(rec)
                delta_b5 = rec["cand_blind5"] - rec["base_blind5"]
                rel_b5 = (delta_b5 / rec["base_blind5"]) * 100.0 if rec["base_blind5"] > 0 else 0.0
                print(
                    f"[{idx:2d}/{total:2d}] {case_id:<18} | "
                    f"Blind5: {rec['base_blind5']:.4f} -> {rec['cand_blind5']:.4f} ({delta_b5:+.4f} m, {rel_b5:+.2f}%) | "
                    f"ATE: {rec['base_ate']:.4f} -> {rec['cand_ate']:.4f}"
                )
                shutil.rmtree(case_dir, ignore_errors=True)

        # Measure resource budgets on MH_01_easy
        test_case_dir = work_path / "budget_test"
        test_case_dir.mkdir(parents=True, exist_ok=True)
        test_imu, _, _ = harness_cand.prepare_degraded_dataset(REAL_SEQUENCES[0], 1, test_case_dir / "deg")
        rss_mb, ms_per_s = measure_budget(cand_bin, test_imu, test_case_dir / "traj.txt")

    # Overall arrays
    base_blind = np.array([r["base_blind5"] for r in records])
    cand_blind = np.array([r["cand_blind5"] for r in records])
    diff_blind = cand_blind - base_blind
    ci_blind = bootstrap_ci(diff_blind)

    base_ate = np.array([r["base_ate"] for r in records])
    cand_ate = np.array([r["cand_ate"] for r in records])
    diff_ate = cand_ate - base_ate
    ci_ate = bootstrap_ci(diff_ate)

    base_gap = np.array([r["base_gap2"] for r in records])
    cand_gap = np.array([r["cand_gap2"] for r in records])
    diff_gap = cand_gap - base_gap
    ci_gap = bootstrap_ci(diff_gap)

    base_rot = np.array([r["base_rot"] for r in records])
    cand_rot = np.array([r["cand_rot"] for r in records])
    diff_rot = cand_rot - base_rot
    ci_rot = bootstrap_ci(diff_rot)

    print(f"\n{'-'*70}")
    print(f" SUMMARY SCORECARD: {label}")
    print(f"{'-'*70}")
    def print_stat(name, base, cand, diff, ci, unit="m"):
        b_m, c_m, d_m = np.mean(base), np.mean(cand), np.mean(diff)
        rel = (d_m / b_m) * 100.0 if b_m > 0 else 0.0
        print(f"  {name:<18}: Base = {b_m:7.4f} {unit} | Cand = {c_m:7.4f} {unit} | Delta = {d_m:+7.4f} {unit} ({rel:+.2f}%) | 95% CI: [{ci[0]:+7.4f}, {ci[1]:+7.4f}]")

    print_stat("Blind 5s RMSE", base_blind, cand_blind, diff_blind, ci_blind)
    print_stat("Waypoint ATE", base_ate, cand_ate, diff_ate, ci_ate)
    print_stat("Gap 2s RMSE", base_gap, cand_gap, diff_gap, ci_gap)
    print_stat("Attitude Error", base_rot, cand_rot, diff_rot, ci_rot, unit="deg")
    print(f"  Peak RSS          : {rss_mb:.2f} MB (Target <= 4.0 MB)")
    print(f"  Compute Latency   : {ms_per_s:.2f} ms/s (Target <= 5.0 ms/s)")

    # Per sequence
    print(f"\nPER-SEQUENCE BREAKDOWN:")
    print(f"{'Sequence':<15} | {'Base Blind':<10} | {'Cand Blind':<10} | {'Blind Rel %':<11} | {'Base ATE':<10} | {'Cand ATE':<10} | {'ATE Rel %':<10}")
    print(f"{'-'*85}")
    seq_summary = {}
    for seq in REAL_SEQUENCES:
        sname = seq["name"]
        s_recs = [r for r in records if r["sequence"] == sname]
        sb = float(np.mean([r["base_blind5"] for r in s_recs]))
        sc = float(np.mean([r["cand_blind5"] for r in s_recs]))
        sd = sc - sb
        s_rel_b = (sd / sb) * 100.0 if sb > 0 else 0.0

        sa_b = float(np.mean([r["base_ate"] for r in s_recs]))
        sa_c = float(np.mean([r["cand_ate"] for r in s_recs]))
        sa_d = sa_c - sa_b
        sa_rel = (sa_d / sa_b) * 100.0 if sa_b > 0 else 0.0

        print(f"{sname:<15} | {sb:10.4f} | {sc:10.4f} | {s_rel_b:+10.2f}% | {sa_b:10.4f} | {sa_c:10.4f} | {sa_rel:+9.2f}%")
        seq_summary[sname] = {
            "base_blind5": sb,
            "cand_blind5": sc,
            "diff_blind5": sd,
            "rel_blind5_pct": s_rel_b,
            "base_ate": sa_b,
            "cand_ate": sa_c,
            "diff_ate": sa_d,
            "rel_ate_pct": sa_rel,
        }

    summary = {
        "label": label,
        "candidate": str(cand_bin),
        "baseline": str(base_bin),
        "overall": {
            "blind5": {
                "base_mean": float(np.mean(base_blind)),
                "cand_mean": float(np.mean(cand_blind)),
                "diff_mean": float(np.mean(diff_blind)),
                "rel_pct": float((np.mean(diff_blind) / np.mean(base_blind)) * 100.0),
                "ci_95": [float(ci_blind[0]), float(ci_blind[1])],
            },
            "ate": {
                "base_mean": float(np.mean(base_ate)),
                "cand_mean": float(np.mean(cand_ate)),
                "diff_mean": float(np.mean(diff_ate)),
                "rel_pct": float((np.mean(diff_ate) / np.mean(base_ate)) * 100.0),
                "ci_95": [float(ci_ate[0]), float(ci_ate[1])],
            },
            "gap2": {
                "base_mean": float(np.mean(base_gap)),
                "cand_mean": float(np.mean(cand_gap)),
                "diff_mean": float(np.mean(diff_gap)),
                "rel_pct": float((np.mean(diff_gap) / np.mean(base_gap)) * 100.0),
                "ci_95": [float(ci_gap[0]), float(ci_gap[1])],
            },
            "rot": {
                "base_mean": float(np.mean(base_rot)),
                "cand_mean": float(np.mean(cand_rot)),
                "diff_mean": float(np.mean(diff_rot)),
                "rel_pct": float((np.mean(diff_rot) / np.mean(base_rot)) * 100.0),
                "ci_95": [float(ci_rot[0]), float(ci_rot[1])],
            },
            "budget": {
                "rss_mb": rss_mb,
                "ms_per_s": ms_per_s,
            }
        },
        "per_sequence": seq_summary,
        "records": records,
    }
    return summary


def main():
    parser = argparse.ArgumentParser(description="EXP-E65 Evaluation")
    parser.add_argument("--candidate", type=Path, default=REPO_ROOT / "bin" / "main_combo_e65")
    parser.add_argument("--baseline", type=Path, default=REPO_ROOT / "bin" / "main_baseline")
    parser.add_argument("--seeds", type=str, default="1,2,3,4,5,6,7,8,9,10")
    parser.add_argument("--label", type=str, default="Combo E63 Adaptive Q + E64 Regime Curvature")
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "reports" / "runs" / "E65" / "results.json")
    args = parser.parse_args()

    seeds = [int(s.strip()) for s in args.seeds.split(",") if s.strip()]
    summary = run_evaluation(args.candidate, args.baseline, seeds, args.label)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\nSaved results to {args.out}")


if __name__ == "__main__":
    main()
