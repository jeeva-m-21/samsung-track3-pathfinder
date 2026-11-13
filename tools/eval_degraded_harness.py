#!/usr/bin/env python3
"""eval_degraded_harness.py - Track 3 Degradation Evaluation Harness & LOOCV Benchmark.

Evaluates pure IMU dead-reckoning engines under controlled sensor degradation:
- Real sequences:
    EuRoC: MH_01_easy, V1_01_easy, V2_02_medium
    TUM-VI: room1, room2
- Degradation injector: tools.degrade (seeds: 42, 123, 456, 789, 999)
- Target competition metrics:
    1. Waypoint / Trajectory ATE error (SE(3) aligned ATE RMSE in meters)
    2. Final 5 seconds error (blind 5s cutoff RMSE in meters)
    3. 2-second gap blackout error (gap blackout RMSE in meters)
    Along with composite loss J normalized to naive baseline.
- Leave-One-Out Cross-Validation (LOOCV):
    5 folds holding out 1 sequence, evaluated on disjoint train/test seeds.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.degrade import write_degraded_dataset, load_degrade_config
from tools.metrics import compute_trajectory_metrics, compute_objective_J

# 5 Real Competition Sequences
REAL_SEQUENCES: List[Dict[str, str]] = [
    {
        "name": "MH_01_easy",
        "imu": "data/euroc/MH_01_easy/mav0/imu0/data.csv",
        "gt": "data/euroc/gt/MH_01_easy.txt",
        "type": "EuRoC",
    },
    {
        "name": "V1_01_easy",
        "imu": "data/euroc/V1_01_easy/mav0/imu0/data.csv",
        "gt": "data/euroc/gt/V1_01_easy.txt",
        "type": "EuRoC",
    },
    {
        "name": "V2_02_medium",
        "imu": "data/euroc/V2_02_medium/mav0/imu0/data.csv",
        "gt": "data/euroc/gt/V2_02_medium.txt",
        "type": "EuRoC",
    },
    {
        "name": "room1",
        "imu": "data/tumvi/room1/mav0/imu0/data.csv",
        "gt": "data/tumvi/gt_room1.txt",
        "type": "TUM-VI",
    },
    {
        "name": "room2",
        "imu": "data/tumvi/room2/mav0/imu0/data.csv",
        "gt": "data/tumvi/gt_room2.txt",
        "type": "TUM-VI",
    },
]

DEFAULT_SEEDS: List[int] = [42, 123, 456, 789, 999]
DEFAULT_TRAIN_SEEDS: List[int] = [42, 123, 456]
DEFAULT_TEST_SEEDS: List[int] = [789, 999]


def compute_3way_loss(
    m_cand: Dict[str, float],
    m_naive: Dict[str, float],
    w_ate: float = 0.50,
    w_blind: float = 0.33,
    w_gap: float = 0.17,
) -> float:
    """Compute normalized 3-metric composite loss across target competition metrics."""
    ate_ratio = m_cand.get("ate_rmse_aligned", 0.0) / max(m_naive.get("ate_rmse_aligned", 1.0), 1e-6)
    blind_ratio = m_cand.get("blind5_rmse", 0.0) / max(m_naive.get("blind5_rmse", 1.0), 1e-6)
    gap_ratio = m_cand.get("gap_2s_rmse", 0.0) / max(m_naive.get("gap_2s_rmse", 1.0), 1e-6)
    return float(w_ate * ate_ratio + w_blind * blind_ratio + w_gap * gap_ratio)


class DegradedHarness:
    """Evaluation harness managing degradation, execution, metric scoring, and LOOCV."""

    def __init__(
        self,
        binary_path: str = "./main",
        config_path: str = "configs/degrade_default.json",
        cache_dir: Optional[str] = None,
        append_blind: bool = True,
        extra_flags: Optional[List[str]] = None,
        verbose: bool = True,
    ):
        self.binary_path = Path(binary_path).resolve()
        self.config_path = Path(config_path).resolve()
        self.cache_dir = Path(cache_dir).resolve() if cache_dir else None
        self.append_blind = append_blind
        self.extra_flags = extra_flags or []
        self.verbose = verbose

        if not self.binary_path.exists():
            raise FileNotFoundError(f"Binary not found: {self.binary_path}")
        if not self.config_path.exists():
            raise FileNotFoundError(f"Config not found: {self.config_path}")

    def prepare_degraded_dataset(
        self,
        seq_cfg: Dict[str, str],
        seed: int,
        target_dir: Path,
    ) -> Tuple[Path, Path, Path]:
        """Generate degraded dataset using tools.degrade deterministically."""
        clean_imu = REPO_ROOT / seq_cfg["imu"]
        clean_gt = REPO_ROOT / seq_cfg["gt"]

        if not clean_imu.exists():
            raise FileNotFoundError(f"Clean IMU missing: {clean_imu}")
        if not clean_gt.exists():
            raise FileNotFoundError(f"Clean GT missing: {clean_gt}")

        target_dir.mkdir(parents=True, exist_ok=True)
        deg_imu = target_dir / "imu0" / "data.csv"
        deg_gt = target_dir / "groundtruth.txt"
        sidecar = target_dir / "degrade_info.json"

        if deg_imu.exists() and deg_gt.exists() and sidecar.exists():
            return deg_imu, deg_gt, sidecar

        write_degraded_dataset(
            imu_in=clean_imu,
            gt_in=clean_gt,
            out_dir=target_dir,
            config_path=self.config_path,
            seed=seed,
        )
        return deg_imu, deg_gt, sidecar

    def run_estimator(
        self,
        imu_csv: Path,
        out_traj: Path,
        ablation_mode: Optional[str] = None,
    ) -> float:
        """Run the binary or ablation mode and return runtime in seconds."""
        cmd = [str(self.binary_path), str(imu_csv), str(out_traj)]
        if self.append_blind:
            cmd.append("--append-blind")
        if ablation_mode:
            cmd.extend(["--ablation", ablation_mode])
        if self.extra_flags:
            cmd.extend(self.extra_flags)

        t_start = time.perf_counter()
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        runtime = time.perf_counter() - t_start

        if res.returncode != 0:
            raise RuntimeError(
                f"Estimator exited with code {res.returncode}:\n{res.stderr.strip()}"
            )
        return runtime

    def evaluate_case(
        self,
        seq_cfg: Dict[str, str],
        seed: int,
        work_dir: Path,
    ) -> Dict[str, Any]:
        """Evaluate a single sequence and seed under candidate and naive baseline."""
        seq_name = seq_cfg["name"]
        case_id = f"{seq_name}_s{seed}"
        case_deg_dir = work_dir / case_id / "deg"

        # 1. Prepare degraded dataset
        deg_imu, deg_gt, sidecar = self.prepare_degraded_dataset(
            seq_cfg, seed, case_deg_dir
        )

        # 2. Run Candidate Binary
        cand_traj = work_dir / case_id / "cand_traj.txt"
        cand_time = self.run_estimator(deg_imu, cand_traj)
        cand_metrics = compute_trajectory_metrics(deg_gt, cand_traj, sidecar_path=sidecar)

        # 3. Run Naive Baseline
        naive_traj = work_dir / case_id / "naive_traj.txt"
        naive_time = self.run_estimator(deg_imu, naive_traj, ablation_mode="naive")
        naive_metrics = compute_trajectory_metrics(deg_gt, naive_traj, sidecar_path=sidecar)

        # 4. Compute Loss Objectives
        J_standard = compute_objective_J(cand_metrics, naive_metrics)
        J_3way = compute_3way_loss(cand_metrics, naive_metrics)

        record = {
            "case_id": case_id,
            "sequence": seq_name,
            "type": seq_cfg["type"],
            "seed": seed,
            "runtime_s": round(cand_time, 4),
            "cand_metrics": cand_metrics,
            "naive_metrics": naive_metrics,
            # Key competition target metrics
            "ate_rmse_aligned": cand_metrics["ate_rmse_aligned"],
            "blind5_rmse": cand_metrics["blind5_rmse"],
            "gap_2s_rmse": cand_metrics["gap_2s_rmse"],
            "rpe_trans_1s": cand_metrics["rpe_trans_1s"],
            "rot_err_deg": cand_metrics["rot_err_deg"],
            "J_standard": J_standard,
            "J_3way": J_3way,
            # Naive baseline values for comparison
            "naive_ate_rmse_aligned": naive_metrics["ate_rmse_aligned"],
            "naive_blind5_rmse": naive_metrics["blind5_rmse"],
            "naive_gap_2s_rmse": naive_metrics["gap_2s_rmse"],
        }
        return record

    def run_full_matrix(
        self,
        sequences: Optional[List[Dict[str, str]]] = None,
        seeds: Optional[List[int]] = None,
    ) -> List[Dict[str, Any]]:
        """Run all combinations of sequences and seeds."""
        seq_list = sequences or REAL_SEQUENCES
        seed_list = seeds or DEFAULT_SEEDS
        total = len(seq_list) * len(seed_list)

        if self.verbose:
            print(f"\n=======================================================")
            print(f" Track 3 Degradation Evaluation Harness: Full Matrix")
            print(f" Binary: {self.binary_path}")
            print(f" Sequences ({len(seq_list)}): {[s['name'] for s in seq_list]}")
            print(f" Seeds ({len(seed_list)}): {seed_list}")
            print(f" Total Runs: {total}")
            print(f"=======================================================\n")

        all_records = []
        with tempfile.TemporaryDirectory(prefix="harness_eval_") as tmpdir:
            work_path = Path(tmpdir) if not self.cache_dir else self.cache_dir
            idx = 0
            for seq in seq_list:
                for seed in seed_list:
                    idx += 1
                    t0 = time.perf_counter()
                    rec = self.evaluate_case(seq, seed, work_path)
                    elapsed = time.perf_counter() - t0
                    all_records.append(rec)
                    if self.verbose:
                        print(
                            f"[{idx:2d}/{total:2d}] {rec['case_id']:<22} | "
                            f"ATE: {rec['ate_rmse_aligned']:7.4f} m | "
                            f"Blind-5s: {rec['blind5_rmse']:7.4f} m | "
                            f"Gap-2s: {rec['gap_2s_rmse']:7.4f} m | "
                            f"J: {rec['J_standard']:.4f} | ({elapsed:.2f}s)"
                        )
        return all_records

    def run_loocv(
        self,
        sequences: Optional[List[Dict[str, str]]] = None,
        train_seeds: Optional[List[int]] = None,
        test_seeds: Optional[List[int]] = None,
    ) -> Dict[str, Any]:
        """Execute 5-fold Leave-One-Out Cross-Validation."""
        seq_list = sequences or REAL_SEQUENCES
        tr_seeds = train_seeds or DEFAULT_TRAIN_SEEDS
        te_seeds = test_seeds or DEFAULT_TEST_SEEDS
        all_eval_seeds = sorted(list(set(tr_seeds + te_seeds)))

        if self.verbose:
            print(f"\n=======================================================")
            print(f" 5-Fold Leave-One-Out Cross-Validation (LOOCV)")
            print(f" Sequences: {[s['name'] for s in seq_list]}")
            print(f" Train Seeds: {tr_seeds} | Test Seeds: {te_seeds}")
            print(f"=======================================================\n")

        # First evaluate full matrix across the union of seeds
        matrix_records = self.run_full_matrix(sequences=seq_list, seeds=all_eval_seeds)
        records_by_key = {(r["sequence"], r["seed"]): r for r in matrix_records}

        folds_results = []
        n_seqs = len(seq_list)

        for fold_idx in range(n_seqs):
            test_seq = seq_list[fold_idx]
            train_seqs = [s for j, s in enumerate(seq_list) if j != fold_idx]

            # Collect train runs: train_seqs on tr_seeds
            train_runs = []
            for s in train_seqs:
                for seed in tr_seeds:
                    train_runs.append(records_by_key[(s["name"], seed)])

            # Collect test runs: test_seq on te_seeds
            test_runs = []
            for seed in te_seeds:
                test_runs.append(records_by_key[(test_seq["name"], seed)])

            fold_summary = {
                "fold": fold_idx + 1,
                "held_out_sequence": test_seq["name"],
                "train_sequences": [s["name"] for s in train_seqs],
                "train_seeds": tr_seeds,
                "test_seeds": te_seeds,
                "train_count": len(train_runs),
                "test_count": len(test_runs),
                "train_ate_mean": float(np.mean([r["ate_rmse_aligned"] for r in train_runs])),
                "train_ate_median": float(np.median([r["ate_rmse_aligned"] for r in train_runs])),
                "train_blind5_mean": float(np.mean([r["blind5_rmse"] for r in train_runs])),
                "train_gap2s_mean": float(np.mean([r["gap_2s_rmse"] for r in train_runs])),
                "train_J_mean": float(np.mean([r["J_standard"] for r in train_runs])),
                "train_J3_mean": float(np.mean([r["J_3way"] for r in train_runs])),
                "test_ate_mean": float(np.mean([r["ate_rmse_aligned"] for r in test_runs])),
                "test_ate_median": float(np.median([r["ate_rmse_aligned"] for r in test_runs])),
                "test_blind5_mean": float(np.mean([r["blind5_rmse"] for r in test_runs])),
                "test_gap2s_mean": float(np.mean([r["gap_2s_rmse"] for r in test_runs])),
                "test_J_mean": float(np.mean([r["J_standard"] for r in test_runs])),
                "test_J3_mean": float(np.mean([r["J_3way"] for r in test_runs])),
                "train_runs": train_runs,
                "test_runs": test_runs,
            }
            folds_results.append(fold_summary)

        # Compute cross-fold aggregate metrics
        test_ates = [f["test_ate_mean"] for f in folds_results]
        test_blind5s = [f["test_blind5_mean"] for f in folds_results]
        test_gap2s = [f["test_gap2s_mean"] for f in folds_results]
        test_Js = [f["test_J_mean"] for f in folds_results]
        test_J3s = [f["test_J3_mean"] for f in folds_results]

        train_ates = [f["train_ate_mean"] for f in folds_results]
        train_blind5s = [f["train_blind5_mean"] for f in folds_results]
        train_gap2s = [f["train_gap2s_mean"] for f in folds_results]
        train_Js = [f["train_J_mean"] for f in folds_results]

        aggregate = {
            "mean_test_ate": float(np.mean(test_ates)),
            "median_test_ate": float(np.median(test_ates)),
            "std_test_ate": float(np.std(test_ates)),
            "p95_test_ate": float(np.percentile(test_ates, 95)),
            "worst_test_ate": float(np.max(test_ates)),
            "mean_test_blind5": float(np.mean(test_blind5s)),
            "mean_test_gap2s": float(np.mean(test_gap2s)),
            "mean_test_J": float(np.mean(test_Js)),
            "mean_test_J3": float(np.mean(test_J3s)),
            "mean_train_ate": float(np.mean(train_ates)),
            "mean_train_blind5": float(np.mean(train_blind5s)),
            "mean_train_gap2s": float(np.mean(train_gap2s)),
            "mean_train_J": float(np.mean(train_Js)),
        }

        return {
            "folds": folds_results,
            "aggregate": aggregate,
            "all_runs": matrix_records,
        }


def format_markdown_table(headers: List[str], rows: List[List[Any]]) -> str:
    """Format headers and rows into a Markdown table."""
    col_widths = [len(h) for h in headers]
    str_rows = []
    for r in rows:
        row_strs = [str(val) for val in r]
        str_rows.append(row_strs)
        for i, val in enumerate(row_strs):
            col_widths[i] = max(col_widths[i], len(val))

    header_line = "| " + " | ".join(h.ljust(col_widths[i]) for i, h in enumerate(headers)) + " |"
    sep_line = "| " + " | ".join("-" * col_widths[i] for i in range(len(headers))) + " |"
    row_lines = [
        "| " + " | ".join(val.ljust(col_widths[i]) for i, val in enumerate(row)) + " |"
        for row in str_rows
    ]
    return "\n".join([header_line, sep_line] + row_lines)


def generate_scorecard_markdown(loocv_data: Dict[str, Any], binary_label: str = "./main") -> str:
    """Generate comprehensive GitHub Flavored Markdown scorecard report."""
    md = []
    md.append(f"# Track 3 LOOCV Baseline Evaluation Scorecard: `{binary_label}`\n")
    md.append(f"**Date**: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    md.append(f"**Harness**: `tools/eval_degraded_harness.py`")
    md.append(f"**Evaluation Mode**: 5-Fold Leave-One-Out Cross-Validation (LOOCV)")
    md.append(f"**Degradation Presets**: `configs/degrade_default.json`")
    md.append(f"**Train Seeds**: {DEFAULT_TRAIN_SEEDS} | **Test Seeds**: {DEFAULT_TEST_SEEDS}\n")

    agg = loocv_data["aggregate"]
    md.append("## 1. Executive LOOCV Summary\n")
    md.append(
        "| Metric | Train Folds Mean | Held-Out Test Mean | Held-Out Median | Held-Out P95 | Held-Out Worst |"
    )
    md.append("|:---|:---:|:---:|:---:|:---:|:---:|")
    md.append(
        f"| **Trajectory ATE RMSE (SE(3))** | {agg['mean_train_ate']:.4f} m | **{agg['mean_test_ate']:.4f} m** | {agg['median_test_ate']:.4f} m | {agg['p95_test_ate']:.4f} m | {agg['worst_test_ate']:.4f} m |"
    )
    md.append(
        f"| **Final 5s Blind Cutoff RMSE** | {agg['mean_train_blind5']:.4f} m | **{agg['mean_test_blind5']:.4f} m** | - | - | - |"
    )
    md.append(
        f"| **2s Gap Blackout RMSE** | {agg['mean_train_gap2s']:.4f} m | **{agg['mean_test_gap2s']:.4f} m** | - | - | - |"
    )
    md.append(
        f"| **Composite Objective Loss J** | {agg['mean_train_J']:.4f} | **{agg['mean_test_J']:.4f}** | - | - | - |"
    )
    md.append(
        f"| **3-Metric Target Loss J3** | - | **{agg['mean_test_J3']:.4f}** | - | - | - |\n"
    )

    md.append("## 2. 5-Fold Leave-One-Out Cross-Validation Breakdown\n")
    fold_headers = [
        "Fold",
        "Held-Out Sequence",
        "Train ATE (m)",
        "Test ATE (m)",
        "Test Blind5 (m)",
        "Test Gap2s (m)",
        "Test Loss J",
        "Test Loss J3",
    ]
    fold_rows = []
    for f in loocv_data["folds"]:
        fold_rows.append(
            [
                f"Fold {f['fold']}",
                f"{f['held_out_sequence']}",
                f"{f['train_ate_mean']:.4f}",
                f"{f['test_ate_mean']:.4f}",
                f"{f['test_blind5_mean']:.4f}",
                f"{f['test_gap2s_mean']:.4f}",
                f"{f['test_J_mean']:.4f}",
                f"{f['test_J3_mean']:.4f}",
            ]
        )
    md.append(format_markdown_table(fold_headers, fold_rows))
    md.append("")

    md.append("## 3. Comprehensive Per-Run Matrix (25 Degradation Trials)\n")
    run_headers = [
        "Sequence",
        "Seed",
        "Partition",
        "ATE (m)",
        "Blind-5s (m)",
        "Gap-2s (m)",
        "RPE 1s (m)",
        "Loss J",
        "Loss J3",
        "Naive ATE (m)",
        "Runtime",
    ]
    run_rows = []
    for r in loocv_data["all_runs"]:
        partition = "Train" if r["seed"] in DEFAULT_TRAIN_SEEDS else "Test"
        run_rows.append(
            [
                r["sequence"],
                str(r["seed"]),
                partition,
                f"{r['ate_rmse_aligned']:.4f}",
                f"{r['blind5_rmse']:.4f}",
                f"{r['gap_2s_rmse']:.4f}",
                f"{r['rpe_trans_1s']:.4f}",
                f"{r['J_standard']:.4f}",
                f"{r['J_3way']:.4f}",
                f"{r['naive_ate_rmse_aligned']:.2f}",
                f"{r['runtime_s']:.3f}s",
            ]
        )
    md.append(format_markdown_table(run_headers, run_rows))
    md.append("")

    return "\n".join(md)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Track 3 Degradation Evaluation Harness & LOOCV Benchmark."
    )
    parser.add_argument(
        "--binary",
        type=str,
        default="./main",
        help="Path to estimator executable (default: ./main)",
    )
    parser.add_argument(
        "--config",
        type=str,
        default="configs/degrade_default.json",
        help="Degradation configuration JSON",
    )
    parser.add_argument(
        "--seeds",
        type=str,
        default="42,123,456,789,999",
        help="Comma-separated seeds (default: 42,123,456,789,999)",
    )
    parser.add_argument(
        "--train-seeds",
        type=str,
        default="42,123,456",
        help="Comma-separated train seeds (default: 42,123,456)",
    )
    parser.add_argument(
        "--test-seeds",
        type=str,
        default="789,999",
        help="Comma-separated test seeds (default: 789,999)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["loocv", "matrix", "quick"],
        default="loocv",
        help="Execution mode: loocv (default), matrix, quick (smoke test)",
    )
    parser.add_argument(
        "--json-out",
        type=str,
        default="reports/loocv_baseline_scorecard.json",
        help="Output path for JSON results",
    )
    parser.add_argument(
        "--md-out",
        type=str,
        default="reports/loocv_baseline_scorecard.md",
        help="Output path for Markdown scorecard",
    )
    parser.add_argument(
        "--no-append-blind",
        action="store_true",
        help="Omit --append-blind flag when calling estimator",
    )

    args = parser.parse_args()

    seeds = [int(s.strip()) for s in args.seeds.split(",") if s.strip()]
    train_seeds = [int(s.strip()) for s in args.train_seeds.split(",") if s.strip()]
    test_seeds = [int(s.strip()) for s in args.test_seeds.split(",") if s.strip()]

    harness = DegradedHarness(
        binary_path=args.binary,
        config_path=args.config,
        append_blind=not args.no_append_blind,
        verbose=True,
    )

    if args.mode == "quick":
        # Smoke test: 2 sequences, 2 seeds
        quick_seqs = REAL_SEQUENCES[:2]
        quick_seeds = seeds[:2]
        records = harness.run_full_matrix(sequences=quick_seqs, seeds=quick_seeds)
        print(f"\nQuick test passed: {len(records)} runs completed.")
        return

    if args.mode == "matrix":
        records = harness.run_full_matrix(seeds=seeds)
        print(f"\nFull matrix completed: {len(records)} runs.")
        return

    # LOOCV Mode (default)
    loocv_data = harness.run_loocv(
        sequences=REAL_SEQUENCES,
        train_seeds=train_seeds,
        test_seeds=test_seeds,
    )

    # Generate Markdown Scorecard
    md_content = generate_scorecard_markdown(loocv_data, binary_label=args.binary)
    print("\n" + md_content)

    # Save reports
    if args.json_out:
        json_path = Path(args.json_out)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        # Prepare serializable json
        clean_data = copy.deepcopy(loocv_data)
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(clean_data, f, indent=2)
        print(f"\n[INFO] LOOCV JSON scorecard saved to {json_path}")

    if args.md_out:
        md_path = Path(args.md_out)
        md_path.parent.mkdir(parents=True, exist_ok=True)
        with open(md_path, "w", encoding="utf-8") as f:
            f.write(md_content)
        print(f"[INFO] LOOCV Markdown scorecard saved to {md_path}")


if __name__ == "__main__":
    main()
