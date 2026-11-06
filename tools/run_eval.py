"""Evaluation harness runner and results table generator (SPEC §6.5 & PLAN.md §11).

Automates:
1. Running estimator binaries or python modules over input sequences
2. Validating output trajectory formatting (tools.validate_trajectory)
3. Evaluating segment metrics against ground truth (tools.metrics)
4. Normalizing against naive baseline to compute scalar objective J
5. Formatting and printing Markdown evaluation tables
"""

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Optional
import numpy as np

from tools.synth import generate_trajectory, write_euroc_imu_csv, write_tum_trajectory
from tools.degrade import write_degraded_dataset
from tools.validate_trajectory import validate_trajectory
from tools.metrics import compute_trajectory_metrics, compute_objective_J


def format_markdown_table(headers: List[str], rows: List[List[str]]) -> str:
    """Format headers and rows into a GitHub-flavored Markdown table."""
    col_widths = [len(h) for h in headers]
    for row in rows:
        for idx, val in enumerate(row):
            col_widths[idx] = max(col_widths[idx], len(str(val)))

    header_line = "| " + " | ".join(h.ljust(col_widths[i]) for i, h in enumerate(headers)) + " |"
    sep_line = "| " + " | ".join("-" * col_widths[i] for i in range(len(headers))) + " |"
    row_lines = [
        "| " + " | ".join(str(val).ljust(col_widths[i]) for i, val in enumerate(row)) + " |"
        for row in rows
    ]

    return "\n".join([header_line, sep_line] + row_lines)


def run_smoke_test(output_table_path: Optional[Path] = None) -> None:
    """Run smoke test evaluating naive and Mahony baselines on synthetic circle sequence."""
    print("=== Running Evaluation Harness Smoke Test ===")
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)
        duration = 10.0
        rate = 100.0

        # 1. Generate clean synthetic dataset
        traj = generate_trajectory("circle", duration=duration, rate=rate)
        clean_imu = tmp_path / "clean_imu.csv"
        clean_gt = tmp_path / "clean_gt.txt"
        write_euroc_imu_csv(clean_imu, traj["t_ns"], traj["omega"], traj["acc"])
        write_tum_trajectory(clean_gt, traj["t_ns"], traj["p"], traj["q"])

        # 2. Degrade dataset with standard config
        deg_dir = tmp_path / "degraded_seq"
        write_degraded_dataset(
            imu_in=clean_imu,
            gt_in=clean_gt,
            out_dir=deg_dir,
            config_path="configs/degrade_default.json",
            seed=42,
        )

        deg_imu = deg_dir / "imu0" / "data.csv"
        gt_file = deg_dir / "groundtruth.txt"
        sidecar_file = deg_dir / "degrade_info.json"

        # 3. Run estimators
        estimators = {
            "naive": [sys.executable, "-m", "src.ref.baseline_naive"],
            "mahony": [sys.executable, "-m", "src.ref.baseline_mahony"],
        }

        metrics_results: Dict[str, Dict[str, float]] = {}

        for name, cmd_prefix in estimators.items():
            out_traj = tmp_path / f"{name}_traj.txt"
            full_cmd = cmd_prefix + [str(deg_imu), str(out_traj)]
            subprocess.run(full_cmd, check=True, capture_output=True)

            # Validate output
            is_valid, errors = validate_trajectory(deg_imu, out_traj)
            if not is_valid:
                raise RuntimeError(f"Estimator {name} output failed validation: {errors}")

            # Compute metrics
            m = compute_trajectory_metrics(gt_file, out_traj, sidecar_path=sidecar_file)
            metrics_results[name] = m

        # 4. Compute J normalized to naive baseline
        naive_metrics = metrics_results["naive"]
        for name in estimators:
            metrics_results[name]["J"] = compute_objective_J(metrics_results[name], naive_metrics)

        # 5. Format results table
        headers = [
            "Estimator",
            "ATE (m)",
            "RPE 1s (m)",
            "Start-Anchored (m)",
            "Checkpoint (m)",
            "Blind-5s (m)",
            "Rot Err (°)",
            "Objective J",
        ]
        table_rows = []
        for name in ["naive", "mahony"]:
            m = metrics_results[name]
            table_rows.append(
                [
                    name,
                    f"{m['ate_rmse_aligned']:.4f}",
                    f"{m['rpe_trans_1s']:.4f}",
                    f"{m['start_anchored_rmse']:.4f}",
                    f"{m['checkpoint_err']:.4f}",
                    f"{m['blind5_rmse']:.4f}",
                    f"{m['rot_err_deg']:.2f}",
                    f"{m['J']:.4f}",
                ]
            )

        table_md = format_markdown_table(headers, table_rows)
        print("\n" + table_md + "\n")

        # Save table to results/
        if output_table_path is None:
            output_table_path = Path("results/table_g1.md")
        output_table_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_table_path, "w", encoding="utf-8") as f:
            f.write("# Gate G1 Evaluation Table\n\n" + table_md + "\n")
        print(f"Results table written to {output_table_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluation harness runner and results table generator.")
    parser.add_argument("--smoke-test", action="store_true", help="Run quick smoke test on synthetic sequence")
    parser.add_argument("--output-table", type=str, default=None, help="Output markdown table file path")

    args = parser.parse_args()

    if args.smoke_test:
        out_p = Path(args.output_table) if args.output_table else None
        run_smoke_test(out_p)
    else:
        # Default run smoke test if no other arguments given
        out_p = Path(args.output_table) if args.output_table else None
        run_smoke_test(out_p)


if __name__ == "__main__":
    main()
