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


def run_eval_py_ref(output_table_path: Optional[Path] = None) -> None:
    """Run full evaluation comparing naive, mahony, eskf_basic, and eskf_full across degraded sequences (Gate G2)."""
    print("=== Running Python Reference Pipeline Ablation Evaluation (Gate G2) ===")

    sequence_specs = [
        {
            "name": "circle_degraded",
            "type": "circle",
            "duration": 15.0,
            "rate": 100.0,
            "seed": 42,
            "deg_cfg": {
                "noise_growth_rate": 0.05,
                "bias_drift_walk": 0.001,
                "dropouts": [
                    {"start_s": 5.0, "duration_s": 1.5, "encoding": "missing"},
                    {"start_s": 8.0, "duration_s": 1.0, "encoding": "zeros"},
                ],
                "gravity_rotation": True,
                "failure_at_end_s": 5.0,
            },
        },
        {
            "name": "figure_eight_degraded",
            "type": "figure_eight",
            "duration": 15.0,
            "rate": 100.0,
            "seed": 101,
            "deg_cfg": {
                "noise_growth_rate": 0.05,
                "bias_drift_walk": 0.001,
                "dropouts": [
                    {"start_s": 4.5, "duration_s": 1.2, "encoding": "repeated"},
                    {"start_s": 7.5, "duration_s": 1.5, "encoding": "missing"},
                ],
                "gravity_rotation": True,
                "failure_at_end_s": 5.0,
            },
        },
    ]

    ablation_modes = ["naive", "mahony", "eskf_basic", "eskf_full"]

    headers = [
        "Sequence",
        "Ablation",
        "ATE (m)",
        "Blind-5s (m)",
        "Dropout (m)",
        "RPE 1s (m)",
        "Start-Anchored (m)",
        "Rot Err (°)",
        "Objective J",
    ]

    all_table_rows: List[List[str]] = []
    summary_data: Dict[str, Dict[str, List[float]]] = {
        mode: {"ate": [], "blind5": [], "dropout": [], "rpe": [], "J": []}
        for mode in ablation_modes
    }

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)

        for seq in sequence_specs:
            seq_name = seq["name"]
            print(f"\nEvaluating sequence: {seq_name}...")

            traj = generate_trajectory(seq["type"], duration=seq["duration"], rate=seq["rate"])
            clean_imu = tmp_path / f"{seq_name}_clean_imu.csv"
            clean_gt = tmp_path / f"{seq_name}_clean_gt.txt"
            write_euroc_imu_csv(clean_imu, traj["t_ns"], traj["omega"], traj["acc"])
            write_tum_trajectory(clean_gt, traj["t_ns"], traj["p"], traj["q"])

            cfg_file = tmp_path / f"{seq_name}_deg_cfg.json"
            with open(cfg_file, "w", encoding="utf-8") as f:
                json.dump(seq["deg_cfg"], f)

            deg_dir = tmp_path / seq_name
            write_degraded_dataset(
                clean_imu, clean_gt, deg_dir, config_path=cfg_file, seed=seq["seed"]
            )
            deg_imu = deg_dir / "imu0" / "data.csv"
            gt_file = deg_dir / "groundtruth.txt"
            sidecar_file = deg_dir / "degrade_info.json"

            seq_metrics: Dict[str, Dict[str, float]] = {}

            for mode in ablation_modes:
                out_traj = tmp_path / f"{seq_name}_{mode}.txt"
                cmd = [
                    sys.executable,
                    "-m",
                    "src.ref",
                    str(deg_imu),
                    str(out_traj),
                    "--ablation",
                    mode,
                    "--append-blind",
                ]
                subprocess.run(cmd, check=True, capture_output=True)

                is_valid, errors = validate_trajectory(deg_imu, out_traj, append_blind=True)
                if not is_valid:
                    raise RuntimeError(
                        f"Trajectory validation failed for {mode} on {seq_name}: {errors}"
                    )

                m = compute_trajectory_metrics(gt_file, out_traj, sidecar_path=sidecar_file)
                seq_metrics[mode] = m

            naive_m = seq_metrics["naive"]
            for mode in ablation_modes:
                m = seq_metrics[mode]
                m["J"] = compute_objective_J(m, naive_m)
                summary_data[mode]["ate"].append(m["ate_rmse_aligned"])
                summary_data[mode]["blind5"].append(m["blind5_rmse"])
                summary_data[mode]["dropout"].append(m["dropout_rmse"])
                summary_data[mode]["rpe"].append(m["rpe_trans_1s"])
                summary_data[mode]["J"].append(m["J"])

                all_table_rows.append(
                    [
                        seq_name,
                        mode,
                        f"{m['ate_rmse_aligned']:.4f}",
                        f"{m['blind5_rmse']:.4f}",
                        f"{m['dropout_rmse']:.4f}",
                        f"{m['rpe_trans_1s']:.4f}",
                        f"{m['start_anchored_rmse']:.4f}",
                        f"{m['rot_err_deg']:.2f}",
                        f"{m['J']:.4f}",
                    ]
                )

            # Verification of Gate G2 Acceptance Criteria:
            # Python ESKF beats naive baseline on ATE and blind-5s on all sequences
            eskf_m = seq_metrics["eskf_full"]
            assert eskf_m["ate_rmse_aligned"] < naive_m["ate_rmse_aligned"], (
                f"Gate G2 check failed on {seq_name}: "
                f"eskf_full ATE ({eskf_m['ate_rmse_aligned']:.4f}) >= naive ({naive_m['ate_rmse_aligned']:.4f})"
            )
            assert eskf_m["blind5_rmse"] < naive_m["blind5_rmse"], (
                f"Gate G2 check failed on {seq_name}: "
                f"eskf_full blind-5s ({eskf_m['blind5_rmse']:.4f}) >= naive ({naive_m['blind5_rmse']:.4f})"
            )
            print(f"  [PASS] eskf_full beats naive baseline on {seq_name} (ATE: {eskf_m['ate_rmse_aligned']:.4f} vs {naive_m['ate_rmse_aligned']:.4f}, Blind-5s: {eskf_m['blind5_rmse']:.4f} vs {naive_m['blind5_rmse']:.4f})")

        # Summary averages
        summary_rows = []
        for mode in ablation_modes:
            mean_ate = float(np.mean(summary_data[mode]["ate"]))
            mean_b5 = float(np.mean(summary_data[mode]["blind5"]))
            mean_drop = float(np.mean(summary_data[mode]["dropout"]))
            mean_rpe = float(np.mean(summary_data[mode]["rpe"]))
            mean_j = float(np.mean(summary_data[mode]["J"]))
            summary_rows.append(
                [
                    "MEAN",
                    mode,
                    f"{mean_ate:.4f}",
                    f"{mean_b5:.4f}",
                    f"{mean_drop:.4f}",
                    f"{mean_rpe:.4f}",
                    "-",
                    "-",
                    f"{mean_j:.4f}",
                ]
            )

        table_md = format_markdown_table(headers, all_table_rows)
        summary_table_md = format_markdown_table(headers, summary_rows)

        full_report_md = (
            "# Python Reference Pipeline Ablation Results (Gate G2)\n\n"
            "Evaluation comparing `naive`, `mahony`, `eskf_basic`, and `eskf_full` across degraded benchmark sequences.\n\n"
            "## Per-Sequence Results\n\n"
            f"{table_md}\n\n"
            "## Overall Summary (Mean Across Sequences)\n\n"
            f"{summary_table_md}\n\n"
            "## Gate G2 Acceptance Verification\n\n"
            "- **All output trajectories passed trajectory validation** (`tools/validate_trajectory.py`): PASS\n"
            "- **`eskf_full` beats `naive` baseline on ATE on all held-out sequences**: PASS\n"
            "- **`eskf_full` beats `naive` baseline on Blind-5s RMSE on all held-out sequences**: PASS\n"
            f"- **Mean scalar objective J**: `eskf_full` ({np.mean(summary_data['eskf_full']['J']):.4f}) vs `naive` (1.0000)\n"
        )

        print("\n" + full_report_md)

        if output_table_path is None:
            output_table_path = Path("results/ablation_py.md")
        output_table_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_table_path, "w", encoding="utf-8") as f:
            f.write(full_report_md)
        print(f"Results table successfully saved to {output_table_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluation harness runner and results table generator.")
    parser.add_argument("--smoke-test", action="store_true", help="Run quick smoke test on synthetic sequence")
    parser.add_argument("--eval-py-ref", action="store_true", help="Run full evaluation of Python reference pipeline and ablations across degraded sequences")
    parser.add_argument("--output-table", type=str, default=None, help="Output markdown table file path")

    args = parser.parse_args()

    out_p = Path(args.output_table) if args.output_table else None
    if args.eval_py_ref:
        run_eval_py_ref(out_p)
    elif args.smoke_test:
        run_smoke_test(out_p)
    else:
        # Default run smoke test if no other arguments given
        run_smoke_test(out_p)


if __name__ == "__main__":
    main()
