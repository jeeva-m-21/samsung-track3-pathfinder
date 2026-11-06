"""CLI Entrypoint for Reference 15-state ESKF Estimator Pipeline (SPEC §6.3, WP-28).

Usage:
    python3 -m src.ref <imu_input> <output_trajectory.txt> [--append-blind] [--ablation MODE]

Ablation modes:
    - naive: initial tilt + raw strapdown
    - mahony: Mahony attitude filter
    - eskf_basic: 15-state ESKF with fixed noise, no monitor inflation, no gap substepping, no blind model
    - eskf_full: full system
"""

import argparse
import sys
from pathlib import Path

from src.ref.pipeline import run_pipeline


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Reference 15-state ESKF Estimator Pipeline and Ablation Modes (SPEC §6.3, §11.5, §12)."
    )
    parser.add_argument(
        "imu_input",
        type=str,
        help="Path to input IMU CSV file (or directory containing imu0/data.csv)",
    )
    parser.add_argument(
        "output_trajectory",
        type=str,
        help="Path to output trajectory text file",
    )
    parser.add_argument(
        "--append-blind",
        action="store_true",
        help="Extrapolate 5.0 s poses after input stream ends",
    )
    parser.add_argument(
        "--ablation",
        type=str,
        default="eskf_full",
        choices=["naive", "mahony", "eskf_basic", "eskf_full"],
        help="Ablation mode: naive, mahony, eskf_basic, eskf_full (default: eskf_full)",
    )
    parser.add_argument(
        "--params",
        type=str,
        default=None,
        help="Path to custom parameters JSON file (default: configs/params.json)",
    )

    args = parser.parse_args()

    run_pipeline(
        imu_input=args.imu_input,
        traj_output=args.output_trajectory,
        ablation=args.ablation,
        append_blind=args.append_blind,
        params=args.params,
    )


if __name__ == "__main__":
    main()
