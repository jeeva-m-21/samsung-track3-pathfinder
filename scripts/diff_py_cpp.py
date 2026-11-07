#!/usr/bin/env python3
"""Differential Verification between Python Reference and C++ Streaming Binary (SPEC §12.11, Gate G3).

Verifies numerical parity:
- Synthetic trajectories: max position diff <= 1e-6 m, quaternion diff <= 1e-6.
- Degraded trajectories: max position diff <= 1e-4 m, quaternion diff <= 1e-6.
- Timestamps: identical integer strings on every line.
- Validates trajectory format via tools.validate_trajectory.

Supported modes: naive, mahony, eskf_basic, eskf_full.
Supports --append-blind verification.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import numpy as np

# Ensure repo root is on sys.path
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.synth import generate_trajectory, write_euroc_imu_csv, write_tum_trajectory
from tools.degrade import write_degraded_dataset
from tools.validate_trajectory import validate_trajectory
from src.ref.pipeline import run_pipeline as run_python_pipeline

CPP_BIN = REPO_ROOT / "src/cpp" / "build" / "main"


def load_trajectory_file(path: Path) -> List[Tuple[str, np.ndarray, np.ndarray]]:
    """Load TUM trajectory file into list of (timestamp_str, pos[3], quat[4]).

    Returns quaternion in order [w, x, y, z] mapped from file format [qx, qy, qz, qw].
    """
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts_str = parts[0]
            tx = float(parts[1])
            ty = float(parts[2])
            tz = float(parts[3])
            qx = float(parts[4])
            qy = float(parts[5])
            qz = float(parts[6])
            qw = float(parts[7])
            pos = np.array([tx, ty, tz], dtype=np.float64)
            # Internal Hamilton [w, x, y, z]
            quat = np.array([qw, qx, qy, qz], dtype=np.float64)
            rows.append((ts_str, pos, quat))
    return rows


def compare_trajectories(
    py_rows: List[Tuple[str, np.ndarray, np.ndarray]],
    cpp_rows: List[Tuple[str, np.ndarray, np.ndarray]],
    pos_tol: float,
    quat_tol: float,
) -> Tuple[bool, float, float, Optional[str]]:
    """Compare Python and C++ output trajectories.

    Returns:
        (passed, max_pos_diff, max_quat_diff, error_message)
    """
    if len(py_rows) != len(cpp_rows):
        return (
            False,
            float("inf"),
            float("inf"),
            f"Row count mismatch: Python={len(py_rows)} vs C++={len(cpp_rows)}",
        )

    max_pos_diff = 0.0
    max_quat_diff = 0.0

    for idx, ((ts_py, p_py, q_py), (ts_cpp, p_cpp, q_cpp)) in enumerate(
        zip(py_rows, cpp_rows)
    ):
        if ts_py != ts_cpp:
            return (
                False,
                max_pos_diff,
                max_quat_diff,
                f"Line {idx + 1}: Timestamp string mismatch ('{ts_py}' vs '{ts_cpp}')",
            )

        pos_diff = float(np.max(np.abs(p_py - p_cpp)))
        if pos_diff > max_pos_diff:
            max_pos_diff = pos_diff

        # Align quaternion signs (q and -q represent same orientation)
        q_c = q_cpp.copy()
        if float(np.dot(q_py, q_c)) < 0.0:
            q_c = -q_c

        quat_diff = float(np.max(np.abs(q_py - q_c)))
        if quat_diff > max_quat_diff:
            max_quat_diff = quat_diff

    passed = (max_pos_diff <= pos_tol) and (max_quat_diff <= quat_tol)
    err = None
    if not passed:
        err = f"Diff exceeded tolerance: pos_diff={max_pos_diff:.2e} (tol={pos_tol:.2e}), quat_diff={max_quat_diff:.2e} (tol={quat_tol:.2e})"

    return passed, max_pos_diff, max_quat_diff, err


def run_cpp_pipeline(
    imu_path: Path,
    out_path: Path,
    ablation: str = "eskf_full",
    append_blind: bool = False,
) -> None:
    """Run C++ compiled binary on IMU input."""
    cmd = [str(CPP_BIN), str(imu_path), str(out_path), "--ablation", ablation]
    if append_blind:
        cmd.append("--append-blind")

    proc = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"C++ binary failed (code {proc.returncode}):\n{proc.stderr}\n{proc.stdout}"
        )


def run_single_check(
    name: str,
    imu_path: Path,
    ablation: str,
    append_blind: bool,
    is_degraded: bool,
    work_dir: Path,
) -> Dict[str, any]:
    """Execute differential check for a single case."""
    pos_tol = 1e-4 if is_degraded else 1e-6
    quat_tol = 1e-6

    suffix = f"{name}_{ablation}"
    if append_blind:
        suffix += "_blind"

    py_out = work_dir / f"py_{suffix}.txt"
    cpp_out = work_dir / f"cpp_{suffix}.txt"

    # 1. Run Python reference
    run_python_pipeline(
        imu_input=imu_path,
        traj_output=py_out,
        ablation=ablation,
        append_blind=append_blind,
    )

    # 2. Run C++ binary
    run_cpp_pipeline(
        imu_path=imu_path,
        out_path=cpp_out,
        ablation=ablation,
        append_blind=append_blind,
    )

    # 3. Validate C++ output with trajectory validator
    valid_ok, valid_errs = validate_trajectory(
        imu_input=imu_path, traj_output=cpp_out, append_blind=append_blind
    )
    if not valid_ok:
        return {
            "name": name,
            "ablation": ablation,
            "blind": append_blind,
            "passed": False,
            "pos_diff": float("nan"),
            "quat_diff": float("nan"),
            "pos_tol": pos_tol,
            "quat_tol": quat_tol,
            "error": f"Trajectory validator failed: {valid_errs[:2]}",
        }

    # 4. Compare trajectories
    py_rows = load_trajectory_file(py_out)
    cpp_rows = load_trajectory_file(cpp_out)

    passed, max_p, max_q, err = compare_trajectories(
        py_rows, cpp_rows, pos_tol=pos_tol, quat_tol=quat_tol
    )

    return {
        "name": name,
        "ablation": ablation,
        "blind": append_blind,
        "passed": passed,
        "rows": len(cpp_rows),
        "pos_diff": max_p,
        "quat_diff": max_q,
        "pos_tol": pos_tol,
        "quat_tol": quat_tol,
        "error": err,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Differential verification of Python and C++ implementations."
    )
    parser.add_argument(
        "--all", action="store_true", help="Run comprehensive differential test suite"
    )
    parser.add_argument(
        "--seq", type=str, default=None, help="Filter to specific sequence name"
    )
    parser.add_argument(
        "--ablation", type=str, default=None, help="Filter to specific ablation mode"
    )

    args = parser.parse_args()

    # Verify C++ binary exists and is built
    if not CPP_BIN.is_file():
        print(f"Building C++ binary at {CPP_BIN}...")
        subprocess.run(["make", "-C", "src/cpp"], check=True)

    with tempfile.TemporaryDirectory() as tmp_dir_str:
        tmp_dir = Path(tmp_dir_str)
        synth_dir = tmp_dir / "synth"
        synth_dir.mkdir(parents=True, exist_ok=True)

        # 1. Prepare synthetic trajectories
        synthetic_cases = [
            ("circle", "circle", 10.0, 200.0),
            ("static", "static", 5.0, 200.0),
            ("figure_eight", "figure_eight", 10.0, 200.0),
            ("constant_yaw", "constant_yaw", 10.0, 200.0),
            ("sum_of_sines", "sum_of_sines", 10.0, 200.0),
        ]

        synth_paths: Dict[str, Path] = {}
        # Check if data/synth/circle.csv exists to save generation time
        default_circle = REPO_ROOT / "data" / "synth" / "circle.csv"
        if default_circle.is_file():
            synth_paths["circle"] = default_circle

        for seq_name, traj_type, dur, rate in synthetic_cases:
            if seq_name in synth_paths:
                continue
            imu_csv = synth_dir / f"{seq_name}.csv"
            gt_txt = synth_dir / f"{seq_name}_gt.txt"
            traj = generate_trajectory(traj_type, duration=dur, rate=rate)
            write_euroc_imu_csv(imu_csv, traj["t_ns"], traj["omega"], traj["acc"])
            write_tum_trajectory(gt_txt, traj["t_ns"], traj["p"], traj["q"])
            synth_paths[seq_name] = imu_csv

        # 2. Prepare degraded trajectories
        deg_cases = [
            {
                "name": "circle_degraded",
                "base": "circle",
                "seed": 42,
                "cfg": {
                    "noise_mult": 2.0,
                    "bias_drift": True,
                    "dropout_type": "missing_rows",
                    "dropout_duration_s": 0.5,
                    "gravity_rotation": True,
                    "failure_at_end_s": 2.0,
                },
            },
            {
                "name": "figure_eight_degraded",
                "base": "figure_eight",
                "seed": 101,
                "cfg": {
                    "noise_mult": 3.0,
                    "bias_drift": True,
                    "dropout_type": "nan_values",
                    "dropout_duration_s": 0.5,
                    "gravity_rotation": True,
                    "failure_at_end_s": 2.0,
                },
            },
        ]

        deg_paths: Dict[str, Path] = {}
        for d in deg_cases:
            d_out = tmp_dir / d["name"]
            base_imu = synth_paths[d["base"]]
            base_gt = synth_dir / f"{d['base']}_gt.txt"
            if not base_gt.is_file():
                base_gt = REPO_ROOT / "data" / "synth" / f"{d['base']}_gt.txt"

            write_degraded_dataset(
                imu_in=base_imu,
                gt_in=base_gt if base_gt.is_file() else None,
                out_dir=d_out,
                config_dict=d["cfg"],
                seed=d["seed"],
            )
            deg_paths[d["name"]] = d_out / "imu0" / "data.csv"

        # 3. Build test checklist
        all_ablations = ["naive", "mahony", "eskf_basic", "eskf_full"]
        if args.ablation:
            all_ablations = [args.ablation]

        test_queue = []

        # Synthetic tests
        for seq_name, imu_p in synth_paths.items():
            if args.seq and args.seq != seq_name:
                continue
            for abl in all_ablations:
                # Primary pass
                test_queue.append((seq_name, imu_p, abl, False, False))
                # Append blind test for circle and static
                if (args.all or seq_name in ("circle", "static")) and abl in (
                    "eskf_basic",
                    "eskf_full",
                ):
                    test_queue.append((seq_name, imu_p, abl, True, False))

        # Degraded tests
        for d_name, imu_p in deg_paths.items():
            if args.seq and args.seq != d_name:
                continue
            for abl in all_ablations:
                test_queue.append((d_name, imu_p, abl, False, True))
                if abl == "eskf_full":
                    test_queue.append((d_name, imu_p, abl, True, True))

        # 4. Execute all checks
        print(f"=== Running Differential Verification ({len(test_queue)} test cases) ===")
        results = []
        all_passed = True

        for name, imu_p, abl, blind, deg in test_queue:
            blind_str = "+blind" if blind else ""
            case_label = f"{name} [{abl}{blind_str}]"
            print(f"  Testing {case_label}...", end=" ", flush=True)

            res = run_single_check(
                name=name,
                imu_path=imu_p,
                ablation=abl,
                append_blind=blind,
                is_degraded=deg,
                work_dir=tmp_dir,
            )
            results.append(res)

            if res["passed"]:
                print(
                    f"PASS (rows={res['rows']}, pos_diff={res['pos_diff']:.2e} m, quat_diff={res['quat_diff']:.2e})"
                )
            else:
                print(f"FAIL! {res['error']}")
                all_passed = False

        # 5. Print summary table
        print("\n" + "=" * 90)
        print(
            f"{'Sequence':<24} {'Ablation':<12} {'Blind':<7} {'Rows':<7} {'Max Pos Diff (m)':<18} {'Max Quat Diff':<14} {'Status':<6}"
        )
        print("-" * 90)
        for r in results:
            b_str = "yes" if r["blind"] else "no"
            rows_str = str(r.get("rows", "-"))
            p_str = f"{r['pos_diff']:.2e}" if not np.isnan(r["pos_diff"]) else "N/A"
            q_str = f"{r['quat_diff']:.2e}" if not np.isnan(r["quat_diff"]) else "N/A"
            status = "PASS" if r["passed"] else "FAIL"
            print(
                f"{r['name']:<24} {r['ablation']:<12} {b_str:<7} {rows_str:<7} {p_str:<18} {q_str:<14} {status:<6}"
            )
        print("=" * 90)

        if all_passed:
            print(f"\nALL {len(results)} DIFFERENTIAL TESTS PASSED.")
            sys.exit(0)
        else:
            failed_count = sum(1 for r in results if not r["passed"])
            print(f"\nDIFFERENTIAL VERIFICATION FAILED: {failed_count} tests failed.")
            sys.exit(1)


if __name__ == "__main__":
    main()
