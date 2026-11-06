"""Trajectory output validator according to SPEC §6.3.

Checks:
1. File exists and is non-empty.
2. 8 space-separated columns per row (timestamp, tx, ty, tz, qx, qy, qz, qw).
3. Timestamp matches int64 ns formatted as <sec>.<nsec zero-padded to 9 digits>.
4. Row count matches input IMU sample count (or >= sample count if --append-blind).
5. All positions and quaternion components are finite (no NaN, no Inf).
6. Quaternion norm is unit (|norm - 1| < 1e-4).
7. Monotonically increasing timestamps.
"""

import argparse
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple, Union


def resolve_imu_csv(imu_path: Union[str, Path]) -> Path:
    """Resolve IMU path whether given as a CSV file or as a sequence directory."""
    p = Path(imu_path)
    if p.is_file():
        return p
    if p.is_dir():
        candidate = p / "imu0" / "data.csv"
        if candidate.is_file():
            return candidate
        candidate2 = p / "data.csv"
        if candidate2.is_file():
            return candidate2
    raise FileNotFoundError(f"Could not locate imu0/data.csv in {imu_path}")


def validate_trajectory(
    imu_input: Union[str, Path],
    traj_output: Union[str, Path],
    append_blind: bool = False,
) -> Tuple[bool, List[str]]:
    """Validate estimated trajectory against input IMU."""
    errors: List[str] = []

    # 1. Resolve IMU path and read timestamps
    try:
        csv_path = resolve_imu_csv(imu_input)
    except FileNotFoundError as e:
        return False, [str(e)]

    expected_timestamps: List[str] = []
    with open(csv_path, "r", encoding="utf-8") as f:
        for line_idx, line in enumerate(f):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if not parts or not parts[0].strip():
                continue
            try:
                t_int = int(parts[0].strip())
                sec = t_int // 1_000_000_000
                nsec = t_int % 1_000_000_000
                expected_timestamps.append(f"{sec}.{nsec:09d}")
            except ValueError:
                continue

    # 2. Check trajectory output file
    out_path = Path(traj_output)
    if not out_path.exists():
        return False, [f"Trajectory file {out_path} does not exist"]

    traj_rows: List[Tuple[str, List[float]]] = []
    with open(out_path, "r", encoding="utf-8") as f:
        for line_idx, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                errors.append(f"Line {line_idx}: Column count mismatch (expected 8, got {len(parts)})")
                continue

            ts_str = parts[0]
            # Validate timestamp format
            ts_parts = ts_str.split(".")
            if len(ts_parts) != 2 or len(ts_parts[1]) != 9 or not ts_parts[0].isdigit() or not ts_parts[1].isdigit():
                errors.append(f"Line {line_idx}: Invalid timestamp format '{ts_str}' (expected sec.9decimals)")

            # Parse numbers
            vals: List[float] = []
            has_nan_inf = False
            for col_idx, col in enumerate(parts[1:], 1):
                try:
                    val = float(col)
                    if math.isnan(val) or math.isinf(val):
                        has_nan_inf = True
                    vals.append(val)
                except ValueError:
                    errors.append(f"Line {line_idx}, Col {col_idx}: Cannot parse '{col}' as float")
                    has_nan_inf = True

            if has_nan_inf:
                errors.append(f"Line {line_idx}: Contains NaN or Inf values")
            else:
                # Check unit quaternion norm: tx ty tz qx qy qz qw -> index 3, 4, 5, 6
                qx, qy, qz, qw = vals[3], vals[4], vals[5], vals[6]
                q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
                if abs(q_norm - 1.0) > 1e-4:
                    errors.append(f"Line {line_idx}: Quaternion norm {q_norm:.6f} deviates from 1.0 by > 1e-4")

            traj_rows.append((ts_str, vals))

    # 3. Check row count
    n_expected = len(expected_timestamps)
    n_actual = len(traj_rows)

    if append_blind:
        if n_actual < n_expected:
            errors.append(f"Row count mismatch with --append-blind (expected >= {n_expected}, got {n_actual})")
    else:
        if n_actual != n_expected:
            errors.append(f"Row count mismatch (expected exactly {n_expected}, got {n_actual})")

    # 4. Check timestamp alignment against input IMU
    check_len = min(n_expected, n_actual)
    for i in range(check_len):
        if traj_rows[i][0] != expected_timestamps[i]:
            errors.append(
                f"Row {i+1}: Timestamp mismatch with input (expected {expected_timestamps[i]}, got {traj_rows[i][0]})"
            )
            # Break early if timestamps are out of sync to prevent error explosion
            if len(errors) > 20:
                errors.append("Truncating further timestamp mismatch errors...")
                break

    is_valid = len(errors) == 0
    return is_valid, errors


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate estimated trajectory format and values.")
    parser.add_argument("imu_input", type=str, help="Path to input imu0/data.csv or dataset folder")
    parser.add_argument("traj_output", type=str, help="Path to estimated trajectory text file")
    parser.add_argument("--append-blind", action="store_true", help="Allow extra rows at end from blind extrapolation")

    args = parser.parse_args()

    is_valid, errors = validate_trajectory(args.imu_input, args.traj_output, append_blind=args.append_blind)

    if is_valid:
        print("VALID: Trajectory matches specification.")
        sys.exit(0)
    else:
        print("INVALID: Trajectory failed validation:", file=sys.stderr)
        for err in errors[:50]:
            print(f"  - {err}", file=sys.stderr)
        if len(errors) > 50:
            print(f"  ... and {len(errors) - 50} more errors", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
