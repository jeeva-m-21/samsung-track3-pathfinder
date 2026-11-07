#!/usr/bin/env python3
"""Calculate duration of IMU dataset in seconds.

Parses first and last valid integer nanosecond timestamps and prints:
(t_end - t_start) * 1e-9
"""

import sys
from pathlib import Path

# Ensure repo root is on sys.path
repo_root = Path(__file__).resolve().parent.parent
if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))

from tools.validate_trajectory import resolve_imu_csv


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: data_duration.py <imu_input_csv_or_dir>", file=sys.stderr)
        sys.exit(1)

    input_path = sys.argv[1]
    csv_path = resolve_imu_csv(input_path)

    t_first = None
    t_last = None

    with open(csv_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if not parts or not parts[0].strip():
                continue
            try:
                t_val = int(parts[0].strip())
            except ValueError:
                continue

            if t_first is None:
                t_first = t_val
            t_last = t_val

    if t_first is None or t_last is None or t_last <= t_first:
        print("0.0")
        sys.exit(0)

    duration_s = (t_last - t_first) * 1e-9
    print(f"{duration_s:.9f}")
    sys.exit(0)


if __name__ == "__main__":
    main()
