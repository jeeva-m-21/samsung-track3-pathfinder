#!/usr/bin/env python3
"""Track 03 Path Finder – standalone Python entrypoint.
Reads a degraded IMU CSV (official degrade_imu.py format or EuRoC format),
runs the full 15-state ESKF, writes estimated_trajectory.txt.
No C++ compilation required.
"""
import argparse
import os
import sys
import tempfile
import zipfile
from pathlib import Path

# Ensure repo root is on sys.path so src.ref.* imports work
_REPO_ROOT = Path(__file__).resolve().parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


def _is_zip(p: Path) -> bool:
    try:
        with open(p, "rb") as f:
            return f.read(2) == b"PK"
    except Exception:
        return False


def _extract_zip_csv(p: Path) -> str:
    tmp = tempfile.mkdtemp(prefix="imu_")
    with zipfile.ZipFile(p) as z:
        names = z.namelist()
        candidates = ["imu0/data.csv", "mav0/imu0/data.csv", "data.csv"]
        chosen = None
        for cand in candidates:
            matches = [n for n in names if n.endswith(cand)]
            if matches:
                chosen = matches[0]
                break
        if chosen is None:
            matches = [n for n in names if n.endswith(".csv")]
            if matches:
                chosen = matches[0]
        if chosen is None:
            raise FileNotFoundError(f"No CSV found inside zip: {p}")
        out = os.path.join(tmp, "data.csv")
        with z.open(chosen) as src, open(out, "wb") as dst:
            dst.write(src.read())
    return out


def resolve_imu_csv(raw: str) -> str:
    p = Path(raw)
    if not p.exists():
        raise FileNotFoundError(f"Input not found: {raw}")
    if p.is_dir():
        for cand in [p / "imu0" / "data.csv", p / "mav0" / "imu0" / "data.csv", p / "data.csv"]:
            if cand.is_file():
                return str(cand)
        raise FileNotFoundError(f"No IMU CSV in directory: {raw}")
    if p.is_file() and (raw.lower().endswith(".zip") or _is_zip(p)):
        return _extract_zip_csv(p)
    return str(p)


class _CustomParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write(f"error: {message}\n")
        sys.exit(1)


def main():
    parser = _CustomParser(description="Track 03 Path Finder Estimator")
    parser.add_argument("input", nargs="?", default=None, help="Input IMU CSV, ZIP, or directory")
    parser.add_argument("output", nargs="?", default="estimated_trajectory.txt", help="Output trajectory path")
    parser.add_argument("--append-blind", action="store_true", help="Extrapolate blind window after stream")
    parser.add_argument(
        "--ablation",
        type=str,
        default="eskf_full",
        choices=["naive", "mahony", "eskf_basic", "eskf_full"],
        help="Ablation mode",
    )
    parser.add_argument("--params", type=str, default=None, help="Custom params JSON")

    args = parser.parse_args()
    if not args.input:
        parser.print_usage(sys.stderr)
        sys.exit(1)

    try:
        csv_path = resolve_imu_csv(args.input)
        from src.ref.pipeline import run_pipeline
        run_pipeline(
            csv_path,
            args.output,
            ablation=args.ablation,
            append_blind=args.append_blind,
            params=args.params,
        )
    except Exception as e:
        sys.stderr.write(f"Error: {e}\n")
        sys.exit(1)


if __name__ == "__main__":
    main()
