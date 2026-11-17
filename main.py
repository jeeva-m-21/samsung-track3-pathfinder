#!/usr/bin/env python3
"""Track 03 Path Finder – standalone Python entrypoint.
Reads a degraded IMU CSV (official degrade_imu.py format or EuRoC format),
runs the full 15-state ESKF, writes estimated_trajectory.txt.
No C++ compilation required.
"""
import os, sys, zipfile, tempfile
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


def main():
    args = sys.argv[1:]
    if not args:
        print(f"Usage: {sys.argv[0]} <input_csv_zip_or_dir> [output.txt]", file=sys.stderr)
        sys.exit(1)
    input_path = args[0]
    output_path = args[1] if len(args) > 1 else "estimated_trajectory.txt"

    csv_path = resolve_imu_csv(input_path)
    from src.ref.pipeline import run_eskf_full
    run_eskf_full(csv_path, output_path)


if __name__ == "__main__":
    main()
