#!/usr/bin/env python3
"""
Track 3 (Path Finder) Standalone Entrypoint (Slide 14 & Slide 16).
Accepts raw IMU telemetry as:
  1. A ZIP archive containing imu0/data.csv (e.g. input.zip)
  2. An extracted directory (e.g. path/to/dataset)
  3. A direct CSV file (e.g. imu0/data.csv)

Generates estimated_trajectory.txt in exact TUM pose format (timestamp tx ty tz qx qy qz qw).
"""

import os
import sys
import subprocess
import tempfile
import zipfile
from pathlib import Path


def resolve_imu_input(raw_input: str) -> str:
    p = Path(raw_input)
    if not p.exists():
        raise FileNotFoundError(f"Input path does not exist: {raw_input}")

    # Case 1: Directory input
    if p.is_dir():
        for cand in [p / "imu0" / "data.csv", p / "mav0" / "imu0" / "data.csv", p / "data.csv"]:
            if cand.is_file():
                return str(cand)
        raise FileNotFoundError(f"Could not find imu0/data.csv in directory: {raw_input}")

    # Case 2: Zip archive
    if p.is_file() and (raw_input.lower().endswith(".zip") or is_zip_file(p)):
        tmp_dir = tempfile.mkdtemp(prefix="imu_eval_")
        with zipfile.ZipFile(p, "r") as z:
            names = z.namelist()
            chosen = None
            for cand in ["imu0/data.csv", "mav0/imu0/data.csv", "data.csv"]:
                matches = [n for n in names if n.endswith(cand)]
                if matches:
                    chosen = matches[0]
                    break
            if not chosen:
                matches = [n for n in names if n.endswith(".csv")]
                if matches:
                    chosen = matches[0]
            if not chosen:
                raise FileNotFoundError(f"No IMU CSV found inside zip archive: {raw_input}")

            out_csv = os.path.join(tmp_dir, "data.csv")
            with z.open(chosen) as src_f, open(out_csv, "wb") as dst_f:
                dst_f.write(src_f.read())
            return out_csv

    # Case 3: Regular CSV file
    return str(p)


def is_zip_file(p: Path) -> bool:
    try:
        with open(p, "rb") as f:
            sig = f.read(4)
            return len(sig) == 4 and sig[:2] == b"PK"
    except Exception:
        return False


def find_or_build_cpp_binary(src_dir: Path) -> str | None:
    # Look for compiled binary
    candidates = [
        src_dir / "build" / "main",
        src_dir / "main",
        src_dir.parent / "main",
        Path.cwd() / "main",
    ]
    for cand in candidates:
        if cand.is_file() and os.access(cand, os.X_OK):
            return str(cand)

    # Attempt compilation if Makefile exists
    makefile = src_dir / "Makefile"
    if makefile.is_file():
        try:
            subprocess.run(["make", "-C", str(src_dir), "all"], check=True, capture_output=True)
            for cand in candidates:
                if cand.is_file() and os.access(cand, os.X_OK):
                    return str(cand)
        except Exception:
            pass

    return None


def main():
    args = sys.argv[1:]
    input_path = None
    output_path = "estimated_trajectory.txt"
    append_blind = False

    i = 0
    while i < len(args):
        arg = args[i]
        if arg == "--append-blind":
            append_blind = True
        elif not arg.startswith("-"):
            if input_path is None:
                input_path = arg
            elif output_path == "estimated_trajectory.txt":
                output_path = arg
        i += 1

    # Fallback to local dataset candidates if no argument provided
    if input_path is None:
        for cand in ["imu0/data.csv", "mav0/imu0/data.csv", "data.csv"]:
            if os.path.isfile(cand):
                input_path = cand
                break

    if input_path is None:
        print(f"Usage: {sys.argv[0]} <input_zip_dir_or_csv> [output_trajectory.txt] [--append-blind]", file=sys.stderr)
        sys.exit(1)

    src_dir = Path(__file__).resolve().parent

    # Try C++ binary for maximal performance (2.9 MB RAM, 1.0 ms/s latency)
    cpp_binary = find_or_build_cpp_binary(src_dir)
    if cpp_binary:
        cmd = [cpp_binary, input_path, output_path]
        if append_blind:
            cmd.append("--append-blind")
        res = subprocess.run(cmd)
        sys.exit(res.returncode)

    # Python reference fallback
    resolved_csv = resolve_imu_input(input_path)
    try:
        from src.ref.pipeline import run_pipeline
        run_pipeline(resolved_csv, output_path, ablation="eskf_full", append_blind=append_blind)
    except ImportError:
        # Fallback if invoked from inside src/
        sys.path.insert(0, str(src_dir.parent))
        from src.ref.pipeline import run_pipeline
        run_pipeline(resolved_csv, output_path, ablation="eskf_full", append_blind=append_blind)


if __name__ == "__main__":
    main()
