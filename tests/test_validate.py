import subprocess
import sys
from pathlib import Path
import pytest
import numpy as np

from tools.synth import generate_trajectory, write_euroc_imu_csv, write_tum_trajectory
from tools.validate_trajectory import validate_trajectory


@pytest.fixture
def synthetic_sample(tmp_path):
    traj = generate_trajectory("circle", duration=1.0, rate=100.0)
    imu_dir = tmp_path / "seq" / "imu0"
    imu_file = imu_dir / "data.csv"
    gt_file = tmp_path / "estimated_trajectory.txt"

    write_euroc_imu_csv(imu_file, traj["t_ns"], traj["omega"], traj["acc"])
    write_tum_trajectory(gt_file, traj["t_ns"], traj["p"], traj["q"])

    return {
        "seq_dir": tmp_path / "seq",
        "imu_file": imu_file,
        "gt_file": gt_file,
        "traj": traj,
    }


def test_valid_trajectory(synthetic_sample):
    # Test with direct file path
    is_valid, errors = validate_trajectory(synthetic_sample["imu_file"], synthetic_sample["gt_file"])
    assert is_valid, f"Validation failed with errors: {errors}"
    assert len(errors) == 0

    # Test with directory path
    is_valid_dir, errors_dir = validate_trajectory(synthetic_sample["seq_dir"], synthetic_sample["gt_file"])
    assert is_valid_dir, f"Directory validation failed with errors: {errors_dir}"


def test_row_count_mismatch(synthetic_sample, tmp_path):
    bad_gt = tmp_path / "bad_gt_count.txt"
    lines = synthetic_sample["gt_file"].read_text().splitlines()
    # Remove last 5 rows
    bad_gt.write_text("\n".join(lines[:-5]) + "\n")

    is_valid, errors = validate_trajectory(synthetic_sample["imu_file"], bad_gt)
    assert not is_valid
    assert any("row count mismatch" in e.lower() for e in errors)


def test_column_count_mismatch(synthetic_sample, tmp_path):
    bad_gt = tmp_path / "bad_gt_cols.txt"
    lines = synthetic_sample["gt_file"].read_text().splitlines()
    # Drop last column from data rows
    bad_lines = []
    for line in lines:
        if line.startswith("#"):
            bad_lines.append(line)
        else:
            parts = line.split()
            bad_lines.append(" ".join(parts[:-1]))
    bad_gt.write_text("\n".join(bad_lines) + "\n")

    is_valid, errors = validate_trajectory(synthetic_sample["imu_file"], bad_gt)
    assert not is_valid
    assert any("column count" in e.lower() for e in errors)


def test_nan_or_inf(synthetic_sample, tmp_path):
    bad_gt = tmp_path / "bad_gt_nan.txt"
    lines = synthetic_sample["gt_file"].read_text().splitlines()
    bad_lines = []
    for idx, line in enumerate(lines):
        if idx == 10:
            parts = line.split()
            parts[1] = "nan"
            bad_lines.append(" ".join(parts))
        elif idx == 15:
            parts = line.split()
            parts[2] = "inf"
            bad_lines.append(" ".join(parts))
        else:
            bad_lines.append(line)
    bad_gt.write_text("\n".join(bad_lines) + "\n")

    is_valid, errors = validate_trajectory(synthetic_sample["imu_file"], bad_gt)
    assert not is_valid
    assert any("nan or inf" in e.lower() for e in errors)


def test_unnormalized_quaternion(synthetic_sample, tmp_path):
    bad_gt = tmp_path / "bad_gt_quat.txt"
    lines = synthetic_sample["gt_file"].read_text().splitlines()
    bad_lines = []
    for idx, line in enumerate(lines):
        if idx == 5:
            parts = line.split()
            # Double quaternion components
            parts[4] = f"{float(parts[4])*2.0:.9f}"
            bad_lines.append(" ".join(parts))
        else:
            bad_lines.append(line)
    bad_gt.write_text("\n".join(bad_lines) + "\n")

    is_valid, errors = validate_trajectory(synthetic_sample["imu_file"], bad_gt)
    assert not is_valid
    assert any("quaternion norm" in e.lower() for e in errors)


def test_timestamp_mismatch(synthetic_sample, tmp_path):
    bad_gt = tmp_path / "bad_gt_ts.txt"
    lines = synthetic_sample["gt_file"].read_text().splitlines()
    bad_lines = []
    for idx, line in enumerate(lines):
        if idx == 5:
            parts = line.split()
            parts[0] = "12345.678"  # Wrong timestamp format and value
            bad_lines.append(" ".join(parts))
        else:
            bad_lines.append(line)
    bad_gt.write_text("\n".join(bad_lines) + "\n")

    is_valid, errors = validate_trajectory(synthetic_sample["imu_file"], bad_gt)
    assert not is_valid
    assert any("timestamp" in e.lower() for e in errors)


def test_cli_execution(synthetic_sample):
    cmd_good = [
        sys.executable,
        "-m",
        "tools.validate_trajectory",
        str(synthetic_sample["imu_file"]),
        str(synthetic_sample["gt_file"]),
    ]
    res_good = subprocess.run(cmd_good, capture_output=True, text=True)
    assert res_good.returncode == 0
    assert "VALID" in res_good.stdout
