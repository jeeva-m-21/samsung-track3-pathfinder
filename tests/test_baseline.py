import subprocess
import sys
from pathlib import Path
import numpy as np
import pytest

from tools.synth import generate_trajectory, write_euroc_imu_csv
from tools.validate_trajectory import validate_trajectory
from src.ref.baseline_naive import run_naive_baseline
from src.ref.baseline_mahony import run_mahony_baseline


@pytest.fixture
def synthetic_imu(tmp_path):
    duration = 10.0
    rate = 100.0
    traj = generate_trajectory("circle", duration=duration, rate=rate)

    imu_file = tmp_path / "imu0" / "data.csv"
    write_euroc_imu_csv(imu_file, traj["t_ns"], traj["omega"], traj["acc"])

    return {
        "imu_file": imu_file,
        "traj": traj,
        "duration": duration,
        "rate": rate,
    }


def test_naive_baseline_valid_output(synthetic_imu, tmp_path):
    out_file = tmp_path / "naive_traj.txt"
    run_naive_baseline(synthetic_imu["imu_file"], out_file)

    assert out_file.exists()
    is_valid, errors = validate_trajectory(synthetic_imu["imu_file"], out_file)
    assert is_valid, f"Naive trajectory validation failed: {errors}"


def test_mahony_baseline_valid_output(synthetic_imu, tmp_path):
    out_file = tmp_path / "mahony_traj.txt"
    run_mahony_baseline(synthetic_imu["imu_file"], out_file)

    assert out_file.exists()
    is_valid, errors = validate_trajectory(synthetic_imu["imu_file"], out_file)
    assert is_valid, f"Mahony trajectory validation failed: {errors}"


def test_naive_baseline_quadratic_drift(tmp_path):
    # Static IMU with an injected acceleration bias (0.05 m/s^2 along x) starting after 1.0s init
    duration = 10.0
    rate = 100.0
    traj = generate_trajectory("static", duration=duration, rate=rate)

    t_s_raw = (traj["t_ns"] - traj["t_ns"][0]) * 1e-9
    biased_acc = traj["acc"].copy()
    bias_val = 0.05
    bias_mask = t_s_raw > 1.0
    biased_acc[bias_mask, 0] += bias_val  # Add x bias after initialisation

    imu_file = tmp_path / "biased_static_imu.csv"
    write_euroc_imu_csv(imu_file, traj["t_ns"], traj["omega"], biased_acc)

    out_file = tmp_path / "naive_biased.txt"
    run_naive_baseline(imu_file, out_file)

    # Read estimated positions
    p_est = []
    t_s = []
    with open(out_file, "r") as f:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.split()
            t_s.append(float(parts[0]))
            p_est.append([float(parts[1]), float(parts[2]), float(parts[3])])

    p_arr = np.array(p_est)
    t_arr = np.array(t_s)
    t_rel = t_arr - t_arr[0]

    # For constant acceleration bias a_bias starting at t=1.0s,
    # double integration gives p(t) = 0.5 * a_bias * (t - 1.0)^2 for t > 1.0
    dt_active = np.maximum(0.0, t_rel - 1.0)
    expected_drift = 0.5 * bias_val * (dt_active**2)
    actual_drift_x = p_arr[:, 0]

    # Check quadratic growth: actual drift should match 0.5 * a_bias * dt_active^2
    np.testing.assert_allclose(actual_drift_x, expected_drift, rtol=1e-2, atol=1e-3)

    # Drift ratio between 10s and 5.5s: dt_active(10) = 9.0s, dt_active(5.5) = 4.5s
    # Expected ratio = (9.0 / 4.5)^2 = 4.0
    idx_55s = np.argmin(np.abs(t_rel - 5.5))
    idx_10s = len(t_rel) - 1
    drift_ratio = actual_drift_x[idx_10s] / actual_drift_x[idx_55s]
    assert drift_ratio == pytest.approx(4.0, rel=0.05)


def test_cli_execution(synthetic_imu, tmp_path):
    out_naive = tmp_path / "cli_naive.txt"
    out_mahony = tmp_path / "cli_mahony.txt"

    # Naive CLI
    cmd_naive = [sys.executable, "-m", "src.ref.baseline_naive", str(synthetic_imu["imu_file"]), str(out_naive)]
    subprocess.run(cmd_naive, check=True)
    assert out_naive.exists()

    # Mahony CLI
    cmd_mahony = [sys.executable, "-m", "src.ref.baseline_mahony", str(synthetic_imu["imu_file"]), str(out_mahony)]
    subprocess.run(cmd_mahony, check=True)
    assert out_mahony.exists()
