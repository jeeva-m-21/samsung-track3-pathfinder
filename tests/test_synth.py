import pytest
import numpy as np
import tempfile
from pathlib import Path

from tools.synth import (
    generate_trajectory,
    strapdown_integrate,
    write_euroc_imu_csv,
    write_tum_trajectory,
    TRAJECTORY_TYPES,
)


def test_trajectory_types():
    assert set(TRAJECTORY_TYPES) == {"static", "constant_yaw", "circle", "figure_eight", "sum_of_sines"}


@pytest.mark.parametrize("traj_type", TRAJECTORY_TYPES)
def test_generate_trajectory_shapes_and_finite(traj_type):
    rate = 200.0
    duration = 2.0
    traj = generate_trajectory(traj_type=traj_type, duration=duration, rate=rate, seed=42)

    n_samples = int(duration * rate)
    assert len(traj["t_ns"]) == n_samples
    assert traj["p"].shape == (n_samples, 3)
    assert traj["v"].shape == (n_samples, 3)
    assert traj["q"].shape == (n_samples, 4)  # [w, x, y, z]
    assert traj["omega"].shape == (n_samples, 3)
    assert traj["acc"].shape == (n_samples, 3)

    # Check finite
    for k in ["p", "v", "q", "omega", "acc"]:
        assert np.all(np.isfinite(traj[k])), f"Non-finite values in {k}"

    # Check quaternion unit norm
    q_norms = np.linalg.norm(traj["q"], axis=1)
    np.testing.assert_allclose(q_norms, 1.0, atol=1e-12)


def test_static_trajectory_physics():
    duration = 10.0
    rate = 200.0
    g_mag = 9.80665
    traj = generate_trajectory(traj_type="static", duration=duration, rate=rate, g_mag=g_mag)

    # Position, velocity, omega are all 0
    np.testing.assert_allclose(traj["p"], 0.0, atol=1e-12)
    np.testing.assert_allclose(traj["v"], 0.0, atol=1e-12)
    np.testing.assert_allclose(traj["omega"], 0.0, atol=1e-12)

    # Orientation is identity [w, x, y, z] = [1, 0, 0, 0]
    expected_q = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (len(traj["q"]), 1))
    np.testing.assert_allclose(traj["q"], expected_q, atol=1e-12)

    # Accelerometer measures upward specific force balancing gravity: [0, 0, g_mag]
    expected_acc = np.tile(np.array([0.0, 0.0, g_mag]), (len(traj["acc"]), 1))
    np.testing.assert_allclose(traj["acc"], expected_acc, atol=1e-12)


def test_constant_yaw_physics():
    duration = 4.0
    rate = 200.0
    yaw_rate = 0.5  # rad/s
    traj = generate_trajectory(traj_type="constant_yaw", duration=duration, rate=rate, yaw_rate=yaw_rate)

    # Position and velocity are 0
    np.testing.assert_allclose(traj["p"], 0.0, atol=1e-12)
    np.testing.assert_allclose(traj["v"], 0.0, atol=1e-12)

    # Omega body is [0, 0, yaw_rate]
    expected_omega = np.tile(np.array([0.0, 0.0, yaw_rate]), (len(traj["omega"]), 1))
    np.testing.assert_allclose(traj["omega"], expected_omega, atol=1e-12)

    # Accel body is [0, 0, g_mag]
    np.testing.assert_allclose(traj["acc"][:, :2], 0.0, atol=1e-12)
    np.testing.assert_allclose(traj["acc"][:, 2], 9.80665, atol=1e-12)


def test_strapdown_integration_drift_static():
    duration = 10.0
    rate = 200.0
    traj = generate_trajectory(traj_type="static", duration=duration, rate=rate)

    p_est, v_est, q_est = strapdown_integrate(traj["t_ns"], traj["acc"], traj["omega"], traj["q"][0])

    max_p_err = np.max(np.linalg.norm(p_est - traj["p"], axis=1))
    max_v_err = np.max(np.linalg.norm(v_est - traj["v"], axis=1))

    # Over 10s static with zero noise, drift should be strictly < 1e-6 m
    assert max_p_err < 1e-6, f"Static position drift {max_p_err} exceeds 1e-6 m"
    assert max_v_err < 1e-6, f"Static velocity drift {max_v_err} exceeds 1e-6 m/s"


def test_strapdown_integration_drift_constant_yaw():
    duration = 10.0
    rate = 200.0
    traj = generate_trajectory(traj_type="constant_yaw", duration=duration, rate=rate)

    p_est, v_est, q_est = strapdown_integrate(traj["t_ns"], traj["acc"], traj["omega"], traj["q"][0])

    max_p_err = np.max(np.linalg.norm(p_est - traj["p"], axis=1))
    assert max_p_err < 1e-6, f"Constant yaw position drift {max_p_err} exceeds 1e-6 m"


def test_strapdown_integration_dynamic():
    for traj_type in ["circle", "figure_eight", "sum_of_sines"]:
        traj = generate_trajectory(traj_type=traj_type, duration=3.0, rate=200.0)
        p_est, v_est, q_est = strapdown_integrate(
            traj["t_ns"], traj["acc"], traj["omega"], traj["q"][0], p0=traj["p"][0], v0=traj["v"][0]
        )
        max_p_err = np.max(np.linalg.norm(p_est - traj["p"], axis=1))
        # Within standard Euler integration truncation error (< 0.1 m over 3s)
        assert max_p_err < 0.1, f"{traj_type} drift {max_p_err} exceeds 0.1 m"


def test_cli_execution():
    import subprocess
    import sys

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)
        csv_file = tmp_path / "imu0" / "data.csv"
        gt_file = tmp_path / "gt.txt"

        cmd = [
            sys.executable,
            "-m",
            "tools.synth",
            "--type",
            "constant_yaw",
            "--duration",
            "1.0",
            "--rate",
            "100.0",
            "--output-imu",
            str(csv_file),
            "--output-gt",
            str(gt_file),
        ]
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        assert csv_file.exists()
        assert gt_file.exists()
        assert "Wrote 100 IMU samples" in res.stdout


def test_write_and_format():
    duration = 0.5
    rate = 100.0
    traj = generate_trajectory(traj_type="circle", duration=duration, rate=rate)

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)
        csv_file = tmp_path / "imu0" / "data.csv"
        gt_file = tmp_path / "groundtruth.txt"

        write_euroc_imu_csv(csv_file, traj["t_ns"], traj["omega"], traj["acc"])
        write_tum_trajectory(gt_file, traj["t_ns"], traj["p"], traj["q"])

        assert csv_file.exists()
        assert gt_file.exists()

        csv_lines = [line.strip() for line in csv_file.read_text().splitlines() if line.strip()]
        gt_lines = [line.strip() for line in gt_file.read_text().splitlines() if line.strip()]

        # Header + 50 rows = 51 lines
        assert len(csv_lines) == 51
        assert csv_lines[0].startswith("#")

        # First CSV data line check
        parts_csv = csv_lines[1].split(",")
        assert len(parts_csv) == 7
        assert parts_csv[0].isdigit()  # int64 nanoseconds

        # Ground truth check: 51 lines (header + 50 rows)
        assert len(gt_lines) == 51
        assert gt_lines[0].startswith("#")
        parts_gt = gt_lines[1].split()
        assert len(parts_gt) == 8
        # Timestamp must have decimal dot: <sec>.<nsec>
        assert "." in parts_gt[0]
