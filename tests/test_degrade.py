import json
import tempfile
from pathlib import Path
import numpy as np
import pytest

from tools.synth import generate_trajectory, write_euroc_imu_csv, write_tum_trajectory
from tools.degrade import (
    apply_degradations,
    load_degrade_config,
    read_euroc_imu_csv,
    write_degraded_dataset,
)


@pytest.fixture
def clean_synthetic_data(tmp_path):
    duration = 60.0
    rate = 100.0  # 100 Hz for faster testing
    traj = generate_trajectory("circle", duration=duration, rate=rate)

    imu_path = tmp_path / "clean_imu.csv"
    gt_path = tmp_path / "clean_gt.txt"
    write_euroc_imu_csv(imu_path, traj["t_ns"], traj["omega"], traj["acc"])
    write_tum_trajectory(gt_path, traj["t_ns"], traj["p"], traj["q"])

    return {
        "imu_path": imu_path,
        "gt_path": gt_path,
        "traj": traj,
        "duration": duration,
        "rate": rate,
    }


def test_load_degrade_config():
    config = load_degrade_config("configs/degrade_default.json")
    assert "noise_growth_rate" in config
    assert "bias_drift_walk" in config
    assert "dropouts" in config
    assert len(config["dropouts"]) >= 4


def test_determinism_by_seed(clean_synthetic_data):
    config = load_degrade_config("configs/degrade_default.json")
    t_ns, omega, acc = read_euroc_imu_csv(clean_synthetic_data["imu_path"])

    res1 = apply_degradations(t_ns, omega, acc, config, seed=123)
    res2 = apply_degradations(t_ns, omega, acc, config, seed=123)
    res3 = apply_degradations(t_ns, omega, acc, config, seed=456)

    # Identical seeds must produce identical arrays
    np.testing.assert_array_equal(res1["t_ns"], res2["t_ns"])
    np.testing.assert_array_equal(res1["omega"], res2["omega"])
    np.testing.assert_array_equal(res1["acc"], res2["acc"])

    # Different seeds must produce different noise / bias
    assert not np.array_equal(res1["acc"], res3["acc"])


def test_dropout_encodings(clean_synthetic_data):
    t_ns, omega, acc = read_euroc_imu_csv(clean_synthetic_data["imu_path"])
    config = {
        "noise_growth_rate": 0.0,
        "bias_drift_walk": 0.0,
        "gravity_rotation": False,
        "failure_at_end_s": 0.0,
        "dropouts": [
            {"start_s": 2.0, "duration_s": 1.0, "encoding": "missing"},
            {"start_s": 5.0, "duration_s": 1.0, "encoding": "nan"},
            {"start_s": 8.0, "duration_s": 1.0, "encoding": "zeros"},
            {"start_s": 11.0, "duration_s": 1.0, "encoding": "repeated"},
        ],
    }

    res = apply_degradations(t_ns, omega, acc, config, seed=42)
    deg_t_ns = res["t_ns"]
    deg_omega = res["omega"]
    deg_acc = res["acc"]

    t0 = t_ns[0]
    t_s = (deg_t_ns - t0) * 1e-9

    # 1. Missing rows check: no timestamps between 2.0 and 3.0s
    missing_mask = (t_s >= 2.0) & (t_s < 3.0)
    assert np.sum(missing_mask) == 0

    # 2. NaN check: between 5.0 and 6.0s, all values are NaN
    nan_mask = (t_s >= 5.0) & (t_s < 6.0)
    assert np.sum(nan_mask) > 0
    assert np.all(np.isnan(deg_omega[nan_mask]))
    assert np.all(np.isnan(deg_acc[nan_mask]))

    # 3. Zeros check: between 8.0 and 9.0s, all values are 0.0
    zeros_mask = (t_s >= 8.0) & (t_s < 9.0)
    assert np.sum(zeros_mask) > 0
    assert np.all(deg_omega[zeros_mask] == 0.0)
    assert np.all(deg_acc[zeros_mask] == 0.0)

    # 4. Repeated check: between 11.0 and 12.0s, all rows match the row right before 11.0s
    rep_mask = (t_s >= 11.0) & (t_s < 12.0)
    assert np.sum(rep_mask) > 0
    # Values inside repeated window must have zero variance
    assert np.all(np.ptp(deg_omega[rep_mask], axis=0) == 0.0)
    assert np.all(np.ptp(deg_acc[rep_mask], axis=0) == 0.0)


def test_failure_at_end(clean_synthetic_data):
    t_ns, omega, acc = read_euroc_imu_csv(clean_synthetic_data["imu_path"])
    fail_s = 5.0
    config = {
        "noise_growth_rate": 0.0,
        "bias_drift_walk": 0.0,
        "gravity_rotation": False,
        "failure_at_end_s": fail_s,
        "dropouts": [],
    }

    res = apply_degradations(t_ns, omega, acc, config, seed=42)
    deg_t_ns = res["t_ns"]

    total_dur = (t_ns[-1] - t_ns[0]) * 1e-9
    deg_dur = (deg_t_ns[-1] - deg_t_ns[0]) * 1e-9

    # Degraded IMU stream stops at failure point (total_dur - 5.0 s)
    assert abs(deg_dur - (total_dur - fail_s)) < 0.05
    assert res["sidecar"]["failure_time_s"] == pytest.approx(total_dur - fail_s, rel=1e-3)


def test_sidecar_generation(tmp_path, clean_synthetic_data):
    out_dir = tmp_path / "degraded_run"
    config_path = "configs/degrade_default.json"

    write_degraded_dataset(
        imu_in=clean_synthetic_data["imu_path"],
        gt_in=clean_synthetic_data["gt_path"],
        out_dir=out_dir,
        config_path=config_path,
        seed=42,
    )

    out_imu = out_dir / "imu0" / "data.csv"
    out_gt = out_dir / "groundtruth.txt"
    out_sidecar = out_dir / "degrade_info.json"

    assert out_imu.exists()
    assert out_gt.exists()
    assert out_sidecar.exists()

    sidecar = json.loads(out_sidecar.read_text())
    assert "dropout_windows" in sidecar
    assert "failure_time_s" in sidecar
    assert "checkpoints_s" in sidecar
    assert sidecar["seed"] == 42
