import json
from pathlib import Path
import numpy as np
import pytest

from tools.synth import generate_trajectory, write_tum_trajectory
from tools.metrics import (
    compute_trajectory_metrics,
    compute_objective_J,
    load_tum_trajectory,
    align_start_anchored,
    geodesic_distance_deg,
)


@pytest.fixture
def synthetic_pair(tmp_path):
    duration = 30.0
    rate = 100.0
    traj_ref = generate_trajectory("circle", duration=duration, rate=rate)

    ref_file = tmp_path / "gt.txt"
    est_file = tmp_path / "est.txt"
    sidecar_file = tmp_path / "sidecar.json"

    # Write reference trajectory
    write_tum_trajectory(ref_file, traj_ref["t_ns"], traj_ref["p"], traj_ref["q"])

    # Create estimated trajectory with small known linear drift
    t_s = (traj_ref["t_ns"] - traj_ref["t_ns"][0]) * 1e-9
    drift_p = traj_ref["p"] + 0.05 * t_s[:, np.newaxis]
    write_tum_trajectory(est_file, traj_ref["t_ns"], drift_p, traj_ref["q"])

    # Create sidecar metadata
    sidecar_data = {
        "seed": 42,
        "duration_s": duration,
        "t0_ns": int(traj_ref["t_ns"][0]),
        "tend_ns": int(traj_ref["t_ns"][-1]),
        "failure_time_s": 25.0,
        "failure_t_ns": int(traj_ref["t_ns"][0] + 25 * 1e9),
        "dropout_windows": [
            {
                "start_s": 10.0,
                "duration_s": 1.5,
                "end_s": 11.5,
                "encoding": "nan",
                "start_ns": int(traj_ref["t_ns"][0] + 10 * 1e9),
                "end_ns": int(traj_ref["t_ns"][0] + 11.5 * 1e9),
            }
        ],
        "checkpoints_s": [5.0, 10.0, 15.0, 20.0],
    }
    sidecar_file.write_text(json.dumps(sidecar_data, indent=2))

    return {
        "ref_file": ref_file,
        "est_file": est_file,
        "sidecar_file": sidecar_file,
        "traj_ref": traj_ref,
    }


def test_load_tum_trajectory(synthetic_pair):
    t_ns, p, q = load_tum_trajectory(synthetic_pair["ref_file"])
    assert len(t_ns) == len(synthetic_pair["traj_ref"]["t_ns"])
    assert p.shape == (len(t_ns), 3)
    assert q.shape == (len(t_ns), 4)
    # Check unit quaternion
    q_norm = np.linalg.norm(q, axis=1)
    np.testing.assert_allclose(q_norm, 1.0, atol=1e-6)


def test_geodesic_distance_deg():
    q1 = np.array([1.0, 0.0, 0.0, 0.0])
    # 90 deg rotation around z: [cos(45°), 0, 0, sin(45°)]
    q2 = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])
    deg = geodesic_distance_deg(q1[np.newaxis, :], q2[np.newaxis, :])
    assert deg[0] == pytest.approx(90.0, abs=1e-4)


def test_start_anchored_alignment():
    # If both start at (0, 0, 0), start anchored error at t=0 is 0
    p_ref = np.array([[10.0, 5.0, 0.0], [11.0, 5.0, 0.0]])
    q_ref = np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]])
    p_est = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    q_est = np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]])

    p_aligned, _ = align_start_anchored(p_est, q_est, p_ref[0], q_ref[0])
    np.testing.assert_allclose(p_aligned[0], p_ref[0], atol=1e-12)
    np.testing.assert_allclose(p_aligned[1], p_ref[1], atol=1e-12)


def test_compute_trajectory_metrics(synthetic_pair):
    metrics = compute_trajectory_metrics(
        synthetic_pair["ref_file"],
        synthetic_pair["est_file"],
        sidecar_path=synthetic_pair["sidecar_file"],
    )

    # Verify all expected keys exist
    expected_keys = [
        "ate_rmse_aligned",
        "ate_rmse_unaligned",
        "rpe_trans_1s",
        "start_anchored_rmse",
        "checkpoint_err",
        "blind5_rmse",
        "blind5_final_err",
        "dropout_rmse",
        "rot_err_deg",
    ]
    for key in expected_keys:
        assert key in metrics, f"Missing metric {key}"
        assert np.isfinite(metrics[key]), f"Metric {key} is not finite: {metrics[key]}"
        assert metrics[key] >= 0.0, f"Metric {key} should be non-negative: {metrics[key]}"


def test_scalar_objective_J():
    metrics = {
        "checkpoint_err": 0.5,
        "blind5_rmse": 1.0,
        "dropout_rmse": 0.3,
        "ate_rmse_aligned": 0.4,
        "rpe_trans_1s": 0.1,
    }
    naive = {
        "checkpoint_err": 1.0,
        "blind5_rmse": 2.0,
        "dropout_rmse": 0.6,
        "ate_rmse_aligned": 0.8,
        "rpe_trans_1s": 0.2,
    }
    # Each ratio is exactly 0.5
    J = compute_objective_J(metrics, naive)
    assert J == pytest.approx(0.5, abs=1e-5)
