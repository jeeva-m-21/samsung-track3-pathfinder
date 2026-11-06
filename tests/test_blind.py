"""Tests for Sensor Failure Detector and Blind-5s Predictor (SPEC §12.7, §12.9, WP-27).

Verifies:
1. Pure extrapolation functions:
   - Decaying-velocity position prediction: p(tau) = p_f + v_f * shrink_v * tau_v * (1 - exp(-tau / tau_v))
   - Decaying-turn-rate orientation prediction: q(tau) = normalize(q_f (x) exp(omega_f * tau_omega * (1 - exp(-tau / tau_omega))))
   - Stationary threshold: |v_f| < 0.05 m/s holds stationary: p(tau) = p_f
   - Clamping: poses beyond tau = 5.0 s clamp to the pose at tau = 5.0 s
   - Pose continuity at tau = 0
2. Failure declaration:
   - Failure declared after continuous invalid samples >= T_fail (1.0 s)
   - Reset of invalid duration when a valid sample intervenes
   - Failure declaration at stream termination
3. State rollback:
   - Retrieval of last reliable state prior to failure onset from fixed ring buffer
4. Acceptance on synthetic trajectories:
   - Circle and figure-eight trajectories with complete IMU failure at t = 5.0 s
   - Blind 5s RMSE is strictly less than naive open-loop integration RMSE
5. Parameter loading from configs/params.json and memory invariants
"""

import json
import math
from pathlib import Path
import numpy as np
import pytest

from tools.synth import generate_trajectory, quat_mult, quat_to_rot_matrix
from src.ref.blind import (
    BlindPredictor,
    ReliableRingBuffer,
    load_params,
    predict_blind_orientation,
    predict_blind_position,
    predict_blind_velocity,
    predict_blind_pose,
    rollback_to_reliable,
)
from src.ref.eskf import NominalState


# ---------------------------------------------------------------------------
# 1. Parameter Loading and Configuration Tests
# ---------------------------------------------------------------------------


def test_load_params_defaults():
    """Verify parameters are loaded from configs/params.json correctly."""
    params = load_params()
    assert "T_fail" in params
    assert params["T_fail"] == 1.0
    assert params["tau_v"] == 3.0
    assert params["tau_omega"] == 1.5
    assert params["shrink_v"] == 0.9
    assert params["stationary_v_thresh"] == 0.05
    assert params["ring_buffer_size"] == 64


# ---------------------------------------------------------------------------
# 2. Position Extrapolation Pure Function Tests
# ---------------------------------------------------------------------------


def test_predict_blind_position_tau_zero():
    """At tau = 0, extrapolated position must equal p_f exactly."""
    p_f = np.array([1.2, -3.4, 5.6], dtype=np.float64)
    v_f = np.array([0.5, 1.0, -0.2], dtype=np.float64)

    p_0 = predict_blind_position(p_f, v_f, tau=0.0)
    np.testing.assert_allclose(p_0, p_f, atol=1e-12)

    # Negative tau should also clamp to tau = 0
    p_neg = predict_blind_position(p_f, v_f, tau=-1.5)
    np.testing.assert_allclose(p_neg, p_f, atol=1e-12)


def test_predict_blind_position_math():
    """Verify decaying-velocity position equation against exact analytic calculation."""
    p_f = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    v_f = np.array([2.0, -1.0, 0.5], dtype=np.float64)
    tau_v = 3.0
    shrink_v = 0.9
    stationary_thresh = 0.05

    for tau in [0.5, 1.0, 2.5, 4.0, 5.0]:
        expected = p_f + v_f * shrink_v * tau_v * (1.0 - math.exp(-tau / tau_v))
        p_tau = predict_blind_position(
            p_f,
            v_f,
            tau=tau,
            tau_v=tau_v,
            shrink_v=shrink_v,
            stationary_v_thresh=stationary_thresh,
        )
        np.testing.assert_allclose(p_tau, expected, atol=1e-12)


def test_predict_blind_position_clamping_beyond_5s():
    """Poses beyond tau = 5.0 s must clamp to pose at tau = 5.0 s."""
    p_f = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    v_f = np.array([3.0, 0.0, 0.0], dtype=np.float64)

    p_5 = predict_blind_position(p_f, v_f, tau=5.0)
    p_6 = predict_blind_position(p_f, v_f, tau=6.0)
    p_10 = predict_blind_position(p_f, v_f, tau=10.0)
    p_100 = predict_blind_position(p_f, v_f, tau=100.0)

    np.testing.assert_allclose(p_6, p_5, atol=1e-12)
    np.testing.assert_allclose(p_10, p_5, atol=1e-12)
    np.testing.assert_allclose(p_100, p_5, atol=1e-12)


def test_predict_blind_position_stationary_threshold():
    """If |v_f| < stationary_v_thresh (0.05 m/s), position remains stationary p(tau) = p_f."""
    p_f = np.array([5.0, 6.0, 7.0], dtype=np.float64)

    # Sub-threshold velocity norm (|v| = 0.04 < 0.05)
    v_sub = np.array([0.02, 0.02, 0.02], dtype=np.float64)  # norm ~= 0.0346
    assert np.linalg.norm(v_sub) < 0.05

    for tau in [0.1, 1.0, 3.0, 5.0, 10.0]:
        p_tau = predict_blind_position(p_f, v_sub, tau=tau, stationary_v_thresh=0.05)
        np.testing.assert_allclose(p_tau, p_f, atol=1e-12)

    # Super-threshold velocity norm (|v| = 0.06 > 0.05)
    v_super = np.array([0.06, 0.0, 0.0], dtype=np.float64)
    p_tau_super = predict_blind_position(p_f, v_super, tau=2.0, stationary_v_thresh=0.05)
    assert not np.allclose(p_tau_super, p_f)


def test_predict_blind_velocity():
    """Verify velocity extrapolation and stationary handling."""
    v_f = np.array([2.0, -1.0, 0.5], dtype=np.float64)
    tau_v = 3.0
    shrink_v = 0.9

    v_0 = predict_blind_velocity(v_f, tau=0.0, tau_v=tau_v, shrink_v=shrink_v)
    np.testing.assert_allclose(v_0, v_f * shrink_v, atol=1e-12)

    v_2 = predict_blind_velocity(v_f, tau=2.0, tau_v=tau_v, shrink_v=shrink_v)
    expected_v_2 = v_f * shrink_v * math.exp(-2.0 / tau_v)
    np.testing.assert_allclose(v_2, expected_v_2, atol=1e-12)

    # Clamping beyond 5s freezes position, so velocity becomes 0
    v_6 = predict_blind_velocity(v_f, tau=6.0, tau_v=tau_v, shrink_v=shrink_v)
    np.testing.assert_allclose(v_6, np.zeros(3), atol=1e-12)

    # Sub-threshold velocity
    v_sub = np.array([0.01, 0.01, 0.0], dtype=np.float64)
    v_sub_out = predict_blind_velocity(v_sub, tau=1.0)
    np.testing.assert_allclose(v_sub_out, np.zeros(3), atol=1e-12)


# ---------------------------------------------------------------------------
# 3. Orientation Extrapolation Pure Function Tests
# ---------------------------------------------------------------------------


def test_predict_blind_orientation_tau_zero():
    """At tau = 0, extrapolated orientation must equal q_f."""
    q_f = np.array([0.70710678, 0.0, 0.70710678, 0.0], dtype=np.float64)
    omega_f = np.array([0.1, -0.2, 0.5], dtype=np.float64)

    q_0 = predict_blind_orientation(q_f, omega_f, tau=0.0)
    np.testing.assert_allclose(q_0, q_f, atol=1e-12)

    q_neg = predict_blind_orientation(q_f, omega_f, tau=-1.0)
    np.testing.assert_allclose(q_neg, q_f, atol=1e-12)


def test_predict_blind_orientation_math():
    """Verify decaying-turn-rate orientation equation."""
    q_f = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_f = np.array([0.0, 0.0, 0.5], dtype=np.float64)  # pure z yaw rate
    tau_omega = 1.5

    for tau in [0.5, 1.0, 2.0, 5.0]:
        decay_angle = 0.5 * tau_omega * (1.0 - math.exp(-tau / tau_omega))
        expected_w = math.cos(0.5 * decay_angle)
        expected_z = math.sin(0.5 * decay_angle)
        expected_q = np.array([expected_w, 0.0, 0.0, expected_z], dtype=np.float64)

        q_tau = predict_blind_orientation(q_f, omega_f, tau=tau, tau_omega=tau_omega)
        np.testing.assert_allclose(q_tau, expected_q, atol=1e-10)
        assert abs(np.linalg.norm(q_tau) - 1.0) < 1e-12


def test_predict_blind_orientation_clamping_beyond_5s():
    """Orientation beyond tau = 5.0 s must clamp to orientation at tau = 5.0 s."""
    q_f = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_f = np.array([0.2, -0.3, 0.4], dtype=np.float64)

    q_5 = predict_blind_orientation(q_f, omega_f, tau=5.0)
    q_6 = predict_blind_orientation(q_f, omega_f, tau=6.0)
    q_15 = predict_blind_orientation(q_f, omega_f, tau=15.0)

    np.testing.assert_allclose(q_6, q_5, atol=1e-12)
    np.testing.assert_allclose(q_15, q_5, atol=1e-12)


def test_predict_blind_pose_combined():
    """Verify combined pose predictor."""
    p_f = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    v_f = np.array([0.5, 0.0, 0.0], dtype=np.float64)
    q_f = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_f = np.array([0.0, 0.0, 0.2], dtype=np.float64)

    p, q = predict_blind_pose(p_f, v_f, q_f, omega_f, tau=2.0)
    assert p.shape == (3,)
    assert q.shape == (4,)
    assert abs(np.linalg.norm(q) - 1.0) < 1e-12


# ---------------------------------------------------------------------------
# 4. Ring Buffer and Rollback Tests
# ---------------------------------------------------------------------------


def test_reliable_ring_buffer_fifo():
    """Test fixed ring buffer insertion and capacity bounds (no heap realloc)."""
    buf = ReliableRingBuffer(capacity=64)
    assert len(buf) == 0

    # Push 100 snapshots
    for i in range(100):
        t_ns = 1_000_000_000 + i * 5_000_000
        p = np.array([float(i), 0.0, 0.0])
        v = np.array([1.0, 0.0, 0.0])
        q = np.array([1.0, 0.0, 0.0, 0.0])
        w = np.array([0.0, 0.0, 0.1])
        buf.push(t_ns, p, v, q, w)

    assert len(buf) == 64
    latest = buf.get_latest()
    assert latest is not None
    assert latest.t_ns == 1_000_000_000 + 99 * 5_000_000
    assert latest.p[0] == 99.0

    oldest = buf.get_oldest()
    assert oldest is not None
    assert oldest.t_ns == 1_000_000_000 + (100 - 64) * 5_000_000
    assert oldest.p[0] == 36.0


def test_rollback_to_reliable_prior_to_onset():
    """Verify rollback retrieves the snapshot immediately prior to failure onset."""
    buf = ReliableRingBuffer(capacity=64)

    # Push samples from t = 0.0s to t = 0.3s at 200 Hz (dt = 5ms = 5_000_000 ns)
    for i in range(60):
        t_ns = i * 5_000_000
        p = np.array([float(i) * 0.1, 0.0, 0.0])
        v = np.array([1.0, 0.0, 0.0])
        q = np.array([1.0, 0.0, 0.0, 0.0])
        w = np.array([0.0, 0.0, 0.05])
        buf.push(t_ns, p, v, q, w)

    # Suppose failure onset occurred at t = 50 * 5_000_000 ns
    onset_ns = 50 * 5_000_000
    snap = rollback_to_reliable(buf, onset_ns)

    # Should retrieve sample index 49 (latest with t_ns < onset_ns)
    expected_t_ns = 49 * 5_000_000
    assert snap.t_ns == expected_t_ns
    assert abs(snap.p[0] - 4.9) < 1e-9


# ---------------------------------------------------------------------------
# 5. Failure Detector and Streaming BlindPredictor Tests
# ---------------------------------------------------------------------------


def test_blind_predictor_continuous_invalid_trigger():
    """Failure is declared when invalid samples persist continuously for >= T_fail (1.0 s)."""
    predictor = BlindPredictor()
    assert not predictor.is_failed

    dt_ns = 5_000_000  # 200 Hz
    t0_ns = 1_000_000_000

    # 1. Feed valid samples for 1.0 s (200 samples)
    for i in range(200):
        t_ns = t0_ns + i * dt_ns
        p = np.array([float(i) * 0.01, 0.0, 0.0])
        v = np.array([1.0, 0.0, 0.0])
        q = np.array([1.0, 0.0, 0.0, 0.0])
        w = np.array([0.0, 0.0, 0.1])
        failed = predictor.update(t_ns, is_valid=True, p=p, v=v, q=q, omega_s=w)
        assert not failed
        assert not predictor.is_failed

    # 2. Feed invalid samples for 0.8 s (160 samples < 1.0 s T_fail)
    last_valid_t_ns = t0_ns + 199 * dt_ns
    for i in range(160):
        t_ns = last_valid_t_ns + (i + 1) * dt_ns
        failed = predictor.update(t_ns, is_valid=False)
        assert not failed
        assert not predictor.is_failed

    # 3. Intervene with a single valid sample: resets continuous invalid counter!
    t_intervene_ns = last_valid_t_ns + 161 * dt_ns
    p = np.array([2.0, 0.0, 0.0])
    v = np.array([1.0, 0.0, 0.0])
    q = np.array([1.0, 0.0, 0.0, 0.0])
    w = np.array([0.0, 0.0, 0.1])
    failed = predictor.update(t_intervene_ns, is_valid=True, p=p, v=v, q=q, omega_s=w)
    assert not failed
    assert not predictor.is_failed
    assert predictor.invalid_start_ns is None

    # 4. Now feed invalid samples continuously for >= 1.0 s (205 samples to ensure >= 1.0s)
    onset_t_ns = t_intervene_ns + dt_ns
    for i in range(205):
        t_ns = onset_t_ns + i * dt_ns
        failed = predictor.update(t_ns, is_valid=False)
        dur = (i * dt_ns) * 1e-9
        if dur < 1.0:
            assert not failed
            assert not predictor.is_failed
        else:
            # Reached 1.0 s!
            assert failed
            assert predictor.is_failed
            break

    assert predictor.is_failed
    assert predictor.frozen_state is not None
    # Verify rollback retrieved pre-failure state (timestamp <= t_intervene_ns)
    assert predictor.frozen_state.t_ns <= t_intervene_ns


def test_blind_predictor_stream_end():
    """Verify failure declaration at stream end."""
    predictor = BlindPredictor()

    # Feed 10 valid samples
    for i in range(10):
        t_ns = (i + 1) * 5_000_000
        p = np.array([float(i), 0.0, 0.0])
        v = np.array([1.0, 0.0, 0.0])
        q = np.array([1.0, 0.0, 0.0, 0.0])
        w = np.array([0.0, 0.0, 0.0])
        predictor.update(t_ns, is_valid=True, p=p, v=v, q=q, omega_s=w)

    assert not predictor.is_failed
    predictor.notify_stream_end()
    assert predictor.is_failed
    assert predictor.frozen_state is not None
    assert predictor.frozen_state.t_ns == 10 * 5_000_000


def test_blind_predictor_generate_blind_poses():
    """Verify generate_blind_poses for --append-blind."""
    predictor = BlindPredictor()
    p_f = np.array([1.0, 2.0, 3.0])
    v_f = np.array([2.0, 0.0, 0.0])
    q_f = np.array([1.0, 0.0, 0.0, 0.0])
    w_f = np.array([0.0, 0.0, 0.5])

    t_f_ns = 1_000_000_000
    predictor.set_frozen_state(t_f_ns, p_f, v_f, q_f, w_f)
    assert predictor.is_failed

    dt_ns = 5_000_000  # 200 Hz -> 1000 steps for 5.0 s
    poses = predictor.generate_blind_poses(start_t_ns=t_f_ns, dt_ns=dt_ns, duration_s=5.0)

    assert len(poses) == 1000
    first_t, first_p, first_q = poses[0]
    assert first_t == t_f_ns + dt_ns
    assert abs(np.linalg.norm(first_q) - 1.0) < 1e-10

    last_t, last_p, last_q = poses[-1]
    assert last_t == t_f_ns + 1000 * dt_ns

    # Further extrapolation beyond 5s clamps
    p_ext, q_ext = predictor.predict(t_f_ns + 7_000_000_000)
    np.testing.assert_allclose(p_ext, last_p, atol=1e-12)
    np.testing.assert_allclose(q_ext, last_q, atol=1e-12)


# ---------------------------------------------------------------------------
# 6. Acceptance Tests on Synthetic Trajectories (Circle & Figure Eight)
# ---------------------------------------------------------------------------


def test_acceptance_synth_circle_blind_rmse_vs_naive():
    """Acceptance test: 10s synthetic circle with complete failure at t = 5.0 s.

    Confirm blind-5s RMSE is strictly less than naive open-loop integration RMSE.
    """
    duration = 10.0
    rate = 200.0
    traj = generate_trajectory("circle", duration=duration, rate=rate)

    t_ns = traj["t_ns"]
    t_s = traj["t"]
    p_gt = traj["p"]
    v_gt = traj["v"]
    q_gt = traj["q"]
    w_gt = traj["omega"]
    acc_gt = traj["acc"]

    fail_idx = int(round(5.0 * rate))
    t_fail_ns = t_ns[fail_idx]

    # Pre-failure state at t = 5.0 s
    p_f = p_gt[fail_idx].copy()
    v_f = v_gt[fail_idx].copy()
    q_f = q_gt[fail_idx].copy()
    w_f = w_gt[fail_idx].copy()

    # 1. Blind Predictor extrapolation from t = 5.0 s to 10.0 s
    blind_predictor = BlindPredictor()
    blind_predictor.set_frozen_state(t_fail_ns, p_f, v_f, q_f, w_f)

    blind_positions = []
    for idx in range(fail_idx, len(t_ns)):
        tau = (t_ns[idx] - t_fail_ns) * 1e-9
        p_b, _ = blind_predictor.predict(t_ns[idx])
        blind_positions.append(p_b)
    blind_positions = np.array(blind_positions)

    # 2. Naive open-loop integration over failure period:
    # During failure, IMU values are lost (a_k = 0, w_k = 0).
    # Naive baseline strapdown integrates specific force with world gravity:
    # a_w = R @ a_k + g_w = [0, 0, -9.80665] m/s^2.
    g_w = np.array([0.0, 0.0, -9.80665], dtype=np.float64)
    dt_step = 1.0 / rate

    p_naive = p_f.copy()
    v_naive = v_f.copy()
    q_naive = q_f.copy()

    naive_open_loop_positions = []
    naive_linear_positions = []
    for idx in range(fail_idx, len(t_ns)):
        tau = (t_ns[idx] - t_fail_ns) * 1e-9
        # Linear constant-velocity tangent
        naive_linear_positions.append(p_f + v_f * tau)

        # Naive open-loop strapdown integration with failed IMU (a=0, w=0)
        R_mat = quat_to_rot_matrix(q_naive)
        a_w = R_mat @ np.zeros(3) + g_w
        p_naive = p_naive + v_naive * dt_step + 0.5 * a_w * (dt_step**2)
        v_naive = v_naive + a_w * dt_step
        naive_open_loop_positions.append(p_naive.copy())

    naive_open_loop_positions = np.array(naive_open_loop_positions)
    naive_linear_positions = np.array(naive_linear_positions)

    # Ground truth segment
    gt_segment = p_gt[fail_idx:]

    blind_errors = np.linalg.norm(blind_positions - gt_segment, axis=1)
    blind_rmse = float(np.sqrt(np.mean(blind_errors**2)))

    open_loop_errors = np.linalg.norm(naive_open_loop_positions - gt_segment, axis=1)
    open_loop_rmse = float(np.sqrt(np.mean(open_loop_errors**2)))

    linear_errors = np.linalg.norm(naive_linear_positions - gt_segment, axis=1)
    linear_rmse = float(np.sqrt(np.mean(linear_errors**2)))

    # Confirm blind RMSE is strictly less than naive open-loop integration RMSE
    assert blind_rmse < open_loop_rmse, (
        f"Blind RMSE ({blind_rmse:.4f}) must be strictly less than open loop RMSE ({open_loop_rmse:.4f})"
    )
    # Also strictly beats linear tangent extrapolation
    assert blind_rmse < linear_rmse, (
        f"Blind RMSE ({blind_rmse:.4f}) must be strictly less than linear tangent RMSE ({linear_rmse:.4f})"
    )


def test_acceptance_synth_figure_eight_blind_rmse_vs_naive():
    """Acceptance test: 10s synthetic figure-eight with complete failure at t = 5.0 s.

    Confirm blind-5s RMSE is strictly less than naive open-loop integration RMSE.
    """
    duration = 10.0
    rate = 200.0
    traj = generate_trajectory("figure_eight", duration=duration, rate=rate)

    t_ns = traj["t_ns"]
    p_gt = traj["p"]
    v_gt = traj["v"]
    q_gt = traj["q"]
    w_gt = traj["omega"]

    fail_idx = int(round(5.0 * rate))
    t_fail_ns = t_ns[fail_idx]

    p_f = p_gt[fail_idx].copy()
    v_f = v_gt[fail_idx].copy()
    q_f = q_gt[fail_idx].copy()
    w_f = w_gt[fail_idx].copy()

    blind_predictor = BlindPredictor()
    blind_predictor.set_frozen_state(t_fail_ns, p_f, v_f, q_f, w_f)

    blind_positions = []
    naive_open_loop_positions = []
    g_w = np.array([0.0, 0.0, -9.80665], dtype=np.float64)
    dt_step = 1.0 / rate

    p_naive = p_f.copy()
    v_naive = v_f.copy()
    q_naive = q_f.copy()

    for idx in range(fail_idx, len(t_ns)):
        tau = (t_ns[idx] - t_fail_ns) * 1e-9
        p_b, _ = blind_predictor.predict(t_ns[idx])
        blind_positions.append(p_b)

        # Naive open loop integration on failed IMU
        R_mat = quat_to_rot_matrix(q_naive)
        a_w = R_mat @ np.zeros(3) + g_w
        p_naive = p_naive + v_naive * dt_step + 0.5 * a_w * (dt_step**2)
        v_naive = v_naive + a_w * dt_step
        naive_open_loop_positions.append(p_naive.copy())

    blind_positions = np.array(blind_positions)
    naive_open_loop_positions = np.array(naive_open_loop_positions)
    gt_segment = p_gt[fail_idx:]

    blind_rmse = float(np.sqrt(np.mean(np.linalg.norm(blind_positions - gt_segment, axis=1) ** 2)))
    naive_rmse = float(np.sqrt(np.mean(np.linalg.norm(naive_open_loop_positions - gt_segment, axis=1) ** 2)))

    assert blind_rmse < naive_rmse, (
        f"Blind RMSE ({blind_rmse:.4f}) must be strictly less than naive RMSE ({naive_rmse:.4f})"
    )
