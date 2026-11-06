"""Unit tests for filter initialisation (SPEC §12.2).

Tests cover:
- Static window tilt and bias initialization (T_init = 1.0 s, fallback 0.5 s).
- Gravity clamping to [9.0, 10.5] and g_w = [0, 0, -g_mag].
- Motion detection fallback (gyro std > 0.2 rad/s or accel-norm std > 0.5).
- Covariance initialization P0 with defaults and 5x multiplier on sigma_theta on motion.
- Online streaming initializer window accumulation.
"""

import numpy as np
import pytest

from src.ref.mathlib import quat_to_rot_matrix
from src.ref.init import (
    NominalState,
    compute_initial_tilt,
    detect_motion,
    initialize_covariance,
    initialize_from_window,
    OnlineInitializer,
    load_params,
)


def test_compute_initial_tilt_canonical_axes():
    """Verify tilt quaternion aligns specific force to world +z with zero yaw."""
    # Case 1: Already aligned with world +z: a = [0, 0, 9.81]
    a_up = np.array([0.0, 0.0, 9.81])
    q_up = compute_initial_tilt(a_up)
    R_up = quat_to_rot_matrix(q_up)
    np.testing.assert_allclose(R_up @ (a_up / 9.81), [0.0, 0.0, 1.0], atol=1e-12)
    np.testing.assert_allclose(q_up, [1.0, 0.0, 0.0, 0.0], atol=1e-12)

    # Case 2: Inverted body (accel points down -z): a = [0, 0, -9.81]
    a_down = np.array([0.0, 0.0, -9.81])
    q_down = compute_initial_tilt(a_down)
    R_down = quat_to_rot_matrix(q_down)
    np.testing.assert_allclose(R_down @ (a_down / 9.81), [0.0, 0.0, 1.0], atol=1e-12)

    # Case 3: Pointing along +x: a = [9.81, 0, 0]
    a_x = np.array([9.81, 0.0, 0.0])
    q_x = compute_initial_tilt(a_x)
    R_x = quat_to_rot_matrix(q_x)
    np.testing.assert_allclose(R_x @ (a_x / 9.81), [0.0, 0.0, 1.0], atol=1e-12)

    # Case 4: Pointing along +y: a = [0, 9.81, 0]
    a_y = np.array([0.0, 9.81, 0.0])
    q_y = compute_initial_tilt(a_y)
    R_y = quat_to_rot_matrix(q_y)
    np.testing.assert_allclose(R_y @ (a_y / 9.81), [0.0, 0.0, 1.0], atol=1e-12)

    # Case 5: Arbitrary diagonal vector
    a_diag = np.array([1.5, -2.3, 9.2])
    q_diag = compute_initial_tilt(a_diag)
    R_diag = quat_to_rot_matrix(q_diag)
    np.testing.assert_allclose(
        R_diag @ (a_diag / np.linalg.norm(a_diag)), [0.0, 0.0, 1.0], atol=1e-12
    )
    # Rotation axis k = v x u has kz == 0, so q_z should be 0.0 (zero yaw)
    assert q_diag[3] == pytest.approx(0.0, abs=1e-12)


def test_static_window_initialization_1s():
    """Verify static initialization over 1.0 s window (SPEC §12.2)."""
    params = load_params()
    rate = 200.0  # Hz
    dt = 1.0 / rate
    n_samples = int(1.5 * rate)  # 1.5 seconds of data (300 samples)
    t_ns = np.arange(n_samples, dtype=np.int64) * int(dt * 1e9)

    # Injected true biases and static gravity
    true_bg = np.array([0.015, -0.02, 0.005], dtype=np.float64)
    true_acc = np.array([0.1, -0.2, 9.80], dtype=np.float64)

    # Add small zero-mean noise with std well below motion threshold
    np.random.seed(42)
    omega = true_bg + np.random.normal(0.0, 0.005, size=(n_samples, 3))
    acc = true_acc + np.random.normal(0.0, 0.01, size=(n_samples, 3))

    res = initialize_from_window(t_ns, omega, acc, params=params)

    # Samples used should correspond to first 1.0 s
    expected_samples = int(params["T_init"] * rate) + 1
    assert res.samples_used == pytest.approx(expected_samples, abs=2)
    assert res.motion_detected is False

    # Position and velocity must be zero
    np.testing.assert_allclose(res.state.p, [0.0, 0.0, 0.0], atol=1e-14)
    np.testing.assert_allclose(res.state.v, [0.0, 0.0, 0.0], atol=1e-14)

    # Accelerometer bias initial must be zero
    np.testing.assert_allclose(res.state.b_a, [0.0, 0.0, 0.0], atol=1e-14)

    # Gyro bias should estimate mean gyro over the 1.0s window
    window_mask = (t_ns - t_ns[0]) <= int(params["T_init"] * 1e9)
    expected_bg = np.mean(omega[window_mask], axis=0)
    np.testing.assert_allclose(res.state.b_g, expected_bg, atol=1e-12)

    # Tilt quaternion aligns mean accel to +z
    expected_amean = np.mean(acc[window_mask], axis=0)
    R0 = quat_to_rot_matrix(res.state.q)
    aligned_a = R0 @ (expected_amean / np.linalg.norm(expected_amean))
    np.testing.assert_allclose(aligned_a, [0.0, 0.0, 1.0], atol=1e-12)

    # Gravity vector g_w = [0, 0, -g_mag]
    expected_g_mag = np.clip(
        np.linalg.norm(expected_amean),
        params["g_mag_clamp_min"],
        params["g_mag_clamp_max"],
    )
    assert res.g_mag == pytest.approx(expected_g_mag, abs=1e-12)
    np.testing.assert_allclose(res.g_w, [0.0, 0.0, -expected_g_mag], atol=1e-12)

    # Covariance P0 is diagonal with defaults
    assert res.P.shape == (15, 15)
    diag_P = np.diag(res.P)
    expected_diag = np.array([
        params["sigma_theta_0"] ** 2] * 3 +
        [params["sigma_v_0"] ** 2] * 3 +
        [params["sigma_p_0"] ** 2] * 3 +
        [params["sigma_bg_0"] ** 2] * 3 +
        [params["sigma_ba_0"] ** 2] * 3
    )
    np.testing.assert_allclose(diag_P, expected_diag, atol=1e-14)
    # Off-diagonals must be strictly zero
    np.testing.assert_allclose(res.P - np.diag(diag_P), np.zeros((15, 15)), atol=1e-14)


def test_fallback_window_0_5s():
    """Verify fallback initialization when duration is between 0.5 s and 1.0 s."""
    params = load_params()
    rate = 200.0
    dt = 1.0 / rate
    # 0.7 seconds of data (less than 1.0s, but >= fallback 0.5s)
    n_samples = int(0.7 * rate)
    t_ns = np.arange(n_samples, dtype=np.int64) * int(dt * 1e9)
    omega = np.zeros((n_samples, 3), dtype=np.float64)
    acc = np.tile([0.0, 0.0, 9.81], (n_samples, 1))

    res = initialize_from_window(t_ns, omega, acc, params=params)
    assert res.samples_used == n_samples
    assert res.motion_detected is False
    np.testing.assert_allclose(res.state.q, [1.0, 0.0, 0.0, 0.0], atol=1e-12)


def test_insufficient_samples_raises():
    """Verify error raised when valid duration is less than fallback 0.5 s."""
    params = load_params()
    rate = 200.0
    dt = 1.0 / rate
    # 0.3 seconds of data (< 0.5s fallback)
    n_samples = int(0.3 * rate)
    t_ns = np.arange(n_samples, dtype=np.int64) * int(dt * 1e9)
    omega = np.zeros((n_samples, 3))
    acc = np.tile([0.0, 0.0, 9.81], (n_samples, 1))

    with pytest.raises(ValueError, match="Insufficient samples"):
        initialize_from_window(t_ns, omega, acc, params=params)


def test_gravity_clamping():
    """Verify gravity magnitude is clamped to [9.0, 10.5] (SPEC §12.2)."""
    params = load_params()
    rate = 200.0
    dt = 1.0 / rate
    n_samples = int(1.0 * rate) + 1
    t_ns = np.arange(n_samples, dtype=np.int64) * int(dt * 1e9)
    omega = np.zeros((n_samples, 3))

    # Low gravity: norm = 8.5 m/s^2 (< 9.0)
    acc_low = np.tile([0.0, 0.0, 8.5], (n_samples, 1))
    res_low = initialize_from_window(t_ns, omega, acc_low, params=params)
    assert res_low.g_mag == pytest.approx(9.0, abs=1e-12)
    np.testing.assert_allclose(res_low.g_w, [0.0, 0.0, -9.0], atol=1e-12)

    # High gravity: norm = 11.5 m/s^2 (> 10.5)
    acc_high = np.tile([0.0, 0.0, 11.5], (n_samples, 1))
    res_high = initialize_from_window(t_ns, omega, acc_high, params=params)
    assert res_high.g_mag == pytest.approx(10.5, abs=1e-12)
    np.testing.assert_allclose(res_high.g_w, [0.0, 0.0, -10.5], atol=1e-12)


def test_motion_detection_gyro_std():
    """Verify motion fallback triggers when gyro std > 0.2 rad/s (SPEC §12.2)."""
    params = load_params()
    rate = 200.0
    dt = 1.0 / rate
    n_samples = int(1.2 * rate)
    t_ns = np.arange(n_samples, dtype=np.int64) * int(dt * 1e9)

    # Injected high-variance gyro (std ~ 0.35 rad/s > 0.2)
    np.random.seed(123)
    omega = np.random.normal(0.0, 0.35, size=(n_samples, 3))
    # Accel with first sample tilted differently from rest
    acc = np.tile([0.0, 0.0, 9.81], (n_samples, 1))
    acc[0] = [1.0, 0.0, 9.75]  # Distinct first sample

    res = initialize_from_window(t_ns, omega, acc, params=params)

    assert res.motion_detected is True
    # Bias gyro must be set to 0.0 (cannot assume static)
    np.testing.assert_allclose(res.state.b_g, [0.0, 0.0, 0.0], atol=1e-14)
    # Tilt must use first sample's tilt
    expected_q0 = compute_initial_tilt(acc[0])
    np.testing.assert_allclose(res.state.q, expected_q0, atol=1e-12)

    # sigma_theta_0 must be multiplied by 5.0 (motion_detect_sigma_theta_mult)
    mult = params["motion_detect_sigma_theta_mult"]
    expected_var_theta = (params["sigma_theta_0"] * mult) ** 2
    diag_P = np.diag(res.P)
    np.testing.assert_allclose(diag_P[0:3], [expected_var_theta] * 3, atol=1e-14)


def test_motion_detection_acc_norm_std():
    """Verify motion fallback triggers when accel-norm std > 0.5 m/s^2 (SPEC §12.2)."""
    params = load_params()
    rate = 200.0
    dt = 1.0 / rate
    n_samples = int(1.2 * rate)
    t_ns = np.arange(n_samples, dtype=np.int64) * int(dt * 1e9)

    omega = np.zeros((n_samples, 3))
    # Accel norm varying strongly (std > 0.5)
    np.random.seed(456)
    acc_noise = np.random.normal(0.0, 0.8, size=(n_samples, 1))
    acc = np.hstack([np.zeros((n_samples, 2)), 9.81 + acc_noise])
    acc[0] = [0.0, 1.0, 9.75]

    res = initialize_from_window(t_ns, omega, acc, params=params)

    assert res.motion_detected is True
    np.testing.assert_allclose(res.state.b_g, [0.0, 0.0, 0.0], atol=1e-14)
    expected_q0 = compute_initial_tilt(acc[0])
    np.testing.assert_allclose(res.state.q, expected_q0, atol=1e-12)
    mult = params["motion_detect_sigma_theta_mult"]
    expected_var_theta = (params["sigma_theta_0"] * mult) ** 2
    np.testing.assert_allclose(np.diag(res.P)[0:3], [expected_var_theta] * 3, atol=1e-14)


def test_online_initializer_streaming():
    """Verify OnlineInitializer streaming accumulator behaves identically to batch."""
    params = load_params()
    rate = 200.0
    dt = 1.0 / rate
    n_samples = int(1.5 * rate)
    t_ns = np.arange(n_samples, dtype=np.int64) * int(dt * 1e9)
    omega = np.tile([0.01, -0.01, 0.005], (n_samples, 1))
    acc = np.tile([0.0, 0.0, 9.81], (n_samples, 1))

    initializer = OnlineInitializer(params=params)
    initialized = False

    for k in range(n_samples):
        ready = initializer.feed(t_ns[k], omega[k], acc[k], is_valid=True)
        if ready:
            initialized = True
            break

    assert initialized is True
    res_online = initializer.get_result()
    assert res_online is not None
    assert res_online.motion_detected is False
    np.testing.assert_allclose(res_online.state.b_g, [0.01, -0.01, 0.005], atol=1e-12)
    np.testing.assert_allclose(res_online.state.q, [1.0, 0.0, 0.0, 0.0], atol=1e-12)
