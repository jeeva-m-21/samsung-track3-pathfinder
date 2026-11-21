"""Unit and regression tests for Degradation Monitor and Quality Score (SPEC §12.6, WP-25).

Tests:
1. Pure high-frequency residual computation:
   - r_k = ||a_k - a_{k-1}|| and rho_k = ||omega_k - omega_{k-1}||
   - Zero residual for identical successive measurements
   - Euclidean norm correctness for 3D vectors
2. Exponential moving variance:
   - var_k = (1 - alpha) * var_{k-1} + alpha * residual^2 with alpha = 0.01
   - Asymptotic convergence under constant residual
3. Reference variance calibration over initial static window:
   - During static window (first T_init = 1.0 s), reference variance tracks moving variance
   - Once t > T_init, reference variance is frozen
   - Quality score stays 1.0 and kappa stays 1.0 during calibration
4. Noise ratio, quality score, and inflation factor kappa:
   - Noise ratio n = max(sigma_now / sigma_ref, 1.0)
   - Quality score = clamp(1.0 / n, 0.05, 1.0)
   - Inflation factor kappa = clamp(n, 1.0, kappa_max) where kappa_max = 50.0
   - Proper clamping at both lower and upper boundaries
5. Progressive noise injection monotonicity:
   - Quality score falls monotonically as noise is progressively injected into IMU measurements
   - Inflation factor kappa increases monotonically as noise grows
6. Stationary gyro bias observation helper:
   - Measurement matrix H = [0, 0, 0, I, 0] (3x15)
   - Measurement noise R = sigma_bgm^2 * I3 (sigma_bgm = 5e-3)
   - Innovation y = omega_static_mean - b_g
   - Bias trend exponential moving average
   - Gyro bias ESKF update reduces bias error and covariance
7. Streaming DegradationMonitor class:
   - Fixed-capacity float64 operations
   - Deterministic reset and state consistency
"""

import math
import numpy as np
import pytest

from src.ref.eskf import NominalState, load_params
from src.ref.init import initialize_covariance
from src.ref.monitor import (
    DegradationMonitor,
    MonitorState,
    build_gyro_bias_h_matrix,
    build_gyro_bias_r_matrix,
    compute_kappa,
    compute_noise_ratio,
    compute_quality_score,
    compute_residual,
    gyro_bias_innovation,
    gyro_bias_update,
    monitor_step,
    update_bias_trend,
    update_moving_variance,
)


# ===========================================================================
# 1. High-frequency residual tests
# ===========================================================================


def test_residual_identical_vectors():
    """Identical successive samples yield zero residual."""
    v1 = np.array([1.2, -3.4, 9.8], dtype=np.float64)
    v2 = np.array([1.2, -3.4, 9.8], dtype=np.float64)
    res = compute_residual(v2, v1)
    assert isinstance(res, float)
    assert res == pytest.approx(0.0, abs=1e-12)


def test_residual_known_3d_norm():
    """Residual matches Euclidean norm ||v_k - v_{k-1}||."""
    v_prev = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    # Difference: [1.0, 2.0, 2.0] -> norm = sqrt(1 + 4 + 4) = 3.0
    v_curr = np.array([2.0, 4.0, 5.0], dtype=np.float64)
    res = compute_residual(v_curr, v_prev)
    assert res == pytest.approx(3.0, abs=1e-9)


def test_residual_list_input_handling():
    """Residual function accepts lists and array-like inputs."""
    r = compute_residual([0.0, 3.0, 4.0], [0.0, 0.0, 0.0])
    assert r == pytest.approx(5.0, abs=1e-9)


# ===========================================================================
# 2. Exponential moving variance tests
# ===========================================================================


def test_moving_variance_formula():
    """Moving variance implements var_k = (1 - alpha) * var_{k-1} + alpha * res^2."""
    alpha = 0.01
    prev_var = 2.0
    residual = 3.0  # res^2 = 9.0
    expected = (1.0 - alpha) * 2.0 + alpha * 9.0  # 0.99 * 2.0 + 0.01 * 9.0 = 1.98 + 0.09 = 2.07
    updated = update_moving_variance(prev_var, residual, alpha=alpha)
    assert updated == pytest.approx(expected, abs=1e-12)


def test_moving_variance_convergence():
    """Under constant residual, moving variance converges to residual squared."""
    alpha = 0.05
    res = 1.5
    var = 0.0
    for _ in range(300):
        var = update_moving_variance(var, res, alpha=alpha)
    assert var == pytest.approx(res**2, rel=1e-4)


# ===========================================================================
# 3. Noise ratio, Quality score, and Kappa inflation tests
# ===========================================================================


def test_noise_ratio_formula():
    """Noise ratio n = max(sigma_now / sigma_ref, 1.0)."""
    assert compute_noise_ratio(2.0, 1.0) == pytest.approx(2.0, abs=1e-9)
    assert compute_noise_ratio(1.0, 1.0) == pytest.approx(1.0, abs=1e-9)
    # If sigma_now < sigma_ref, ratio clamped to 1.0
    assert compute_noise_ratio(0.5, 1.0) == pytest.approx(1.0, abs=1e-9)
    # Zero or negative sigma_ref clamped safely
    assert compute_noise_ratio(1.0, 0.0) == pytest.approx(1.0, abs=1e-9)


def test_quality_score_clamping():
    """Quality score = clamp(1.0 / n, 0.05, 1.0)."""
    # Nominal n = 1.0 -> quality = 1.0
    assert compute_quality_score(1.0) == pytest.approx(1.0, abs=1e-9)
    # Doubled noise n = 2.0 -> quality = 0.5
    assert compute_quality_score(2.0) == pytest.approx(0.5, abs=1e-9)
    # 10x noise n = 10.0 -> quality = 0.1
    assert compute_quality_score(10.0) == pytest.approx(0.1, abs=1e-9)
    # 20x noise n = 20.0 -> quality = 0.05
    assert compute_quality_score(20.0) == pytest.approx(0.05, abs=1e-9)
    # 50x noise n = 50.0 -> clamped at floor 0.05
    assert compute_quality_score(50.0) == pytest.approx(0.05, abs=1e-9)
    # 100x noise n = 100.0 -> clamped at floor 0.05
    assert compute_quality_score(100.0) == pytest.approx(0.05, abs=1e-9)
    # Sub-nominal n < 1.0 -> clamped at 1.0
    assert compute_quality_score(0.5) == pytest.approx(1.0, abs=1e-9)


def test_kappa_inflation_clamping():
    """Inflation factor kappa = clamp(n, 1.0, kappa_max) with kappa_max = 50.0."""
    assert compute_kappa(0.5, kappa_max=50.0) == pytest.approx(1.0, abs=1e-9)
    assert compute_kappa(1.0, kappa_max=50.0) == pytest.approx(1.0, abs=1e-9)
    assert compute_kappa(5.0, kappa_max=50.0) == pytest.approx(5.0, abs=1e-9)
    assert compute_kappa(50.0, kappa_max=50.0) == pytest.approx(50.0, abs=1e-9)
    # Clamped at kappa_max
    assert compute_kappa(100.0, kappa_max=50.0) == pytest.approx(50.0, abs=1e-9)


# ===========================================================================
# 4. Reference variance calibration during initial static window
# ===========================================================================


def test_reference_calibration_static_window():
    """Reference variance tracks moving variance during static window, then freezes."""
    params = load_params()
    t_init = float(params.get("T_init", 1.0))
    monitor = DegradationMonitor(params=params)

    dt = 0.005  # 200 Hz
    rng = np.random.RandomState(42)

    # 1. Feed static nominal data during [0.0, 1.0] s
    nominal_sigma_a = 0.02
    nominal_sigma_g = 0.005

    n_init_steps = int(round(t_init / dt))
    for step_i in range(n_init_steps + 1):
        t = step_i * dt
        acc = np.array([0.0, 0.0, 9.80665], dtype=np.float64) + rng.normal(
            0.0, nominal_sigma_a, size=3
        )
        omega = np.array([0.0, 0.0, 0.0], dtype=np.float64) + rng.normal(
            0.0, nominal_sigma_g, size=3
        )
        monitor.step(t, omega, acc)

    assert monitor.calibrated is True
    assert monitor.var_acc_ref > 0.0
    assert monitor.var_gyro_ref > 0.0
    frozen_ref_a = monitor.var_acc_ref
    frozen_ref_g = monitor.var_gyro_ref

    # Quality should be 1.0 and kappa 1.0 during nominal static
    assert monitor.quality == pytest.approx(1.0, abs=1e-2)
    assert monitor.kappa == pytest.approx(1.0, abs=1e-2)

    # 2. Feed higher noise post static window: ref variance must remain frozen
    for _ in range(50):
        t += dt
        acc = np.array([0.0, 0.0, 9.80665], dtype=np.float64) + rng.normal(
            0.0, nominal_sigma_a * 5.0, size=3
        )
        omega = np.array([0.0, 0.0, 0.0], dtype=np.float64) + rng.normal(
            0.0, nominal_sigma_g * 5.0, size=3
        )
        monitor.step(t, omega, acc)

    assert monitor.var_acc_ref == pytest.approx(frozen_ref_a, abs=1e-12)
    assert monitor.var_gyro_ref == pytest.approx(frozen_ref_g, abs=1e-12)
    # But current variance grew
    assert monitor.var_acc > frozen_ref_a
    assert monitor.var_gyro > frozen_ref_g


# ===========================================================================
# 5. Monotonic degradation under progressive noise injection
# ===========================================================================


def test_quality_score_monotonic_decrease_with_noise():
    """Verify quality score falls monotonically as noise is progressively injected."""
    params = load_params()
    monitor = DegradationMonitor(params=params)

    rng = np.random.RandomState(123)
    dt = 0.005  # 200 Hz
    t = 0.0

    base_noise_a = 0.01
    base_noise_g = 0.002

    # Calibrate on nominal data for 1.0 s
    while t <= 1.0:
        acc = np.array([0.0, 0.0, 9.80665], dtype=np.float64) + rng.normal(
            0.0, base_noise_a, size=3
        )
        omega = np.zeros(3, dtype=np.float64) + rng.normal(0.0, base_noise_g, size=3)
        monitor.step(t, omega, acc)
        t += dt

    # Test increasing noise multipliers
    multipliers = [1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0]
    qualities = []
    kappas = []

    for mult in multipliers:
        # Run long enough for exponential moving average to converge (~300 steps with alpha=0.01)
        for _ in range(400):
            acc = np.array([0.0, 0.0, 9.80665], dtype=np.float64) + rng.normal(
                0.0, base_noise_a * mult, size=3
            )
            omega = np.zeros(3, dtype=np.float64) + rng.normal(
                0.0, base_noise_g * mult, size=3
            )
            monitor.step(t, omega, acc)
            t += dt
        qualities.append(monitor.quality)
        kappas.append(monitor.kappa)

    # 1. Check monotonic non-increasing quality score
    for i in range(len(qualities) - 1):
        assert (
            qualities[i] >= qualities[i + 1] - 1e-4
        ), f"Quality violated monotonic decrease at index {i}: {qualities[i]} < {qualities[i+1]}"

    # 2. Check monotonic non-decreasing kappa
    for i in range(len(kappas) - 1):
        assert (
            kappas[i] <= kappas[i + 1] + 1e-4
        ), f"Kappa violated monotonic increase at index {i}: {kappas[i]} > {kappas[i+1]}"

    # 3. Check boundary conditions
    assert qualities[0] == pytest.approx(1.0, abs=0.05)
    assert qualities[-1] == pytest.approx(0.05, abs=1e-3)
    assert kappas[0] == pytest.approx(1.0, abs=0.05)
    assert kappas[-1] == pytest.approx(50.0, abs=1e-3)


# ===========================================================================
# 6. Stationary gyro bias observation helper tests (SPEC §12.6)
# ===========================================================================


def test_build_gyro_bias_matrices():
    """Verify H and R matrices for stationary gyro bias observation."""
    params = load_params()
    sigma_bgm = float(params.get("sigma_bgm", 5e-3))

    H = build_gyro_bias_h_matrix()
    assert H.shape == (3, 15)
    assert H.dtype == np.float64

    # SPEC §12.6: H = [0, 0, 0, I, 0] (delta_b_g block at 9:12 is I3)
    assert np.allclose(H[:, 0:9], 0.0)
    assert np.allclose(H[:, 9:12], np.eye(3))
    assert np.allclose(H[:, 12:15], 0.0)

    # SPEC §12.6: R = sigma_bgm^2 * I3
    R = build_gyro_bias_r_matrix(params=params, r_scale=1.0)
    assert R.shape == (3, 3)
    expected_var = sigma_bgm**2
    assert np.allclose(R, expected_var * np.eye(3))

    # r_scale scaling
    R_scaled = build_gyro_bias_r_matrix(params=params, r_scale=4.0)
    assert np.allclose(R_scaled, 4.0 * expected_var * np.eye(3))


def test_gyro_bias_innovation():
    """Innovation y = omega_static_mean - b_g (SPEC §12.6)."""
    state = NominalState(
        p=np.zeros(3),
        v=np.zeros(3),
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        b_g=np.array([0.02, -0.01, 0.03], dtype=np.float64),
        b_a=np.zeros(3),
    )
    omega_static_mean = np.array([0.005, -0.005, 0.01], dtype=np.float64)

    y = gyro_bias_innovation(state, omega_static_mean)
    expected_y = omega_static_mean - state.b_g
    assert np.allclose(y, expected_y)


def test_update_bias_trend():
    """Exponential moving average of stationary gyro mean."""
    alpha = 0.01
    prev_trend = np.array([0.01, 0.02, 0.03], dtype=np.float64)
    omega_mean = np.array([0.02, 0.01, 0.04], dtype=np.float64)

    expected = (1.0 - alpha) * prev_trend + alpha * omega_mean
    trend = update_bias_trend(prev_trend, omega_mean, alpha=alpha)
    assert np.allclose(trend, expected)


def test_gyro_bias_eskf_update():
    """Stationary gyro bias pseudo-update pulls b_g towards measurement and reduces covariance."""
    params = load_params()
    P_init = initialize_covariance(params)

    # Initial state with non-zero estimated bias
    initial_bg = np.array([0.04, -0.03, 0.02], dtype=np.float64)
    state = NominalState(
        p=np.array([1.0, 2.0, 3.0]),
        v=np.array([0.1, -0.2, 0.0]),
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        b_g=initial_bg,
        b_a=np.zeros(3),
    )

    # Stationary gyro observation (true bias is close to 0)
    omega_static_mean = np.array([0.001, -0.001, 0.000], dtype=np.float64)

    bg_prior_cov_trace = float(np.trace(P_init[9:12, 9:12]))

    new_state, new_P, delta_x = gyro_bias_update(
        state, P_init, omega_static_mean, params=params
    )

    # 1. State bias moves closer to observation
    dist_old = np.linalg.norm(state.b_g - omega_static_mean)
    dist_new = np.linalg.norm(new_state.b_g - omega_static_mean)
    assert dist_new < dist_old

    # 2. Covariance of gyro bias is reduced
    bg_post_cov_trace = float(np.trace(new_P[9:12, 9:12]))
    assert bg_post_cov_trace < bg_prior_cov_trace

    # 3. Kinematic states (p, v, q, b_a) are unaffected because H only couples to b_g
    assert np.allclose(new_state.p, state.p)
    assert np.allclose(new_state.v, state.v)
    assert np.allclose(new_state.q, state.q)
    assert np.allclose(new_state.b_a, state.b_a)


# ===========================================================================
# 7. Pure function step and DegradationMonitor class tests
# ===========================================================================


def test_monitor_pure_step():
    """monitor_step is a pure function returning a new MonitorState without mutating."""
    params = load_params()
    state0 = MonitorState()

    t1 = 0.0
    omega1 = np.array([0.01, -0.01, 0.02], dtype=np.float64)
    acc1 = np.array([0.0, 0.0, 9.8], dtype=np.float64)

    state1 = monitor_step(state0, t1, omega1, acc1, params=params)
    assert state1 is not state0
    assert state0.sample_count == 0
    assert state1.sample_count == 1

    t2 = 0.005
    omega2 = np.array([0.02, -0.01, 0.03], dtype=np.float64)
    acc2 = np.array([0.1, 0.0, 9.8], dtype=np.float64)

    state2 = monitor_step(state1, t2, omega2, acc2, params=params)
    assert state2 is not state1
    assert state2.sample_count == 2
    assert state2.residual_acc > 0.0
    assert state2.residual_gyro > 0.0


def test_monitor_reset():
    """DegradationMonitor reset restores default state."""
    monitor = DegradationMonitor()
    monitor.step(0.0, [0.1, 0.0, 0.0], [0.0, 0.0, 9.8])
    monitor.step(0.005, [0.2, 0.0, 0.0], [0.1, 0.0, 9.8])
    assert monitor.count == 2

    monitor.reset()
    assert monitor.count == 0
    assert monitor.calibrated is False
    assert monitor.quality == 1.0
    assert monitor.kappa == 1.0
    assert monitor.var_acc == 0.0
    assert monitor.var_gyro == 0.0
