"""Unit tests for strapdown nominal state propagation (SPEC §12.3).

Tests cover:
- Nominal state propagation equations (SPEC §12.3) step-by-step verification.
- Pure function immutability (input state not mutated).
- Bias subtraction for gyro and accelerometer.
- Constant acceleration double-integration precision.
- Constant angular velocity rotation integration precision.
- Zero-noise static IMU over 60 s drifts < 1e-6 m (both upright and arbitrarily tilted).
"""

import numpy as np
import pytest

from src.ref.init import NominalState
from src.ref.mathlib import (
    quat_exp,
    quat_mult,
    quat_normalize,
    quat_sign_continuity,
    quat_to_rot_matrix,
)
from src.ref.strapdown import strapdown_step, strapdown_propagate_sequence


def test_strapdown_step_equations_exact():
    """Verify exact numerical agreement with SPEC §12.3 equations."""
    # Arbitrary initial state
    q0 = quat_normalize(np.array([0.70710678, 0.0, 0.70710678, 0.0]))  # 90 deg pitch
    v0 = np.array([1.2, -0.5, 0.8], dtype=np.float64)
    p0 = np.array([10.0, -20.0, 5.0], dtype=np.float64)
    bg = np.array([0.01, -0.02, 0.03], dtype=np.float64)
    ba = np.array([0.05, 0.02, -0.04], dtype=np.float64)

    state0 = NominalState(p=p0.copy(), v=v0.copy(), q=q0.copy(), b_g=bg.copy(), b_a=ba.copy())

    omega = np.array([0.1, 0.2, -0.15], dtype=np.float64)
    acc = np.array([0.3, -0.1, 9.85], dtype=np.float64)
    dt = 0.01  # 10 ms
    g_w = np.array([0.0, 0.0, -9.80665], dtype=np.float64)

    # Hand-calculate expected next state according to SPEC §12.3
    omega_hat = omega - bg
    a_hat = acc - ba

    # q_new = normalize(q_old (x) exp(omega_hat * dt))
    delta_q = quat_exp(omega_hat * dt)
    expected_q = quat_normalize(quat_mult(q0, delta_q))
    expected_q = quat_sign_continuity(expected_q, q0)

    # a_w = R(q_old) * a_hat + g_w
    R_old = quat_to_rot_matrix(q0)
    a_w = R_old @ a_hat + g_w

    # p_new = p0 + v0 * dt + 0.5 * a_w * dt^2
    expected_p = p0 + v0 * dt + 0.5 * a_w * (dt**2)

    # v_new = v0 + a_w * dt
    expected_v = v0 + a_w * dt

    # Perform strapdown step
    state1 = strapdown_step(state0, omega, acc, dt, g_w)

    np.testing.assert_allclose(state1.p, expected_p, atol=1e-14)
    np.testing.assert_allclose(state1.v, expected_v, atol=1e-14)
    np.testing.assert_allclose(state1.q, expected_q, atol=1e-14)
    np.testing.assert_allclose(state1.b_g, bg, atol=1e-14)
    np.testing.assert_allclose(state1.b_a, ba, atol=1e-14)


def test_pure_function_immutability():
    """Verify strapdown_step does not mutate the input state or sample arrays."""
    p0 = np.array([1.0, 2.0, 3.0])
    v0 = np.array([0.1, 0.2, 0.3])
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    bg = np.array([0.01, 0.0, 0.0])
    ba = np.array([0.0, 0.01, 0.0])

    state0 = NominalState(p=p0.copy(), v=v0.copy(), q=q0.copy(), b_g=bg.copy(), b_a=ba.copy())

    omega = np.array([0.05, 0.0, 0.0])
    acc = np.array([0.0, 0.0, 9.81])
    dt = 0.01
    g_w = np.array([0.0, 0.0, -9.81])

    state1 = strapdown_step(state0, omega, acc, dt, g_w)

    # Verify state0 has not changed
    np.testing.assert_allclose(state0.p, p0, atol=1e-15)
    np.testing.assert_allclose(state0.v, v0, atol=1e-15)
    np.testing.assert_allclose(state0.q, q0, atol=1e-15)
    np.testing.assert_allclose(state0.b_g, bg, atol=1e-15)
    np.testing.assert_allclose(state0.b_a, ba, atol=1e-15)

    # Verify state1 is a distinct instance
    assert state1 is not state0


def test_constant_acceleration_analytical():
    """Verify constant acceleration integration matches analytic kinematics."""
    dt = 0.005  # 200 Hz
    n_steps = 1000  # 5 seconds
    g_w = np.array([0.0, 0.0, -9.81])

    # Sensor oriented with world frame (q = identity)
    # Accelerating along body x with 2.0 m/s^2, while specific force measures a_x and gravity
    a_net = np.array([2.0, -1.0, 0.5])
    acc_body = a_net - g_w  # body specific force: a = a_net - g_w
    omega_body = np.array([0.0, 0.0, 0.0])

    state = NominalState(
        p=np.zeros(3),
        v=np.zeros(3),
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        b_g=np.zeros(3),
        b_a=np.zeros(3),
    )

    for _ in range(n_steps):
        state = strapdown_step(state, omega_body, acc_body, dt, g_w)

    total_time = n_steps * dt
    expected_v = a_net * total_time
    expected_p = 0.5 * a_net * (total_time**2)

    np.testing.assert_allclose(state.v, expected_v, atol=1e-10)
    np.testing.assert_allclose(state.p, expected_p, atol=1e-10)


def test_constant_angular_velocity_analytical():
    """Verify constant angular velocity integrates to correct quaternion."""
    dt = 0.005  # 200 Hz
    n_steps = 400  # 2.0 seconds
    omega_z = 0.5  # rad/s around z
    omega_body = np.array([0.0, 0.0, omega_z])
    acc_body = np.array([0.0, 0.0, 9.81])
    g_w = np.array([0.0, 0.0, -9.81])

    state = NominalState(
        p=np.zeros(3),
        v=np.zeros(3),
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        b_g=np.zeros(3),
        b_a=np.zeros(3),
    )

    for _ in range(n_steps):
        state = strapdown_step(state, omega_body, acc_body, dt, g_w)

    total_angle = omega_z * (n_steps * dt)  # 1.0 radian
    expected_q = np.array([np.cos(0.5 * total_angle), 0.0, 0.0, np.sin(0.5 * total_angle)])

    np.testing.assert_allclose(state.q, expected_q, atol=1e-12)


def test_zero_noise_static_imu_60s_drift_upright():
    """Zero-noise static IMU over 60 s drifts < 1e-6 m (upright)."""
    rate = 200.0  # Hz
    duration = 60.0  # seconds
    n_samples = int(duration * rate)  # 12,000 samples
    dt = 1.0 / rate

    g = 9.80665
    g_w = np.array([0.0, 0.0, -g])
    acc = np.array([0.0, 0.0, g])
    omega = np.array([0.0, 0.0, 0.0])

    state = NominalState(
        p=np.zeros(3),
        v=np.zeros(3),
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        b_g=np.zeros(3),
        b_a=np.zeros(3),
    )

    for _ in range(n_samples):
        state = strapdown_step(state, omega, acc, dt, g_w)

    pos_drift = np.linalg.norm(state.p)
    vel_drift = np.linalg.norm(state.v)
    quat_err = np.linalg.norm(state.q - np.array([1.0, 0.0, 0.0, 0.0]))

    assert pos_drift < 1e-6, f"Position drift {pos_drift:.2e} m exceeds 1e-6 m"
    assert vel_drift < 1e-6, f"Velocity drift {vel_drift:.2e} m/s exceeds 1e-6 m/s"
    assert quat_err < 1e-6, f"Attitude drift {quat_err:.2e} exceeds 1e-6"


def test_zero_noise_static_imu_60s_drift_tilted():
    """Zero-noise static IMU over 60 s drifts < 1e-6 m (arbitrary 3D tilt)."""
    rate = 200.0
    duration = 60.0
    n_samples = int(duration * rate)
    dt = 1.0 / rate

    # Arbitrary orientation (e.g. roll=25 deg, pitch=35 deg, yaw=50 deg)
    q_arbitrary = quat_normalize(np.array([0.8, 0.3, -0.4, 0.3]))
    R_world_from_body = quat_to_rot_matrix(q_arbitrary)

    g = 9.80665
    g_w = np.array([0.0, 0.0, -g])

    # Body specific force in static condition: a_body = R^T * (-g_w) = R^T * [0, 0, g]
    acc_body = R_world_from_body.T @ np.array([0.0, 0.0, g])
    omega_body = np.array([0.0, 0.0, 0.0])

    state = NominalState(
        p=np.zeros(3),
        v=np.zeros(3),
        q=q_arbitrary.copy(),
        b_g=np.zeros(3),
        b_a=np.zeros(3),
    )

    for _ in range(n_samples):
        state = strapdown_step(state, omega_body, acc_body, dt, g_w)

    pos_drift = np.linalg.norm(state.p)
    assert pos_drift < 1e-6, f"Tilted static drift {pos_drift:.2e} m exceeds 1e-6 m"


def test_strapdown_propagate_sequence():
    """Verify vectorized/sequence propagation function matches step-by-step."""
    rate = 100.0
    dt = 1.0 / rate
    n_samples = 50
    g_w = np.array([0.0, 0.0, -9.81])

    omega_seq = np.random.normal(0.0, 0.05, size=(n_samples, 3))
    acc_seq = np.random.normal(0.0, 0.1, size=(n_samples, 3))
    acc_seq[:, 2] += 9.81
    dt_seq = np.full(n_samples, dt)

    state_init = NominalState(
        p=np.array([1.0, -1.0, 2.0]),
        v=np.array([0.5, 0.1, -0.2]),
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        b_g=np.array([0.005, -0.002, 0.001]),
        b_a=np.array([0.01, 0.02, -0.01]),
    )

    # Step-by-step
    state_step = state_init.copy()
    for k in range(n_samples):
        state_step = strapdown_step(state_step, omega_seq[k], acc_seq[k], dt_seq[k], g_w)

    # Sequence function
    state_seq, p_hist, v_hist, q_hist = strapdown_propagate_sequence(
        state_init, omega_seq, acc_seq, dt_seq, g_w
    )

    np.testing.assert_allclose(state_seq.p, state_step.p, atol=1e-14)
    np.testing.assert_allclose(state_seq.v, state_step.v, atol=1e-14)
    np.testing.assert_allclose(state_seq.q, state_step.q, atol=1e-14)
    assert len(p_hist) == n_samples
    assert len(v_hist) == n_samples
    assert len(q_hist) == n_samples
