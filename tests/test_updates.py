"""Unit tests for Measurement Updates: Tilt update and ZUPT (SPEC §12.5).

Tests:
1. Tilt update gating (| |a| - g_mag | < threshold and |omega| < omega_max).
2. Tilt update gating threshold expansion with degraded quality.
3. Tilt innovation y = (a - b_a)/|a - b_a| - R(q)^T [0,0,1], H = [skew(h_hat), 0...].
4. Tilt measurement covariance R scaling: (sigma_tilt / max(quality, 0.1))^2 I3.
5. Tilt update convergence on synthetic data: start with an intentional 5-degree tilt error,
   run filter with strapdown + tilt updates over 1.0 - 2.0s, confirm tilt error < 0.5 deg.
6. ZUPT stationary detector over sliding window (zupt_window_s = 0.25s):
   mean gyro < 0.03 rad/s, |acc - g_mag| < 0.15 m/s^2, quality >= 0.3.
7. ZUPT disabled when quality < 0.3.
8. ZUPT disabled when gyro motion is present (mean gyro >= 0.03 rad/s).
9. ZUPT disabled when accel deviation exceeds threshold (|acc - g_mag| >= 0.15 m/s^2).
10. ZUPT measurement update: y = -v, H = [0, I, 0, 0, 0], R_zupt = sigma_zupt^2 I3.
11. ZUPT velocity update zeroes out accumulated drift.
12. Streaming stationary ring buffer (StationaryWindowBuffer) behavior.
13. Pure function immutability: inputs are not mutated by updates.
"""

from typing import Tuple
import numpy as np
import pytest

from src.ref.eskf import (
    NominalState,
    build_f_matrix,
    build_q_discrete,
    eskf_predict_covariance,
    load_params,
)
from src.ref.mathlib import (
    quat_exp,
    quat_to_rot_matrix,
    skew,
)
from src.ref.strapdown import strapdown_step
from src.ref.updates import (
    StationaryWindowBuffer,
    build_tilt_h_matrix,
    build_tilt_r_matrix,
    build_zupt_h_matrix,
    build_zupt_r_matrix,
    compute_tilt_innovation,
    tilt_gating,
    tilt_update,
    zupt_detect_stationary,
    zupt_step,
    zupt_update,
)


@pytest.fixture
def default_params():
    return load_params()


@pytest.fixture
def nominal_state_and_cov(default_params) -> Tuple[NominalState, np.ndarray]:
    q0 = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    state = NominalState(
        p=np.zeros(3, dtype=np.float64),
        v=np.zeros(3, dtype=np.float64),
        q=q0,
        b_g=np.zeros(3, dtype=np.float64),
        b_a=np.zeros(3, dtype=np.float64),
    )
    p_diag = np.array([
        default_params["sigma_theta_0"] ** 2] * 3 +
        [default_params["sigma_v_0"] ** 2] * 3 +
        [default_params["sigma_p_0"] ** 2] * 3 +
        [default_params["sigma_bg_0"] ** 2] * 3 +
        [default_params["sigma_ba_0"] ** 2] * 3,
        dtype=np.float64,
    )
    P = np.diag(p_diag)
    return state, P


# ---------------------------------------------------------------------------
# Tilt Update Tests
# ---------------------------------------------------------------------------


def test_tilt_gating_normal(default_params):
    """Tilt gating accepts sample when acc norm is close to g_mag and omega is small."""
    g_mag = 9.80665
    acc = np.array([0.0, 0.0, g_mag], dtype=np.float64)
    omega = np.array([0.05, -0.02, 0.01], dtype=np.float64)
    quality = 1.0

    passed = tilt_gating(acc, omega, g_mag=g_mag, quality=quality, params=default_params)
    assert passed is True


def test_tilt_gating_acc_excessive(default_params):
    """Tilt gating rejects sample when acc norm deviates too far from g_mag."""
    g_mag = 9.80665
    # Deviation 0.4 > tau_a = 0.3 (at quality 1.0)
    acc = np.array([0.0, 0.0, g_mag + 0.4], dtype=np.float64)
    omega = np.array([0.01, 0.0, 0.0], dtype=np.float64)

    passed = tilt_gating(acc, omega, g_mag=g_mag, quality=1.0, params=default_params)
    assert passed is False


def test_tilt_gating_omega_excessive(default_params):
    """Tilt gating rejects sample when omega exceeds omega_max (1.0 rad/s)."""
    g_mag = 9.80665
    acc = np.array([0.0, 0.0, g_mag], dtype=np.float64)
    omega = np.array([0.8, 0.8, 0.0], dtype=np.float64)  # norm = 1.13 > 1.0

    passed = tilt_gating(acc, omega, g_mag=g_mag, quality=1.0, params=default_params)
    assert passed is False


def test_tilt_gating_quality_expansion(default_params):
    """Gating threshold expands with lower quality: tau_eff = tau_a * (1 + 5*(1-quality))."""
    g_mag = 9.80665
    tau_a = float(default_params["tilt_tau_a"])  # 0.3
    # At quality = 0.5: threshold = 0.3 * (1 + 5*0.5) = 0.3 * 3.5 = 1.05
    acc_dev = 0.6  # Rejected at quality=1.0, accepted at quality=0.5
    acc = np.array([0.0, 0.0, g_mag + acc_dev], dtype=np.float64)
    omega = np.array([0.05, 0.0, 0.0], dtype=np.float64)

    assert tilt_gating(acc, omega, g_mag=g_mag, quality=1.0, params=default_params) is False
    assert tilt_gating(acc, omega, g_mag=g_mag, quality=0.5, params=default_params) is True


def test_tilt_innovation_and_h_matrix(nominal_state_and_cov):
    """Test innovation y = z - h_hat and measurement matrix H = [skew(h_hat), 0...]."""
    state, _ = nominal_state_and_cov
    # Add a small accel bias
    state.b_a = np.array([0.1, -0.05, 0.0], dtype=np.float64)

    # Measured accel
    acc = np.array([0.1, -0.05, 9.8], dtype=np.float64)
    # Unbiased accel: [0, 0, 9.8] -> z = [0, 0, 1]
    y, h_hat = compute_tilt_innovation(state, acc)

    expected_z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    R_q = quat_to_rot_matrix(state.q)
    expected_h = R_q.T @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    expected_y = expected_z - expected_h

    assert np.allclose(h_hat, expected_h, atol=1e-12)
    assert np.allclose(y, expected_y, atol=1e-12)

    H = build_tilt_h_matrix(h_hat)
    assert H.shape == (3, 15)
    assert np.allclose(H[:, 0:3], skew(h_hat), atol=1e-12)
    assert np.allclose(H[:, 3:], 0.0, atol=1e-12)


def test_tilt_r_matrix_scaling(default_params):
    """Test R_tilt = (sigma_tilt / max(quality, 0.1))^2 I3."""
    sigma_base = float(default_params["tilt_sigma"])  # 0.02

    # quality = 1.0 -> sigma = 0.02
    R1 = build_tilt_r_matrix(quality=1.0, params=default_params)
    expected_var1 = (sigma_base / 1.0) ** 2
    assert np.allclose(R1, expected_var1 * np.eye(3), atol=1e-12)

    # quality = 0.2 -> sigma = 0.10
    R2 = build_tilt_r_matrix(quality=0.2, params=default_params)
    expected_var2 = (sigma_base / 0.2) ** 2
    assert np.allclose(R2, expected_var2 * np.eye(3), atol=1e-12)

    # quality = 0.02 clamped to 0.1 -> sigma = 0.20
    R3 = build_tilt_r_matrix(quality=0.02, params=default_params)
    expected_var3 = (sigma_base / 0.1) ** 2
    assert np.allclose(R3, expected_var3 * np.eye(3), atol=1e-12)


def test_tilt_update_convergence_pitch_error(default_params):
    """Start with intentional 5-degree pitch error (around Y), confirm convergence < 0.5 deg."""
    # 5 degrees around Y axis
    theta_err = np.radians(5.0)
    q_init = quat_exp(np.array([0.0, theta_err, 0.0], dtype=np.float64))

    state = NominalState(
        p=np.zeros(3, dtype=np.float64),
        v=np.zeros(3, dtype=np.float64),
        q=q_init,
        b_g=np.zeros(3, dtype=np.float64),
        b_a=np.zeros(3, dtype=np.float64),
    )
    p_diag = np.array([
        default_params["sigma_theta_0"] ** 2] * 3 +
        [default_params["sigma_v_0"] ** 2] * 3 +
        [default_params["sigma_p_0"] ** 2] * 3 +
        [default_params["sigma_bg_0"] ** 2] * 3 +
        [default_params["sigma_ba_0"] ** 2] * 3,
        dtype=np.float64,
    )
    P = np.diag(p_diag)

    g_mag = 9.80665
    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)
    dt = 0.005  # 200 Hz
    num_steps = 200  # 1.0 second

    # True orientation is identity [1, 0, 0, 0]; static sensor measures [0, 0, g_mag]
    acc = np.array([0.0, 0.0, g_mag], dtype=np.float64)
    omega = np.zeros(3, dtype=np.float64)

    for _ in range(num_steps):
        # 1. Strapdown and ESKF predict
        state = strapdown_step(state, omega, acc, dt, g_w)
        F = build_f_matrix(state, omega, acc, dt)
        Q = build_q_discrete(dt, default_params)
        P = eskf_predict_covariance(P, F, Q)

        # 2. Tilt update
        state, P, _, updated = tilt_update(
            state, P, acc, omega, g_mag=g_mag, quality=1.0, params=default_params
        )
        assert updated is True

    # Compute final tilt error angle (angle between R(q)^T [0,0,1] and [0,0,1])
    R_final = quat_to_rot_matrix(state.q)
    h_final = R_final.T @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    cos_angle = np.clip(np.dot(h_final, np.array([0.0, 0.0, 1.0])), -1.0, 1.0)
    final_tilt_err_deg = np.degrees(np.arccos(cos_angle))

    assert final_tilt_err_deg < 0.5, f"Final tilt error {final_tilt_err_deg:.4f} deg >= 0.5 deg"


def test_tilt_update_convergence_roll_error(default_params):
    """Start with intentional 5-degree roll error (around X), confirm convergence < 0.5 deg."""
    theta_err = np.radians(5.0)
    q_init = quat_exp(np.array([theta_err, 0.0, 0.0], dtype=np.float64))

    state = NominalState(
        p=np.zeros(3, dtype=np.float64),
        v=np.zeros(3, dtype=np.float64),
        q=q_init,
        b_g=np.zeros(3, dtype=np.float64),
        b_a=np.zeros(3, dtype=np.float64),
    )
    p_diag = np.array([
        default_params["sigma_theta_0"] ** 2] * 3 +
        [default_params["sigma_v_0"] ** 2] * 3 +
        [default_params["sigma_p_0"] ** 2] * 3 +
        [default_params["sigma_bg_0"] ** 2] * 3 +
        [default_params["sigma_ba_0"] ** 2] * 3,
        dtype=np.float64,
    )
    P = np.diag(p_diag)

    g_mag = 9.80665
    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)
    dt = 0.005  # 200 Hz
    num_steps = 200  # 1.0 second

    acc = np.array([0.0, 0.0, g_mag], dtype=np.float64)
    omega = np.zeros(3, dtype=np.float64)

    for _ in range(num_steps):
        state = strapdown_step(state, omega, acc, dt, g_w)
        F = build_f_matrix(state, omega, acc, dt)
        Q = build_q_discrete(dt, default_params)
        P = eskf_predict_covariance(P, F, Q)

        state, P, _, updated = tilt_update(
            state, P, acc, omega, g_mag=g_mag, quality=1.0, params=default_params
        )
        assert updated is True

    R_final = quat_to_rot_matrix(state.q)
    h_final = R_final.T @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    cos_angle = np.clip(np.dot(h_final, np.array([0.0, 0.0, 1.0])), -1.0, 1.0)
    final_tilt_err_deg = np.degrees(np.arccos(cos_angle))

    assert final_tilt_err_deg < 0.5, f"Final tilt error {final_tilt_err_deg:.4f} deg >= 0.5 deg"


def test_tilt_update_convergence_combined_error(default_params):
    """Start with combined 5-degree tilt error (both roll and pitch), verify convergence < 0.5 deg."""
    from tools.synth import generate_trajectory

    # Generate 1.5s of static synthetic trajectory at 200 Hz
    traj = generate_trajectory("static", duration=1.5, rate=200.0)
    acc_data = traj["acc"]
    omega_data = traj["omega"]
    n_samples = len(acc_data)

    # 5 degree rotation around axis [1, 1, 0] / sqrt(2)
    axis = np.array([1.0, 1.0, 0.0], dtype=np.float64)
    axis /= np.linalg.norm(axis)
    theta_err = np.radians(5.0)
    q_init = quat_exp(axis * theta_err)

    state = NominalState(
        p=np.zeros(3, dtype=np.float64),
        v=np.zeros(3, dtype=np.float64),
        q=q_init,
        b_g=np.zeros(3, dtype=np.float64),
        b_a=np.zeros(3, dtype=np.float64),
    )
    p_diag = np.array([
        default_params["sigma_theta_0"] ** 2] * 3 +
        [default_params["sigma_v_0"] ** 2] * 3 +
        [default_params["sigma_p_0"] ** 2] * 3 +
        [default_params["sigma_bg_0"] ** 2] * 3 +
        [default_params["sigma_ba_0"] ** 2] * 3,
        dtype=np.float64,
    )
    P = np.diag(p_diag)

    g_mag = 9.80665
    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)
    dt = 0.005

    for k in range(n_samples):
        acc = acc_data[k]
        omega = omega_data[k]
        state = strapdown_step(state, omega, acc, dt, g_w)
        F = build_f_matrix(state, omega, acc, dt)
        Q = build_q_discrete(dt, default_params)
        P = eskf_predict_covariance(P, F, Q)

        state, P, _, updated = tilt_update(
            state, P, acc, omega, g_mag=g_mag, quality=1.0, params=default_params
        )
        assert updated is True

    R_final = quat_to_rot_matrix(state.q)
    h_final = R_final.T @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    cos_angle = np.clip(np.dot(h_final, np.array([0.0, 0.0, 1.0])), -1.0, 1.0)
    final_tilt_err_deg = np.degrees(np.arccos(cos_angle))

    assert final_tilt_err_deg < 0.5, f"Combined tilt error {final_tilt_err_deg:.4f} deg >= 0.5 deg"


def test_tilt_update_skipped_when_gating_fails(nominal_state_and_cov, default_params):
    """Verify tilt_update returns updated=False and preserves state and P when gating fails."""
    state, P = nominal_state_and_cov
    acc_excessive = np.array([0.0, 0.0, 15.0], dtype=np.float64)
    omega = np.zeros(3, dtype=np.float64)

    new_state, new_P, dx, updated = tilt_update(
        state, P, acc_excessive, omega, g_mag=9.80665, quality=1.0, params=default_params
    )
    assert updated is False
    assert np.allclose(dx, 0.0)
    assert np.allclose(new_state.q, state.q)
    assert np.allclose(new_P, P)


# ---------------------------------------------------------------------------
# ZUPT Tests
# ---------------------------------------------------------------------------


def test_zupt_detector_stationary(default_params):
    """ZUPT detector recognizes stationary window (0.25s at 200 Hz = 50 samples)."""
    n_samples = 50
    g_mag = 9.80665
    gyro_window = np.zeros((n_samples, 3), dtype=np.float64)
    acc_window = np.tile(np.array([0.0, 0.0, g_mag]), (n_samples, 1))

    is_stat = zupt_detect_stationary(
        gyro_window, acc_window, g_mag=g_mag, quality=1.0, params=default_params
    )
    assert is_stat is True


def test_zupt_detector_fails_on_gyro_motion(default_params):
    """ZUPT detector rejects window when mean gyro norm exceeds 0.03 rad/s."""
    n_samples = 50
    g_mag = 9.80665
    # mean gyro 0.05 > 0.03 rad/s
    gyro_window = np.full((n_samples, 3), 0.05 / np.sqrt(3), dtype=np.float64)
    acc_window = np.tile(np.array([0.0, 0.0, g_mag]), (n_samples, 1))

    is_stat = zupt_detect_stationary(
        gyro_window, acc_window, g_mag=g_mag, quality=1.0, params=default_params
    )
    assert is_stat is False


def test_zupt_detector_fails_on_acc_deviation(default_params):
    """ZUPT detector rejects window when acc norm deviation from g_mag exceeds 0.15 m/s^2."""
    n_samples = 50
    g_mag = 9.80665
    gyro_window = np.zeros((n_samples, 3), dtype=np.float64)
    # acc deviation 0.20 > 0.15
    acc_window = np.tile(np.array([0.0, 0.0, g_mag + 0.20]), (n_samples, 1))

    is_stat = zupt_detect_stationary(
        gyro_window, acc_window, g_mag=g_mag, quality=1.0, params=default_params
    )
    assert is_stat is False


def test_zupt_detector_fails_on_acc_variance(default_params):
    """ZUPT detector rejects window when acc variance exceeds threshold even if mean matches g_mag."""
    n_samples = 50
    g_mag = 9.80665
    gyro_window = np.zeros((n_samples, 3), dtype=np.float64)
    # Alternating +/- 0.3 m/s^2: mean deviation is 0, but variance is 0.09 > (0.15)^2 = 0.0225
    acc_window = np.zeros((n_samples, 3), dtype=np.float64)
    for i in range(n_samples):
        dev = 0.3 if i % 2 == 0 else -0.3
        acc_window[i, 2] = g_mag + dev

    is_stat = zupt_detect_stationary(
        gyro_window, acc_window, g_mag=g_mag, quality=1.0, params=default_params
    )
    assert is_stat is False


def test_zupt_detector_disabled_on_low_quality(default_params):
    """ZUPT is disabled when monitor quality < 0.3 (SPEC §12.5)."""
    n_samples = 50
    g_mag = 9.80665
    gyro_window = np.zeros((n_samples, 3), dtype=np.float64)
    acc_window = np.tile(np.array([0.0, 0.0, g_mag]), (n_samples, 1))

    # quality = 0.29 < 0.3 -> disabled
    assert zupt_detect_stationary(
        gyro_window, acc_window, g_mag=g_mag, quality=0.29, params=default_params
    ) is False

    # quality = 0.35 >= 0.3 -> enabled
    assert zupt_detect_stationary(
        gyro_window, acc_window, g_mag=g_mag, quality=0.35, params=default_params
    ) is True


def test_zupt_h_matrix_and_innovation(nominal_state_and_cov, default_params):
    """Test ZUPT measurement matrix H = [0, I, 0, 0, 0] and innovation y = -v."""
    state, P = nominal_state_and_cov
    state.v = np.array([0.5, -0.2, 0.1], dtype=np.float64)

    H = build_zupt_h_matrix()
    assert H.shape == (3, 15)
    assert np.allclose(H[:, 0:3], 0.0)
    assert np.allclose(H[:, 3:6], np.eye(3))
    assert np.allclose(H[:, 6:15], 0.0)

    R = build_zupt_r_matrix(default_params)
    assert R.shape == (3, 3)
    assert np.allclose(R, (default_params["zupt_sigma"] ** 2) * np.eye(3))


def test_zupt_update_zeroes_velocity_drift(nominal_state_and_cov, default_params):
    """Verify ZUPT measurement update successfully zeroes out accumulated velocity drift."""
    state, P = nominal_state_and_cov
    initial_drift = np.array([0.8, -0.5, 0.3], dtype=np.float64)
    state.v = initial_drift.copy()

    # Covariance on velocity reflects accumulated drift
    P[3:6, 3:6] = (0.5 ** 2) * np.eye(3, dtype=np.float64)

    # In 2 ZUPT updates, velocity is reduced by >99%
    for _ in range(2):
        state, P, dx = zupt_update(state, P, params=default_params)

    final_v_norm = np.linalg.norm(state.v)
    assert final_v_norm < 0.01, f"ZUPT failed to zero drift: final v_norm = {final_v_norm}"


def test_zupt_step_conditional(nominal_state_and_cov, default_params):
    """Test zupt_step only applies update when stationary is detected."""
    state, P = nominal_state_and_cov
    state.v = np.array([0.4, 0.4, 0.0], dtype=np.float64)

    # When moving (e.g. gyro = 0.1), zupt_step should skip
    g_mag = 9.80665
    n_samples = 50
    gyro_moving = np.full((n_samples, 3), 0.1, dtype=np.float64)
    acc_stationary = np.tile(np.array([0.0, 0.0, g_mag]), (n_samples, 1))

    st_after, P_after, _, applied = zupt_step(
        state, P, gyro_moving, acc_stationary, g_mag=g_mag, quality=1.0, params=default_params
    )
    assert applied is False
    assert np.allclose(st_after.v, state.v)

    # When stationary, zupt_step applies
    gyro_stat = np.zeros((n_samples, 3), dtype=np.float64)
    st_after2, P_after2, _, applied2 = zupt_step(
        state, P, gyro_stat, acc_stationary, g_mag=g_mag, quality=1.0, params=default_params
    )
    assert applied2 is True
    assert np.linalg.norm(st_after2.v) < np.linalg.norm(state.v)


def test_stationary_window_buffer(default_params):
    """Test fixed-size ring buffer for streaming stationary detection."""
    buf = StationaryWindowBuffer(capacity=64)
    g_mag = 9.80665

    # Buffer starts not stationary (not enough samples)
    assert buf.is_stationary(g_mag=g_mag, quality=1.0, params=default_params) is False

    # Feed 50 stationary samples at 200 Hz (dt = 0.005s)
    t = 0.0
    for _ in range(50):
        buf.add_sample(t, np.zeros(3), np.array([0.0, 0.0, g_mag]))
        t += 0.005

    assert buf.count == 50
    assert buf.is_stationary(g_mag=g_mag, quality=1.0, params=default_params) is True

    # Feed 10 moving samples with gyro = 0.2 rad/s: buffer mean gyro exceeds threshold
    for _ in range(10):
        buf.add_sample(t, np.array([0.2, 0.0, 0.0]), np.array([0.0, 0.0, g_mag]))
        t += 0.005
    assert buf.is_stationary(g_mag=g_mag, quality=1.0, params=default_params) is False


def test_pure_functions_immutability(nominal_state_and_cov, default_params):
    """Verify input NominalState and covariance P are NOT mutated in-place by updates."""
    state, P = nominal_state_and_cov
    state_v_orig = state.v.copy()
    P_orig = P.copy()

    acc = np.array([0.0, 0.0, 9.80665], dtype=np.float64)
    omega = np.zeros(3, dtype=np.float64)

    new_state, new_P, _, _ = tilt_update(
        state, P, acc, omega, g_mag=9.80665, quality=1.0, params=default_params
    )

    assert np.allclose(state.v, state_v_orig)
    assert np.allclose(P, P_orig)
