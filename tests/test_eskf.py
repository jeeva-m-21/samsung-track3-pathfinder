"""Unit tests for 15-state ESKF Covariance Propagation and Joseph Update (SPEC §12.4, §12.5).

Tests:
1. Discrete Q construction: block diagonal, scaling with dt and kappa.
2. Covariance conditioning: symmetrization and diagonal clamping [cov_diag_floor, cov_diag_cap].
3. Single-step covariance propagation: P_pred = F P F^T + Q.
4. Long-horizon numerical stability: P remains symmetric and positive-definite after 10,000 steps.
5. Closed-form 3x3 matrix inverse helper.
6. Generic Joseph-form measurement update:
       S = H P H^T + R
       K = P H^T S^{-1}
       P = (I - KH) P (I - KH)^T + K R K^T
7. State injection: nominal state update (q, v, p, b_g, b_a) with unit quaternion normalization.
8. Error state reset: covariance reset P <- G P G^T with G = I, G[0:3, 0:3] = I - skew(delta_theta / 2).
9. Full measurement update step combining Joseph update, injection, and reset.
"""

import numpy as np
import pytest

from src.ref.eskf import (
    build_f_matrix,
    build_q_discrete,
    condition_covariance,
    eskf_inject_nominal_state,
    eskf_joseph_update,
    eskf_measurement_update,
    eskf_predict_covariance,
    eskf_reset_covariance,
    invert_3x3,
)
from src.ref.init import NominalState, load_params
from src.ref.mathlib import quat_exp, quat_mult, quat_to_rot_matrix, skew


@pytest.fixture
def default_params():
    return load_params()


@pytest.fixture
def initial_state_and_cov(default_params):
    q0 = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    state = NominalState(
        p=np.zeros(3, dtype=np.float64),
        v=np.zeros(3, dtype=np.float64),
        q=q0,
        b_g=np.zeros(3, dtype=np.float64),
        b_a=np.zeros(3, dtype=np.float64),
    )
    p_diag = np.array([
        default_params["sigma_theta_0"] ** 2, default_params["sigma_theta_0"] ** 2, default_params["sigma_theta_0"] ** 2,
        default_params["sigma_v_0"] ** 2, default_params["sigma_v_0"] ** 2, default_params["sigma_v_0"] ** 2,
        default_params["sigma_p_0"] ** 2, default_params["sigma_p_0"] ** 2, default_params["sigma_p_0"] ** 2,
        default_params["sigma_bg_0"] ** 2, default_params["sigma_bg_0"] ** 2, default_params["sigma_bg_0"] ** 2,
        default_params["sigma_ba_0"] ** 2, default_params["sigma_ba_0"] ** 2, default_params["sigma_ba_0"] ** 2,
    ], dtype=np.float64)
    P = np.diag(p_diag)
    return state, P


def test_discrete_q_construction(default_params):
    """Test discrete Q matrix structure, scaling with dt and kappa."""
    dt = 0.005
    kappa = 1.5
    Q = build_q_discrete(dt, default_params, kappa=kappa)

    assert Q.shape == (15, 15)
    assert Q.dtype == np.float64
    assert np.allclose(Q, Q.T, atol=1e-15)

    # Compute expected block standard deviations
    sigma_g = default_params["sigma_g_base"] * default_params["noise_mult_gyro"] * kappa
    sigma_a = default_params["sigma_a_base"] * default_params["noise_mult_acc"] * kappa
    sigma_bg = default_params["sigma_bg_base"] * default_params["walk_mult_bg"] * kappa
    sigma_ba = default_params["sigma_ba_base"] * default_params["walk_mult_ba"] * kappa

    expected_q_theta = (sigma_g ** 2) * dt * np.eye(3)
    expected_q_v = (sigma_a ** 2) * dt * np.eye(3)
    expected_q_p = np.zeros((3, 3))
    expected_q_bg = (sigma_bg ** 2) * dt * np.eye(3)
    expected_q_ba = (sigma_ba ** 2) * dt * np.eye(3)

    assert np.allclose(Q[0:3, 0:3], expected_q_theta, atol=1e-15)
    assert np.allclose(Q[3:6, 3:6], expected_q_v, atol=1e-15)
    assert np.allclose(Q[6:9, 6:9], expected_q_p, atol=1e-15)
    assert np.allclose(Q[9:12, 9:12], expected_q_bg, atol=1e-15)
    assert np.allclose(Q[12:15, 12:15], expected_q_ba, atol=1e-15)

    # Zero dt produces all-zero Q
    Q_zero = build_q_discrete(0.0, default_params)
    assert np.allclose(Q_zero, np.zeros((15, 15)), atol=1e-15)


def test_covariance_symmetrization_and_clamping(default_params):
    """Test safe covariance conditioning: symmetrization and diagonal clamping."""
    floor_val = default_params["cov_diag_floor"]  # 1e-12
    cap_val = default_params["cov_diag_cap"]      # 1e6

    # Create an asymmetric matrix with diagonal violating bounds
    P_raw = np.eye(15, dtype=np.float64)
    P_raw[0, 1] = 0.5
    P_raw[1, 0] = 0.1  # Asymmetric!
    P_raw[0, 0] = 1e-16  # Below floor
    P_raw[1, 1] = -5.0   # Negative
    P_raw[2, 2] = 1e8    # Above cap

    P_conditioned = condition_covariance(P_raw, diag_floor=floor_val, diag_cap=cap_val)

    # Must be strictly symmetric
    assert np.allclose(P_conditioned, P_conditioned.T, atol=1e-15)
    assert np.isclose(P_conditioned[0, 1], 0.3, atol=1e-15)
    assert np.isclose(P_conditioned[1, 0], 0.3, atol=1e-15)

    # Clamping checks
    assert np.isclose(P_conditioned[0, 0], floor_val, atol=1e-15)
    assert np.isclose(P_conditioned[1, 1], floor_val, atol=1e-15)
    assert np.isclose(P_conditioned[2, 2], cap_val, atol=1e-15)


def test_covariance_propagation_step(default_params, initial_state_and_cov):
    """Test single-step covariance propagation P_pred = F P F^T + Q."""
    state, P = initial_state_and_cov
    dt = 0.005
    omega = np.array([0.1, -0.2, 0.3])
    acc = np.array([0.5, 0.2, 9.8])

    F = build_f_matrix(state, omega, acc, dt)
    Q = build_q_discrete(dt, default_params)

    P_pred = eskf_predict_covariance(P, F, Q, diag_floor=default_params["cov_diag_floor"], diag_cap=default_params["cov_diag_cap"])

    # Manual calculation
    P_expected = condition_covariance(F @ P @ F.T + Q, default_params["cov_diag_floor"], default_params["cov_diag_cap"])

    assert np.allclose(P_pred, P_expected, atol=1e-14)
    assert np.allclose(P_pred, P_pred.T, atol=1e-14)


def test_p_remains_symmetric_and_positive_definite_10000_steps(default_params, initial_state_and_cov):
    """Verify that P remains strictly symmetric and positive definite after 10,000 steps."""
    state, P = initial_state_and_cov
    dt = 0.005  # 200 Hz -> 50 seconds of propagation

    omega = np.array([0.05, -0.08, 0.12])
    acc = np.array([0.3, -0.4, 9.81])

    diag_floor = default_params["cov_diag_floor"]
    diag_cap = default_params["cov_diag_cap"]

    for step in range(10000):
        F = build_f_matrix(state, omega, acc, dt)
        Q = build_q_discrete(dt, default_params)
        P = eskf_predict_covariance(P, F, Q, diag_floor=diag_floor, diag_cap=diag_cap)

    # 1. Symmetry check
    asym = np.max(np.abs(P - P.T))
    assert asym < 1e-12, f"Asymmetry after 10,000 steps was {asym}"

    # 2. Diagonal bounds check
    diag_vals = np.diag(P)
    assert np.all(diag_vals >= diag_floor)
    assert np.all(diag_vals <= diag_cap)

    # 3. Positive definiteness: eigenvalues strictly positive
    eigvals = np.linalg.eigvalsh(P)
    assert np.all(eigvals > 0), f"Minimum eigenvalue: {np.min(eigvals)}"

    # 4. Cholesky decomposition succeeds without error
    L = np.linalg.cholesky(P)
    assert L.shape == (15, 15)


def test_invert_3x3():
    """Test closed-form 3x3 matrix inverse helper against numpy.linalg.inv."""
    # Identity
    assert np.allclose(invert_3x3(np.eye(3)), np.eye(3), atol=1e-14)

    # Diagonal
    D = np.diag([2.0, 4.0, 8.0])
    assert np.allclose(invert_3x3(D), np.diag([0.5, 0.25, 0.125]), atol=1e-14)

    # Random positive-definite matrix
    np.random.seed(42)
    A = np.random.randn(3, 3)
    M = A @ A.T + np.eye(3) * 0.1
    inv_closed = invert_3x3(M)
    inv_np = np.linalg.inv(M)
    assert np.allclose(inv_closed, inv_np, atol=1e-12)

    # Singular / near-singular matrix does not blow up into NaN or Inf
    singular = np.array([
        [1.0, 2.0, 3.0],
        [2.0, 4.0, 6.0],
        [1.0, 1.0, 1.0],
    ])
    inv_sing = invert_3x3(singular, det_floor=1e-12)
    assert np.all(np.isfinite(inv_sing))


def test_generic_joseph_update(default_params, initial_state_and_cov):
    """Test generic Joseph-form measurement update equations and covariance conditioning."""
    _, P = initial_state_and_cov

    # Set up velocity observation: y = -v, H has identity at [3:6, 3:6]
    H = np.zeros((3, 15), dtype=np.float64)
    H[0:3, 3:6] = np.eye(3)

    y = np.array([0.15, -0.08, 0.05], dtype=np.float64)
    sigma_v = 0.05
    R = (sigma_v ** 2) * np.eye(3, dtype=np.float64)

    delta_x, P_post, S = eskf_joseph_update(
        P, H, y, R,
        diag_floor=default_params["cov_diag_floor"],
        diag_cap=default_params["cov_diag_cap"],
    )

    assert delta_x.shape == (15,)
    assert P_post.shape == (15, 15)
    assert S.shape == (3, 3)

    # Check S = H P H^T + R
    S_expected = H @ P @ H.T + R
    assert np.allclose(S, S_expected, atol=1e-14)

    # Check Kalman gain: K = P H^T S^{-1}
    K_expected = P @ H.T @ invert_3x3(S)
    delta_x_expected = K_expected @ y
    assert np.allclose(delta_x, delta_x_expected, atol=1e-12)

    # Check Joseph form covariance: P_post = (I - KH) P (I - KH)^T + K R K^T
    I_KH = np.eye(15) - K_expected @ H
    P_joseph_expected = condition_covariance(
        I_KH @ P @ I_KH.T + K_expected @ R @ K_expected.T,
        default_params["cov_diag_floor"],
        default_params["cov_diag_cap"],
    )
    assert np.allclose(P_post, P_joseph_expected, atol=1e-12)

    # Check covariance reduction on observed states (velocity block)
    assert np.trace(P_post[3:6, 3:6]) < np.trace(P[3:6, 3:6])

    # Check symmetry and positive definiteness
    assert np.allclose(P_post, P_post.T, atol=1e-14)
    assert np.all(np.linalg.eigvalsh(P_post) > 0)


def test_state_injection():
    """Test injection of error state delta_x into NominalState (SPEC §12.5)."""
    q0 = quat_exp(np.array([0.1, -0.2, 0.3]))
    state = NominalState(
        p=np.array([1.0, 2.0, 3.0]),
        v=np.array([0.5, -0.4, 0.2]),
        q=q0,
        b_g=np.array([0.01, -0.02, 0.03]),
        b_a=np.array([0.04, -0.05, 0.06]),
    )

    delta_theta = np.array([0.02, -0.01, 0.03])
    delta_v = np.array([0.1, 0.2, -0.3])
    delta_p = np.array([0.4, -0.2, 0.1])
    delta_bg = np.array([1e-3, -2e-3, 3e-3])
    delta_ba = np.array([5e-3, -4e-3, 2e-3])

    delta_x = np.concatenate([delta_theta, delta_v, delta_p, delta_bg, delta_ba])

    new_state = eskf_inject_nominal_state(state, delta_x)

    # Check quaternion update: q <- normalize(q (x) exp(delta_theta))
    expected_q = quat_mult(state.q, quat_exp(delta_theta))
    expected_q = expected_q / np.linalg.norm(expected_q)
    assert np.isclose(np.linalg.norm(new_state.q), 1.0, atol=1e-12)
    assert np.allclose(new_state.q, expected_q, atol=1e-12)

    # Check vector updates
    assert np.allclose(new_state.v, state.v + delta_v, atol=1e-14)
    assert np.allclose(new_state.p, state.p + delta_p, atol=1e-14)
    assert np.allclose(new_state.b_g, state.b_g + delta_bg, atol=1e-14)
    assert np.allclose(new_state.b_a, state.b_a + delta_ba, atol=1e-14)

    # Verify original state was not mutated (purity)
    assert np.allclose(state.v, np.array([0.5, -0.4, 0.2]), atol=1e-14)


def test_error_state_reset(default_params, initial_state_and_cov):
    """Test error state covariance reset P <- G P G^T (SPEC §12.5)."""
    _, P = initial_state_and_cov
    delta_theta = np.array([0.03, -0.02, 0.04])

    P_reset = eskf_reset_covariance(
        P, delta_theta,
        diag_floor=default_params["cov_diag_floor"],
        diag_cap=default_params["cov_diag_cap"],
    )

    # Analytical G matrix
    G = np.eye(15, dtype=np.float64)
    G[0:3, 0:3] = np.eye(3) - skew(0.5 * delta_theta)
    P_expected = condition_covariance(
        G @ P @ G.T,
        default_params["cov_diag_floor"],
        default_params["cov_diag_cap"],
    )

    assert np.allclose(P_reset, P_expected, atol=1e-14)
    assert np.allclose(P_reset, P_reset.T, atol=1e-14)
    assert np.all(np.linalg.eigvalsh(P_reset) > 0)


def test_full_measurement_update_step(default_params, initial_state_and_cov):
    """Test full eskf_measurement_update step combining Joseph update, injection, and reset."""
    state, P = initial_state_and_cov

    H = np.zeros((3, 15), dtype=np.float64)
    H[0:3, 3:6] = np.eye(3)
    y = np.array([0.1, -0.2, 0.3])
    R = 0.0025 * np.eye(3)

    new_state, new_P, delta_x = eskf_measurement_update(
        state, P, H, y, R, default_params, apply_reset=True
    )

    assert isinstance(new_state, NominalState)
    assert new_P.shape == (15, 15)
    assert delta_x.shape == (15,)

    # State should have been adjusted
    assert not np.allclose(new_state.v, state.v)
    # Covariance should remain valid and symmetric
    assert np.allclose(new_P, new_P.T, atol=1e-14)
    assert np.all(np.linalg.eigvalsh(new_P) > 0)
