"""Unit tests for ESKF Analytical vs Numerical Jacobians (SPEC §12.4).

Verifies the analytical error-state transition matrix F (15x15) against
finite differences of strapdown nominal propagation under small perturbations:
    delta_x = [delta_theta(0:3), delta_v(3:6), delta_p(6:9), delta_b_g(9:12), delta_b_a(12:15)]

Equations (SPEC §12.4):
    F = I15
    F[0:3, 0:3]   = R(omega_hat * dt)^T
    F[0:3, 9:12]  = -I * dt
    F[3:6, 0:3]   = -R(q_old) * skew(a_hat) * dt
    F[3:6, 12:15] = -R(q_old) * dt
    F[6:9, 3:6]   = I * dt
"""

import numpy as np
import pytest

from src.ref.eskf import build_f_matrix
from src.ref.init import NominalState
from src.ref.mathlib import (
    quat_exp,
    quat_log,
    quat_mult,
    quat_normalize,
    quat_to_rot_matrix,
    skew,
)
from src.ref.strapdown import strapdown_step


def compute_numerical_jacobian(
    state: NominalState,
    omega: np.ndarray,
    acc: np.ndarray,
    dt: float,
    g_w: np.ndarray,
    eps: float = 1e-7,
) -> np.ndarray:
    """Compute 15x15 error-state transition Jacobian F_num by central finite differences.

    Perturbation convention (SPEC §12.4, §12.5, Solà):
        q(delta_theta) = normalize(q (x) exp(delta_theta))
        v(delta_v)     = v + delta_v
        p(delta_p)     = p + delta_p
        b_g(delta_bg)  = b_g + delta_bg
        b_a(delta_ba)  = b_a + delta_ba
    """
    nom_next = strapdown_step(state, omega, acc, dt, g_w)
    F_num = np.zeros((15, 15), dtype=np.float64)

    # Conjugate of unperturbed next quaternion for computing relative orientation error
    q_nom_conj = np.array([nom_next.q[0], -nom_next.q[1], -nom_next.q[2], -nom_next.q[3]], dtype=np.float64)

    for i in range(15):
        pert = np.zeros(15, dtype=np.float64)
        pert[i] = eps

        # Positive perturbation state
        sp = NominalState(
            p=state.p + pert[6:9],
            v=state.v + pert[3:6],
            q=quat_normalize(quat_mult(state.q, quat_exp(pert[0:3]))),
            b_g=state.b_g + pert[9:12],
            b_a=state.b_a + pert[12:15],
        )

        # Negative perturbation state
        sn = NominalState(
            p=state.p - pert[6:9],
            v=state.v - pert[3:6],
            q=quat_normalize(quat_mult(state.q, quat_exp(-pert[0:3]))),
            b_g=state.b_g - pert[9:12],
            b_a=state.b_a - pert[12:15],
        )

        next_p = strapdown_step(sp, omega, acc, dt, g_w)
        next_n = strapdown_step(sn, omega, acc, dt, g_w)

        # Relative error states relative to nominal next state
        # Orientation error: q_p = nom_next.q (x) exp(dq_p) -> q_nom* (x) q_p = exp(dq_p)
        dq_p = quat_log(quat_mult(q_nom_conj, next_p.q))
        dq_n = quat_log(quat_mult(q_nom_conj, next_n.q))

        d_p = np.concatenate([
            dq_p,
            next_p.v - nom_next.v,
            next_p.p - nom_next.p,
            next_p.b_g - nom_next.b_g,
            next_p.b_a - nom_next.b_a,
        ])
        d_n = np.concatenate([
            dq_n,
            next_n.v - nom_next.v,
            next_n.p - nom_next.p,
            next_n.b_g - nom_next.b_g,
            next_n.b_a - nom_next.b_a,
        ])

        F_num[:, i] = (d_p - d_n) / (2.0 * eps)

    return F_num


def test_analytical_f_structure():
    """Verify analytical F matrix dimensions and block structure."""
    state = NominalState(
        p=np.array([1.0, 2.0, 3.0]),
        v=np.array([0.5, -0.2, 0.1]),
        q=quat_exp(np.array([0.1, 0.2, -0.3])),
        b_g=np.array([0.01, -0.02, 0.005]),
        b_a=np.array([0.05, -0.04, 0.03]),
    )
    omega = np.array([0.3, -0.1, 0.4])
    acc = np.array([0.2, 0.8, 9.7])
    dt = 0.005

    F = build_f_matrix(state, omega, acc, dt)

    assert F.shape == (15, 15)
    assert F.dtype == np.float64

    # Diagonal blocks
    R_rot = F[0:3, 0:3]
    # F[0:3, 0:3] should be an orthogonal rotation matrix
    assert np.allclose(R_rot @ R_rot.T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(R_rot), 1.0, atol=1e-12)
    assert np.allclose(F[3:6, 3:6], np.eye(3), atol=1e-14)
    assert np.allclose(F[6:9, 6:9], np.eye(3), atol=1e-14)
    assert np.allclose(F[9:12, 9:12], np.eye(3), atol=1e-14)
    assert np.allclose(F[12:15, 12:15], np.eye(3), atol=1e-14)

    # Off-diagonal blocks
    omega_hat = omega - state.b_g
    a_hat = acc - state.b_a
    R_old = quat_to_rot_matrix(state.q)

    assert np.allclose(F[0:3, 9:12], -np.eye(3) * dt, atol=1e-14)
    assert np.allclose(F[3:6, 0:3], -R_old @ skew(a_hat) * dt, atol=1e-14)
    assert np.allclose(F[3:6, 12:15], -R_old * dt, atol=1e-14)
    assert np.allclose(F[6:9, 3:6], np.eye(3) * dt, atol=1e-14)

    # Zero blocks
    mask_expected_nonzero = np.zeros((15, 15), dtype=bool)
    mask_expected_nonzero[0:3, 0:3] = True
    mask_expected_nonzero[0:3, 9:12] = True
    mask_expected_nonzero[3:6, 0:3] = True
    mask_expected_nonzero[3:6, 3:6] = True
    mask_expected_nonzero[3:6, 12:15] = True
    mask_expected_nonzero[6:9, 3:6] = True
    mask_expected_nonzero[6:9, 6:9] = True
    mask_expected_nonzero[9:12, 9:12] = True
    mask_expected_nonzero[12:15, 12:15] = True

    assert np.all(F[~mask_expected_nonzero] == 0.0)


def test_f_against_finite_differences_small_dt():
    """Verify analytical F matches numerical finite differences as dt -> 0."""
    state = NominalState(
        p=np.array([1.2, -0.4, 3.1]),
        v=np.array([0.8, -1.2, 0.5]),
        q=quat_exp(np.array([0.2, -0.15, 0.4])),
        b_g=np.array([0.02, -0.01, 0.03]),
        b_a=np.array([0.1, -0.05, 0.08]),
    )
    omega = np.array([0.5, -0.3, 0.7])
    acc = np.array([1.1, -0.5, 9.6])
    g_w = np.array([0.0, 0.0, -9.81])

    dt = 1e-4
    F_analytical = build_f_matrix(state, omega, acc, dt)
    F_numerical = compute_numerical_jacobian(state, omega, acc, dt, g_w, eps=1e-7)

    # For dt = 1e-4, analytical matches numerical to within 1e-6 across the entire 15x15 matrix
    max_diff = np.max(np.abs(F_analytical - F_numerical))
    assert max_diff < 1e-6, f"Max diff at dt=1e-4 was {max_diff}"


def test_f_against_finite_differences_nominal_dt():
    """Verify individual analytical blocks match numerical derivatives at nominal 200 Hz (dt=0.005s)."""
    state = NominalState(
        p=np.array([0.5, 1.0, -0.2]),
        v=np.array([0.2, 0.3, -0.1]),
        q=quat_exp(np.array([-0.1, 0.2, 0.05])),
        b_g=np.array([0.015, -0.025, 0.01]),
        b_a=np.array([0.04, -0.02, 0.06]),
    )
    omega = np.array([0.2, -0.4, 0.3])
    acc = np.array([0.8, 0.4, 9.8])
    g_w = np.array([0.0, 0.0, -9.81])

    dt = 0.005
    F_analytical = build_f_matrix(state, omega, acc, dt)
    F_numerical = compute_numerical_jacobian(state, omega, acc, dt, g_w, eps=1e-7)

    # Orientation blocks
    assert np.allclose(F_analytical[0:3, 0:3], F_numerical[0:3, 0:3], atol=1e-7)
    assert np.allclose(F_analytical[0:3, 9:12], F_numerical[0:3, 9:12], atol=1e-4)

    # Velocity blocks
    assert np.allclose(F_analytical[3:6, 0:3], F_numerical[3:6, 0:3], atol=1e-7)
    assert np.allclose(F_analytical[3:6, 3:6], F_numerical[3:6, 3:6], atol=1e-7)
    assert np.allclose(F_analytical[3:6, 12:15], F_numerical[3:6, 12:15], atol=1e-7)

    # Position blocks
    assert np.allclose(F_analytical[6:9, 3:6], F_numerical[6:9, 3:6], atol=1e-7)
    assert np.allclose(F_analytical[6:9, 6:9], F_numerical[6:9, 6:9], atol=1e-10)

    # Bias random walks
    assert np.allclose(F_analytical[9:12, 9:12], F_numerical[9:12, 9:12], atol=1e-10)
    assert np.allclose(F_analytical[12:15, 12:15], F_numerical[12:15, 12:15], atol=1e-10)


def test_f_at_zero_dt():
    """Verify F matrix is exactly identity when dt = 0."""
    state = NominalState(
        p=np.zeros(3),
        v=np.zeros(3),
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        b_g=np.zeros(3),
        b_a=np.zeros(3),
    )
    F = build_f_matrix(state, np.zeros(3), np.zeros(3), dt=0.0)
    assert np.allclose(F, np.eye(15), atol=1e-15)
