import numpy as np
import pytest

from src.ref.mathlib import (
    skew,
    quat_mult,
    quat_conj,
    quat_normalize,
    quat_exp,
    quat_log,
    quat_to_rot_matrix,
    rot_matrix_to_quat,
    quat_sign_continuity,
)


def test_skew():
    v = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    u = np.array([4.0, 5.0, 6.0], dtype=np.float64)

    S = skew(v)
    assert S.shape == (3, 3)

    # Antisymmetry
    np.testing.assert_allclose(S.T, -S, atol=1e-15)

    # Cross product equivalence: skew(v) @ u == v x u
    expected_cross = np.cross(v, u)
    actual_cross = S @ u
    np.testing.assert_allclose(actual_cross, expected_cross, atol=1e-15)


def test_quat_mult_hamilton():
    # Identity
    q_id = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    q = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float64)
    np.testing.assert_allclose(quat_mult(q, q_id), q, atol=1e-15)
    np.testing.assert_allclose(quat_mult(q_id, q), q, atol=1e-15)

    # Conjugate inverse for unit quaternion
    q_c = quat_conj(q)
    np.testing.assert_allclose(quat_mult(q, q_c), q_id, atol=1e-15)

    # Standard Hamilton basis rules: i^2 = j^2 = k^2 = ijk = -1, i*j = k
    qi = np.array([0.0, 1.0, 0.0, 0.0])
    qj = np.array([0.0, 0.0, 1.0, 0.0])
    qk = np.array([0.0, 0.0, 0.0, 1.0])

    np.testing.assert_allclose(quat_mult(qi, qj), qk, atol=1e-15)
    np.testing.assert_allclose(quat_mult(qj, qk), qi, atol=1e-15)
    np.testing.assert_allclose(quat_mult(qk, qi), qj, atol=1e-15)
    np.testing.assert_allclose(quat_mult(qi, qi), -q_id, atol=1e-15)


def test_quat_exp_and_log():
    # Zero vector
    phi_zero = np.zeros(3, dtype=np.float64)
    np.testing.assert_allclose(quat_exp(phi_zero), [1.0, 0.0, 0.0, 0.0], atol=1e-15)

    # Near-zero vector < 1e-8
    phi_tiny = np.array([1e-9, 2e-9, -1e-9], dtype=np.float64)
    q_tiny = quat_exp(phi_tiny)
    assert np.all(np.isfinite(q_tiny))
    np.testing.assert_allclose(np.linalg.norm(q_tiny), 1.0, atol=1e-15)

    # 90 degrees around z
    phi_z = np.array([0.0, 0.0, np.pi / 2.0], dtype=np.float64)
    q_z = quat_exp(phi_z)
    expected_q_z = np.array([np.cos(np.pi / 4.0), 0.0, 0.0, np.sin(np.pi / 4.0)])
    np.testing.assert_allclose(q_z, expected_q_z, atol=1e-15)

    # Exp - Log roundtrip
    phi = np.array([0.2, -0.3, 0.4], dtype=np.float64)
    q = quat_exp(phi)
    phi_rec = quat_log(q)
    np.testing.assert_allclose(phi_rec, phi, atol=1e-12)


def test_rot_matrix_cookbook_spec():
    # 90° about z gives q=[0.70710678,0,0,0.70710678]
    val = np.sqrt(0.5)
    q_90z = np.array([val, 0.0, 0.0, val], dtype=np.float64)
    R = quat_to_rot_matrix(q_90z)

    # R^T R = I
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-14)
    assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-14)

    # R(q) v equals q (x) [0, v] (x) q*
    v = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    Rv = R @ v

    qv = np.array([0.0, v[0], v[1], v[2]])
    q_rot_v = quat_mult(quat_mult(q_90z, qv), quat_conj(q_90z))
    np.testing.assert_allclose(Rv, q_rot_v[1:], atol=1e-14)

    # Roundtrip R to quat
    q_rec = rot_matrix_to_quat(R)
    # Check orientation equivalence (q or -q)
    sign = np.sign(np.dot(q_rec, q_90z))
    np.testing.assert_allclose(q_rec * sign, q_90z, atol=1e-14)


def test_quat_sign_continuity():
    q_prev = np.array([1.0, 0.0, 0.0, 0.0])
    q_flipped = np.array([-0.999, 0.01, 0.0, 0.0])

    q_cont = quat_sign_continuity(q_flipped, q_prev)
    assert np.dot(q_cont, q_prev) > 0.0
    np.testing.assert_allclose(q_cont, -q_flipped, atol=1e-15)


def test_quat_normalize_floor():
    # Extremely small quaternion
    q_tiny = np.array([1e-15, 0.0, 0.0, 0.0])
    q_norm = quat_normalize(q_tiny)
    assert np.all(np.isfinite(q_norm))
    assert np.linalg.norm(q_norm) > 0.0


def test_rot_matrix_shepperd_branches():
    # Branch 1: trace > 0 (already tested with 90 deg z, but also identity)
    R_id = np.eye(3, dtype=np.float64)
    q_id = rot_matrix_to_quat(R_id)
    np.testing.assert_allclose(q_id, [1.0, 0.0, 0.0, 0.0], atol=1e-14)

    # Branch 2: r00 dominant (180 deg rotation about x)
    # R_x(pi) = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    R_x180 = np.diag([1.0, -1.0, -1.0])
    q_x180 = rot_matrix_to_quat(R_x180)
    # Equivalent to q = [0, 1, 0, 0] or [0, -1, 0, 0]
    assert np.abs(q_x180[1]) == pytest.approx(1.0, abs=1e-14)
    assert np.abs(q_x180[0]) == pytest.approx(0.0, abs=1e-14)
    np.testing.assert_allclose(quat_to_rot_matrix(q_x180), R_x180, atol=1e-14)

    # Branch 3: r11 dominant (180 deg rotation about y)
    # R_y(pi) = [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]
    R_y180 = np.diag([-1.0, 1.0, -1.0])
    q_y180 = rot_matrix_to_quat(R_y180)
    assert np.abs(q_y180[2]) == pytest.approx(1.0, abs=1e-14)
    assert np.abs(q_y180[0]) == pytest.approx(0.0, abs=1e-14)
    np.testing.assert_allclose(quat_to_rot_matrix(q_y180), R_y180, atol=1e-14)

    # Branch 4: r22 dominant (180 deg rotation about z)
    # R_z(pi) = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]
    R_z180 = np.diag([-1.0, -1.0, 1.0])
    q_z180 = rot_matrix_to_quat(R_z180)
    assert np.abs(q_z180[3]) == pytest.approx(1.0, abs=1e-14)
    assert np.abs(q_z180[0]) == pytest.approx(0.0, abs=1e-14)
    np.testing.assert_allclose(quat_to_rot_matrix(q_z180), R_z180, atol=1e-14)


def test_quat_mult_properties():
    # Associativity
    q1 = quat_normalize(np.array([1.0, 2.0, 3.0, 4.0]))
    q2 = quat_normalize(np.array([0.5, -0.5, 1.0, -0.2]))
    q3 = quat_normalize(np.array([2.0, 0.1, -0.3, 0.4]))

    lhs = quat_mult(q1, quat_mult(q2, q3))
    rhs = quat_mult(quat_mult(q1, q2), q3)
    np.testing.assert_allclose(lhs, rhs, atol=1e-14)

    # Norm multiplicativity: |q1 (x) q2| == |q1| * |q2|
    q_raw1 = np.array([1.2, -0.3, 0.5, 0.8])
    q_raw2 = np.array([0.4, 1.1, -0.9, 0.2])
    q_prod = quat_mult(q_raw1, q_raw2)
    np.testing.assert_allclose(
        np.linalg.norm(q_prod),
        np.linalg.norm(q_raw1) * np.linalg.norm(q_raw2),
        atol=1e-14,
    )


def test_quat_exp_log_consistency():
    # Test that quat_log with negative scalar w produces valid rotation vector
    phi = np.array([0.3, -0.5, 0.2])
    q = quat_exp(phi)
    # Negate q: -q represents the exact same rotation in SO(3)
    phi_neg = quat_log(-q)
    # The logarithmic map with sign normalization should recover the same phi
    np.testing.assert_allclose(phi_neg, phi, atol=1e-12)

    # Cross product of parallel vector is 0
    v = np.array([3.0, -2.0, 5.0])
    np.testing.assert_allclose(skew(v) @ v, np.zeros(3), atol=1e-15)

