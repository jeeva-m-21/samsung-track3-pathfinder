"""Quaternion and SO(3) Kinematics Math Library.

SPEC §12.1 Implementation:
- Quaternion q = [w, x, y, z], Hamilton product q1 (x) q2, body -> world.
- Skew-symmetric cross product matrix: skew(v).
- Exponential map: quat_exp(phi) with angle a = |phi| (Taylor expansion for a < 1e-8).
- Logarithmic map: quat_log(q).
- Rotation matrix conversions: quat_to_rot_matrix(q) and rot_matrix_to_quat(R).
- Quaternion normalization with floor 1e-12.
- Sign continuity: quat_sign_continuity(q_new, q_old).
"""

from typing import Union
import numpy as np


def skew(v: Union[np.ndarray, list]) -> np.ndarray:
    """Return 3x3 skew-symmetric cross-product matrix of a 3-vector.

    skew(v) @ u == v x u.
    skew(v) = [[ 0,   -vz,  vy],
               [ vz,   0,  -vx],
               [-vy,  vx,   0 ]]
    """
    vx = float(v[0])
    vy = float(v[1])
    vz = float(v[2])
    return np.array([
        [0.0, -vz, vy],
        [vz, 0.0, -vx],
        [-vy, vx, 0.0]
    ], dtype=np.float64)


def quat_mult(q1: Union[np.ndarray, list], q2: Union[np.ndarray, list]) -> np.ndarray:
    """Hamilton quaternion product q1 (x) q2 where q = [w, x, y, z]."""
    w1, x1, y1, z1 = float(q1[0]), float(q1[1]), float(q1[2]), float(q1[3])
    w2, x2, y2, z2 = float(q2[0]), float(q2[1]), float(q2[2]), float(q2[3])

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z], dtype=np.float64)


def quat_conj(q: Union[np.ndarray, list]) -> np.ndarray:
    """Quaternion conjugate q* = [w, -x, -y, -z] where q = [w, x, y, z]."""
    return np.array([float(q[0]), -float(q[1]), -float(q[2]), -float(q[3])], dtype=np.float64)


def quat_normalize(q: Union[np.ndarray, list]) -> np.ndarray:
    """Normalize quaternion q = [w, x, y, z] to unit norm with floor 1e-12."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    norm = float(np.sqrt(w * w + x * x + y * y + z * z))
    if norm < 1e-12:
        norm = 1e-12
    inv_norm = 1.0 / norm
    return np.array([w * inv_norm, x * inv_norm, y * inv_norm, z * inv_norm], dtype=np.float64)


def quat_exp(phi: Union[np.ndarray, list]) -> np.ndarray:
    """Exponential map of rotation vector phi to unit Hamilton quaternion [w, x, y, z].

    Angle a = |phi|: exp(phi) = [cos(a/2), sin(a/2)*phi/a].
    For a < 1e-8, uses [1, phi/2] then normalizes.
    """
    phi_x = float(phi[0])
    phi_y = float(phi[1])
    phi_z = float(phi[2])
    a = float(np.sqrt(phi_x * phi_x + phi_y * phi_y + phi_z * phi_z))

    if a < 1e-8:
        raw = np.array([1.0, 0.5 * phi_x, 0.5 * phi_y, 0.5 * phi_z], dtype=np.float64)
        return quat_normalize(raw)

    half_a = 0.5 * a
    factor = float(np.sin(half_a) / a)
    return np.array([
        float(np.cos(half_a)),
        factor * phi_x,
        factor * phi_y,
        factor * phi_z
    ], dtype=np.float64)


def quat_log(q: Union[np.ndarray, list]) -> np.ndarray:
    """Logarithmic map of unit Hamilton quaternion q = [w, x, y, z] to rotation vector phi."""
    w = float(q[0])
    x = float(q[1])
    y = float(q[2])
    z = float(q[3])

    # Ensure w >= 0 so that angle is in [0, pi]
    if w < 0.0:
        w = -w
        x = -x
        y = -y
        z = -z

    v_norm = float(np.sqrt(x * x + y * y + z * z))
    if v_norm < 1e-8:
        scale = 2.0 / (w if w > 1e-12 else 1e-12)
        return np.array([scale * x, scale * y, scale * z], dtype=np.float64)

    angle = 2.0 * float(np.arctan2(v_norm, w))
    scale = angle / v_norm
    return np.array([scale * x, scale * y, scale * z], dtype=np.float64)


def quat_to_rot_matrix(q: Union[np.ndarray, list]) -> np.ndarray:
    """Convert Hamilton quaternion [w, x, y, z] to 3x3 rotation matrix R (body to world)."""
    w = float(q[0])
    x = float(q[1])
    y = float(q[2])
    z = float(q[3])

    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - w * z)
    r02 = 2.0 * (x * z + w * y)

    r10 = 2.0 * (x * y + w * z)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - w * x)

    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    return np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22]
    ], dtype=np.float64)


def rot_matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to unit Hamilton quaternion [w, x, y, z] using Shepperd's algorithm."""
    r00, r01, r02 = float(R[0, 0]), float(R[0, 1]), float(R[0, 2])
    r10, r11, r12 = float(R[1, 0]), float(R[1, 1]), float(R[1, 2])
    r20, r21, r22 = float(R[2, 0]), float(R[2, 1]), float(R[2, 2])

    trace = r00 + r11 + r22
    if trace > 0.0:
        s = 0.5 / float(np.sqrt(trace + 1.0))
        w = 0.25 / s
        x = (r21 - r12) * s
        y = (r02 - r20) * s
        z = (r10 - r01) * s
    elif (r00 > r11) and (r00 > r22):
        s = 2.0 * float(np.sqrt(1.0 + r00 - r11 - r22))
        w = (r21 - r12) / s
        x = 0.25 * s
        y = (r01 + r10) / s
        z = (r02 + r20) / s
    elif r11 > r22:
        s = 2.0 * float(np.sqrt(1.0 + r11 - r00 - r22))
        w = (r02 - r20) / s
        x = (r01 + r10) / s
        y = 0.25 * s
        z = (r12 + r21) / s
    else:
        s = 2.0 * float(np.sqrt(1.0 + r22 - r00 - r11))
        w = (r10 - r01) / s
        x = (r02 + r20) / s
        y = (r12 + r21) / s
        z = 0.25 * s

    q = np.array([w, x, y, z], dtype=np.float64)
    return quat_normalize(q)


def quat_sign_continuity(q_new: Union[np.ndarray, list], q_old: Union[np.ndarray, list]) -> np.ndarray:
    """Enforce quaternion sign continuity dot(q_new, q_old) >= 0."""
    w1, x1, y1, z1 = float(q_new[0]), float(q_new[1]), float(q_new[2]), float(q_new[3])
    w2, x2, y2, z2 = float(q_old[0]), float(q_old[1]), float(q_old[2]), float(q_old[3])
    dot = w1 * w2 + x1 * x2 + y1 * y2 + z1 * z2
    q_arr = np.array([w1, x1, y1, z1], dtype=np.float64)
    if dot < 0.0:
        return -q_arr
    return q_arr
