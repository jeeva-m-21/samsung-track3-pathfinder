"""15-State Error-State Kalman Filter (ESKF) Covariance Propagation and Joseph Update.

SPEC §12.4 and §12.5 Implementation:
- 15-state error state vector:
    delta_x = [delta_theta (0:3), delta_v (3:6), delta_p (6:9), delta_b_g (9:12), delta_b_a (12:15)]
- Analytical error-state transition matrix F:
    F = I15
    F[0:3, 0:3]   = R(omega_hat * dt)^T
    F[0:3, 9:12]  = -I * dt
    F[3:6, 0:3]   = -R(q_old) * skew(a_hat) * dt
    F[3:6, 12:15] = -R(q_old) * dt
    F[6:9, 3:6]   = I * dt
- Discrete process noise covariance Q:
    Q = diag(sigma_g^2*dt*I, sigma_a^2*dt*I, 0, sigma_bg^2*dt*I, sigma_ba^2*dt*I)
- Covariance conditioning:
    Symmetrization: P = 0.5 * (P + P^T)
    Diagonal clamping: P[i, i] in [cov_diag_floor, cov_diag_cap] ([1e-12, 1e6])
- Generic Joseph-form measurement update:
    S = H P H^T + R
    K = P H^T S^{-1}  (S is 3x3: invert with closed form + det floor)
    delta_x = K y
    P = (I - KH) P (I - KH)^T + K R K^T
- State injection:
    q <- normalize(q (x) exp(delta_theta))
    v += delta_v
    p += delta_p
    b_g += delta_bg
    b_a += delta_ba
- Error state reset:
    P <- G P G^T where G = I except G[0:3, 0:3] = I - skew(delta_theta / 2)

Constraints:
- Pure functions: no global state, no mutation of inputs.
- No dynamic lists or dicts inside the filter math loop.
- No printing in the filter loop.
- Portable to fixed-size C++17 Mat<15, 15> and Mat<3, 3>.
"""

from typing import Any, Dict, Optional, Tuple, Union
import numpy as np

from src.ref.init import NominalState, load_params
from src.ref.mathlib import (
    quat_exp,
    quat_mult,
    quat_normalize,
    quat_sign_continuity,
    quat_to_rot_matrix,
    skew,
)


def build_f_matrix(
    state: NominalState,
    omega: Union[np.ndarray, list],
    acc: Union[np.ndarray, list],
    dt: float,
) -> np.ndarray:
    """Build analytical 15x15 error-state transition matrix F (SPEC §12.4).

    Error state ordering:
        delta_x = [delta_theta(3), delta_v(3), delta_p(3), delta_b_g(3), delta_b_a(3)]

    Equations:
        F = I15
        F[0:3, 0:3]   = R(omega_hat * dt)^T
        F[0:3, 9:12]  = -I * dt
        F[3:6, 0:3]   = -R(q_old) * skew(a_hat) * dt
        F[3:6, 12:15] = -R(q_old) * dt
        F[6:9, 3:6]   = I * dt

    Args:
        state: Current NominalState (contains q, b_g, b_a).
        omega: (3,) angular velocity sample [rad/s].
        acc: (3,) specific force sample [m/s^2].
        dt: Sample delta time in seconds.

    Returns:
        F: (15, 15) float64 transition matrix.
    """
    dt_f = float(dt)
    F = np.eye(15, dtype=np.float64)

    if dt_f <= 0.0:
        return F

    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)

    # Unbiased measurements (SPEC §12.3)
    omega_hat = omega_arr - state.b_g
    a_hat = acc_arr - state.b_a

    # Orientation transition: R(omega_hat * dt)^T
    delta_q = quat_exp(omega_hat * dt_f)
    R_delta = quat_to_rot_matrix(delta_q)
    F[0:3, 0:3] = R_delta.T

    # Gyro bias effect on orientation
    F[0:3, 9:12] = -np.eye(3, dtype=np.float64) * dt_f

    # Velocity transitions
    R_old = quat_to_rot_matrix(state.q)
    F[3:6, 0:3] = -R_old @ skew(a_hat) * dt_f
    F[3:6, 12:15] = -R_old * dt_f

    # Position transition
    F[6:9, 3:6] = np.eye(3, dtype=np.float64) * dt_f

    return F


def build_q_discrete(
    dt: float,
    params: Dict[str, Any],
    kappa: float = 1.0,
) -> np.ndarray:
    """Build discrete 15x15 process noise covariance matrix Q (SPEC §12.4).

    Equations:
        sigma_g  = sigma_g_base  * noise_mult_gyro * kappa
        sigma_a  = sigma_a_base  * noise_mult_acc  * kappa
        sigma_bg = sigma_bg_base * walk_mult_bg    * kappa
        sigma_ba = sigma_ba_base * walk_mult_ba    * kappa

        Q = diag(
            sigma_g^2  * dt * I3,
            sigma_a^2  * dt * I3,
            0 * I3,
            sigma_bg^2 * dt * I3,
            sigma_ba^2 * dt * I3,
        )

    Args:
        dt: Sample delta time in seconds.
        params: Configuration parameter dictionary.
        kappa: Quality inflation factor from degradation monitor (default 1.0).

    Returns:
        Q: (15, 15) float64 discrete process noise covariance matrix.
    """
    dt_f = float(dt)
    Q = np.zeros((15, 15), dtype=np.float64)

    if dt_f <= 0.0:
        return Q

    kappa_f = float(kappa)

    sigma_g = float(params["sigma_g_base"]) * float(params["noise_mult_gyro"]) * kappa_f
    sigma_a = float(params["sigma_a_base"]) * float(params["noise_mult_acc"]) * kappa_f
    sigma_bg = float(params["sigma_bg_base"]) * float(params["walk_mult_bg"]) * kappa_f
    sigma_ba = float(params["sigma_ba_base"]) * float(params["walk_mult_ba"]) * kappa_f

    q_theta = (sigma_g * sigma_g) * dt_f
    q_v = (sigma_a * sigma_a) * dt_f
    q_bg = (sigma_bg * sigma_bg) * dt_f
    q_ba = (sigma_ba * sigma_ba) * dt_f

    Q[0, 0] = q_theta
    Q[1, 1] = q_theta
    Q[2, 2] = q_theta

    Q[3, 3] = q_v
    Q[4, 4] = q_v
    Q[5, 5] = q_v

    # p block (6:9) is 0

    Q[9, 9] = q_bg
    Q[10, 10] = q_bg
    Q[11, 11] = q_bg

    Q[12, 12] = q_ba
    Q[13, 13] = q_ba
    Q[14, 14] = q_ba

    return Q


def condition_covariance(
    P: np.ndarray,
    diag_floor: float = 1e-12,
    diag_cap: float = 1e6,
) -> np.ndarray:
    """Safely condition covariance matrix P via symmetrisation and diagonal clamping.

    P <- 0.5 * (P + P^T)
    P[i, i] <- clamp(P[i, i], diag_floor, diag_cap)
    When diagonal elements exceed diag_cap, row and column are scaled by sqrt(diag_cap / P[i,i])
    (congruence transformation D P D) to preserve correlation structure and positive-definiteness.

    Args:
        P: (N, N) covariance matrix.
        diag_floor: Minimum allowable diagonal element (default 1e-12).
        diag_cap: Maximum allowable diagonal element (default 1e6).

    Returns:
        P_cond: (N, N) conditioned covariance matrix.
    """
    P_sym = 0.5 * (P + P.T)
    n = P_sym.shape[0]

    # Floor handling
    for i in range(n):
        if P_sym[i, i] < diag_floor:
            P_sym[i, i] = diag_floor

    # Cap handling with positive-definiteness-preserving congruence transformation
    for i in range(n):
        if P_sym[i, i] > diag_cap:
            scale = float(np.sqrt(diag_cap / P_sym[i, i]))
            P_sym[i, :] *= scale
            P_sym[:, i] *= scale
            P_sym[i, i] = diag_cap

    return P_sym


def eskf_predict_covariance(
    P: np.ndarray,
    F: np.ndarray,
    Q: np.ndarray,
    diag_floor: float = 1e-12,
    diag_cap: float = 1e6,
) -> np.ndarray:
    """Propagate 15-state error covariance: P <- F P F^T + Q with safe conditioning (SPEC §12.4).

    Args:
        P: (15, 15) current covariance matrix.
        F: (15, 15) error-state transition matrix.
        Q: (15, 15) discrete process noise covariance matrix.
        diag_floor: Minimum diagonal value.
        diag_cap: Maximum diagonal value.

    Returns:
        P_next: (15, 15) propagated and conditioned covariance matrix.
    """
    P_pred = F @ P @ F.T + Q
    return condition_covariance(P_pred, diag_floor=diag_floor, diag_cap=diag_cap)


def invert_3x3(A: np.ndarray, det_floor: float = 1e-12) -> np.ndarray:
    """Closed-form 3x3 matrix inverse with determinant floor (SPEC §12.5).

    Directly portable to fixed-size C++ without heap allocation.

    Args:
        A: (3, 3) float64 matrix.
        det_floor: Determinant magnitude floor to prevent division by zero.

    Returns:
        A_inv: (3, 3) inverse matrix.
    """
    a00 = float(A[0, 0])
    a01 = float(A[0, 1])
    a02 = float(A[0, 2])
    a10 = float(A[1, 0])
    a11 = float(A[1, 1])
    a12 = float(A[1, 2])
    a20 = float(A[2, 0])
    a21 = float(A[2, 1])
    a22 = float(A[2, 2])

    c00 = a11 * a22 - a12 * a21
    c01 = -(a10 * a22 - a12 * a20)
    c02 = a10 * a21 - a11 * a20

    c10 = -(a01 * a22 - a02 * a21)
    c11 = a00 * a22 - a02 * a20
    c12 = -(a00 * a21 - a01 * a20)

    c20 = a01 * a12 - a02 * a11
    c21 = -(a00 * a12 - a02 * a10)
    c22 = a00 * a11 - a01 * a10

    det = a00 * c00 + a01 * c01 + a02 * c02
    if abs(det) < det_floor:
        det = det_floor if det >= 0.0 else -det_floor

    inv_det = 1.0 / det

    return np.array([
        [c00 * inv_det, c10 * inv_det, c20 * inv_det],
        [c01 * inv_det, c11 * inv_det, c21 * inv_det],
        [c02 * inv_det, c12 * inv_det, c22 * inv_det],
    ], dtype=np.float64)


def eskf_joseph_update(
    P: np.ndarray,
    H: np.ndarray,
    y: np.ndarray,
    R: np.ndarray,
    diag_floor: float = 1e-12,
    diag_cap: float = 1e6,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generic Joseph-form Kalman measurement update (SPEC §12.5).

    Equations:
        S = H P H^T + R
        K = P H^T S^{-1}   (S is 3x3: inverted in closed form with det floor)
        delta_x = K y
        P_post = (I - KH) P (I - KH)^T + K R K^T
        P_post conditioned (symmetrised and clamped)

    Args:
        P: (15, 15) prior error covariance.
        H: (M, 15) measurement matrix (typically 3x15).
        y: (M,) measurement innovation.
        R: (M, M) measurement noise covariance.
        diag_floor: Diagonal floor.
        diag_cap: Diagonal cap.

    Returns:
        delta_x: (15,) error state update vector.
        P_post: (15, 15) posterior error covariance.
        S: (M, M) innovation covariance matrix.
    """
    H_arr = np.asarray(H, dtype=np.float64)
    y_arr = np.asarray(y, dtype=np.float64).reshape(-1)
    R_arr = np.asarray(R, dtype=np.float64)

    # Innovation covariance S = H P H^T + R
    S = H_arr @ P @ H_arr.T + R_arr

    # Invert S (use closed-form 3x3 helper if M == 3)
    if S.shape == (3, 3):
        S_inv = invert_3x3(S, det_floor=diag_floor)
    else:
        S_inv = np.linalg.inv(S)

    # Kalman gain K = P H^T S^{-1}
    K = P @ H_arr.T @ S_inv

    # Error state correction delta_x = K y
    delta_x = K @ y_arr

    # Joseph-form covariance update: (I - KH) P (I - KH)^T + K R K^T
    I_KH = np.eye(15, dtype=np.float64) - K @ H_arr
    P_post = I_KH @ P @ I_KH.T + K @ R_arr @ K.T

    # Condition covariance (symmetrisation + clamping)
    P_post = condition_covariance(P_post, diag_floor=diag_floor, diag_cap=diag_cap)

    return delta_x, P_post, S


def eskf_inject_nominal_state(
    state: NominalState,
    delta_x: np.ndarray,
) -> NominalState:
    """Inject error state delta_x into NominalState (SPEC §12.5).

    Equations:
        q   <- normalize( q (x) exp(delta_theta) )
        v   <- v + delta_v
        p   <- p + delta_p
        b_g <- b_g + delta_b_g
        b_a <- b_a + delta_b_a

    Args:
        state: Current NominalState.
        delta_x: (15,) error state correction.

    Returns:
        new_state: Updated NominalState with unit quaternion normalization.
    """
    dx = np.asarray(delta_x, dtype=np.float64).reshape(15)

    delta_theta = dx[0:3]
    delta_v = dx[3:6]
    delta_p = dx[6:9]
    delta_bg = dx[9:12]
    delta_ba = dx[12:15]

    # Orientation injection
    delta_q = quat_exp(delta_theta)
    q_new = quat_normalize(quat_mult(state.q, delta_q))
    q_new = quat_sign_continuity(q_new, state.q)

    # Vector injections
    v_new = state.v + delta_v
    p_new = state.p + delta_p
    b_g_new = state.b_g + delta_bg
    b_a_new = state.b_a + delta_ba

    return NominalState(
        p=p_new,
        v=v_new,
        q=q_new,
        b_g=b_g_new,
        b_a=b_a_new,
    )


def eskf_reset_covariance(
    P: np.ndarray,
    delta_theta: np.ndarray,
    diag_floor: float = 1e-12,
    diag_cap: float = 1e6,
) -> np.ndarray:
    """Reset error-state covariance after injection (SPEC §12.5).

    Equation:
        P <- G P G^T where G = I15 except G[0:3, 0:3] = I3 - skew(delta_theta / 2)

    Args:
        P: (15, 15) posterior covariance after Joseph update.
        delta_theta: (3,) injected orientation correction.
        diag_floor: Minimum diagonal value.
        diag_cap: Maximum diagonal value.

    Returns:
        P_reset: (15, 15) reset and conditioned error covariance.
    """
    d_th = np.asarray(delta_theta, dtype=np.float64).reshape(3)
    G = np.eye(15, dtype=np.float64)
    G[0:3, 0:3] = np.eye(3, dtype=np.float64) - skew(0.5 * d_th)

    P_reset = G @ P @ G.T
    return condition_covariance(P_reset, diag_floor=diag_floor, diag_cap=diag_cap)


def eskf_measurement_update(
    state: NominalState,
    P: np.ndarray,
    H: np.ndarray,
    y: np.ndarray,
    R: np.ndarray,
    params: Optional[Dict[str, Any]] = None,
    apply_reset: bool = True,
) -> Tuple[NominalState, np.ndarray, np.ndarray]:
    """Execute complete ESKF measurement update: Joseph update + injection + reset (SPEC §12.5).

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        H: (M, 15) measurement matrix.
        y: (M,) measurement innovation.
        R: (M, M) measurement noise covariance.
        params: Optional parameter dict (loads configs/params.json if None).
        apply_reset: Whether to apply covariance reset P <- G P G^T (default True).

    Returns:
        new_state: Updated NominalState after injection.
        new_P: Updated and conditioned error covariance matrix.
        delta_x: (15,) computed error-state correction vector.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    diag_floor = float(p_cfg.get("cov_diag_floor", 1e-12))
    diag_cap = float(p_cfg.get("cov_diag_cap", 1e6))

    # 1. Joseph-form update
    delta_x, P_post, _ = eskf_joseph_update(
        P, H, y, R, diag_floor=diag_floor, diag_cap=diag_cap
    )

    # 2. State injection
    new_state = eskf_inject_nominal_state(state, delta_x)

    # 3. Covariance reset
    if apply_reset:
        new_P = eskf_reset_covariance(
            P_post, delta_x[0:3], diag_floor=diag_floor, diag_cap=diag_cap
        )
    else:
        new_P = P_post

    return new_state, new_P, delta_x
