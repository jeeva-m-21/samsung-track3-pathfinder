"""Strapdown Nominal State Integration (SPEC §12.3).

Implements the nominal continuous-discrete kinematic propagation equations:
    omega_hat = omega - b_g
    a_hat = acc - b_a
    q <- normalize( q (x) exp(omega_hat * dt) )   # sign continuity maintained
    a_w = R(q_old) * a_hat + g_w                  # uses q_old for R
    p <- p + v * dt + 0.5 * a_w * dt^2
    v <- v + a_w * dt
    b_g, b_a unchanged

Constraints:
- Pure functions: step(state, sample, dt, g_w) -> new_state
- No printing in the filter loop.
- Fixed-size numpy float64 arrays portable to fixed-size C++.
"""

from typing import Tuple, Union
import numpy as np

from src.ref.init import NominalState
from src.ref.mathlib import (
    quat_exp,
    quat_mult,
    quat_normalize,
    quat_sign_continuity,
    quat_to_rot_matrix,
)


def strapdown_step(
    state: NominalState,
    omega: Union[np.ndarray, list],
    acc: Union[np.ndarray, list],
    dt: float,
    g_w: Union[np.ndarray, list],
) -> NominalState:
    """Pure function for single-step strapdown state propagation (SPEC §12.3).

    Args:
        state: Current NominalState (p, v, q, b_g, b_a).
        omega: (3,) angular velocity measurement in body frame [rad/s].
        acc: (3,) specific force measurement in body frame [m/s^2].
        dt: Time increment in seconds.
        g_w: (3,) world gravity vector [0, 0, -g_mag].

    Returns:
        New NominalState after dt propagation. Input state is not modified.
    """
    dt_f = float(dt)
    if dt_f <= 0.0:
        return state.copy()

    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)
    g_w_arr = np.asarray(g_w, dtype=np.float64).reshape(3)

    # 1. Unbiased measurements (SPEC §12.3)
    omega_hat = omega_arr - state.b_g
    a_hat = acc_arr - state.b_a

    # 2. Update orientation quaternion
    delta_q = quat_exp(omega_hat * dt_f)
    q_new = quat_normalize(quat_mult(state.q, delta_q))
    q_new = quat_sign_continuity(q_new, state.q)

    # 3. Compute acceleration in world frame using q_old
    R_old = quat_to_rot_matrix(state.q)
    a_w = R_old @ a_hat + g_w_arr

    # 4. Integrate position and velocity
    dt_sq_half = 0.5 * (dt_f * dt_f)
    p_new = state.p + state.v * dt_f + a_w * dt_sq_half
    v_new = state.v + a_w * dt_f

    # 5. Biases unchanged
    b_g_new = state.b_g.copy()
    b_a_new = state.b_a.copy()

    return NominalState(
        p=p_new,
        v=v_new,
        q=q_new,
        b_g=b_g_new,
        b_a=b_a_new,
    )


def strapdown_propagate_sequence(
    state_init: NominalState,
    omega_seq: np.ndarray,
    acc_seq: np.ndarray,
    dt_seq: Union[np.ndarray, float],
    g_w: np.ndarray,
) -> Tuple[NominalState, np.ndarray, np.ndarray, np.ndarray]:
    """Propagate nominal state across a sequence of IMU samples.

    Args:
        state_init: Initial NominalState.
        omega_seq: (N, 3) float64 array of gyro samples.
        acc_seq: (N, 3) float64 array of accelerometer samples.
        dt_seq: (N,) float64 array of dt increments or float dt.
        g_w: (3,) float64 world gravity vector.

    Returns:
        final_state: NominalState at the end of sequence.
        p_hist: (N, 3) position history.
        v_hist: (N, 3) velocity history.
        q_hist: (N, 4) quaternion history.
    """
    n = len(omega_seq)
    if isinstance(dt_seq, (int, float)):
        dts = np.full(n, float(dt_seq), dtype=np.float64)
    else:
        dts = np.asarray(dt_seq, dtype=np.float64)

    p_hist = np.empty((n, 3), dtype=np.float64)
    v_hist = np.empty((n, 3), dtype=np.float64)
    q_hist = np.empty((n, 4), dtype=np.float64)

    curr_state = state_init.copy()
    for k in range(n):
        curr_state = strapdown_step(curr_state, omega_seq[k], acc_seq[k], dts[k], g_w)
        p_hist[k] = curr_state.p
        v_hist[k] = curr_state.v
        q_hist[k] = curr_state.q

    return curr_state, p_hist, v_hist, q_hist
