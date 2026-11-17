"""Measurement Updates: Tilt Update and Zero Velocity Update (ZUPT) (SPEC §12.5).

Implements:
1. Tilt measurement update:
   - Gating condition:
       | ||a|| - g_mag | < tau_a * (1 + 5 * (1 - quality))  and  ||omega|| < omega_max
   - Normalized gravity observation in body frame:
       z = (a - b_a) / ||a - b_a||
   - Predicted gravity observation from current orientation:
       h_hat = R(q)^T [0, 0, 1]
   - Innovation:
       y = z - h_hat
   - Measurement matrix H (3x15):
       H = [skew(h_hat), 0, 0, 0, 0]  (only the delta_theta block is nonzero)
   - Measurement noise covariance R:
       sigma_tilt = tilt_sigma / max(quality, 0.1)
       R_tilt = sigma_tilt^2 * I3

2. Zero Velocity Update (ZUPT):
   - Stationary detector over sliding window (zupt_window_s = 0.25s):
       mean gyro norm < zupt_gyro_mean (0.03 rad/s)
       accel norm deviation from g_mag < zupt_acc_dev (0.15 m/s^2)
       quality >= zupt_min_quality (0.3)
   - Measurement:
       y = -v
   - Measurement matrix H (3x15):
       H = [0, I3, 0, 0, 0]  (only the delta_v block is nonzero)
   - Measurement noise covariance R:
       R_zupt = zupt_sigma^2 * I3 (zupt_sigma = 0.05)

3. StationaryWindowBuffer:
   - Fixed-capacity circular ring buffer for streaming stationary detection.
   - Fully portable to fixed-size C++ ring buffer without dynamic allocations.

Constraints:
- Pure functions: no global state, no mutation of inputs.
- No dynamic lists or dicts inside filter loops.
- No printing in the filter loop.
- Portable to fixed-size C++17 Mat<15, 15>, Mat<3, 15>, and Mat<3, 3>.
"""

from typing import Any, Dict, Optional, Tuple, Union
import numpy as np

from src.ref.eskf import (
    NominalState,
    eskf_measurement_update,
    joseph_update,
    load_params,
)
from src.ref.mathlib import (
    quat_to_rot_matrix,
    skew,
)


def tilt_gating(
    acc: Union[np.ndarray, list],
    omega: Union[np.ndarray, list],
    g_mag: float = 9.80665,
    quality: float = 1.0,
    params: Optional[Dict[str, Any]] = None,
) -> bool:
    """Evaluate gating conditions for tilt measurement update (SPEC §12.5).

    Applied when:
        | ||a|| - g_mag | < tau_a * (1.0 + 5.0 * (1.0 - quality))
        and
        ||omega|| < omega_max

    Args:
        acc: (3,) accelerometer measurement [m/s^2].
        omega: (3,) gyroscope measurement [rad/s].
        g_mag: Reference gravity magnitude [m/s^2].
        quality: Signal quality score in [0.05, 1.0].
        params: Optional parameter dict (loads configs/params.json if None).

    Returns:
        True if the sample passes gating and tilt update can be applied.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    tau_a = float(p_cfg.get("tilt_tau_a", 0.3))
    omega_max = float(p_cfg.get("tilt_omega_max", 1.0))

    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)
    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)

    acc_norm = float(np.linalg.norm(acc_arr))
    omega_norm = float(np.linalg.norm(omega_arr))

    # Gyro check
    if omega_norm >= omega_max:
        return False

    # Quality-inflated accel threshold
    q_clamped = float(np.clip(quality, 0.0, 1.0))
    tau_eff = tau_a * (1.0 + 5.0 * (1.0 - q_clamped))

    acc_dev = abs(acc_norm - float(g_mag))
    return bool(acc_dev < tau_eff)


def compute_tilt_innovation(
    state: NominalState,
    acc: Union[np.ndarray, list],
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute tilt innovation y = z - h_hat and predicted gravity h_hat (SPEC §12.5).

    Equations:
        z = (a - b_a) / ||a - b_a||
        h_hat = R(q)^T [0, 0, 1]
        y = z - h_hat

    Args:
        state: Current NominalState (q, b_a).
        acc: (3,) specific force measurement [m/s^2].

    Returns:
        y: (3,) innovation vector.
        h_hat: (3,) predicted gravity direction in body frame.
    """
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)
    a_unbiased = acc_arr - state.b_a

    norm_a = float(np.linalg.norm(a_unbiased))
    if norm_a < 1e-12:
        norm_a = 1e-12

    z = a_unbiased / norm_a

    R_bw = quat_to_rot_matrix(state.q)
    # R_bw is body-to-world; R_wb = R_bw.T maps world +z to body frame
    h_hat = R_bw.T @ np.array([0.0, 0.0, 1.0], dtype=np.float64)

    y = z - h_hat
    return y, h_hat


def build_tilt_h_matrix(h_hat: Union[np.ndarray, list]) -> np.ndarray:
    """Build 3x15 tilt measurement Jacobian matrix H (SPEC §12.5).

    Equation:
        H = [skew(h_hat), 0, 0, 0, 0]  (3x15, theta block at 0:3 is nonzero)

    Args:
        h_hat: (3,) predicted gravity direction in body frame.

    Returns:
        H: (3, 15) float64 measurement matrix.
    """
    h_arr = np.asarray(h_hat, dtype=np.float64).reshape(3)
    H = np.zeros((3, 15), dtype=np.float64)
    H[:, 0:3] = skew(h_arr)
    return H


def build_tilt_r_matrix(
    quality: float = 1.0,
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
) -> np.ndarray:
    """Build 3x3 tilt measurement noise covariance matrix R (SPEC §12.5).

    Equation:
        sigma_tilt = tilt_sigma / max(quality, 0.1)
        R_tilt = (sigma_tilt^2 * r_scale) * I3

    Args:
        quality: Signal quality score in [0.05, 1.0].
        params: Optional parameter dict (loads configs/params.json if None).
        r_scale: Multiplier for gap recovery inflation (SPEC §12.7, default 1.0).

    Returns:
        R: (3, 3) float64 covariance matrix.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    sigma_base = float(p_cfg.get("tilt_sigma", 0.02))
    q_clamped = max(float(quality), 0.1)

    sigma_tilt = sigma_base / q_clamped
    var_tilt = (sigma_tilt * sigma_tilt) * float(r_scale)

    return var_tilt * np.eye(3, dtype=np.float64)


def tilt_update(
    state: NominalState,
    P: np.ndarray,
    acc: Union[np.ndarray, list],
    omega: Union[np.ndarray, list],
    g_mag: float = 9.80665,
    quality: float = 1.0,
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
    apply_reset: bool = True,
) -> Tuple[NominalState, np.ndarray, np.ndarray, bool]:
    """Execute complete tilt measurement update with gating check (SPEC §12.5).

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        acc: (3,) accelerometer sample [m/s^2].
        omega: (3,) gyroscope sample [rad/s].
        g_mag: Gravity magnitude [m/s^2].
        quality: Signal quality score [0.05, 1.0].
        params: Optional configuration dictionary.
        r_scale: Covariance scaling multiplier (e.g. from gap recovery).
        apply_reset: Whether to apply error-state reset G P G^T.

    Returns:
        new_state: Updated NominalState (or unchanged copy if rejected).
        new_P: Updated error covariance (or unchanged copy if rejected).
        delta_x: (15,) error state update (zeros if rejected).
        updated: True if update was applied, False if rejected by gating.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    if not tilt_gating(acc, omega, g_mag=g_mag, quality=quality, params=p_cfg):
        return state.copy(), P.copy(), np.zeros(15, dtype=np.float64), False

    y, h_hat = compute_tilt_innovation(state, acc)
    H = build_tilt_h_matrix(h_hat)

    # Dynamic noise scaling based on linear acceleration residual and gyro rate
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)
    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    acc_norm = float(np.linalg.norm(acc_arr))
    delta_a = abs(acc_norm - float(g_mag))
    a_scale = float(p_cfg.get("tilt_a_scale", p_cfg.get("tilt_tau_a", 0.3)))
    a_ratio = delta_a / a_scale if a_scale > 1e-6 else 0.0

    omega_norm = float(np.linalg.norm(omega_arr))
    omega_scale = float(p_cfg.get("tilt_omega_scale", p_cfg.get("tilt_omega_max", 1.0)))
    omega_ratio = omega_norm / omega_scale if omega_scale > 1e-6 else 0.0

    alpha_a = float(p_cfg.get("tilt_alpha_a", 2.0))
    alpha_omega = float(p_cfg.get("tilt_alpha_omega", 8.0))

    dynamic_scale = 1.0 + alpha_a * (a_ratio * a_ratio) + alpha_omega * (omega_ratio * omega_ratio)
    if dynamic_scale > 1e6:
        dynamic_scale = 1e6

    R = build_tilt_r_matrix(quality=quality, params=p_cfg, r_scale=r_scale * dynamic_scale)

    gate_threshold = float(p_cfg.get("tilt_gate_threshold", 1e9))
    new_state, new_P, updated = joseph_update(
        state, P, H, y, R, gate_threshold=gate_threshold, params=p_cfg, m=3, apply_reset=apply_reset
    )

    return new_state, new_P, np.zeros(15, dtype=np.float64), updated


# ---------------------------------------------------------------------------
# Zero Velocity Update (ZUPT)
# ---------------------------------------------------------------------------


def zupt_detect_stationary(
    gyro_samples: np.ndarray,
    acc_samples: np.ndarray,
    g_mag: float = 9.80665,
    quality: float = 1.0,
    params: Optional[Dict[str, Any]] = None,
) -> bool:
    """Evaluate sliding window stationary condition for ZUPT (SPEC §12.5).

    Stationary conditions:
        1. quality >= zupt_min_quality (0.3)
        2. mean gyro norm < zupt_gyro_mean (0.03 rad/s)
        3. |mean accel norm - g_mag| < zupt_acc_dev (0.15 m/s^2)
        4. accel variance < (zupt_acc_dev)^2

    Args:
        gyro_samples: (N, 3) gyroscope measurements over window.
        acc_samples: (N, 3) accelerometer measurements over window.
        g_mag: Reference gravity magnitude [m/s^2].
        quality: Current signal quality score.
        params: Optional parameter dict (loads configs/params.json if None).

    Returns:
        True if stationary conditions are met and ZUPT can be applied.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    min_quality = float(p_cfg.get("zupt_min_quality", 0.3))
    if float(quality) < min_quality:
        return False

    if gyro_samples is None or acc_samples is None:
        return False

    g_arr = np.asarray(gyro_samples, dtype=np.float64).reshape(-1, 3)
    a_arr = np.asarray(acc_samples, dtype=np.float64).reshape(-1, 3)

    n = len(g_arr)
    if n == 0 or len(a_arr) != n:
        return False

    # 1. Mean gyro norm and latest sample gating
    gyro_thresh = float(p_cfg.get("zupt_omega_thresh", p_cfg.get("zupt_gyro_mean", 0.02)))
    gyro_norms = np.linalg.norm(g_arr, axis=1)
    mean_gyro = float(np.mean(gyro_norms))
    if mean_gyro >= gyro_thresh:
        return False

    if float(gyro_norms[-1]) > gyro_thresh * 1.5:
        return False

    # 2. Accel norm deviation from g_mag
    acc_dev_thresh = float(p_cfg.get("zupt_acc_dev", 0.15))
    acc_norms = np.linalg.norm(a_arr, axis=1)
    mean_acc = float(np.mean(acc_norms))
    acc_dev = abs(mean_acc - float(g_mag))
    if acc_dev >= acc_dev_thresh:
        return False

    # 3. Accel variance
    acc_var = float(np.var(acc_norms))
    var_thresh = float(p_cfg.get("zupt_acc_var", acc_dev_thresh * acc_dev_thresh))
    if acc_var >= var_thresh:
        return False

    return True


def build_zupt_h_matrix() -> np.ndarray:
    """Build 3x15 ZUPT measurement Jacobian matrix H (SPEC §12.5).

    Equation:
        H = [0, I3, 0, 0, 0]  (3x15, velocity block at 3:6 is identity)

    Returns:
        H: (3, 15) float64 measurement matrix.
    """
    H = np.zeros((3, 15), dtype=np.float64)
    H[:, 3:6] = np.eye(3, dtype=np.float64)
    return H


def build_zupt_r_matrix(
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
) -> np.ndarray:
    """Build 3x3 ZUPT measurement noise covariance matrix R (SPEC §12.5).

    Equation:
        R_zupt = (zupt_sigma^2 * r_scale) * I3  (default zupt_sigma = 0.05)

    Args:
        params: Optional parameter dict (loads configs/params.json if None).
        r_scale: Covariance scaling multiplier (e.g. from gap recovery).

    Returns:
        R: (3, 3) float64 covariance matrix.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    sigma_z = float(p_cfg.get("zupt_sigma", 0.05))
    var_z = (sigma_z * sigma_z) * float(r_scale)
    return var_z * np.eye(3, dtype=np.float64)


def zupt_update(
    state: NominalState,
    P: np.ndarray,
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
    apply_reset: bool = True,
) -> Tuple[NominalState, np.ndarray, np.ndarray]:
    """Execute Zero Velocity Update (ZUPT) measurement update (SPEC §12.5).

    Equations:
        y = -v
        H = [0, I3, 0, 0, 0]
        R = zupt_sigma^2 * I3

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        params: Optional configuration dictionary.
        r_scale: Covariance scaling multiplier.
        apply_reset: Whether to apply error-state reset G P G^T.

    Returns:
        new_state: Updated NominalState with reduced velocity.
        new_P: Posterior conditioned covariance matrix.
        delta_x: (15,) error state update vector.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    y = -state.v
    H = build_zupt_h_matrix()
    R = build_zupt_r_matrix(params=p_cfg, r_scale=r_scale)

    new_state, new_P, delta_x = eskf_measurement_update(
        state, P, H, y, R, params=p_cfg, apply_reset=apply_reset
    )

    return new_state, new_P, delta_x


def zupt_step(
    state: NominalState,
    P: np.ndarray,
    gyro_samples: np.ndarray,
    acc_samples: np.ndarray,
    g_mag: float = 9.80665,
    quality: float = 1.0,
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
    apply_reset: bool = True,
) -> Tuple[NominalState, np.ndarray, np.ndarray, bool]:
    """Check stationary conditions and optionally apply ZUPT update (SPEC §12.5).

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        gyro_samples: (N, 3) window of gyro measurements.
        acc_samples: (N, 3) window of accel measurements.
        g_mag: Gravity magnitude [m/s^2].
        quality: Signal quality score.
        params: Optional configuration dictionary.
        r_scale: Covariance scaling multiplier.
        apply_reset: Whether to apply covariance reset.

    Returns:
        new_state: Updated NominalState (or unchanged copy if not stationary).
        new_P: Updated covariance matrix (or unchanged copy if not stationary).
        delta_x: (15,) error state correction (zeros if not stationary).
        updated: True if ZUPT was applied, False otherwise.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    is_stat = zupt_detect_stationary(
        gyro_samples, acc_samples, g_mag=g_mag, quality=quality, params=p_cfg
    )

    if not is_stat:
        return state.copy(), P.copy(), np.zeros(15, dtype=np.float64), False

    new_state, new_P, delta_x = zupt_update(
        state, P, params=p_cfg, r_scale=r_scale, apply_reset=apply_reset
    )
    return new_state, new_P, delta_x, True


# ---------------------------------------------------------------------------
# Streaming Stationary Window Buffer
# ---------------------------------------------------------------------------


class StationaryWindowBuffer:
    """Fixed-capacity circular ring buffer for streaming stationary detection.

    Designed for direct zero-heap porting to C++ std::array.
    Maintains a fixed history of timestamps, gyro measurements, and accel measurements.
    """

    def __init__(self, capacity: int = 64) -> None:
        """Initialize fixed-size buffer with capacity slots.

        Args:
            capacity: Maximum number of samples to store (default 64).
        """
        self.capacity: int = int(capacity)
        self.timestamps: np.ndarray = np.zeros(self.capacity, dtype=np.float64)
        self.gyro: np.ndarray = np.zeros((self.capacity, 3), dtype=np.float64)
        self.acc: np.ndarray = np.zeros((self.capacity, 3), dtype=np.float64)
        self.head: int = 0
        self.count: int = 0
        self.acc_filter = Butterworth2LowPass()

    def clear(self) -> None:
        """Reset the buffer state."""
        self.head = 0
        self.count = 0
        self.timestamps.fill(0.0)
        self.gyro.fill(0.0)
        self.acc.fill(0.0)
        self.acc_filter = Butterworth2LowPass()

    def add_sample(
        self,
        t: float,
        omega: Union[np.ndarray, list],
        acc: Union[np.ndarray, list],
        b_g: Optional[Union[np.ndarray, list]] = None,
    ) -> None:
        """Add a sample to circular ring buffer with gyro bias subtraction and accel filtering.

        Args:
            t: Sample timestamp in seconds.
            omega: (3,) gyroscope measurement [rad/s].
            acc: (3,) accelerometer measurement [m/s^2].
            b_g: Optional (3,) estimated gyro bias [rad/s].
        """
        idx = self.head
        self.timestamps[idx] = float(t)
        omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
        acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)
        bg_arr = np.zeros(3, dtype=np.float64) if b_g is None else np.asarray(b_g, dtype=np.float64).reshape(3)

        self.gyro[idx, :] = omega_arr - bg_arr
        self.acc[idx, :] = self.acc_filter.step(acc_arr)

        self.head = (idx + 1) % self.capacity
        if self.count < self.capacity:
            self.count += 1

    def get_window_samples(self, window_s: float) -> Tuple[np.ndarray, np.ndarray]:
        """Extract samples falling within window_s seconds from the most recent sample.

        Args:
            window_s: Sliding window duration in seconds.

        Returns:
            gyro_win: (K, 3) gyro measurements in window.
            acc_win: (K, 3) accel measurements in window.
        """
        if self.count == 0:
            return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.float64)

        latest_idx = (self.head - 1) % self.capacity
        latest_t = self.timestamps[latest_idx]
        t_thresh = latest_t - float(window_s)

        # Collect sample indices going backward
        indices = []
        for i in range(self.count):
            curr_idx = (self.head - 1 - i) % self.capacity
            if self.timestamps[curr_idx] < t_thresh:
                break
            indices.append(curr_idx)

        if not indices:
            return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.float64)

        # Order chronologically
        indices.reverse()
        idx_arr = np.array(indices, dtype=np.int64)

        return self.gyro[idx_arr].copy(), self.acc[idx_arr].copy()

    def is_stationary(
        self,
        g_mag: float = 9.80665,
        quality: float = 1.0,
        params: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Evaluate stationary detector over the recent window_s history.

        Args:
            g_mag: Reference gravity magnitude [m/s^2].
            quality: Signal quality score.
            params: Optional parameter dict.

        Returns:
            True if buffer contains a full valid window and stationary conditions are met.
        """
        if params is None:
            p_cfg = load_params()
        else:
            p_cfg = params

        window_s = float(p_cfg.get("zupt_window_s", 0.25))

        if self.count < 2:
            return False

        latest_idx = (self.head - 1) % self.capacity
        latest_t = self.timestamps[latest_idx]

        # Determine oldest sample time in the buffer
        oldest_idx = (self.head - self.count) % self.capacity
        oldest_t = self.timestamps[oldest_idx]

        # Require that accumulated history covers at least 90% of window_s
        if (latest_t - oldest_t) < (window_s * 0.9):
            return False

        gyro_win, acc_win = self.get_window_samples(window_s)
        if len(gyro_win) < 2:
            return False

        return zupt_detect_stationary(
            gyro_win, acc_win, g_mag=g_mag, quality=quality, params=p_cfg
        )


class Butterworth2LowPass:
    """2nd-order Butterworth low-pass filter (fc = 14 Hz, fs = 200 Hz)."""

    b0: float = 0.03657483584392802
    b1: float = 0.07314967168785604
    b2: float = 0.03657483584392802
    a1: float = -1.3908952814253894
    a2: float = 0.5371946248011015

    def __init__(self) -> None:
        self.d1 = np.zeros(3, dtype=np.float64)
        self.d2 = np.zeros(3, dtype=np.float64)
        self.initialized = False

    def reset(self, x0: np.ndarray) -> None:
        x0_arr = np.asarray(x0, dtype=np.float64).reshape(3)
        self.d1 = x0_arr * (self.b1 + self.b2 - self.a1 - self.a2)
        self.d2 = x0_arr * (self.b2 - self.a2)
        self.initialized = True

    def step(self, x: np.ndarray) -> np.ndarray:
        x_arr = np.asarray(x, dtype=np.float64).reshape(3)
        if not self.initialized:
            self.reset(x_arr)
            return x_arr.copy()
        y = x_arr * self.b0 + self.d1
        self.d1 = x_arr * self.b1 - y * self.a1 + self.d2
        self.d2 = x_arr * self.b2 - y * self.a2
        return y


def apply_velocity_bounding(
    state: NominalState,
    P: np.ndarray,
    max_speed: Optional[float] = None,
    sigma_v: Optional[float] = None,
    params: Optional[Dict[str, Any]] = None,
) -> Tuple[NominalState, np.ndarray, bool]:
    """Soft velocity bounding pseudo-measurement update."""
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    if not bool(p_cfg.get("enable_velocity_bounding", True)):
        return state, P, False

    limit = float(max_speed if max_speed is not None else p_cfg.get("max_velocity_mps", 3.0))
    sig = float(sigma_v if sigma_v is not None else p_cfg.get("velocity_bounding_sigma", 0.2))

    v = state.v
    v_norm = float(np.linalg.norm(v))
    if v_norm <= limit or v_norm < 1e-12:
        return state, P, False

    target_v = v * (limit / v_norm)
    y_v = target_v - v

    # Decouple P_pv and P_vp cross-covariance blocks so velocity bounding
    # pseudo-measurement does not inject unphysical position jumps/kicks
    P_decoupled = P.copy()
    P_decoupled[6:9, 3:6] = 0.0
    P_decoupled[3:6, 6:9] = 0.0

    H_v = np.zeros((3, 15), dtype=np.float64)
    H_v[0:3, 3:6] = np.eye(3, dtype=np.float64)
    R_v = (sig ** 2) * np.eye(3, dtype=np.float64)

    updated_state, updated_P, _ = eskf_measurement_update(state, P_decoupled, H_v, y_v, R_v)
    return updated_state, updated_P, True


def apply_rotor_drag_update(
    state: NominalState,
    P: np.ndarray,
    acc_raw: np.ndarray,
    omega_raw: np.ndarray,
    g_w: np.ndarray,
    kd: float = 0.22,
    sigma: float = 0.8,
    gate_threshold: float = 12.0,
    params: Optional[Dict[str, Any]] = None,
) -> Tuple[NominalState, np.ndarray, bool]:
    """Rotor drag measurement update for multirotor drone flight."""
    if params is not None and not bool(params.get("enable_rotor_drag", True)):
        return state, P, False

    R_b2w = quat_to_rot_matrix(state.q)
    R_w2b = R_b2w.T

    acc_arr = np.asarray(acc_raw, dtype=np.float64).reshape(3)
    f_b_xy = acc_arr[:2] - state.b_a[:2]
    v_b = R_w2b @ state.v
    v_b_xy = v_b[:2]

    # Innovation: y = a_xy^b + kd * v_xy^b
    y = f_b_xy + kd * v_b_xy

    # Decouple P_pv and P_vp cross covariance
    P_decoupled = P.copy()
    P_decoupled[6:9, 3:6] = 0.0
    P_decoupled[3:6, 6:9] = 0.0

    # H_v = -kd * R_b2w(0:2, :)^T => shape (2, 15)
    H = np.zeros((2, 15), dtype=np.float64)
    for i in range(2):
        for j in range(3):
            H[i, 3 + j] = -kd * R_b2w[j, i]

    R_drag = (sigma ** 2) * np.eye(2, dtype=np.float64)

    return joseph_update(
        state, P_decoupled, H, y, R_drag, gate_threshold=gate_threshold, params=params, m=0
    )


def apply_pedestrian_prior(
    state: NominalState,
    P: np.ndarray,
    sigma_lat: float = 0.5,
    sigma_vert: float = 0.3,
    gate_threshold: float = 9.0,
    params: Optional[Dict[str, Any]] = None,
) -> Tuple[NominalState, np.ndarray, bool]:
    """Pedestrian kinematic motion prior (lateral non-holonomic and vertical damping)."""
    if params is not None and not bool(params.get("enable_pedestrian_prior", True)):
        return state, P, False

    R_b2w = quat_to_rot_matrix(state.q)
    R_w2b = R_b2w.T

    v_b = R_w2b @ state.v
    v_lat = v_b[1]
    v_vert = state.v[2]

    # Innovation: y = target - current = [0 - v_lat, 0 - v_vert]^T = [-v_lat, -v_vert]^T
    y = np.array([-v_lat, -v_vert], dtype=np.float64)

    P_decoupled = P.copy()
    P_decoupled[6:9, 3:6] = 0.0
    P_decoupled[3:6, 6:9] = 0.0

    H = np.zeros((2, 15), dtype=np.float64)
    H[0, 3:6] = R_w2b[1, :]
    H[1, 3 + 2] = 1.0

    R_ped = np.diag([sigma_lat ** 2, sigma_vert ** 2])

    return joseph_update(
        state, P_decoupled, H, y, R_ped, gate_threshold=gate_threshold, params=params, m=2
    )


