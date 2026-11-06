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
    R = build_tilt_r_matrix(quality=quality, params=p_cfg, r_scale=r_scale)

    new_state, new_P, delta_x = eskf_measurement_update(
        state, P, H, y, R, params=p_cfg, apply_reset=apply_reset
    )

    return new_state, new_P, delta_x, True


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

    # 1. Mean gyro norm
    gyro_thresh = float(p_cfg.get("zupt_gyro_mean", 0.03))
    gyro_norms = np.linalg.norm(g_arr, axis=1)
    mean_gyro = float(np.mean(gyro_norms))
    if mean_gyro >= gyro_thresh:
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

    def clear(self) -> None:
        """Reset the buffer state."""
        self.head = 0
        self.count = 0
        self.timestamps.fill(0.0)
        self.gyro.fill(0.0)
        self.acc.fill(0.0)

    def add_sample(
        self,
        t: float,
        omega: Union[np.ndarray, list],
        acc: Union[np.ndarray, list],
    ) -> None:
        """Add a sample to circular ring buffer.

        Args:
            t: Sample timestamp in seconds.
            omega: (3,) gyroscope measurement [rad/s].
            acc: (3,) accelerometer measurement [m/s^2].
        """
        idx = self.head
        self.timestamps[idx] = float(t)
        self.gyro[idx, :] = np.asarray(omega, dtype=np.float64).reshape(3)
        self.acc[idx, :] = np.asarray(acc, dtype=np.float64).reshape(3)

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
