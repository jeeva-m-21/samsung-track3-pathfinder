"""Degradation Monitor and Signal Quality Estimation (SPEC §12.6).

Implements:
1. High-frequency residual tracking:
   - r_k = ||a_k - a_{k-1}|| (accel norm)
   - rho_k = ||omega_k - omega_{k-1}|| (gyro norm)
2. Exponential moving variance:
   - var_k = (1 - alpha) * var_{k-1} + alpha * residual^2
   - alpha = 0.01 (monitor_alpha from configs/params.json)
3. Reference variance calibration:
   - Tracks nominal variance during initial static window (T_init = 1.0 s)
   - Freezes reference variance sigma_ref = sqrt(var_ref) once t > T_init
4. Adaptive noise ratio, quality score, and covariance inflation:
   - Noise ratio n = max(sigma_now / sigma_ref, 1.0)
   - Quality score = clamp(1.0 / n, 0.05, 1.0)
   - Covariance inflation factor kappa = clamp(n, 1.0, kappa_max) (kappa_max = 50.0)
5. Stationary gyro bias observation pseudo-update (SPEC §12.6):
   - Innovation y = omega_static_mean - b_g
   - Measurement matrix H = [0, 0, 0, I, 0] (3x15)
   - Measurement noise R = sigma_bgm^2 * I3 (sigma_bgm = 5e-3)
   - Exponential moving trend of static gyro mean

Constraints:
- Pure functions: no global state, no mutation of inputs.
- No dynamic lists or dicts inside filter loops.
- No printing in the filter loop.
- Portable to fixed-size C++17 Mat<15, 15> and scalar arithmetic.
"""

from dataclasses import dataclass, field
import math
from typing import Any, Dict, Optional, Tuple, Union
import numpy as np

from src.ref.eskf import (
    NominalState,
    eskf_measurement_update,
    load_params,
)


# ---------------------------------------------------------------------------
# Pure Helper Functions (Cookbook SPEC §12.6)
# ---------------------------------------------------------------------------


def compute_residual(
    curr: Union[np.ndarray, list],
    prev: Union[np.ndarray, list],
) -> float:
    """Compute Euclidean norm high-frequency residual between successive measurements (SPEC §12.6).

    Equations:
        r_k   = ||a_k - a_{k-1}||
        rho_k = ||omega_k - omega_{k-1}||

    Args:
        curr: Current 3-element vector [x, y, z].
        prev: Previous 3-element vector [x, y, z].

    Returns:
        Euclidean norm of vector difference as float.
    """
    diff = np.asarray(curr, dtype=np.float64).reshape(3) - np.asarray(
        prev, dtype=np.float64
    ).reshape(3)
    return float(np.linalg.norm(diff))


def update_moving_variance(
    prev_var: float,
    residual: float,
    alpha: float = 0.01,
) -> float:
    """Update exponential moving variance (SPEC §12.6).

    Equation:
        var_k = (1.0 - alpha) * var_{k-1} + alpha * residual^2

    Args:
        prev_var: Previous variance estimate.
        residual: High-frequency residual (norm of successive differences).
        alpha: Exponential smoothing factor (default 0.01).

    Returns:
        Updated variance estimate.
    """
    res_sq = float(residual) * float(residual)
    a = float(alpha)
    return (1.0 - a) * float(prev_var) + a * res_sq


def compute_noise_ratio(
    sigma_now: float,
    sigma_ref: float,
) -> float:
    """Compute noise ratio n = max(sigma_now / sigma_ref, 1.0) (SPEC §12.6).

    Args:
        sigma_now: Current noise standard deviation estimate.
        sigma_ref: Reference nominal noise standard deviation.

    Returns:
        Noise ratio n >= 1.0.
    """
    s_ref = float(sigma_ref)
    if s_ref <= 1e-12:
        return 1.0
    s_now = float(sigma_now)
    ratio = s_now / s_ref
    return max(ratio, 1.0)


def compute_quality_score(
    noise_ratio: float,
    min_quality: float = 0.05,
    max_quality: float = 1.0,
) -> float:
    """Compute bounded quality score (SPEC §12.6).

    Equation:
        quality = clamp(1.0 / n, min_quality, max_quality)

    Args:
        noise_ratio: Noise ratio n >= 1.0.
        min_quality: Floor bound (default 0.05).
        max_quality: Ceiling bound (default 1.0).

    Returns:
        Quality score in [min_quality, max_quality].
    """
    n = max(float(noise_ratio), 1e-12)
    val = 1.0 / n
    return max(float(min_quality), min(float(max_quality), val))


def compute_kappa(
    noise_ratio: float,
    kappa_max: float = 50.0,
) -> float:
    """Compute process noise covariance inflation factor kappa (SPEC §12.6).

    Equation:
        kappa = clamp(n, 1.0, kappa_max)

    Args:
        noise_ratio: Noise ratio n.
        kappa_max: Maximum allowed inflation (default 50.0).

    Returns:
        Inflation factor in [1.0, kappa_max].
    """
    n = float(noise_ratio)
    return max(1.0, min(float(kappa_max), n))


# ---------------------------------------------------------------------------
# Stationary Gyro Bias Observation Helpers (SPEC §12.6)
# ---------------------------------------------------------------------------


def build_gyro_bias_h_matrix() -> np.ndarray:
    """Build 3x15 measurement Jacobian matrix H for gyro bias pseudo-update (SPEC §12.6).

    Equation:
        H = [0, 0, 0, I, 0]  (3x15, delta_b_g block at 9:12 is I3)

    Returns:
        H: (3, 15) float64 measurement matrix.
    """
    H = np.zeros((3, 15), dtype=np.float64)
    H[:, 9:12] = np.eye(3, dtype=np.float64)
    return H


def build_gyro_bias_r_matrix(
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
) -> np.ndarray:
    """Build 3x3 measurement covariance matrix R for stationary gyro bias (SPEC §12.6).

    Equation:
        R = sigma_bgm^2 * r_scale * I3

    Args:
        params: Optional configuration dictionary.
        r_scale: Scaling factor (default 1.0).

    Returns:
        R: (3, 3) float64 diagonal covariance matrix.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    sigma_bgm = float(p_cfg.get("sigma_bgm", 5e-3))
    var_bgm = (sigma_bgm * sigma_bgm) * float(r_scale)
    return var_bgm * np.eye(3, dtype=np.float64)


def gyro_bias_innovation(
    state: NominalState,
    omega_meas: Union[np.ndarray, list],
) -> np.ndarray:
    """Compute stationary gyro bias innovation y (SPEC §12.6).

    Equation:
        y = omega_static_mean - b_g

    Args:
        state: Current NominalState.
        omega_meas: (3,) stationary window gyro mean.

    Returns:
        y: (3,) innovation vector.
    """
    w_arr = np.asarray(omega_meas, dtype=np.float64).reshape(3)
    return w_arr - state.b_g


def update_bias_trend(
    prev_trend: Union[np.ndarray, list],
    omega_mean: Union[np.ndarray, list],
    alpha: float = 0.01,
) -> np.ndarray:
    """Update exponential average of stationary gyro mean (SPEC §12.6).

    Equation:
        trend_k = (1.0 - alpha) * trend_{k-1} + alpha * omega_mean

    Args:
        prev_trend: (3,) previous bias trend vector.
        omega_mean: (3,) stationary window gyro mean.
        alpha: Smoothing factor (default 0.01).

    Returns:
        Updated (3,) float64 bias trend vector.
    """
    prev_arr = np.asarray(prev_trend, dtype=np.float64).reshape(3)
    mean_arr = np.asarray(omega_mean, dtype=np.float64).reshape(3)
    a = float(alpha)
    return (1.0 - a) * prev_arr + a * mean_arr


def gyro_bias_update(
    state: NominalState,
    P: np.ndarray,
    omega_meas: Union[np.ndarray, list],
    params: Optional[Dict[str, Any]] = None,
    r_scale: float = 1.0,
    apply_reset: bool = True,
) -> Tuple[NominalState, np.ndarray, np.ndarray]:
    """Execute ESKF measurement update using stationary gyro bias pseudo-measurement (SPEC §12.6).

    Equations:
        y = omega_meas - b_g
        H = [0, 0, 0, I, 0]
        R = sigma_bgm^2 * I3

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        omega_meas: (3,) stationary window gyro mean.
        params: Optional configuration dictionary.
        r_scale: Covariance scaling factor.
        apply_reset: Whether to apply error-state reset.

    Returns:
        new_state: Updated NominalState with refined gyro bias.
        new_P: Posterior conditioned covariance matrix.
        delta_x: (15,) error state correction vector.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    y = gyro_bias_innovation(state, omega_meas)
    H = build_gyro_bias_h_matrix()
    R = build_gyro_bias_r_matrix(params=p_cfg, r_scale=r_scale)

    new_state, new_P, delta_x = eskf_measurement_update(
        state, P, H, y, R, params=p_cfg, apply_reset=apply_reset
    )
    return new_state, new_P, delta_x


# ---------------------------------------------------------------------------
# Monitor State Dataclass & Pure Step Function
# ---------------------------------------------------------------------------


@dataclass
class MonitorState:
    """Immutable state container for degradation monitor (SPEC §12.6)."""

    t_prev: float = 0.0
    t_start: float = 0.0
    sample_count: int = 0
    prev_acc: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    prev_omega: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    var_acc: float = 0.0
    var_gyro: float = 0.0
    var_acc_ref: float = 0.0
    var_gyro_ref: float = 0.0
    calibrated: bool = False
    n_acc: float = 1.0
    n_gyro: float = 1.0
    noise_ratio: float = 1.0
    quality: float = 1.0
    kappa: float = 1.0
    residual_acc: float = 0.0
    residual_gyro: float = 0.0
    bias_trend: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    bias_trend_count: int = 0

    def copy(self) -> "MonitorState":
        """Return a deep copy with cloned arrays."""
        return MonitorState(
            t_prev=self.t_prev,
            t_start=self.t_start,
            sample_count=self.sample_count,
            prev_acc=self.prev_acc.copy(),
            prev_omega=self.prev_omega.copy(),
            var_acc=self.var_acc,
            var_gyro=self.var_gyro,
            var_acc_ref=self.var_acc_ref,
            var_gyro_ref=self.var_gyro_ref,
            calibrated=self.calibrated,
            n_acc=self.n_acc,
            n_gyro=self.n_gyro,
            noise_ratio=self.noise_ratio,
            quality=self.quality,
            kappa=self.kappa,
            residual_acc=self.residual_acc,
            residual_gyro=self.residual_gyro,
            bias_trend=self.bias_trend.copy(),
            bias_trend_count=self.bias_trend_count,
        )


def monitor_step(
    state: MonitorState,
    t: float,
    omega: Union[np.ndarray, list],
    acc: Union[np.ndarray, list],
    params: Optional[Dict[str, Any]] = None,
    is_stationary: bool = False,
    omega_static_mean: Optional[Union[np.ndarray, list]] = None,
) -> MonitorState:
    """Execute one degradation monitor update step as a pure function (SPEC §12.6).

    Args:
        state: Current MonitorState.
        t: Current sample timestamp in seconds.
        omega: (3,) gyroscope measurement [rad/s].
        acc: (3,) accelerometer measurement [m/s^2].
        params: Optional configuration dictionary.
        is_stationary: Whether current sample was detected stationary.
        omega_static_mean: Optional stationary window gyro mean.

    Returns:
        new_state: Updated MonitorState.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    alpha = float(p_cfg.get("monitor_alpha", 0.01))
    t_init = float(p_cfg.get("T_init", 1.0))
    kappa_max = float(p_cfg.get("kappa_max", 50.0))

    t_f = float(t)
    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)

    if state.sample_count == 0:
        new_state = state.copy()
        new_state.t_start = t_f
        new_state.t_prev = t_f
        new_state.prev_acc = acc_arr
        new_state.prev_omega = omega_arr
        new_state.sample_count = 1
        new_state.residual_acc = 0.0
        new_state.residual_gyro = 0.0
        new_state.quality = 1.0
        new_state.kappa = 1.0
        new_state.noise_ratio = 1.0
        return new_state

    # 1. Compute high-frequency residuals
    r_k = compute_residual(acc_arr, state.prev_acc)
    rho_k = compute_residual(omega_arr, state.prev_omega)

    # 2. Update exponential moving variance
    if state.sample_count == 1:
        var_acc = r_k * r_k
        var_gyro = rho_k * rho_k
    else:
        var_acc = update_moving_variance(state.var_acc, r_k, alpha=alpha)
        var_gyro = update_moving_variance(state.var_gyro, rho_k, alpha=alpha)

    # 3. Reference calibration logic over static window
    dt_elapsed = t_f - state.t_start
    if dt_elapsed < t_init:
        # Calibration phase: reference tracks current moving variance
        calibrated = False
        var_acc_ref = var_acc
        var_gyro_ref = var_gyro
        n_acc = 1.0
        n_gyro = 1.0
        noise_ratio = 1.0
        quality = 1.0
        kappa = 1.0
    else:
        # Calibration complete: reference variance is frozen
        if not state.calibrated:
            calibrated = True
            var_acc_ref = state.var_acc_ref if state.var_acc_ref > 0.0 else var_acc
            var_gyro_ref = state.var_gyro_ref if state.var_gyro_ref > 0.0 else var_gyro
        else:
            calibrated = True
            var_acc_ref = state.var_acc_ref
            var_gyro_ref = state.var_gyro_ref

        sigma_a_now = math.sqrt(max(var_acc, 0.0))
        sigma_g_now = math.sqrt(max(var_gyro, 0.0))
        sigma_a_ref = math.sqrt(max(var_acc_ref, 0.0))
        sigma_g_ref = math.sqrt(max(var_gyro_ref, 0.0))

        n_acc = compute_noise_ratio(sigma_a_now, sigma_a_ref)
        n_gyro = compute_noise_ratio(sigma_g_now, sigma_g_ref)
        noise_ratio = max(n_acc, n_gyro, 1.0)
        quality = compute_quality_score(noise_ratio)
        kappa = compute_kappa(noise_ratio, kappa_max=kappa_max)

    # 4. Stationary gyro bias trend tracking
    bias_trend = state.bias_trend.copy()
    trend_count = state.bias_trend_count
    if is_stationary and omega_static_mean is not None:
        mean_arr = np.asarray(omega_static_mean, dtype=np.float64).reshape(3)
        if trend_count == 0:
            bias_trend = mean_arr.copy()
        else:
            bias_trend = update_bias_trend(bias_trend, mean_arr, alpha=alpha)
        trend_count += 1

    return MonitorState(
        t_prev=t_f,
        t_start=state.t_start,
        sample_count=state.sample_count + 1,
        prev_acc=acc_arr,
        prev_omega=omega_arr,
        var_acc=var_acc,
        var_gyro=var_gyro,
        var_acc_ref=var_acc_ref,
        var_gyro_ref=var_gyro_ref,
        calibrated=calibrated,
        n_acc=n_acc,
        n_gyro=n_gyro,
        noise_ratio=noise_ratio,
        quality=quality,
        kappa=kappa,
        residual_acc=r_k,
        residual_gyro=rho_k,
        bias_trend=bias_trend,
        bias_trend_count=trend_count,
    )


# ---------------------------------------------------------------------------
# Streaming DegradationMonitor Class
# ---------------------------------------------------------------------------


class DegradationMonitor:
    """Streaming Degradation Monitor class for real-time filter execution (SPEC §12.6).

    Zero-allocation in the main loop, portable to C++.
    """

    def __init__(self, params: Optional[Dict[str, Any]] = None) -> None:
        """Initialize monitor with configuration parameters.

        Args:
            params: Optional parameter dictionary (loads configs/params.json if None).
        """
        if params is None:
            self._params = load_params()
        else:
            self._params = params

        self._state = MonitorState()

    def reset(self) -> None:
        """Reset internal state to initial conditions."""
        self._state = MonitorState()

    def step(
        self,
        t: float,
        omega: Union[np.ndarray, list],
        acc: Union[np.ndarray, list],
        is_stationary: bool = False,
        omega_static_mean: Optional[Union[np.ndarray, list]] = None,
    ) -> None:
        """Process incoming sample and update degradation metrics.

        Args:
            t: Timestamp in seconds.
            omega: (3,) gyro measurement [rad/s].
            acc: (3,) accel measurement [m/s^2].
            is_stationary: Optional boolean if current sample is stationary.
            omega_static_mean: Optional (3,) stationary window gyro mean.
        """
        self._state = monitor_step(
            self._state,
            t,
            omega,
            acc,
            params=self._params,
            is_stationary=is_stationary,
            omega_static_mean=omega_static_mean,
        )

    @property
    def quality(self) -> float:
        """Current quality score in [0.05, 1.0]."""
        return self._state.quality

    @property
    def kappa(self) -> float:
        """Current process noise inflation factor in [1.0, kappa_max]."""
        return self._state.kappa

    @property
    def noise_ratio(self) -> float:
        """Current noise ratio n = max(sigma_now / sigma_ref, 1.0)."""
        return self._state.noise_ratio

    @property
    def n(self) -> float:
        """Alias for noise_ratio."""
        return self._state.noise_ratio

    @property
    def calibrated(self) -> bool:
        """True if initial static calibration window is complete."""
        return self._state.calibrated

    @property
    def count(self) -> int:
        """Total number of processed samples."""
        return self._state.sample_count

    @property
    def var_acc(self) -> float:
        """Current moving variance of accel residual."""
        return self._state.var_acc

    @property
    def var_gyro(self) -> float:
        """Current moving variance of gyro residual."""
        return self._state.var_gyro

    @property
    def var_acc_ref(self) -> float:
        """Reference variance of accel residual."""
        return self._state.var_acc_ref

    @property
    def var_gyro_ref(self) -> float:
        """Reference variance of gyro residual."""
        return self._state.var_gyro_ref

    @property
    def residual_acc(self) -> float:
        """Most recent accel residual r_k."""
        return self._state.residual_acc

    @property
    def residual_gyro(self) -> float:
        """Most recent gyro residual rho_k."""
        return self._state.residual_gyro

    @property
    def bias_trend(self) -> np.ndarray:
        """Current stationary gyro bias trend vector."""
        return self._state.bias_trend
