"""Filter Initialisation (SPEC §12.2).

Performs:
- Static window tilt and bias estimation over T_init = 1.0 s (fallback 0.5 s).
- Gravity magnitude estimation clamped to [g_mag_clamp_min, g_mag_clamp_max] ([9.0, 10.5]).
- World gravity vector construction: g_w = [0, 0, -g_mag].
- Initial gyro bias: b_g = mean(gyro) when static; b_a = [0, 0, 0].
- Initial orientation: unit quaternion q aligning mean body specific force to world +z with zero yaw.
- Initial position p = 0 and velocity v = 0.
- Initial error covariance P = diag(sigma_theta_0^2, sigma_v_0^2, sigma_p_0^2, sigma_bg_0^2, sigma_ba_0^2).
- Motion fallback: if gyro std > motion_detect_gyro_std or accel-norm std > motion_detect_acc_std,
  use first sample's tilt, b_g = 0, and multiply sigma_theta_0 by motion_detect_sigma_theta_mult (5x).
"""

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Dict, NamedTuple, Optional, Tuple, Union
import numpy as np

from src.ref.mathlib import quat_normalize, quat_to_rot_matrix


@dataclass
class NominalState:
    """16-parameter nominal kinematic state (SPEC §12.2, §12.3).

    Attributes:
        p: Position in world frame, shape (3,), float64.
        v: Velocity in world frame, shape (3,), float64.
        q: Orientation quaternion [w, x, y, z] (body to world), shape (4,), float64.
        b_g: Gyroscope bias in body frame, shape (3,), float64.
        b_a: Accelerometer bias in body frame, shape (3,), float64.
    """

    p: np.ndarray
    v: np.ndarray
    q: np.ndarray
    b_g: np.ndarray
    b_a: np.ndarray

    def __post_init__(self) -> None:
        self.p = np.asarray(self.p, dtype=np.float64).reshape(3)
        self.v = np.asarray(self.v, dtype=np.float64).reshape(3)
        self.q = np.asarray(self.q, dtype=np.float64).reshape(4)
        self.b_g = np.asarray(self.b_g, dtype=np.float64).reshape(3)
        self.b_a = np.asarray(self.b_a, dtype=np.float64).reshape(3)

    def copy(self) -> "NominalState":
        """Return a deep copy with cloned numpy arrays."""
        return NominalState(
            p=self.p.copy(),
            v=self.v.copy(),
            q=self.q.copy(),
            b_g=self.b_g.copy(),
            b_a=self.b_a.copy(),
        )

    def to_array(self) -> np.ndarray:
        """Serialize state to a flat 16-element float64 array [p(3), v(3), q(4), b_g(3), b_a(3)]."""
        arr = np.empty(16, dtype=np.float64)
        arr[0:3] = self.p
        arr[3:6] = self.v
        arr[6:10] = self.q
        arr[10:13] = self.b_g
        arr[13:16] = self.b_a
        return arr

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "NominalState":
        """Construct NominalState from a 16-element array."""
        arr = np.asarray(arr, dtype=np.float64).reshape(16)
        return cls(
            p=arr[0:3].copy(),
            v=arr[3:6].copy(),
            q=arr[6:10].copy(),
            b_g=arr[10:13].copy(),
            b_a=arr[13:16].copy(),
        )


class InitResult(NamedTuple):
    """Result of filter initialisation."""

    state: NominalState
    P: np.ndarray  # (15, 15) float64
    g_w: np.ndarray  # (3,) float64
    g_mag: float
    motion_detected: bool
    samples_used: int
    t0: Union[int, float]


def get_default_params_path() -> Path:
    """Return path to configs/params.json."""
    return Path(__file__).resolve().parent.parent.parent / "configs" / "params.json"


def load_params(params: Optional[Union[Dict[str, Any], str, Path]] = None) -> Dict[str, Any]:
    """Load parameter dictionary or return default parameters from configs/params.json."""
    if params is None:
        params_path = get_default_params_path()
        if params_path.is_file():
            with open(params_path, "r", encoding="utf-8") as f:
                return json.load(f)
        return {
            "T_init": 1.0,
            "T_init_fallback": 0.5,
            "g_mag_clamp_min": 9.0,
            "g_mag_clamp_max": 10.5,
            "sigma_theta_0": 0.02,
            "sigma_v_0": 0.05,
            "sigma_p_0": 0.01,
            "sigma_bg_0": 0.02,
            "sigma_ba_0": 0.2,
            "motion_detect_gyro_std": 0.2,
            "motion_detect_acc_std": 0.5,
            "motion_detect_sigma_theta_mult": 5.0,
        }
    if isinstance(params, (str, Path)):
        with open(params, "r", encoding="utf-8") as f:
            return json.load(f)
    return dict(params)


def compute_initial_tilt(a_mean: np.ndarray) -> np.ndarray:
    """Compute initial unit quaternion [w, x, y, z] aligning specific force to world +z (SPEC §12.2).

    R(q) * (a_mean / |a_mean|) = [0, 0, 1]^T with zero yaw.
    Body specific force maps to world +z.

    Args:
        a_mean: (3,) specific force vector in body frame.

    Returns:
        q: (4,) unit Hamilton quaternion [w, x, y, z] with q_w >= 0.
    """
    a_vec = np.asarray(a_mean, dtype=np.float64).reshape(3)
    norm_a = float(np.linalg.norm(a_vec))
    if norm_a < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    v = a_vec / norm_a  # unit body vector
    u = np.array([0.0, 0.0, 1.0], dtype=np.float64)  # target world +z

    cos_theta = float(v[2])  # dot(v, [0, 0, 1]) == v[2]

    # Threshold for parallel vectors
    if cos_theta > 0.999999999:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    # Threshold for anti-parallel vectors (180 deg rotation about x)
    if cos_theta < -0.999999999:
        return np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)

    # Cross product v x u = [v_y * 1 - v_z * 0, v_z * 0 - v_x * 1, v_x * 0 - v_y * 0]
    # v x u = [v[1], -v[0], 0.0]
    # Notice the z-component is identically zero, ensuring zero yaw.
    # Half-angle quaternion: q_raw = [1 + cos_theta, (v x u)_x, (v x u)_y, (v x u)_z]
    q_raw = np.array([1.0 + cos_theta, v[1], -v[0], 0.0], dtype=np.float64)
    q = quat_normalize(q_raw)

    # Ensure canonical positive scalar part
    if q[0] < 0.0:
        q = -q
    return q


def detect_motion(
    omega_window: np.ndarray,
    acc_window: np.ndarray,
    params: Optional[Dict[str, Any]] = None,
) -> bool:
    """Detect dynamic motion in the initialisation window (SPEC §12.2).

    Condition: gyro std > motion_detect_gyro_std (0.2 rad/s) OR
               accel-norm std > motion_detect_acc_std (0.5 m/s^2).

    Args:
        omega_window: (N, 3) gyro readings.
        acc_window: (N, 3) accelerometer readings.
        params: Configuration dictionary.

    Returns:
        True if dynamic motion is detected, False if static.
    """
    cfg = load_params(params)
    gyro_thresh = float(cfg.get("motion_detect_gyro_std", 0.2))
    acc_thresh = float(cfg.get("motion_detect_acc_std", 0.5))

    omega_arr = np.asarray(omega_window, dtype=np.float64)
    acc_arr = np.asarray(acc_window, dtype=np.float64)

    if len(omega_arr) < 2 or len(acc_arr) < 2:
        return False

    # Check axis-wise and norm gyro variation
    gyro_std_axes = np.std(omega_arr, axis=0)
    gyro_std_norm = np.std(np.linalg.norm(omega_arr, axis=1))

    # Check accel norm variation
    acc_norms = np.linalg.norm(acc_arr, axis=1)
    acc_norm_std = np.std(acc_norms)

    is_motion = bool(
        np.any(gyro_std_axes > gyro_thresh)
        or (gyro_std_norm > gyro_thresh)
        or (acc_norm_std > acc_thresh)
    )
    return is_motion


def initialize_covariance(
    params: Optional[Dict[str, Any]] = None,
    motion_detected: bool = False,
) -> np.ndarray:
    """Construct initial 15x15 error state covariance matrix P0 (SPEC §12.2).

    P0 = diag(sigma_theta_0^2, sigma_v_0^2, sigma_p_0^2, sigma_bg_0^2, sigma_ba_0^2).
    If motion is detected, sigma_theta_0 is multiplied by motion_detect_sigma_theta_mult (default 5.0).

    Args:
        params: Configuration parameters dictionary.
        motion_detected: Boolean flag indicating if motion fallback was triggered.

    Returns:
        P0: (15, 15) float64 diagonal matrix.
    """
    cfg = load_params(params)

    sigma_theta = float(cfg.get("sigma_theta_0", 0.02))
    if motion_detected:
        sigma_theta *= float(cfg.get("motion_detect_sigma_theta_mult", 5.0))

    sigma_v = float(cfg.get("sigma_v_0", 0.05))
    sigma_p = float(cfg.get("sigma_p_0", 0.01))
    sigma_bg = float(cfg.get("sigma_bg_0", 0.02))
    sigma_ba = float(cfg.get("sigma_ba_0", 0.2))

    diag_vals = np.array(
        [sigma_theta**2] * 3
        + [sigma_v**2] * 3
        + [sigma_p**2] * 3
        + [sigma_bg**2] * 3
        + [sigma_ba**2] * 3,
        dtype=np.float64,
    )
    return np.diag(diag_vals)


def initialize_from_window(
    t: Union[np.ndarray, list],
    omega: Union[np.ndarray, list],
    acc: Union[np.ndarray, list],
    params: Optional[Dict[str, Any]] = None,
) -> InitResult:
    """Perform filter initialisation on a batch window of IMU samples (SPEC §12.2).

    Selects the first T_init = 1.0 s of valid samples (fallback T_init_fallback = 0.5 s).

    Args:
        t: Array of timestamps (in nanoseconds or seconds).
        omega: Array of shape (N, 3), gyro readings [rad/s].
        acc: Array of shape (N, 3), accelerometer readings [m/s^2].
        params: Configuration dictionary.

    Returns:
        InitResult containing initial NominalState, P0, g_w, g_mag, motion flag, samples used, t0.

    Raises:
        ValueError: If valid sample duration is less than T_init_fallback.
    """
    cfg = load_params(params)
    t_init_target = float(cfg.get("T_init", 1.0))
    t_init_fallback = float(cfg.get("T_init_fallback", 0.5))
    g_min = float(cfg.get("g_mag_clamp_min", 9.0))
    g_max = float(cfg.get("g_mag_clamp_max", 10.5))

    t_arr = np.asarray(t)
    omega_arr = np.asarray(omega, dtype=np.float64)
    acc_arr = np.asarray(acc, dtype=np.float64)

    n_samples = len(t_arr)
    if n_samples < 2:
        raise ValueError(f"Insufficient samples for initialisation: n={n_samples} < 2")

    # Determine time units (nanoseconds vs seconds)
    total_span_raw = float(t_arr[-1] - t_arr[0])
    is_ns = total_span_raw > 1e6
    time_scale = 1e-9 if is_ns else 1.0
    t_s = (t_arr - t_arr[0]) * time_scale

    total_duration = float(t_s[-1])
    if total_duration < t_init_fallback:
        raise ValueError(
            f"Insufficient samples for initialisation: duration {total_duration:.3f}s "
            f"is less than fallback {t_init_fallback:.3f}s"
        )

    # Window selection: within T_init if available, otherwise fallback window
    if total_duration >= t_init_target:
        mask = t_s <= t_init_target
    else:
        mask = np.ones(n_samples, dtype=bool)

    omega_win = omega_arr[mask]
    acc_win = acc_arr[mask]
    samples_used = int(np.sum(mask))

    # Detect motion
    motion = detect_motion(omega_win, acc_win, cfg)

    # Initial estimates
    if motion:
        # Use first sample's tilt and zero gyro bias
        b_g = np.zeros(3, dtype=np.float64)
        a_ref = acc_win[0]
    else:
        # Mean gyro as bias and mean accel for tilt
        b_g = np.mean(omega_win, axis=0).astype(np.float64)
        a_ref = np.mean(acc_win, axis=0).astype(np.float64)

    b_a = np.zeros(3, dtype=np.float64)
    p = np.zeros(3, dtype=np.float64)
    v = np.zeros(3, dtype=np.float64)

    # Tilt quaternion
    q0 = compute_initial_tilt(a_ref)

    # Gravity magnitude and world gravity vector
    g_raw = float(np.linalg.norm(a_ref))
    g_mag = float(np.clip(g_raw, g_min, g_max))
    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)

    # Initial covariance
    P0 = initialize_covariance(cfg, motion_detected=motion)

    state = NominalState(p=p, v=v, q=q0, b_g=b_g, b_a=b_a)

    return InitResult(
        state=state,
        P=P0,
        g_w=g_w,
        g_mag=g_mag,
        motion_detected=motion,
        samples_used=samples_used,
        t0=t_arr[0],
    )


class OnlineInitializer:
    """Streaming accumulator for filter initialisation (SPEC §12.2).

    Portable to fixed-size C++ ring buffer / preallocated arrays.
    Accumulates valid IMU samples until T_init (1.0 s) is reached.
    """

    MAX_BUFFER_SAMPLES: int = 1000

    def __init__(self, params: Optional[Dict[str, Any]] = None) -> None:
        self.params = load_params(params)
        self.t_init_target = float(self.params.get("T_init", 1.0))
        self.t_init_fallback = float(self.params.get("T_init_fallback", 0.5))

        # Preallocated buffers for C++ portability
        self._t_buf = np.empty(self.MAX_BUFFER_SAMPLES, dtype=np.int64)
        self._omega_buf = np.empty((self.MAX_BUFFER_SAMPLES, 3), dtype=np.float64)
        self._acc_buf = np.empty((self.MAX_BUFFER_SAMPLES, 3), dtype=np.float64)
        self._count = 0
        self._is_ready = False
        self._result: Optional[InitResult] = None

    @property
    def is_ready(self) -> bool:
        """Return True if initialisation has completed."""
        return self._is_ready

    def feed(
        self,
        t_ns: int,
        omega: np.ndarray,
        acc: np.ndarray,
        is_valid: bool = True,
    ) -> bool:
        """Feed a sample into the initializer.

        Returns True as soon as initialisation is complete.
        """
        if self._is_ready:
            return True
        if not is_valid:
            return False

        if self._count < self.MAX_BUFFER_SAMPLES:
            self._t_buf[self._count] = int(t_ns)
            self._omega_buf[self._count] = np.asarray(omega, dtype=np.float64)
            self._acc_buf[self._count] = np.asarray(acc, dtype=np.float64)
            self._count += 1

        # Check if duration reached T_init
        if self._count >= 2:
            dt_span_s = (self._t_buf[self._count - 1] - self._t_buf[0]) * 1e-9
            if dt_span_s >= self.t_init_target:
                self._compute_initialisation()
                return True

        return False

    def finalize_fallback(self) -> bool:
        """Attempt initialisation using fallback window if T_init was not reached."""
        if self._is_ready:
            return True
        if self._count < 2:
            return False

        dt_span_s = (self._t_buf[self._count - 1] - self._t_buf[0]) * 1e-9
        if dt_span_s >= self.t_init_fallback:
            self._compute_initialisation()
            return True
        return False

    def _compute_initialisation(self) -> None:
        """Compute initial filter state from buffered samples."""
        t_slice = self._t_buf[: self._count]
        omega_slice = self._omega_buf[: self._count]
        acc_slice = self._acc_buf[: self._count]

        self._result = initialize_from_window(t_slice, omega_slice, acc_slice, params=self.params)
        self._is_ready = True

    def get_result(self) -> Optional[InitResult]:
        """Return the initialization result, or None if not ready."""
        return self._result
