"""Gap, Invalid-Sample, and Dropout Handler (SPEC §12.7, WP-26).

Handles:
1. Classification of invalid IMU samples:
   - NaN or Inf in any field
   - All six values exactly zero
   - Stuck sensor (identical values in all six fields for >= 3 consecutive rows)
   - Out-of-range sensor readings (|omega| > 35 rad/s or |a| > 160 m/s^2)
2. Non-positive delta time (dt <= 0):
   - Duplicate or backward timestamps freeze / copy previous pose
   - Skips integration and updates
3. Normal dt propagation:
   - dt <= 2.5 * dt_nom and valid sample
4. Large dt gap (dt > 2.5 * dt_nom):
   - Sub-stepping with n = ceil(dt / dt_nom) capped at gap_substep_max (800)
   - Velocity decay: v <- v * exp(-dt_sub / tau_gap) with tau_gap = 2.0s
   - Orientation integration with smoothed gyro omega_s (tau = 0.25s)
   - Covariance Q inflated by kappa_gap = 5.0
   - Sets recovery counter = recover_steps (100)
5. Single invalid row handling:
   - Treated as 1 dt_nom gap sub-step, still emits one pose row
6. Post-gap recovery:
   - Measurement noise covariance R multiplied by (1 + 10 * recover / recover_steps)
   - Decays toward 1.0 as recover counts down
7. Fixed-size circular ring buffer:
   - Capacity 64 states for rollback to the last reliable state

Constraints:
- Pure functions where possible: step(state, sample, params) -> state.
- No dynamic lists or dicts inside the filter math loop.
- No printing in the filter loop.
- Portable to fixed-size C++ Mat<15, 15> and scalar arithmetic.
- Parameters from configs/params.json.
"""

from enum import Enum
import math
from typing import Any, Dict, List, NamedTuple, Optional, Tuple, Union
import numpy as np

from src.ref.eskf import (
    NominalState,
    build_f_matrix,
    build_q_discrete,
    condition_covariance,
    load_params,
)
from src.ref.mathlib import (
    quat_exp,
    quat_mult,
    quat_normalize,
    quat_sign_continuity,
)


class GapCase(str, Enum):
    """Classification of time-step and sample conditions (SPEC §12.7)."""
    NORMAL = "normal"
    ZERO_OR_NEGATIVE_DT = "zero_or_negative_dt"
    GAP = "gap"
    INVALID_SAMPLE = "invalid_sample"


class GapStepResult(NamedTuple):
    """Result of processing a single sample through the GapHandler."""
    state: NominalState
    P: np.ndarray
    case: GapCase
    substeps: int
    apply_updates: bool
    r_scale: float
    is_valid: bool
    reason: str


# ---------------------------------------------------------------------------
# 1. Sample Classification Pure Functions
# ---------------------------------------------------------------------------


def classify_sample(
    omega: Union[np.ndarray, list],
    acc: Union[np.ndarray, list],
    prev_sample: Optional[Union[np.ndarray, list]] = None,
    repeat_count: int = 0,
    params: Optional[Dict[str, Any]] = None,
) -> Tuple[bool, str]:
    """Classify an IMU sample as valid or invalid according to SPEC §12.7.

    A sample is invalid if:
    - Any field is non-finite (NaN or Inf)
    - All six values are exactly zero
    - Sample equals previous one in all six fields for >= 3 consecutive rows (stuck)
    - |omega| > invalid_gyro_max (35.0 rad/s) or |a| > invalid_acc_max (160.0 m/s^2)

    Args:
        omega: (3,) angular velocity vector [rad/s].
        acc: (3,) specific force vector [m/s^2].
        prev_sample: Optional (6,) vector of previous [omega, acc].
        repeat_count: Number of consecutive identical rows seen prior to this sample.
        params: Optional configuration dictionary.

    Returns:
        (is_valid, reason): Tuple of bool and classification reason string.
    """
    if params is None:
        p_cfg = load_params()
    else:
        p_cfg = params

    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)

    # 1. Check for NaN or Inf in any field
    if not (np.all(np.isfinite(omega_arr)) and np.all(np.isfinite(acc_arr))):
        return False, "nan_or_inf"

    # 2. Check for all six zeros
    if np.all(omega_arr == 0.0) and np.all(acc_arr == 0.0):
        return False, "all_zeros"

    # 3. Check out of range bounds
    gyro_max = float(p_cfg.get("invalid_gyro_max", 35.0))
    acc_max = float(p_cfg.get("invalid_acc_max", 160.0))

    if np.any(np.abs(omega_arr) > gyro_max) or float(np.linalg.norm(omega_arr)) > gyro_max:
        return False, "out_of_range_gyro"

    if np.any(np.abs(acc_arr) > acc_max) or float(np.linalg.norm(acc_arr)) > acc_max:
        return False, "out_of_range_acc"

    # 4. Check stuck sensor (>= 3 consecutive identical rows)
    if prev_sample is not None:
        curr_6 = np.concatenate([omega_arr, acc_arr])
        prev_6 = np.asarray(prev_sample, dtype=np.float64).reshape(6)
        if np.array_equal(curr_6, prev_6):
            if repeat_count + 1 >= 3:
                return False, "stuck"

    return True, "valid"


def compute_dt_nom(
    dt_samples: Union[np.ndarray, List[float]],
    default_dt: float = 0.005,
) -> float:
    """Compute nominal sampling interval dt_nom as median of valid dt samples (SPEC §12.7).

    Args:
        dt_samples: Sequence of dt values (first up to 200 valid samples).
        default_dt: Fallback dt if insufficient samples (default 0.005s = 200 Hz).

    Returns:
        dt_nom: Median dt in seconds.
    """
    if len(dt_samples) == 0:
        return float(default_dt)

    arr = np.asarray(dt_samples, dtype=np.float64)
    valid = arr[(arr > 0.0) & np.isfinite(arr)]
    if len(valid) == 0:
        return float(default_dt)

    sample_subset = valid[:200]
    return float(np.median(sample_subset))


# ---------------------------------------------------------------------------
# 2. Motion Smoothing and Recovery Functions
# ---------------------------------------------------------------------------


def update_smoothed_gyro(
    omega_s: Union[np.ndarray, list],
    omega: Union[np.ndarray, list],
    b_g: Union[np.ndarray, list],
    dt: float,
    tau: float = 0.25,
) -> np.ndarray:
    """Update smoothed motion angular velocity omega_s (SPEC §12.7).

    Equation:
        omega_corrected = omega - b_g
        alpha = 1.0 - exp(-dt / tau)
        omega_s <- (1.0 - alpha) * omega_s + alpha * omega_corrected

    Args:
        omega_s: Current (3,) smoothed angular velocity [rad/s].
        omega: Current (3,) raw angular velocity measurement [rad/s].
        b_g: Current (3,) estimated gyroscope bias [rad/s].
        dt: Time increment in seconds.
        tau: Time constant in seconds (default 0.25s).

    Returns:
        Updated (3,) smoothed angular velocity array.
    """
    omega_s_arr = np.asarray(omega_s, dtype=np.float64).reshape(3)
    if dt <= 0.0:
        return omega_s_arr.copy()

    omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
    bg_arr = np.asarray(b_g, dtype=np.float64).reshape(3)

    omega_corr = omega_arr - bg_arr
    alpha = 1.0 - math.exp(-float(dt) / float(tau))
    return (1.0 - alpha) * omega_s_arr + alpha * omega_corr


def get_recovery_multiplier(
    recover: int,
    recover_steps: int = 100,
) -> float:
    """Compute measurement covariance inflation multiplier during post-gap recovery (SPEC §12.7).

    Equation:
        multiplier = 1.0 + 10.0 * (recover / recover_steps)  for recover > 0
        multiplier = 1.0                                     for recover <= 0

    Args:
        recover: Current remaining recovery step count.
        recover_steps: Total duration of recovery period (default 100).

    Returns:
        Scalar covariance multiplier >= 1.0.
    """
    if recover <= 0:
        return 1.0
    steps = max(1, int(recover_steps))
    return 1.0 + 10.0 * (float(recover) / float(steps))


# ---------------------------------------------------------------------------
# 3. Gap Sub-step and Propagation Pure Functions
# ---------------------------------------------------------------------------


def gap_substep(
    state: NominalState,
    P: np.ndarray,
    omega_s: Union[np.ndarray, list],
    dt_sub: float,
    params: Dict[str, Any],
) -> Tuple[NominalState, np.ndarray]:
    """Execute a single gap propagation sub-step (SPEC §12.7).

    Equations:
        tau_gap: velocity decay time constant (default 2.0 s)
        decay = exp(-dt_sub / tau_gap)
        v_next = v * decay
        p_next = p + 0.5 * (v + v_next) * dt_sub
        delta_q = exp(omega_s * dt_sub)
        q_next = normalize(q (x) delta_q)
        P_next = F P F^T + Q * kappa_gap

    Args:
        state: Current NominalState.
        P: (15, 15) error covariance matrix.
        omega_s: (3,) smoothed angular velocity [rad/s].
        dt_sub: Sub-step duration in seconds.
        params: Configuration dictionary containing tau_gap, kappa_gap, etc.

    Returns:
        (next_state, next_P): Propagated nominal state and conditioned error covariance.
    """
    dt_f = float(dt_sub)
    tau_gap = float(params.get("tau_gap", 2.0))
    kappa_gap = float(params.get("kappa_gap", 5.0))
    diag_floor = float(params.get("cov_diag_floor", 1e-12))
    diag_cap = float(params.get("cov_diag_cap", 1e6))

    omega_s_arr = np.asarray(omega_s, dtype=np.float64).reshape(3)

    # 1. Velocity decay and position integration
    decay = math.exp(-dt_f / tau_gap)
    v_next = state.v * decay
    p_next = state.p + 0.5 * (state.v + v_next) * dt_f

    # 2. Orientation integration using smoothed gyro omega_s
    delta_q = quat_exp(omega_s_arr * dt_f)
    q_rot = quat_mult(state.q, delta_q)
    q_next = quat_normalize(q_rot)
    q_next = quat_sign_continuity(q_next, state.q)

    # Biases remain constant across gap propagation
    next_state = NominalState(
        q=q_next,
        v=v_next,
        p=p_next,
        b_g=state.b_g.copy(),
        b_a=state.b_a.copy(),
    )

    # 3. Covariance propagation with inflated process noise
    F = build_f_matrix(state, omega_s_arr + state.b_g, state.b_a, dt_f)

    Q_base = build_q_discrete(dt_f, params, kappa=1.0)
    Q_gap = Q_base * kappa_gap

    P_pred = F @ P @ F.T + Q_gap
    next_P = condition_covariance(P_pred, diag_floor=diag_floor, diag_cap=diag_cap)

    return next_state, next_P


def propagate_gap(
    state: NominalState,
    P: np.ndarray,
    omega_s: Union[np.ndarray, list],
    dt: float,
    dt_nom: float,
    params: Dict[str, Any],
) -> Tuple[NominalState, np.ndarray, int]:
    """Propagate state and covariance across a large time gap via sub-stepping (SPEC §12.7).

    Equation:
        n = min(ceil(dt / dt_nom), gap_substep_max)
        dt_sub = dt / n
        Loop n steps with gap_substep(...)

    Args:
        state: NominalState prior to gap.
        P: Covariance matrix prior to gap.
        omega_s: Smoothed pre-gap angular velocity.
        dt: Total gap duration in seconds.
        dt_nom: Nominal sampling interval in seconds.
        params: Configuration dictionary.

    Returns:
        (state_after, P_after, n_steps): Propagated state, covariance, and number of sub-steps.
    """
    gap_substep_max = int(params.get("gap_substep_max", 800))
    nom_dt = max(float(dt_nom), 1e-6)

    # Numerical epsilon tolerance to prevent FP rounding from inflating exact multiples
    n = int(math.ceil((float(dt) / nom_dt) - 1e-9))
    n = max(1, min(n, gap_substep_max))
    dt_sub = float(dt) / float(n)

    curr_state = state
    curr_P = P
    for _ in range(n):
        curr_state, curr_P = gap_substep(curr_state, curr_P, omega_s, dt_sub, params)

    return curr_state, curr_P, n


# ---------------------------------------------------------------------------
# 4. Ring Buffer for Rollback to Last Reliable State
# ---------------------------------------------------------------------------


class StateRingBuffer:
    """Fixed-capacity circular ring buffer storing reliable state history (SPEC §12.7).

    Capacity is fixed (default 64) with no heap reallocations.
    Stores tuples of (t_ns, NominalState, P, omega_s).
    """

    def __init__(self, capacity: int = 64):
        self.capacity = max(1, int(capacity))
        self._t_ns: List[int] = [0] * self.capacity
        self._states: List[Optional[NominalState]] = [None] * self.capacity
        self._covs: List[Optional[np.ndarray]] = [None] * self.capacity
        self._omega_s: List[Optional[np.ndarray]] = [None] * self.capacity
        self._head: int = 0
        self._count: int = 0

    def push(
        self,
        t_ns: int,
        state: NominalState,
        P: np.ndarray,
        omega_s: np.ndarray,
    ) -> None:
        """Push a new snapshot into the ring buffer."""
        idx = self._head
        self._t_ns[idx] = int(t_ns)
        self._states[idx] = NominalState(
            q=state.q.copy(),
            v=state.v.copy(),
            p=state.p.copy(),
            b_g=state.b_g.copy(),
            b_a=state.b_a.copy(),
        )
        self._covs[idx] = P.copy()
        self._omega_s[idx] = omega_s.copy()

        self._head = (self._head + 1) % self.capacity
        if self._count < self.capacity:
            self._count += 1

    def __len__(self) -> int:
        return self._count

    def get_latest(self) -> Optional[Tuple[int, NominalState, np.ndarray, np.ndarray]]:
        """Retrieve most recently pushed snapshot."""
        if self._count == 0:
            return None
        latest_idx = (self._head - 1 + self.capacity) % self.capacity
        return (
            self._t_ns[latest_idx],
            self._states[latest_idx],  # type: ignore[return-value]
            self._covs[latest_idx],  # type: ignore[return-value]
            self._omega_s[latest_idx],  # type: ignore[return-value]
        )

    def get_oldest(self) -> Optional[Tuple[int, NominalState, np.ndarray, np.ndarray]]:
        """Retrieve oldest snapshot currently stored in the ring buffer."""
        if self._count == 0:
            return None
        oldest_idx = (self._head - self._count + self.capacity) % self.capacity
        return (
            self._t_ns[oldest_idx],
            self._states[oldest_idx],  # type: ignore[return-value]
            self._covs[oldest_idx],  # type: ignore[return-value]
            self._omega_s[oldest_idx],  # type: ignore[return-value]
        )

    def find_closest(
        self,
        t_target_ns: int,
    ) -> Optional[Tuple[int, NominalState, np.ndarray, np.ndarray]]:
        """Find stored snapshot with timestamp closest to t_target_ns."""
        if self._count == 0:
            return None

        best_idx = -1
        best_diff = float("inf")
        for i in range(self._count):
            idx = (self._head - 1 - i + self.capacity) % self.capacity
            diff = abs(self._t_ns[idx] - t_target_ns)
            if diff < best_diff:
                best_diff = diff
                best_idx = idx

        if best_idx < 0:
            return None

        return (
            self._t_ns[best_idx],
            self._states[best_idx],  # type: ignore[return-value]
            self._covs[best_idx],  # type: ignore[return-value]
            self._omega_s[best_idx],  # type: ignore[return-value]
        )

    def clear(self) -> None:
        """Clear all entries in the ring buffer."""
        self._head = 0
        self._count = 0


# ---------------------------------------------------------------------------
# 5. Streaming GapHandler Class
# ---------------------------------------------------------------------------


class GapHandler:
    """Streaming gap, dropout, and invalid-sample handler (SPEC §12.7).

    Orchestrates:
    - Sample validity checks
    - dt_nom tracking and calibration over first 200 valid samples
    - Large gap sub-stepping with velocity decay and covariance inflation
    - Pose freezing on non-positive dt
    - Post-gap measurement recovery counter and R_meas scaling
    - Gyro motion smoothing omega_s
    - State snapshot ring buffer for failure rollback
    """

    def __init__(self, params: Optional[Dict[str, Any]] = None):
        self.params: Dict[str, Any] = load_params() if params is None else dict(params)
        self.recover_steps: int = int(self.params.get("recover_steps", 100))
        self.recover: int = 0
        self.dt_nom: float = 0.005
        self.dt_nom_frozen: bool = False
        self._dt_samples: List[float] = []

        self.omega_s: np.ndarray = np.zeros(3, dtype=np.float64)
        self.prev_t_ns: Optional[int] = None
        self.prev_sample_6: Optional[np.ndarray] = None
        self.repeat_count: int = 0

        ring_size = int(self.params.get("ring_buffer_size", 64))
        self.ring_buffer = StateRingBuffer(capacity=ring_size)

    def process_sample(
        self,
        t_ns: int,
        omega: Union[np.ndarray, list],
        acc: Union[np.ndarray, list],
        state: NominalState,
        P: np.ndarray,
    ) -> GapStepResult:
        """Process an incoming IMU measurement row according to SPEC §12.7.

        Handles:
        1. First sample initialization
        2. Non-positive dt (dt <= 0): freeze/repeat pose, skip updates
        3. Invalid sample: 1 dt_nom sub-step, skip updates, set recover = recover_steps
        4. Gap (dt > 2.5 * dt_nom): sub-step propagation, skip updates, set recover = recover_steps
        5. Normal (dt <= 2.5 * dt_nom and valid): advance recovery counter, allow updates

        Args:
            t_ns: Nanosecond timestamp.
            omega: (3,) angular velocity [rad/s].
            acc: (3,) specific force [m/s^2].
            state: Current NominalState.
            P: Current (15, 15) error covariance matrix.

        Returns:
            GapStepResult containing propagated state, covariance, case enum, and update flags.
        """
        omega_arr = np.asarray(omega, dtype=np.float64).reshape(3)
        acc_arr = np.asarray(acc, dtype=np.float64).reshape(3)

        # 1. First sample handling
        if self.prev_t_ns is None:
            is_valid, reason = classify_sample(
                omega_arr,
                acc_arr,
                prev_sample=None,
                repeat_count=0,
                params=self.params,
            )
            self.prev_t_ns = int(t_ns)
            self.prev_sample_6 = np.concatenate([omega_arr, acc_arr])
            self.repeat_count = 1

            if is_valid:
                self.omega_s = omega_arr - state.b_g
            else:
                self.omega_s = np.zeros(3, dtype=np.float64)

            self.ring_buffer.push(int(t_ns), state, P, self.omega_s)
            return GapStepResult(
                state=state,
                P=P,
                case=GapCase.NORMAL if is_valid else GapCase.INVALID_SAMPLE,
                substeps=1,
                apply_updates=is_valid,
                r_scale=1.0,
                is_valid=is_valid,
                reason=reason,
            )

        # 2. Check delta time: integer difference first to avoid float64 precision loss on 19-digit nanosecond timestamps
        dt_ns = int(t_ns) - int(self.prev_t_ns)
        dt = float(dt_ns) * 1e-9

        # Case dt <= 0: freeze/copy previous pose, skip integration and updates
        if dt <= 0.0:
            frozen_state = NominalState(
                q=state.q.copy(),
                v=state.v.copy(),
                p=state.p.copy(),
                b_g=state.b_g.copy(),
                b_a=state.b_a.copy(),
            )
            return GapStepResult(
                state=frozen_state,
                P=P.copy(),
                case=GapCase.ZERO_OR_NEGATIVE_DT,
                substeps=0,
                apply_updates=False,
                r_scale=get_recovery_multiplier(self.recover, self.recover_steps),
                is_valid=False,
                reason="dt_non_positive",
            )

        # 3. Classify sample validity
        is_valid, reason = classify_sample(
            omega_arr,
            acc_arr,
            prev_sample=self.prev_sample_6,
            repeat_count=self.repeat_count,
            params=self.params,
        )

        curr_6 = np.concatenate([omega_arr, acc_arr])
        if self.prev_sample_6 is not None and np.array_equal(curr_6, self.prev_sample_6):
            self.repeat_count += 1
        else:
            self.repeat_count = 1
            self.prev_sample_6 = curr_6

        # 4. Handle invalid sample
        # "Invalid row | Same as gap mode for one dt_nom step; still emit one row"
        if not is_valid:
            self.recover = self.recover_steps
            r_scale = get_recovery_multiplier(self.recover, self.recover_steps)

            next_state, next_P = gap_substep(
                state,
                P,
                self.omega_s,
                self.dt_nom,
                self.params,
            )
            self.prev_t_ns = int(t_ns)
            self.ring_buffer.push(int(t_ns), next_state, next_P, self.omega_s)

            return GapStepResult(
                state=next_state,
                P=next_P,
                case=GapCase.INVALID_SAMPLE,
                substeps=1,
                apply_updates=False,
                r_scale=r_scale,
                is_valid=False,
                reason=reason,
            )

        # 5. Handle valid sample: update dt_nom calibration if not frozen
        if not self.dt_nom_frozen:
            self._dt_samples.append(dt)
            self.dt_nom = compute_dt_nom(self._dt_samples)
            if len(self._dt_samples) >= 200:
                self.dt_nom_frozen = True

        # Update smoothed angular velocity omega_s
        tau_smooth = float(self.params.get("smooth_omega_tau", 0.25))
        self.omega_s = update_smoothed_gyro(
            self.omega_s,
            omega_arr,
            state.b_g,
            dt,
            tau=tau_smooth,
        )

        # 6. Check if gap: dt > 2.5 * dt_nom
        if dt > 2.5 * self.dt_nom:
            # Gap mode!
            next_state, next_P, n_steps = propagate_gap(
                state,
                P,
                self.omega_s,
                dt,
                self.dt_nom,
                self.params,
            )
            self.recover = self.recover_steps
            r_scale = get_recovery_multiplier(self.recover, self.recover_steps)
            self.prev_t_ns = int(t_ns)
            self.ring_buffer.push(int(t_ns), next_state, next_P, self.omega_s)

            return GapStepResult(
                state=next_state,
                P=next_P,
                case=GapCase.GAP,
                substeps=n_steps,
                apply_updates=False,
                r_scale=r_scale,
                is_valid=True,
                reason="gap",
            )

        # 7. Normal propagation (dt <= 2.5 * dt_nom and valid)
        if self.recover > 0:
            self.recover -= 1

        r_scale = get_recovery_multiplier(self.recover, self.recover_steps)
        self.prev_t_ns = int(t_ns)
        self.ring_buffer.push(int(t_ns), state, P, self.omega_s)

        return GapStepResult(
            state=state,
            P=P,
            case=GapCase.NORMAL,
            substeps=1,
            apply_updates=True,
            r_scale=r_scale,
            is_valid=True,
            reason="valid",
        )
