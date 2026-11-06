"""Sensor Failure Detector and Blind-5s Predictor (SPEC §12.7, §12.9, WP-27).

Provides:
1. Pure extrapolation functions:
   - predict_blind_position(p_f, v_f, tau, tau_v, shrink_v, stationary_v_thresh)
   - predict_blind_orientation(q_f, omega_f, tau, tau_omega)
   - predict_blind_velocity(v_f, tau, tau_v, shrink_v, stationary_v_thresh)
   - predict_blind_pose(p_f, v_f, q_f, omega_f, tau, params)
2. State rollback:
   - ReliableRingBuffer: fixed-size circular ring buffer for reliable states (no dynamic allocations)
   - rollback_to_reliable: retrieve last reliable state prior to failure onset
3. Streaming FailureDetector and BlindPredictor:
   - Tracks invalid sample persistence continuously
   - Declares failure when invalid samples persist >= T_fail (1.0 s) or at stream termination
   - Performs state rollback to pre-failure reliable state
   - Extrapolates trajectory poses up to 5.0 s (clamping beyond 5.0 s)
   - Generates blind poses for --append-blind

Constraints:
- Portable to fixed-size C++: no dynamic heap allocations inside filter loop
- Strictly implements equations from docs/SPEC.md §12.7 and §12.9
- Parameters loaded from configs/params.json
- No print statements in the filter loop
"""

import json
import math
from pathlib import Path
from typing import Any, Dict, List, NamedTuple, Optional, Tuple, Union
import numpy as np

from src.ref.eskf import NominalState
from src.ref.mathlib import (
    quat_exp,
    quat_mult,
    quat_normalize,
    quat_sign_continuity,
)


def load_params(config_path: Optional[Union[str, Path]] = None) -> Dict[str, Any]:
    """Load estimation parameters from configs/params.json or fallback defaults."""
    if config_path is None:
        config_path = Path(__file__).resolve().parent.parent.parent / "configs" / "params.json"
    p = Path(config_path)
    if p.exists():
        with open(p, "r", encoding="utf-8") as f:
            return json.load(f)
    return {
        "T_fail": 1.0,
        "tau_v": 3.0,
        "tau_omega": 1.5,
        "shrink_v": 0.9,
        "stationary_v_thresh": 0.05,
        "ring_buffer_size": 64,
        "smooth_omega_tau": 0.25,
    }


# ---------------------------------------------------------------------------
# 1. Pure Extrapolation Functions
# ---------------------------------------------------------------------------


def predict_blind_position(
    p_f: Union[np.ndarray, list],
    v_f: Union[np.ndarray, list],
    tau: float,
    tau_v: float = 3.0,
    shrink_v: float = 0.9,
    stationary_v_thresh: float = 0.05,
) -> np.ndarray:
    """Extrapolate position using the decaying-velocity kinematic model (SPEC §12.9).

    Equation:
        p(tau) = p_f + v_f * shrink_v * tau_v * (1 - exp(-tau / tau_v))
        Clamped to pose at tau = 5.0 s for tau > 5.0 s.
        If |v_f| < stationary_v_thresh, holds stationary: p(tau) = p_f.

    Args:
        p_f: (3,) frozen position at failure time [m].
        v_f: (3,) frozen velocity at failure time [m/s].
        tau: Elapsed time since failure in seconds (tau = t - t_f).
        tau_v: Velocity decay time constant in seconds (default 3.0 s).
        shrink_v: Velocity shrink multiplier (default 0.9).
        stationary_v_thresh: Velocity norm threshold for stationary hold (default 0.05 m/s).

    Returns:
        (3,) extrapolated position array [m].
    """
    p_f_arr = np.asarray(p_f, dtype=np.float64).reshape(3)
    v_f_arr = np.asarray(v_f, dtype=np.float64).reshape(3)

    tau_val = float(tau)
    if tau_val <= 0.0:
        return p_f_arr.copy()

    # Poses beyond tau = 5.0 s clamp to the pose at tau = 5.0 s
    if tau_val > 5.0:
        tau_val = 5.0

    # Stationary hold
    v_norm = float(np.linalg.norm(v_f_arr))
    if v_norm < float(stationary_v_thresh):
        return p_f_arr.copy()

    tau_v_val = max(1e-6, float(tau_v))
    shrink_val = float(shrink_v)

    decay_factor = 1.0 - math.exp(-tau_val / tau_v_val)
    delta_p = v_f_arr * (shrink_val * tau_v_val * decay_factor)
    return p_f_arr + delta_p


def predict_blind_orientation(
    q_f: Union[np.ndarray, list],
    omega_f: Union[np.ndarray, list],
    tau: float,
    tau_omega: float = 1.5,
) -> np.ndarray:
    """Extrapolate orientation using decaying turn-rate kinematic model (SPEC §12.9).

    Equation:
        theta(tau) = omega_f * tau_omega * (1 - exp(-tau / tau_omega))
        q(tau) = normalize(q_f (x) exp(theta(tau)))
        Clamped to pose at tau = 5.0 s for tau > 5.0 s.

    Args:
        q_f: (4,) frozen unit quaternion [w, x, y, z] at failure time.
        omega_f: (3,) frozen smoothed body angular velocity [rad/s].
        tau: Elapsed time since failure in seconds (tau = t - t_f).
        tau_omega: Angular velocity decay time constant in seconds (default 1.5 s).

    Returns:
        (4,) extrapolated unit quaternion [w, x, y, z].
    """
    q_f_arr = np.asarray(q_f, dtype=np.float64).reshape(4)
    omega_f_arr = np.asarray(omega_f, dtype=np.float64).reshape(3)

    tau_val = float(tau)
    if tau_val <= 0.0:
        return quat_normalize(q_f_arr)

    # Clamping beyond 5.0 s
    if tau_val > 5.0:
        tau_val = 5.0

    tau_w_val = max(1e-6, float(tau_omega))
    decay_factor = 1.0 - math.exp(-tau_val / tau_w_val)
    rot_vec = omega_f_arr * (tau_w_val * decay_factor)

    delta_q = quat_exp(rot_vec)
    q_rot = quat_mult(q_f_arr, delta_q)
    q_out = quat_normalize(q_rot)
    return quat_sign_continuity(q_out, q_f_arr)


def predict_blind_velocity(
    v_f: Union[np.ndarray, list],
    tau: float,
    tau_v: float = 3.0,
    shrink_v: float = 0.9,
    stationary_v_thresh: float = 0.05,
) -> np.ndarray:
    """Compute instantaneous velocity along the decaying trajectory.

    Derivative:
        v(tau) = v_f * shrink_v * exp(-tau / tau_v)   for tau < 5.0 s
        v(tau) = 0                                     for tau >= 5.0 s (clamped pose)
        v(tau) = 0                                     if |v_f| < stationary_v_thresh

    Args:
        v_f: (3,) frozen velocity at failure time [m/s].
        tau: Elapsed time since failure in seconds.
        tau_v: Velocity decay time constant in seconds.
        shrink_v: Velocity shrink factor.
        stationary_v_thresh: Velocity norm threshold for stationary hold.

    Returns:
        (3,) instantaneous velocity array [m/s].
    """
    v_f_arr = np.asarray(v_f, dtype=np.float64).reshape(3)
    tau_val = float(tau)
    if tau_val < 0.0:
        tau_val = 0.0

    if tau_val >= 5.0:
        return np.zeros(3, dtype=np.float64)

    v_norm = float(np.linalg.norm(v_f_arr))
    if v_norm < float(stationary_v_thresh):
        return np.zeros(3, dtype=np.float64)

    tau_v_val = max(1e-6, float(tau_v))
    shrink_val = float(shrink_v)
    return v_f_arr * (shrink_val * math.exp(-tau_val / tau_v_val))


def predict_blind_pose(
    p_f: Union[np.ndarray, list],
    v_f: Union[np.ndarray, list],
    q_f: Union[np.ndarray, list],
    omega_f: Union[np.ndarray, list],
    tau: float,
    params: Optional[Dict[str, Any]] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Extrapolate complete pose (p, q) at elapsed time tau (SPEC §12.9).

    Args:
        p_f: (3,) frozen position.
        v_f: (3,) frozen velocity.
        q_f: (4,) frozen quaternion [w, x, y, z].
        omega_f: (3,) frozen smoothed gyro.
        tau: Elapsed time in seconds.
        params: Configuration dictionary (tau_v, tau_omega, shrink_v, stationary_v_thresh).

    Returns:
        (p, q): Tuple of (3,) position and (4,) unit quaternion.
    """
    if params is None:
        params = {}

    tau_v = float(params.get("tau_v", 3.0))
    tau_omega = float(params.get("tau_omega", 1.5))
    shrink_v = float(params.get("shrink_v", 0.9))
    stationary_thresh = float(params.get("stationary_v_thresh", 0.05))

    p = predict_blind_position(
        p_f,
        v_f,
        tau,
        tau_v=tau_v,
        shrink_v=shrink_v,
        stationary_v_thresh=stationary_thresh,
    )
    q = predict_blind_orientation(
        q_f,
        omega_f,
        tau,
        tau_omega=tau_omega,
    )
    return p, q


# ---------------------------------------------------------------------------
# 2. Ring Buffer for Reliable States & Rollback
# ---------------------------------------------------------------------------


class ReliableSnapshot(NamedTuple):
    """Reliable state snapshot for pre-failure rollback."""
    t_ns: int
    p: np.ndarray
    v: np.ndarray
    q: np.ndarray
    omega_s: np.ndarray


class ReliableRingBuffer:
    """Fixed-capacity circular ring buffer storing reliable state history (SPEC §12.7).

    Preallocates fixed-size numpy arrays to avoid heap allocations in the filter loop.
    Capacity defaults to 64 snapshots.
    """

    def __init__(self, capacity: int = 64):
        self.capacity = max(1, int(capacity))
        self._t_ns = np.zeros(self.capacity, dtype=np.int64)
        self._p = np.zeros((self.capacity, 3), dtype=np.float64)
        self._v = np.zeros((self.capacity, 3), dtype=np.float64)
        self._q = np.zeros((self.capacity, 4), dtype=np.float64)
        self._omega_s = np.zeros((self.capacity, 3), dtype=np.float64)
        self._head: int = 0
        self._count: int = 0

    def push(
        self,
        t_ns: int,
        p: Union[np.ndarray, list],
        v: Union[np.ndarray, list],
        q: Union[np.ndarray, list],
        omega_s: Union[np.ndarray, list],
    ) -> None:
        """Push a reliable state snapshot into the circular buffer."""
        idx = self._head
        self._t_ns[idx] = int(t_ns)
        self._p[idx, :] = np.asarray(p, dtype=np.float64).reshape(3)
        self._v[idx, :] = np.asarray(v, dtype=np.float64).reshape(3)
        self._q[idx, :] = np.asarray(q, dtype=np.float64).reshape(4)
        self._omega_s[idx, :] = np.asarray(omega_s, dtype=np.float64).reshape(3)

        self._head = (self._head + 1) % self.capacity
        if self._count < self.capacity:
            self._count += 1

    def __len__(self) -> int:
        return self._count

    def get_latest(self) -> Optional[ReliableSnapshot]:
        """Retrieve most recently pushed snapshot."""
        if self._count == 0:
            return None
        latest_idx = (self._head - 1 + self.capacity) % self.capacity
        return ReliableSnapshot(
            t_ns=int(self._t_ns[latest_idx]),
            p=self._p[latest_idx].copy(),
            v=self._v[latest_idx].copy(),
            q=self._q[latest_idx].copy(),
            omega_s=self._omega_s[latest_idx].copy(),
        )

    def get_oldest(self) -> Optional[ReliableSnapshot]:
        """Retrieve oldest snapshot currently stored in the buffer."""
        if self._count == 0:
            return None
        oldest_idx = (self._head - self._count + self.capacity) % self.capacity
        return ReliableSnapshot(
            t_ns=int(self._t_ns[oldest_idx]),
            p=self._p[oldest_idx].copy(),
            v=self._v[oldest_idx].copy(),
            q=self._q[oldest_idx].copy(),
            omega_s=self._omega_s[oldest_idx].copy(),
        )

    def clear(self) -> None:
        """Clear the ring buffer."""
        self._head = 0
        self._count = 0


def rollback_to_reliable(
    ring_buffer: Any,
    t_onset_ns: int,
) -> ReliableSnapshot:
    """Retrieve the last reliable state prior to failure onset from a ring buffer.

    Searches backwards from the newest snapshot for the latest entry with t_ns < t_onset_ns.
    If no entry has t_ns < t_onset_ns, returns the oldest available entry.

    Args:
        ring_buffer: ReliableRingBuffer or StateRingBuffer instance.
        t_onset_ns: Timestamp (ns) of failure onset.

    Returns:
        ReliableSnapshot of (t_ns, p, v, q, omega_s).
    """
    if len(ring_buffer) == 0:
        return ReliableSnapshot(
            t_ns=int(t_onset_ns),
            p=np.zeros(3, dtype=np.float64),
            v=np.zeros(3, dtype=np.float64),
            q=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            omega_s=np.zeros(3, dtype=np.float64),
        )

    # 1. ReliableRingBuffer search
    if isinstance(ring_buffer, ReliableRingBuffer):
        cap = ring_buffer.capacity
        head = ring_buffer._head
        count = ring_buffer._count

        # Scan newest to oldest
        for i in range(count):
            idx = (head - 1 - i + cap) % cap
            if ring_buffer._t_ns[idx] < t_onset_ns:
                return ReliableSnapshot(
                    t_ns=int(ring_buffer._t_ns[idx]),
                    p=ring_buffer._p[idx].copy(),
                    v=ring_buffer._v[idx].copy(),
                    q=ring_buffer._q[idx].copy(),
                    omega_s=ring_buffer._omega_s[idx].copy(),
                )

        # Fallback to oldest entry
        oldest = ring_buffer.get_oldest()
        assert oldest is not None
        return oldest

    # 2. StateRingBuffer (from src.ref.gap) compatibility
    cap = getattr(ring_buffer, "capacity", len(ring_buffer))
    head = getattr(ring_buffer, "_head", 0)
    count = getattr(ring_buffer, "_count", len(ring_buffer))
    t_list = getattr(ring_buffer, "_t_ns", [])
    states = getattr(ring_buffer, "_states", [])
    omegas = getattr(ring_buffer, "_omega_s", [])

    for i in range(count):
        idx = (head - 1 - i + cap) % cap
        if t_list[idx] < t_onset_ns:
            st = states[idx]
            w = omegas[idx]
            return ReliableSnapshot(
                t_ns=int(t_list[idx]),
                p=st.p.copy(),
                v=st.v.copy(),
                q=st.q.copy(),
                omega_s=w.copy() if w is not None else np.zeros(3, dtype=np.float64),
            )

    oldest = ring_buffer.get_oldest()
    if oldest is not None:
        t_ns_old, st_old, _, w_old = oldest
        return ReliableSnapshot(
            t_ns=int(t_ns_old),
            p=st_old.p.copy(),
            v=st_old.v.copy(),
            q=st_old.q.copy(),
            omega_s=w_old.copy() if w_old is not None else np.zeros(3, dtype=np.float64),
        )

    return ReliableSnapshot(
        t_ns=int(t_onset_ns),
        p=np.zeros(3, dtype=np.float64),
        v=np.zeros(3, dtype=np.float64),
        q=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        omega_s=np.zeros(3, dtype=np.float64),
    )


# ---------------------------------------------------------------------------
# 3. Streaming FailureDetector and BlindPredictor Class
# ---------------------------------------------------------------------------


class BlindPredictor:
    """Streaming Failure Detector and Blind-5s Predictor (SPEC §12.7, §12.9).

    Manages:
    - Ring buffer of reliable pre-failure states
    - Continuous invalid sample duration tracking
    - Failure declaration when invalid duration >= T_fail (1.0 s) or at stream termination
    - Rollback to pre-failure reliable state
    - Extrapolation of blind poses clamped beyond 5.0 s
    """

    def __init__(
        self,
        params: Optional[Dict[str, Any]] = None,
        ring_buffer_capacity: Optional[int] = None,
    ):
        self.params = load_params() if params is None else dict(params)

        self.T_fail = float(self.params.get("T_fail", 1.0))
        self.tau_v = float(self.params.get("tau_v", 3.0))
        self.tau_omega = float(self.params.get("tau_omega", 1.5))
        self.shrink_v = float(self.params.get("shrink_v", 0.9))
        self.stationary_v_thresh = float(self.params.get("stationary_v_thresh", 0.05))

        cap = ring_buffer_capacity or int(self.params.get("ring_buffer_size", 64))
        self.ring_buffer = ReliableRingBuffer(capacity=cap)

        self.is_failed: bool = False
        self.failure_time_ns: Optional[int] = None
        self.invalid_start_ns: Optional[int] = None
        self.last_valid_t_ns: Optional[int] = None
        self.frozen_state: Optional[ReliableSnapshot] = None

    def record_reliable_state(
        self,
        t_ns: int,
        p: Optional[Union[np.ndarray, list]] = None,
        v: Optional[Union[np.ndarray, list]] = None,
        q: Optional[Union[np.ndarray, list]] = None,
        omega_s: Optional[Union[np.ndarray, list]] = None,
        state: Optional[NominalState] = None,
    ) -> None:
        """Push a verified reliable state into the ring buffer."""
        if state is not None:
            pos = state.p
            vel = state.v
            ori = state.q
        else:
            pos = p if p is not None else np.zeros(3, dtype=np.float64)
            vel = v if v is not None else np.zeros(3, dtype=np.float64)
            ori = q if q is not None else np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        w_s = omega_s if omega_s is not None else np.zeros(3, dtype=np.float64)

        self.ring_buffer.push(int(t_ns), pos, vel, ori, w_s)
        self.last_valid_t_ns = int(t_ns)
        self.invalid_start_ns = None

    def update(
        self,
        t_ns: int,
        is_valid: bool,
        p: Optional[Union[np.ndarray, list]] = None,
        v: Optional[Union[np.ndarray, list]] = None,
        q: Optional[Union[np.ndarray, list]] = None,
        omega_s: Optional[Union[np.ndarray, list]] = None,
        state: Optional[NominalState] = None,
    ) -> bool:
        """Process sample validity and detect continuous failure.

        Args:
            t_ns: Current sample timestamp in nanoseconds.
            is_valid: True if sample passed validity checks.
            p: Optional position array.
            v: Optional velocity array.
            q: Optional quaternion array.
            omega_s: Optional smoothed angular velocity.
            state: Optional NominalState.

        Returns:
            True if sensor failure has been declared, False otherwise.
        """
        if self.is_failed:
            return True

        if is_valid:
            if p is not None or state is not None:
                self.record_reliable_state(t_ns, p=p, v=v, q=q, omega_s=omega_s, state=state)
            self.invalid_start_ns = None
            self.last_valid_t_ns = int(t_ns)
            return False

        # Invalid sample
        if self.invalid_start_ns is None:
            self.invalid_start_ns = int(t_ns)

        invalid_duration_s = (int(t_ns) - self.invalid_start_ns) * 1e-9
        if invalid_duration_s >= self.T_fail:
            self.declare_failure(failure_time_ns=int(t_ns), failure_onset_ns=self.invalid_start_ns)
            return True

        return False

    def declare_failure(
        self,
        failure_time_ns: int,
        failure_onset_ns: Optional[int] = None,
    ) -> None:
        """Declare sensor failure, rollback state, and enter blind extrapolation mode."""
        if self.is_failed:
            return

        onset_ns = int(failure_onset_ns) if failure_onset_ns is not None else int(failure_time_ns)
        self.frozen_state = rollback_to_reliable(self.ring_buffer, onset_ns)
        self.failure_time_ns = int(failure_time_ns)
        self.is_failed = True

    def notify_stream_end(self, final_t_ns: Optional[int] = None) -> None:
        """Declare failure at stream termination and freeze last reliable state."""
        if self.is_failed:
            return

        if len(self.ring_buffer) > 0:
            latest = self.ring_buffer.get_latest()
            assert latest is not None
            onset_ns = int(final_t_ns) if final_t_ns is not None else latest.t_ns
            self.frozen_state = rollback_to_reliable(self.ring_buffer, onset_ns + 1)
        else:
            t_fallback = int(final_t_ns) if final_t_ns is not None else 0
            self.frozen_state = ReliableSnapshot(
                t_ns=t_fallback,
                p=np.zeros(3, dtype=np.float64),
                v=np.zeros(3, dtype=np.float64),
                q=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
                omega_s=np.zeros(3, dtype=np.float64),
            )

        self.failure_time_ns = int(final_t_ns) if final_t_ns is not None else self.frozen_state.t_ns
        self.is_failed = True

    def set_frozen_state(
        self,
        t_f_ns: int,
        p_f: Union[np.ndarray, list],
        v_f: Union[np.ndarray, list],
        q_f: Union[np.ndarray, list],
        omega_f: Union[np.ndarray, list],
    ) -> None:
        """Directly set frozen pre-failure state and enter failed mode."""
        self.frozen_state = ReliableSnapshot(
            t_ns=int(t_f_ns),
            p=np.asarray(p_f, dtype=np.float64).reshape(3),
            v=np.asarray(v_f, dtype=np.float64).reshape(3),
            q=np.asarray(q_f, dtype=np.float64).reshape(4),
            omega_s=np.asarray(omega_f, dtype=np.float64).reshape(3),
        )
        self.failure_time_ns = int(t_f_ns)
        self.is_failed = True

    def predict(self, t_ns: int) -> Tuple[np.ndarray, np.ndarray]:
        """Predict blind pose at timestamp t_ns >= t_f_ns.

        Args:
            t_ns: Target timestamp in nanoseconds.

        Returns:
            (p, q): Extrapolated position and quaternion.
        """
        if not self.is_failed or self.frozen_state is None:
            raise RuntimeError("Cannot predict blind pose before failure is declared.")

        tau = (int(t_ns) - self.frozen_state.t_ns) * 1e-9
        return predict_blind_pose(
            self.frozen_state.p,
            self.frozen_state.v,
            self.frozen_state.q,
            self.frozen_state.omega_s,
            tau=tau,
            params=self.params,
        )

    def predict_tau(self, tau: float) -> Tuple[np.ndarray, np.ndarray]:
        """Predict blind pose at elapsed time tau in seconds."""
        if not self.is_failed or self.frozen_state is None:
            raise RuntimeError("Cannot predict blind pose before failure is declared.")

        return predict_blind_pose(
            self.frozen_state.p,
            self.frozen_state.v,
            self.frozen_state.q,
            self.frozen_state.omega_s,
            tau=tau,
            params=self.params,
        )

    def generate_blind_poses(
        self,
        start_t_ns: int,
        dt_ns: int,
        duration_s: float = 5.0,
    ) -> List[Tuple[int, np.ndarray, np.ndarray]]:
        """Generate extrapolated poses for --append-blind mode up to duration_s.

        Args:
            start_t_ns: Timestamp of the last emitted sample.
            dt_ns: Nominal sample interval in nanoseconds.
            duration_s: Duration of blind extrapolation in seconds (default 5.0 s).

        Returns:
            List of (t_ns, p, q) tuples.
        """
        n_steps = int(round(float(duration_s) * 1e9 / float(dt_ns)))
        poses: List[Tuple[int, np.ndarray, np.ndarray]] = []

        for step in range(1, n_steps + 1):
            t_step_ns = int(start_t_ns) + step * int(dt_ns)
            p, q = self.predict(t_step_ns)
            poses.append((t_step_ns, p, q))

        return poses
