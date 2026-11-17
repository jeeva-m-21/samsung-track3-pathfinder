"""Full Reference 15-State ESKF Pipeline and Ablation Modes (SPEC §6.3, §11.5, §12).

Assembles:
1. Streaming tolerant CSV parser (SPEC §12.8)
2. Online static window initialization (SPEC §12.2)
3. Nominal strapdown kinematic propagation (SPEC §12.3)
4. Analytical error-state covariance prediction with quality inflation (SPEC §12.4)
5. Degradation monitor tracking residual variance and signal quality (SPEC §12.6)
6. Tilt measurement updates with gating and quality-scaled noise (SPEC §12.5)
7. Zero Velocity Updates (ZUPT) over sliding window (SPEC §12.5)
8. Gap and invalid-sample handler with sub-stepping and recovery (SPEC §12.7)
9. Sensor failure detector, state rollback, and blind-5s predictor (SPEC §12.7, §12.9)

Ablation modes:
- naive: initial tilt + raw open-loop strapdown integration
- mahony: Mahony attitude filter + double integration
- eskf_basic: 15-state ESKF with fixed noise, no monitor inflation, no gap substepping, no blind model
- eskf_full: full system with all components active

Output format (SPEC §6.3):
    timestamp tx ty tz qx qy qz qw
    where timestamp is formatted as <sec>.<nsec zero-padded to 9 digits> from int64 integer math.
    Positions and quaternion formatted with fixed 9 decimal places.
"""

import math
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union
import numpy as np

from tools.validate_trajectory import resolve_imu_csv
from src.ref.baseline_naive import run_naive_baseline
from src.ref.baseline_mahony import run_mahony_baseline
from src.ref.init import (
    InitResult,
    NominalState,
    OnlineInitializer,
    compute_initial_tilt,
    initialize_covariance,
    load_params,
)
from src.ref.strapdown import strapdown_step
from src.ref.eskf import (
    build_f_matrix,
    build_q_discrete,
    eskf_predict_covariance,
)
from src.ref.monitor import DegradationMonitor
from src.ref.updates import (
    Butterworth2LowPass,
    StationaryWindowBuffer,
    apply_pedestrian_prior,
    apply_rotor_drag_update,
    apply_velocity_bounding,
    tilt_update,
    zupt_step,
)
from src.ref.gap import (
    GapCase,
    GapHandler,
)
from src.ref.blind import BlindPredictor
from src.ref.parser import parse_imu_stream


class ChunkedTrajectory:
    """Collects poses from Pass 1 using bounded-memory chunks and temporary file backing for Pass 2 offline filtering."""

    def __init__(self, chunk_size: int = 8192):
        self.chunk_size = int(chunk_size)
        self._count = 0
        self._idx = 0
        self._t_buf = np.empty(self.chunk_size, dtype=np.int64)
        self._p_buf = np.empty((self.chunk_size, 3), dtype=np.float64)
        self._q_buf = np.empty((self.chunk_size, 4), dtype=np.float64)
        self._f_t = tempfile.NamedTemporaryFile()
        self._f_p = tempfile.NamedTemporaryFile()
        self._f_q = tempfile.NamedTemporaryFile()

    def push(self, t_ns: int, p: Union[np.ndarray, list], q: Union[np.ndarray, list]) -> None:
        self._t_buf[self._idx] = int(t_ns)
        self._p_buf[self._idx] = p
        self._q_buf[self._idx] = q
        self._idx += 1
        self._count += 1
        if self._idx == self.chunk_size:
            self._flush_chunk()

    def write_pose(
        self, t_ns: int, px: float, py: float, pz: float, qx: float, qy: float, qz: float, qw: float
    ) -> None:
        self._t_buf[self._idx] = int(t_ns)
        self._p_buf[self._idx, 0] = px
        self._p_buf[self._idx, 1] = py
        self._p_buf[self._idx, 2] = pz
        self._q_buf[self._idx, 0] = qx
        self._q_buf[self._idx, 1] = qy
        self._q_buf[self._idx, 2] = qz
        self._q_buf[self._idx, 3] = qw
        self._idx += 1
        self._count += 1
        if self._idx == self.chunk_size:
            self._flush_chunk()

    def _flush_chunk(self) -> None:
        if self._idx > 0:
            self._f_t.write(self._t_buf[:self._idx].tobytes())
            self._f_p.write(self._p_buf[:self._idx].tobytes())
            self._f_q.write(self._q_buf[:self._idx].tobytes())
            self._idx = 0

    def to_arrays(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        self._flush_chunk()
        self._f_t.flush()
        self._f_p.flush()
        self._f_q.flush()
        # Free buffer memory
        self._t_buf = None
        self._p_buf = None
        self._q_buf = None

        if self._count == 0:
            return (
                np.empty(0, dtype=np.int64),
                np.empty((0, 3), dtype=np.float64),
                np.empty((0, 4), dtype=np.float64),
            )

        t_arr = np.memmap(self._f_t, dtype=np.int64, mode="r+", shape=(self._count,))
        p_arr = np.memmap(self._f_p, dtype=np.float64, mode="r+", shape=(self._count, 3))
        q_arr = np.memmap(self._f_q, dtype=np.float64, mode="r+", shape=(self._count, 4))
        return t_arr, p_arr, q_arr


# Backward-compatible alias
TrajectoryCollector = ChunkedTrajectory


def apply_bounded_workspace_filter(
    pos: np.ndarray, dt_nom: float, cutoff_period: float
) -> np.ndarray:
    count = len(pos)
    if count == 0:
        return pos.copy()

    pos_out = np.array(pos, dtype=np.float64, copy=True)
    invalid_mask = ~np.isfinite(pos_out)
    if np.any(invalid_mask):
        pos_out[invalid_mask] = 0.0

    if count < 15 or dt_nom <= 0.0 or cutoff_period <= 0.0:
        return pos_out - pos_out[0]

    fs = 1.0 / dt_nom
    fc = 1.0 / cutoff_period
    if fc >= 0.5 * fs:
        return pos_out - pos_out[0]

    K = math.tan(math.pi * fc / fs)
    norm = 1.0 + math.sqrt(2.0) * K + K * K
    b0 = 1.0 / norm
    b1 = -2.0 * b0
    b2 = b0
    a1 = 2.0 * (K * K - 1.0) / norm
    a2 = (1.0 - math.sqrt(2.0) * K + K * K) / norm

    for c in range(3):
        # Forward pass
        d1 = (b1 + b2 - a1 - a2) * pos_out[0, c]
        d2 = (b2 - a2) * pos_out[0, c]
        for n in range(count):
            x_n = pos_out[n, c]
            val = b0 * x_n + d1
            d1 = b1 * x_n - a1 * val + d2
            d2 = b2 * x_n - a2 * val
            pos_out[n, c] = val

        # Backward pass
        d1 = (b1 + b2 - a1 - a2) * pos_out[count - 1, c]
        d2 = (b2 - a2) * pos_out[count - 1, c]
        for n in range(count - 1, -1, -1):
            x_n = pos_out[n, c]
            val = b0 * x_n + d1
            d1 = b1 * x_n - a1 * val + d2
            d2 = b2 * x_n - a2 * val
            pos_out[n, c] = val

    pos_out -= pos_out[0]
    return pos_out


class DelayedOutputBuffer:
    """Ring buffer holding delayed output frames to apply smooth kinematic bridge across gaps."""

    def __init__(self, capacity: int = 512, delay_frames: int = 512):
        self.capacity = int(capacity)
        self.delay_frames = int(delay_frames)
        self.buffer = []

    def push(self, t_ns: int, p: np.ndarray, q: np.ndarray, is_gap: bool, sink) -> None:
        if self.delay_frames <= 0:
            sink.write_pose(t_ns, p[0], p[1], p[2], q[1], q[2], q[3], q[0])
            return

        if len(self.buffer) >= self.delay_frames or len(self.buffer) >= self.capacity:
            oldest = self.buffer.pop(0)
            po = oldest['p']
            qo = oldest['q']
            sink.write_pose(oldest['t_ns'], po[0], po[1], po[2], qo[1], qo[2], qo[3], qo[0])

        self.buffer.append({'t_ns': t_ns, 'p': p.copy(), 'q': q.copy(), 'is_gap': is_gap})

    def apply_bridge(self, p_recovered: np.ndarray) -> None:
        if not self.buffer or self.delay_frames <= 0:
            return

        gap_count = 0
        for item in reversed(self.buffer):
            if item['is_gap']:
                gap_count += 1
            else:
                break

        if gap_count == 0:
            return

        last_gap_p = self.buffer[-1]['p']
        delta_p = p_recovered - last_gap_p

        start_idx = len(self.buffer) - gap_count
        for step in range(gap_count):
            idx = start_idx + step
            alpha = float(step + 1) / float(gap_count + 1)
            s = 3.0 * alpha * alpha - 2.0 * alpha * alpha * alpha
            self.buffer[idx]['p'] = self.buffer[idx]['p'] + delta_p * s
            self.buffer[idx]['is_gap'] = False

    def flush(self, sink) -> None:
        while self.buffer:
            oldest = self.buffer.pop(0)
            po = oldest['p']
            qo = oldest['q']
            sink.write_pose(oldest['t_ns'], po[0], po[1], po[2], qo[1], qo[2], qo[3], qo[0])


def _fallback_initialization(
    t_arr: np.ndarray,
    omega_arr: np.ndarray,
    acc_arr: np.ndarray,
    cfg: Dict[str, Any],
) -> InitResult:
    """Fallback initialization when insufficient samples exist to reach T_init."""
    n_samples = len(t_arr)
    if n_samples == 0:
        a_ref = np.array([0.0, 0.0, 9.80665], dtype=np.float64)
        b_g = np.zeros(3, dtype=np.float64)
        t0 = 0
    else:
        a_ref = np.mean(acc_arr, axis=0) if n_samples > 1 else acc_arr[0]
        b_g = np.zeros(3, dtype=np.float64)
        t0 = int(t_arr[0])

    g_raw = float(np.linalg.norm(a_ref))
    g_min = float(cfg.get("g_mag_clamp_min", 9.0))
    g_max = float(cfg.get("g_mag_clamp_max", 10.5))
    g_mag = float(np.clip(g_raw, g_min, g_max))
    g_w = np.array([0.0, 0.0, -g_mag], dtype=np.float64)

    q0 = compute_initial_tilt(a_ref)
    p0 = np.zeros(3, dtype=np.float64)
    v0 = np.zeros(3, dtype=np.float64)
    b_a0 = np.zeros(3, dtype=np.float64)

    state = NominalState(p=p0, v=v0, q=q0, b_g=b_g, b_a=b_a0)
    P0 = initialize_covariance(cfg, motion_detected=True)

    return InitResult(
        state=state,
        P=P0,
        g_w=g_w,
        g_mag=g_mag,
        motion_detected=True,
        samples_used=max(1, n_samples),
        t0=t0,
    )


def run_eskf_full(
    imu_input: Union[str, Path],
    traj_output: Union[str, Path],
    append_blind: bool = False,
    params: Optional[Dict[str, Any]] = None,
) -> None:
    """Run full 15-state ESKF reference pipeline (SPEC §11.5, §12)."""
    csv_path = resolve_imu_csv(imu_input)
    out_path = Path(traj_output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cfg = load_params(params)

    cov_floor = float(cfg.get("cov_diag_floor", 1e-12))
    cov_cap = float(cfg.get("cov_diag_cap", 1e6))
    zupt_window_s = float(cfg.get("zupt_window_s", 0.25))

    initializer = OnlineInitializer(params=cfg)
    gap_handler = GapHandler(params=cfg)
    monitor = DegradationMonitor(params=cfg)
    blind_predictor = BlindPredictor(params=cfg)
    stat_buf = StationaryWindowBuffer(capacity=int(cfg.get("ring_buffer_size", 256)))
    tilt_filter = Butterworth2LowPass()
    delay_buf = DelayedOutputBuffer(
        capacity=512,
        delay_frames=int(cfg.get("delayed_output_frames", 512))
    )
    was_in_gap = False

    # Preallocated initial buffer for zero dynamic allocations during startup
    max_init_cap = OnlineInitializer.MAX_BUFFER_SAMPLES
    init_t = np.empty(max_init_cap, dtype=np.int64)
    init_omega = np.empty((max_init_cap, 3), dtype=np.float64)
    init_acc = np.empty((max_init_cap, 3), dtype=np.float64)
    init_valid = np.empty(max_init_cap, dtype=bool)
    init_count = 0

    is_initialized = False
    init_res: Optional[InitResult] = None

    state: Optional[NominalState] = None
    P: Optional[np.ndarray] = None
    g_w: Optional[np.ndarray] = None
    g_mag: float = 9.80665
    t0_ns: int = 0
    prev_t_ns: int = 0
    vibe_ema: float = 0.0
    ped_energy: float = 0.0
    dyn_acc_ema: float = 0.0
    prev_acc_raw = np.zeros(3, dtype=np.float64)
    has_prev_acc: bool = False

    with open(out_path, "w", encoding="utf-8") as f_out:
        f_out.write("# timestamp tx ty tz qx qy qz qw\n")

        collector = ChunkedTrajectory()

        def process_full_step(t_curr: int, omega_curr: np.ndarray, acc_curr: np.ndarray) -> None:
            nonlocal state, P, prev_t_ns, was_in_gap
            nonlocal vibe_ema, ped_energy, dyn_acc_ema, prev_acc_raw, has_prev_acc
            assert state is not None and P is not None and g_w is not None

            if blind_predictor.is_failed:
                p_blind, q_blind = blind_predictor.predict(t_curr)
                delay_buf.push(t_curr, p_blind, q_blind, False, collector)
                prev_t_ns = t_curr
                return

            step_res = gap_handler.process_sample(t_curr, omega_curr, acc_curr, state, P)

            if step_res.case == GapCase.ZERO_OR_NEGATIVE_DT:
                delay_buf.push(t_curr, state.p, state.q, False, collector)
                return

            if step_res.case in (GapCase.GAP, GapCase.INVALID_SAMPLE):
                state = step_res.state
                P = step_res.P
                was_in_gap = True
                failed = blind_predictor.update(t_curr, is_valid=False)
                if failed or blind_predictor.is_failed:
                    p_out, q_out = blind_predictor.predict(t_curr)
                else:
                    p_out, q_out = state.p, state.q

                delay_buf.push(t_curr, p_out, q_out, True, collector)
                prev_t_ns = t_curr
                return

            # Normal step
            dt = (t_curr - prev_t_ns) * 1e-9
            prev_t_ns = t_curr

            acc_arr = np.asarray(acc_curr, dtype=np.float64).reshape(3)
            if np.all(np.isfinite(acc_arr)):
                if not has_prev_acc:
                    prev_acc_raw = acc_arr.copy()
                    has_prev_acc = True
                else:
                    d_a = float(np.linalg.norm(acc_arr - prev_acc_raw))
                    if np.isfinite(d_a):
                        vibe_ema = 0.98 * vibe_ema + 0.02 * d_a
                        dev_a = abs(float(np.linalg.norm(acc_arr)) - g_mag)
                        ped_energy = 0.99 * ped_energy + 0.01 * (dev_a * dev_a)
                        if monitor.quality >= 0.5 and dev_a < 12.0:
                            a_excess = max(0.0, float(np.linalg.norm(acc_arr)) - g_mag)
                            dyn_acc_ema = 0.95 * dyn_acc_ema + 0.05 * a_excess
                        prev_acc_raw = acc_arr.copy()

            flight_thresh = float(cfg.get("flight_vibe_thresh", 0.4))
            ped_thresh = float(cfg.get("ped_energy_thresh", 0.2))
            is_flight = vibe_ema > flight_thresh
            is_pedestrian = (not is_flight) and (ped_energy > ped_thresh)

            dyn_env = float(cfg.get("max_velocity_mps", 3.0)) + (1.0 if is_flight else 0.0) * dyn_acc_ema
            if dyn_env > 15.0:
                dyn_env = 15.0
            max_v = dyn_env if is_flight else (float(cfg.get("ped_max_velocity_mps", 3.5)) if is_pedestrian else dyn_env)

            state = strapdown_step(state, omega_curr, acc_curr, dt, g_w)
            F = build_f_matrix(state, omega_curr, acc_curr, dt)
            Q = build_q_discrete(dt, cfg, kappa=monitor.kappa)
            P = eskf_predict_covariance(P, F, Q, diag_floor=cov_floor, diag_cap=cov_cap)

            t_sec = (t_curr - t0_ns) * 1e-9
            acc_filt = tilt_filter.step(acc_curr) if cfg.get("enable_iir_filter", False) else acc_curr

            stat_buf.add_sample(t_sec, omega_curr, acc_curr, state.b_g)
            bool_stat = stat_buf.is_stationary(g_mag=g_mag, quality=monitor.quality, params=cfg)
            gyro_win, acc_win = stat_buf.get_window_samples(zupt_window_s)
            omega_stat_mean = np.mean(gyro_win, axis=0) if (bool_stat and len(gyro_win) > 0) else None
            monitor.step(t_sec, omega_curr, acc_curr, is_stationary=bool_stat, omega_static_mean=omega_stat_mean)

            if step_res.apply_updates:
                state, P, _, _ = tilt_update(
                    state, P, acc_filt, omega_curr,
                    g_mag=g_mag, quality=monitor.quality,
                    params=cfg, r_scale=step_res.r_scale,
                )
                if bool_stat:
                    state, P, _, _ = zupt_step(
                        state, P, gyro_win, acc_win,
                        g_mag=g_mag, quality=monitor.quality,
                        params=cfg, r_scale=step_res.r_scale,
                    )
                elif is_flight and bool(cfg.get("enable_rotor_drag", True)):
                    state, P, _ = apply_rotor_drag_update(
                        state, P, acc_curr, omega_curr, g_w,
                        kd=float(cfg.get("rotor_drag_kd", 0.22)),
                        sigma=float(cfg.get("rotor_drag_sigma", 0.8)),
                        gate_threshold=float(cfg.get("rotor_drag_gate", 12.0)),
                        params=cfg,
                    )
                elif is_pedestrian and bool(cfg.get("enable_pedestrian_prior", True)):
                    state, P, _ = apply_pedestrian_prior(
                        state, P,
                        sigma_lat=float(cfg.get("ped_sigma_lat", 0.5)),
                        sigma_vert=float(cfg.get("ped_sigma_vert", 0.3)),
                        gate_threshold=float(cfg.get("ped_gate_threshold", 9.0)),
                        params=cfg,
                    )
                state, P, _ = apply_velocity_bounding(state, P, max_speed=max_v, params=cfg)

            if was_in_gap:
                delay_buf.apply_bridge(state.p)
                was_in_gap = False

            blind_predictor.update(
                t_curr, is_valid=True, state=state, omega_s=gap_handler.omega_s
            )
            delay_buf.push(t_curr, state.p, state.q, False, collector)

        with parse_imu_stream(csv_path, params=cfg) as stream:
            for t_curr, omega_curr, acc_curr, is_valid_curr, _ in stream:
                if not is_initialized:
                    if init_count < max_init_cap:
                        init_t[init_count] = t_curr
                        init_omega[init_count] = omega_curr
                        init_acc[init_count] = acc_curr
                        init_valid[init_count] = is_valid_curr
                        init_count += 1

                    ready = initializer.feed(t_curr, omega_curr, acc_curr, is_valid=is_valid_curr)
                    if ready:
                        is_initialized = True
                        init_res = initializer.get_result()
                        assert init_res is not None
                        state = init_res.state.copy()
                        P = init_res.P.copy()
                        g_w = init_res.g_w.copy()
                        g_mag = float(init_res.g_mag)
                        t0_ns = int(init_res.t0)

                        valid_acc = init_acc[:init_count][init_valid[:init_count]]
                        a_mean = np.mean(valid_acc, axis=0) if len(valid_acc) > 0 else np.array([0.0, 0.0, 9.80665], dtype=np.float64)
                        tilt_filter.reset(a_mean)

                        # Process buffered samples 0 .. init_count - 1
                        for idx in range(init_count):
                            t_smp = int(init_t[idx])
                            w_smp = init_omega[idx]
                            a_smp = init_acc[idx]

                            if idx == 0:
                                prev_t_ns = t_smp
                                blind_predictor.record_reliable_state(
                                    t_smp, state=state, omega_s=np.zeros(3, dtype=np.float64)
                                )
                                gap_handler.process_sample(t_smp, w_smp, a_smp, state, P)
                                delay_buf.push(t_smp, state.p, state.q, False, collector)
                                continue

                            process_full_step(t_smp, w_smp, a_smp)
                        continue
                    continue

                process_full_step(t_curr, omega_curr, acc_curr)

        # Handle file shorter than T_init fallback
        if not is_initialized and init_count > 0:
            ready = initializer.finalize_fallback()
            if ready:
                init_res = initializer.get_result()
            else:
                init_res = _fallback_initialization(
                    init_t[:init_count], init_omega[:init_count], init_acc[:init_count], cfg
                )

            assert init_res is not None
            state = init_res.state.copy()
            P = init_res.P.copy()
            g_w = init_res.g_w.copy()
            g_mag = float(init_res.g_mag)
            t0_ns = int(init_res.t0)

            for idx in range(init_count):
                t_smp = int(init_t[idx])
                w_smp = init_omega[idx]
                a_smp = init_acc[idx]

                if idx == 0:
                    prev_t_ns = t_smp
                else:
                    dt = (t_smp - prev_t_ns) * 1e-9
                    prev_t_ns = t_smp
                    if dt > 0.0 and init_valid[idx]:
                        state = strapdown_step(state, w_smp, a_smp, dt, g_w)
                if not np.all(np.isfinite(state.p)):
                    state.p = np.zeros(3, dtype=np.float64)
                if not np.all(np.isfinite(state.q)):
                    state.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
                delay_buf.push(t_smp, state.p, state.q, False, collector)

        delay_buf.flush(collector)

        # Pass 2: Offline drift-aware bounded-workspace filter
        dt_nom = float(gap_handler.dt_nom if gap_handler.dt_nom > 0.0 else 0.005)
        t_ns_arr, pos_arr, quat_arr = collector.to_arrays()
        stream_count = len(t_ns_arr)

        # Determine if IMU failure occurred during the stream
        active_count = stream_count
        p_raw_split = np.zeros(3, dtype=np.float64)
        failed_in_stream = False

        if blind_predictor.is_failed and stream_count > 0:
            t_fail = blind_predictor.failure_time_ns
            for i in range(stream_count):
                if t_ns_arr[i] >= t_fail:
                    active_count = i
                    failed_in_stream = True
                    if i > 0:
                        p_raw_split = pos_arr[i - 1].copy()
                    break

        if bool(cfg.get("enable_two_pass_filter", True)) and active_count > 0:
            cutoff_T = float(cfg.get("two_pass_cutoff_period", 45.0))
            pos_arr[:active_count] = apply_bounded_workspace_filter(pos_arr[:active_count], dt_nom, cutoff_T)

        if failed_in_stream and active_count > 0 and active_count < stream_count:
            p_filtered_split = pos_arr[active_count - 1]
            delta_align = p_filtered_split - p_raw_split
            pos_arr[active_count:] += delta_align

        for i in range(len(t_ns_arr)):
            t_ns_i = int(t_ns_arr[i])
            sec = t_ns_i // 1_000_000_000
            nsec = t_ns_i % 1_000_000_000
            p_i = pos_arr[i]
            q_i = quat_arr[i]
            f_out.write(
                f"{sec}.{nsec:09d} {p_i[0]:.9f} {p_i[1]:.9f} {p_i[2]:.9f} "
                f"{q_i[0]:.9f} {q_i[1]:.9f} {q_i[2]:.9f} {q_i[3]:.9f}\n"
            )

        # Append blind 5.0s extrapolation from clean filtered endpoint
        if append_blind and prev_t_ns > 0 and stream_count > 0:
            if not blind_predictor.is_failed:
                blind_predictor.notify_stream_end(prev_t_ns)

            p_last_filtered = pos_arr[-1]
            p_raw_endpoint = blind_predictor.frozen_state.p
            delta_endpoint_align = p_last_filtered - p_raw_endpoint

            dt_ns_step = max(1, int(round(dt_nom * 1e9)))
            blind_poses = blind_predictor.generate_blind_poses(prev_t_ns, dt_ns_step, duration_s=5.0)

            for t_extrap, p_extrap, q_extrap in blind_poses:
                p_final = p_extrap + delta_endpoint_align
                sec = t_extrap // 1_000_000_000
                nsec = t_extrap % 1_000_000_000
                f_out.write(
                    f"{sec}.{nsec:09d} {p_final[0]:.9f} {p_final[1]:.9f} {p_final[2]:.9f} "
                    f"{q_extrap[1]:.9f} {q_extrap[2]:.9f} {q_extrap[3]:.9f} {q_extrap[0]:.9f}\n"
                )


def run_eskf_basic(
    imu_input: Union[str, Path],
    traj_output: Union[str, Path],
    append_blind: bool = False,
    params: Optional[Dict[str, Any]] = None,
) -> None:
    """Run eskf_basic ablation: 15-state ESKF with fixed noise, no monitor inflation, no gap substepping, no blind model."""
    csv_path = resolve_imu_csv(imu_input)
    out_path = Path(traj_output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cfg = load_params(params)

    cov_floor = float(cfg.get("cov_diag_floor", 1e-12))
    cov_cap = float(cfg.get("cov_diag_cap", 1e6))
    zupt_window_s = float(cfg.get("zupt_window_s", 0.25))

    initializer = OnlineInitializer(params=cfg)
    stat_buf = StationaryWindowBuffer(capacity=int(cfg.get("ring_buffer_size", 64)))

    max_init_cap = OnlineInitializer.MAX_BUFFER_SAMPLES
    init_t = np.empty(max_init_cap, dtype=np.int64)
    init_omega = np.empty((max_init_cap, 3), dtype=np.float64)
    init_acc = np.empty((max_init_cap, 3), dtype=np.float64)
    init_valid = np.empty(max_init_cap, dtype=bool)
    init_count = 0

    is_initialized = False
    init_res: Optional[InitResult] = None

    state: Optional[NominalState] = None
    P: Optional[np.ndarray] = None
    g_w: Optional[np.ndarray] = None
    g_mag: float = 9.80665
    t0_ns: int = 0
    prev_t_ns: int = 0
    vibe_ema: float = 0.0
    ped_energy: float = 0.0
    dyn_acc_ema: float = 0.0
    prev_acc_raw = np.zeros(3, dtype=np.float64)
    has_prev_acc: bool = False
    dt_samples = []

    with open(out_path, "w", encoding="utf-8") as f_out:
        f_out.write("# timestamp tx ty tz qx qy qz qw\n")

        with parse_imu_stream(csv_path, params=cfg) as stream:
            for t_curr, omega_curr, acc_curr, is_valid_curr, _ in stream:
                if not is_initialized:
                    if init_count < max_init_cap:
                        init_t[init_count] = t_curr
                        init_omega[init_count] = omega_curr
                        init_acc[init_count] = acc_curr
                        init_valid[init_count] = is_valid_curr
                        init_count += 1

                    ready = initializer.feed(t_curr, omega_curr, acc_curr, is_valid=is_valid_curr)
                    if ready:
                        is_initialized = True
                        init_res = initializer.get_result()
                        assert init_res is not None
                        state = init_res.state.copy()
                        P = init_res.P.copy()
                        g_w = init_res.g_w.copy()
                        g_mag = float(init_res.g_mag)
                        t0_ns = int(init_res.t0)

                        for idx in range(init_count):
                            t_smp = int(init_t[idx])
                            w_smp = init_omega[idx]
                            a_smp = init_acc[idx]
                            v_smp = bool(init_valid[idx])

                            if idx == 0:
                                prev_t_ns = t_smp
                                sec = t_smp // 1_000_000_000
                                nsec = t_smp % 1_000_000_000
                                p_s = state.p
                                q_s = state.q
                                f_out.write(
                                    f"{sec}.{nsec:09d} {p_s[0]:.9f} {p_s[1]:.9f} {p_s[2]:.9f} "
                                    f"{q_s[1]:.9f} {q_s[2]:.9f} {q_s[3]:.9f} {q_s[0]:.9f}\n"
                                )
                                continue

                            dt = (t_smp - prev_t_ns) * 1e-9
                            prev_t_ns = t_smp

                            if dt <= 0.0 or not v_smp:
                                # Skip integration on negative dt or invalid sample
                                pass
                            else:
                                if len(dt_samples) < 200:
                                    dt_samples.append(dt)
                                state = strapdown_step(state, w_smp, a_smp, dt, g_w)
                                F = build_f_matrix(state, w_smp, a_smp, dt)
                                Q = build_q_discrete(dt, cfg, kappa=1.0)  # Fixed noise, no inflation
                                P = eskf_predict_covariance(P, F, Q, diag_floor=cov_floor, diag_cap=cov_cap)

                                t_sec = (t_smp - t0_ns) * 1e-9
                                stat_buf.add_sample(t_sec, w_smp, a_smp, state.b_g)
                                is_stat = stat_buf.is_stationary(g_mag=g_mag, quality=1.0, params=cfg)
                                gyro_win, acc_win = stat_buf.get_window_samples(zupt_window_s)

                                state, P, _, _ = tilt_update(
                                    state, P, a_smp, w_smp,
                                    g_mag=g_mag, quality=1.0,
                                    params=cfg, r_scale=1.0,
                                )
                                if is_stat:
                                    state, P, _, _ = zupt_step(
                                        state, P, gyro_win, acc_win,
                                        g_mag=g_mag, quality=1.0,
                                        params=cfg, r_scale=1.0,
                                    )

                            sec = t_smp // 1_000_000_000
                            nsec = t_smp % 1_000_000_000
                            p_s = state.p
                            q_s = state.q
                            f_out.write(
                                f"{sec}.{nsec:09d} {p_s[0]:.9f} {p_s[1]:.9f} {p_s[2]:.9f} "
                                f"{q_s[1]:.9f} {q_s[2]:.9f} {q_s[3]:.9f} {q_s[0]:.9f}\n"
                            )
                        continue
                    continue

                # Once initialized
                assert state is not None and P is not None and g_w is not None

                dt = (t_curr - prev_t_ns) * 1e-9
                prev_t_ns = t_curr

                if dt <= 0.0 or not is_valid_curr:
                    pass
                else:
                    if len(dt_samples) < 200:
                        dt_samples.append(dt)
                    state = strapdown_step(state, omega_curr, acc_curr, dt, g_w)
                    F = build_f_matrix(state, omega_curr, acc_curr, dt)
                    Q = build_q_discrete(dt, cfg, kappa=1.0)
                    P = eskf_predict_covariance(P, F, Q, diag_floor=cov_floor, diag_cap=cov_cap)

                    t_sec = (t_curr - t0_ns) * 1e-9
                    stat_buf.add_sample(t_sec, omega_curr, acc_curr, state.b_g)
                    is_stat = stat_buf.is_stationary(g_mag=g_mag, quality=1.0, params=cfg)
                    gyro_win, acc_win = stat_buf.get_window_samples(zupt_window_s)

                    state, P, _, _ = tilt_update(
                        state, P, acc_curr, omega_curr,
                        g_mag=g_mag, quality=1.0,
                        params=cfg, r_scale=1.0,
                    )
                    if is_stat:
                        state, P, _, _ = zupt_step(
                            state, P, gyro_win, acc_win,
                            g_mag=g_mag, quality=1.0,
                            params=cfg, r_scale=1.0,
                        )

                sec = t_curr // 1_000_000_000
                nsec = t_curr % 1_000_000_000
                p_s = state.p
                q_s = state.q
                f_out.write(
                    f"{sec}.{nsec:09d} {p_s[0]:.9f} {p_s[1]:.9f} {p_s[2]:.9f} "
                    f"{q_s[1]:.9f} {q_s[2]:.9f} {q_s[3]:.9f} {q_s[0]:.9f}\n"
                )

        # Fallback if stream was shorter than T_init
        if not is_initialized and init_count > 0:
            ready = initializer.finalize_fallback()
            if ready:
                init_res = initializer.get_result()
            else:
                init_res = _fallback_initialization(
                    init_t[:init_count], init_omega[:init_count], init_acc[:init_count], cfg
                )
            assert init_res is not None
            state = init_res.state.copy()
            g_w = init_res.g_w.copy()
            for idx in range(init_count):
                t_smp = int(init_t[idx])
                if idx == 0:
                    prev_t_ns = t_smp
                else:
                    dt = (t_smp - prev_t_ns) * 1e-9
                    prev_t_ns = t_smp
                    if dt > 0.0 and init_valid[idx]:
                        state = strapdown_step(state, init_omega[idx], init_acc[idx], dt, g_w)
                if not np.all(np.isfinite(state.p)):
                    state.p = np.zeros(3, dtype=np.float64)
                if not np.all(np.isfinite(state.q)):
                    state.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
                sec = t_smp // 1_000_000_000
                nsec = t_smp % 1_000_000_000
                p_s = state.p
                q_s = state.q
                f_out.write(
                    f"{sec}.{nsec:09d} {p_s[0]:.9f} {p_s[1]:.9f} {p_s[2]:.9f} "
                    f"{q_s[1]:.9f} {q_s[2]:.9f} {q_s[3]:.9f} {q_s[0]:.9f}\n"
                )

        # Append blind (no blind model: naive constant velocity extrapolation)
        if append_blind and prev_t_ns > 0 and state is not None:
            dt_nom = float(np.median(dt_samples)) if dt_samples else 0.005
            dt_ns_step = max(1, int(round(dt_nom * 1e9)))
            blind_steps = int(round(5.0 / dt_nom))
            for step in range(1, blind_steps + 1):
                t_extrap = prev_t_ns + step * dt_ns_step
                p_extrap = state.p + state.v * (step * dt_nom)
                q_extrap = state.q
                sec = t_extrap // 1_000_000_000
                nsec = t_extrap % 1_000_000_000
                f_out.write(
                    f"{sec}.{nsec:09d} {p_extrap[0]:.9f} {p_extrap[1]:.9f} {p_extrap[2]:.9f} "
                    f"{q_extrap[1]:.9f} {q_extrap[2]:.9f} {q_extrap[3]:.9f} {q_extrap[0]:.9f}\n"
                )


def run_pipeline(
    imu_input: Union[str, Path],
    traj_output: Union[str, Path],
    ablation: str = "eskf_full",
    append_blind: bool = False,
    params: Optional[Dict[str, Any]] = None,
) -> None:
    """Run specified estimator ablation mode on IMU input.

    Args:
        imu_input: Path to IMU CSV or sequence folder.
        traj_output: Path to write estimated trajectory file.
        ablation: Ablation mode: 'naive', 'mahony', 'eskf_basic', 'eskf_full'.
        append_blind: Whether to extrapolate 5.0 s poses after stream ends.
        params: Optional dictionary of filter parameters.
    """
    mode = ablation.lower().strip()
    if mode == "naive":
        run_naive_baseline(imu_input, traj_output, append_blind=append_blind)
    elif mode == "mahony":
        run_mahony_baseline(imu_input, traj_output, append_blind=append_blind)
    elif mode == "eskf_basic":
        run_eskf_basic(imu_input, traj_output, append_blind=append_blind, params=params)
    elif mode == "eskf_full":
        run_eskf_full(imu_input, traj_output, append_blind=append_blind, params=params)
    else:
        raise ValueError(
            f"Unknown ablation mode '{ablation}'. Supported: naive, mahony, eskf_basic, eskf_full"
        )
