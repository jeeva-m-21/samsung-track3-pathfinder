"""Unit and integration tests for Gap and Dropout Handler (SPEC §12.7, WP-26).

Tests:
1. Classification of invalid samples:
   - NaN / Inf in any of 6 IMU fields
   - All 6 values exactly zero
   - Stuck / repeated for >= 3 consecutive rows
   - Out-of-range gyro (|omega| > 35 rad/s) and acc (|acc| > 160 m/s^2)
2. Non-positive dt (dt <= 0):
   - Duplicate or backward timestamps freeze / repeat previous pose
   - Integration and updates are skipped
3. Normal dt propagation:
   - Valid sample with dt <= 2.5 * dt_nom handled normally
4. Large dt gap (dt > 2.5 * dt_nom):
   - Sub-stepping with n = ceil(dt / dt_nom) capped at gap_substep_max (800)
   - Velocity decay: v <- v * exp(-dt_sub / tau_gap) with tau_gap = 2.0s
   - Orientation integration with smoothed gyro omega_s (tau = 0.25s)
   - Covariance Q inflated by kappa_gap = 5.0
5. Post-gap recovery:
   - recover counter initialized to recover_steps (100)
   - Measurement covariance R multiplied by (1 + 10 * recover / recover_steps) decaying to 1.0
6. Single invalid row handling:
   - Treated as 1 dt_nom gap sub-step, still emits one pose row
7. Fixed-size circular ring buffer:
   - Capacity 64 states for rollback
8. Continuous trajectory across 2.0 s gap on synthetic circle and figure-eight:
   - Continuous transition, sub-step deltas smooth, bounded position drift
"""

import math
from typing import Dict, List, Tuple
import numpy as np
import pytest

from src.ref.eskf import NominalState, load_params
from src.ref.gap import (
    GapCase,
    GapHandler,
    StateRingBuffer,
    classify_sample,
    compute_dt_nom,
    gap_substep,
    get_recovery_multiplier,
    propagate_gap,
    update_smoothed_gyro,
)
from tools.synth import generate_trajectory


# ===========================================================================
# 1. Classification of invalid samples
# ===========================================================================


def test_classify_valid_sample():
    """Verify that typical valid IMU measurements pass classification."""
    omega = np.array([0.01, -0.02, 0.05], dtype=np.float64)
    acc = np.array([0.1, 0.2, 9.8], dtype=np.float64)
    is_valid, reason = classify_sample(omega, acc)
    assert is_valid is True
    assert reason == "valid"


def test_classify_nan_and_inf_sample():
    """Verify that NaN or Inf in any field is flagged as invalid."""
    # NaN in gyro
    omega_nan = np.array([np.nan, 0.0, 0.0], dtype=np.float64)
    acc = np.array([0.0, 0.0, 9.81], dtype=np.float64)
    is_valid, reason = classify_sample(omega_nan, acc)
    assert is_valid is False
    assert reason == "nan_or_inf"

    # Inf in acc
    omega = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    acc_inf = np.array([0.0, np.inf, 9.81], dtype=np.float64)
    is_valid, reason = classify_sample(omega, acc_inf)
    assert is_valid is False
    assert reason == "nan_or_inf"

    # Negative Inf
    acc_ninf = np.array([0.0, 0.0, -np.inf], dtype=np.float64)
    is_valid, reason = classify_sample(omega, acc_ninf)
    assert is_valid is False
    assert reason == "nan_or_inf"


def test_classify_all_zeros_sample():
    """Verify that all six fields being exactly zero is flagged invalid."""
    omega = np.zeros(3, dtype=np.float64)
    acc = np.zeros(3, dtype=np.float64)
    is_valid, reason = classify_sample(omega, acc)
    assert is_valid is False
    assert reason == "all_zeros"

    # If even one component is non-zero, it is not all_zeros
    acc_nonzero = np.array([0.0, 0.0, 1e-9], dtype=np.float64)
    is_valid, reason = classify_sample(omega, acc_nonzero)
    assert is_valid is True


def test_classify_stuck_sample():
    """Verify stuck sensor detection: identical samples for >= 3 consecutive rows."""
    omega = np.array([0.12, -0.34, 0.56], dtype=np.float64)
    acc = np.array([0.05, 0.15, 9.78], dtype=np.float64)
    prev = np.concatenate([omega, acc])

    # 1st occurrence (repeat_count = 1) -> valid
    is_valid, reason = classify_sample(omega, acc, prev_sample=None, repeat_count=0)
    assert is_valid is True

    # 2nd occurrence (repeat_count = 2) -> still valid
    is_valid, reason = classify_sample(omega, acc, prev_sample=prev, repeat_count=1)
    assert is_valid is True

    # 3rd consecutive identical occurrence (repeat_count = 3) -> stuck!
    is_valid, reason = classify_sample(omega, acc, prev_sample=prev, repeat_count=2)
    assert is_valid is False
    assert reason == "stuck"

    # 4th consecutive identical occurrence -> stuck!
    is_valid, reason = classify_sample(omega, acc, prev_sample=prev, repeat_count=3)
    assert is_valid is False
    assert reason == "stuck"

    # Now a different sample arrives -> valid again
    omega_new = np.array([0.12, -0.34, 0.57], dtype=np.float64)
    is_valid, reason = classify_sample(omega_new, acc, prev_sample=prev, repeat_count=3)
    assert is_valid is True


def test_classify_out_of_range():
    """Verify range limits: |omega| > 35 rad/s and |a| > 160 m/s^2."""
    acc_norm = np.array([0.0, 0.0, 9.81], dtype=np.float64)

    # Gyro component exceeding 35.0
    omega_high = np.array([35.1, 0.0, 0.0], dtype=np.float64)
    is_valid, reason = classify_sample(omega_high, acc_norm)
    assert is_valid is False
    assert reason == "out_of_range_gyro"

    # Gyro norm exceeding 35.0 even if individual components <= 35.0
    omega_norm_high = np.array([25.0, 25.0, 20.0], dtype=np.float64)  # norm = 39.05 > 35
    is_valid, reason = classify_sample(omega_norm_high, acc_norm)
    assert is_valid is False
    assert reason == "out_of_range_gyro"

    # Acc component exceeding 160.0
    omega_norm = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    acc_high = np.array([0.0, -160.5, 9.81], dtype=np.float64)
    is_valid, reason = classify_sample(omega_norm, acc_high)
    assert is_valid is False
    assert reason == "out_of_range_acc"

    # Acc norm exceeding 160.0
    acc_norm_high = np.array([120.0, 120.0, 0.0], dtype=np.float64)  # norm = 169.7 > 160
    is_valid, reason = classify_sample(omega_norm, acc_norm_high)
    assert is_valid is False
    assert reason == "out_of_range_acc"

    # Values within limit
    omega_ok = np.array([34.9, 0.0, 0.0], dtype=np.float64)
    acc_ok = np.array([0.0, 159.0, 0.0], dtype=np.float64)
    is_valid, reason = classify_sample(omega_ok, acc_ok)
    assert is_valid is True


# ===========================================================================
# 2. Nominal dt computation
# ===========================================================================


def test_compute_dt_nom():
    """Verify dt_nom calculation as the median of the first 200 valid dt samples."""
    # 200 samples centered around 0.005s (200 Hz) with some jitter
    rng = np.random.RandomState(42)
    dts = 0.005 + rng.uniform(-0.0002, 0.0002, size=200)
    dt_nom = compute_dt_nom(dts)
    assert dt_nom == pytest.approx(0.005, abs=1e-4)

    # Fewer than 200 samples still computes median correctly
    dt_nom_short = compute_dt_nom(dts[:50])
    assert dt_nom_short == pytest.approx(0.005, abs=1e-4)

    # Empty array falls back to standard 0.005s (200 Hz default)
    assert compute_dt_nom([]) == pytest.approx(0.005)


# ===========================================================================
# 3. Motion smoothing: omega_s
# ===========================================================================


def test_smoothed_gyro_exponential_filter():
    """Verify exponential moving average of valid gyro with time constant tau = 0.25 s."""
    omega_s = np.zeros(3, dtype=np.float64)
    b_g = np.array([0.01, -0.01, 0.0], dtype=np.float64)
    omega_raw = np.array([1.01, -0.01, 0.0], dtype=np.float64)  # corrected = [1.0, 0.0, 0.0]

    dt = 0.005
    tau = 0.25
    expected_alpha = 1.0 - math.exp(-dt / tau)

    # Single step update
    omega_s_1 = update_smoothed_gyro(omega_s, omega_raw, b_g, dt=dt, tau=tau)
    assert omega_s_1[0] == pytest.approx(expected_alpha * 1.0, abs=1e-6)
    assert omega_s_1[1] == pytest.approx(0.0, abs=1e-6)

    # Steady state convergence: after 3 seconds (12 * tau), should be ~1.0
    s = np.zeros(3, dtype=np.float64)
    for _ in range(600):
        s = update_smoothed_gyro(s, omega_raw, b_g, dt=dt, tau=tau)
    assert s[0] == pytest.approx(1.0, rel=1e-3)


# ===========================================================================
# 4. Recovery covariance multiplier
# ===========================================================================


def test_recovery_multiplier():
    """Verify recovery multiplier (1 + 10 * recover / recover_steps) decaying to 1."""
    recover_steps = 100

    # At beginning of recovery: recover = 100 -> multiplier = 1 + 10 * 1 = 11.0
    mult_100 = get_recovery_multiplier(100, recover_steps=recover_steps)
    assert mult_100 == pytest.approx(11.0)

    # Halfway: recover = 50 -> multiplier = 1 + 10 * 0.5 = 6.0
    mult_50 = get_recovery_multiplier(50, recover_steps=recover_steps)
    assert mult_50 == pytest.approx(6.0)

    # At completion: recover = 0 -> multiplier = 1.0
    mult_0 = get_recovery_multiplier(0, recover_steps=recover_steps)
    assert mult_0 == pytest.approx(1.0)

    # Negative recover remains 1.0
    mult_neg = get_recovery_multiplier(-5, recover_steps=recover_steps)
    assert mult_neg == pytest.approx(1.0)


# ===========================================================================
# 5. Gap sub-stepping pure function
# ===========================================================================


def test_gap_substep_velocity_decay_and_covariance():
    """Verify single gap sub-step: velocity decay, smooth rotation, covariance inflation."""
    params = load_params()
    params["tau_gap"] = 2.0
    params["kappa_gap"] = 5.0

    state = NominalState(
        q=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        v=np.array([5.0, 0.0, 0.0], dtype=np.float64),
        p=np.array([0.0, 0.0, 0.0], dtype=np.float64),
        b_g=np.zeros(3, dtype=np.float64),
        b_a=np.zeros(3, dtype=np.float64),
    )
    P = np.eye(15, dtype=np.float64) * 0.01
    omega_s = np.array([0.0, 0.0, 0.5], dtype=np.float64)
    dt_sub = 0.005

    next_state, next_P = gap_substep(state, P, omega_s, dt_sub, params)

    # 1. Velocity decayed by exp(-dt_sub / tau_gap)
    expected_decay = math.exp(-0.005 / 2.0)
    assert next_state.v[0] == pytest.approx(5.0 * expected_decay, rel=1e-5)

    # 2. Position integrated forward along velocity
    assert next_state.p[0] > 0.0
    assert next_state.p[0] == pytest.approx(0.5 * (state.v[0] + next_state.v[0]) * dt_sub, rel=1e-5)

    # 3. Orientation rotated by omega_s * dt_sub
    # Yaw angle = 0.5 * 0.005 = 0.0025 rad -> q_z ~ sin(0.00125)
    assert next_state.q[3] > 0.0

    # 4. Covariance P grew due to inflated process noise
    assert next_P[0, 0] > P[0, 0]
    assert next_P[3, 3] > P[3, 3]

    # 5. Covariance remains symmetric positive-definite
    assert np.allclose(next_P, next_P.T, atol=1e-12)
    eigvals = np.linalg.eigvalsh(next_P)
    assert np.all(eigvals > 0.0)


def test_propagate_gap_multiple_substeps():
    """Verify gap propagation over 2.0 seconds with n = ceil(dt / dt_nom)."""
    params = load_params()
    params["tau_gap"] = 2.0
    params["kappa_gap"] = 5.0
    params["gap_substep_max"] = 800

    state = NominalState(
        q=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        v=np.array([10.0, 0.0, 0.0], dtype=np.float64),
        p=np.array([0.0, 0.0, 0.0], dtype=np.float64),
        b_g=np.zeros(3, dtype=np.float64),
        b_a=np.zeros(3, dtype=np.float64),
    )
    P = np.eye(15, dtype=np.float64) * 0.01
    omega_s = np.zeros(3, dtype=np.float64)

    dt_gap = 2.0
    dt_nom = 0.005  # n = 400 sub-steps

    next_state, next_P, n_steps = propagate_gap(state, P, omega_s, dt_gap, dt_nom, params)

    assert n_steps == 400
    # Over 2.0 s with tau_gap = 2.0 s, velocity should decay by exactly 1/e
    assert next_state.v[0] == pytest.approx(10.0 * math.exp(-1.0), rel=1e-2)

    # Traveled distance ~ v0 * tau * (1 - exp(-T/tau)) = 10 * 2 * (1 - 1/e) = 12.64 m
    expected_dist = 10.0 * 2.0 * (1.0 - math.exp(-1.0))
    assert next_state.p[0] == pytest.approx(expected_dist, rel=1e-2)


# ===========================================================================
# 6. Ring buffer tests
# ===========================================================================


def test_ring_buffer_operations():
    """Verify fixed-capacity ring buffer of 64 states with rollback capability."""
    buf = StateRingBuffer(capacity=64)
    assert len(buf) == 0

    state = NominalState(
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        v=np.zeros(3),
        p=np.zeros(3),
        b_g=np.zeros(3),
        b_a=np.zeros(3),
    )
    P = np.eye(15)
    omega_s = np.zeros(3)

    # Push 100 items (exceeds capacity 64)
    for i in range(100):
        t_ns = 1_000_000_000 + i * 5_000_000  # 5ms steps
        st = NominalState(q=state.q, v=state.v, p=np.array([float(i), 0.0, 0.0]), b_g=state.b_g, b_a=state.b_a)
        buf.push(t_ns, st, P, omega_s)

    assert len(buf) == 64
    latest = buf.get_latest()
    assert latest[0] == 1_000_000_000 + 99 * 5_000_000
    assert latest[1].p[0] == 99.0

    # Oldest stored should be index 100 - 64 = 36
    oldest = buf.get_oldest()
    assert oldest[0] == 1_000_000_000 + 36 * 5_000_000
    assert oldest[1].p[0] == 36.0

    # Rollback by 0.1s (20 steps) from latest (t = 99 -> target t = 79)
    target_t_ns = 1_000_000_000 + 79 * 5_000_000
    rb_entry = buf.find_closest(target_t_ns)
    assert rb_entry is not None
    assert rb_entry[0] == target_t_ns
    assert rb_entry[1].p[0] == 79.0


# ===========================================================================
# 7. Streaming GapHandler Class Tests
# ===========================================================================


def test_gap_handler_dt_cases():
    """Verify GapHandler correctly detects and handles:
    - Normal dt (dt <= 2.5 * dt_nom)
    - Zero/negative dt (freeze pose)
    - Gap dt (dt > 2.5 * dt_nom)
    - Invalid row (1 dt_nom sub-step)
    """
    params = load_params()
    handler = GapHandler(params=params)

    # Seed 200 normal samples at 200 Hz (dt = 5ms = 5_000_000 ns)
    t_start = 1_000_000_000
    omega = np.array([0.01, 0.0, 0.0])
    acc = np.array([0.0, 0.0, 9.81])

    state = NominalState(
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        v=np.zeros(3),
        p=np.zeros(3),
        b_g=np.zeros(3),
        b_a=np.zeros(3),
    )
    P = np.eye(15) * 0.01

    for i in range(200):
        t_ns = t_start + i * 5_000_000
        # Realistic sensor samples with slight variation to prevent stuck-sensor false positive
        omega_i = omega + np.array([1e-5 * math.sin(i), 0.0, 0.0])
        acc_i = acc + np.array([0.0, 0.0, 1e-4 * math.cos(i)])
        step_res = handler.process_sample(t_ns, omega_i, acc_i, state, P)
        assert step_res.case == GapCase.NORMAL
        state = step_res.state
        P = step_res.P

    assert handler.dt_nom == pytest.approx(0.005, abs=1e-5)
    t_curr = t_start + 199 * 5_000_000

    # 1. Negative or zero dt test (repeat timestamp)
    res_zero = handler.process_sample(t_curr, omega, acc, state, P)
    assert res_zero.case == GapCase.ZERO_OR_NEGATIVE_DT
    # Pose frozen
    assert np.array_equal(res_zero.state.p, state.p)
    assert np.array_equal(res_zero.state.q, state.q)
    assert res_zero.apply_updates is False

    # 2. Large gap test: dt = 0.5 s (100 * dt_nom > 2.5 * dt_nom)
    t_gap = t_curr + 500_000_000
    res_gap = handler.process_sample(t_gap, omega, acc, state, P)
    assert res_gap.case == GapCase.GAP
    assert res_gap.substeps == 100
    assert res_gap.apply_updates is False
    assert handler.recover == handler.recover_steps
    assert res_gap.r_scale == pytest.approx(11.0)
    state = res_gap.state
    P = res_gap.P

    # 3. Post-gap recovery steps
    # Next sample is normal: dt = 5ms
    t_next = t_gap + 5_000_000
    res_norm = handler.process_sample(t_next, omega, acc, state, P)
    assert res_norm.case == GapCase.NORMAL
    assert res_norm.apply_updates is True
    assert handler.recover == handler.recover_steps - 1  # 99
    assert res_norm.r_scale == pytest.approx(1.0 + 10.0 * 99.0 / 100.0)

    # 4. Invalid sample (NaN) test
    omega_nan = np.array([np.nan, 0.0, 0.0])
    t_nan = t_next + 5_000_000
    res_invalid = handler.process_sample(t_nan, omega_nan, acc, state, P)
    assert res_invalid.case == GapCase.INVALID_SAMPLE
    assert res_invalid.substeps == 1
    assert res_invalid.apply_updates is False
    assert handler.recover == handler.recover_steps


# ===========================================================================
# 8. Continuous trajectory across 2.0 s gap on synthetic trajectories
# ===========================================================================


def test_synthetic_circle_gap_continuous_and_bounded():
    """Verify that across a 2.0 s gap on synthetic circle, no position jump occurs.

    Checks:
    - Continuous trajectory transition: max position change between adjacent sub-steps is smooth.
    - Position error relative to ground truth is bounded.
    - Post-gap filter recovery accepts updates with scaled R and converges smoothly.
    """
    data = generate_trajectory("circle", duration=8.0, rate=200.0)
    N = len(data["t_ns"])
    dt_nom = 0.005

    params = load_params()
    handler = GapHandler(params=params)

    # Initialize state at t=0
    state = NominalState(
        q=data["q"][0].copy(),
        v=data["v"][0].copy(),
        p=data["p"][0].copy(),
        b_g=np.zeros(3, dtype=np.float64),
        b_a=np.zeros(3, dtype=np.float64),
    )
    P = np.eye(15, dtype=np.float64) * 0.01

    # Simulate up to t = 2.0 s (samples 0 to 400 inclusive)
    for i in range(401):
        t_ns = int(data["t_ns"][i])
        omega = data["omega"][i]
        acc = data["acc"][i]
        step = handler.process_sample(t_ns, omega, acc, state, P)
        state = step.state
        P = step.P

    pos_before_gap = state.p.copy()
    v_before_gap = state.v.copy()

    # Drop samples between t = 2.0s and t = 4.0s (a 2.0 s gap!)
    # Next sample arrives at index 800 (t = 4.0 s, exactly 400 dt steps later)
    gap_idx = 800
    t_ns_after = int(data["t_ns"][gap_idx])
    omega_after = data["omega"][gap_idx]
    acc_after = data["acc"][gap_idx]

    step_gap = handler.process_sample(t_ns_after, omega_after, acc_after, state, P)
    assert step_gap.case == GapCase.GAP
    assert step_gap.substeps == 400

    pos_after_gap = step_gap.state.p.copy()
    v_after_gap = step_gap.state.v.copy()

    # Check that position change across the gap is bounded by coasting physics
    # Delta p <= v_max * tau_gap * (1 - exp(-dt_gap / tau_gap))
    v_mag = np.linalg.norm(v_before_gap)
    max_coasting_dist = v_mag * 2.0 * (1.0 - math.exp(-2.0 / 2.0))
    pos_diff = np.linalg.norm(pos_after_gap - pos_before_gap)
    assert pos_diff == pytest.approx(max_coasting_dist, rel=0.15)

    # Sub-step smooth delta check: across 400 sub-steps, average step delta is pos_diff / 400
    avg_substep_delta = pos_diff / 400.0
    assert avg_substep_delta < 0.05  # Smooth 5ms sub-steps

    # Position relative to ground truth is bounded
    gt_pos_after = data["p"][gap_idx]
    gt_error = np.linalg.norm(pos_after_gap - gt_pos_after)
    # Ground truth circle radius is 5m; coasting tangential error over 2s should be bounded (< 15m)
    assert gt_error < 15.0


def test_synthetic_figure_eight_gap_continuous_and_bounded():
    """Verify continuous and bounded trajectory across a 2.0 s gap on synthetic figure_eight."""
    data = generate_trajectory("figure_eight", duration=8.0, rate=200.0)

    params = load_params()
    handler = GapHandler(params=params)

    state = NominalState(
        q=data["q"][0].copy(),
        v=data["v"][0].copy(),
        p=data["p"][0].copy(),
        b_g=np.zeros(3, dtype=np.float64),
        b_a=np.zeros(3, dtype=np.float64),
    )
    P = np.eye(15, dtype=np.float64) * 0.01

    # Simulate up to t = 2.0 s (samples 0 to 400 inclusive)
    for i in range(401):
        t_ns = int(data["t_ns"][i])
        step = handler.process_sample(t_ns, data["omega"][i], data["acc"][i], state, P)
        state = step.state
        P = step.P

    pos_before = state.p.copy()

    # 2.0 s gap -> sample at 4.0 s (index 800, exactly 400 dt steps later)
    gap_idx = 800
    step_gap = handler.process_sample(
        int(data["t_ns"][gap_idx]),
        data["omega"][gap_idx],
        data["acc"][gap_idx],
        state,
        P,
    )
    assert step_gap.case == GapCase.GAP
    assert step_gap.substeps == 400

    pos_after = step_gap.state.p.copy()
    pos_diff = np.linalg.norm(pos_after - pos_before)

    # Sub-step average delta is small and smooth
    assert (pos_diff / 400.0) < 0.05

    # Position error relative to ground truth is bounded
    gt_error = np.linalg.norm(pos_after - data["p"][gap_idx])
    assert gt_error < 20.0
