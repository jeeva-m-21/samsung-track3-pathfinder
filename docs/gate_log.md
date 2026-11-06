# Gate Verification Log

Append-only record of gate evaluations and outputs.

## Gate G0 Verification (WP-00)
- Date: 2026-11-06T13:40:00+05:30
- Command: `./scripts/gate_g0.sh`
- Output:
```
=== Running Gate G0 Checks ===
Repository layout and basic tools verified.
GATE G0 PASS
```
- Verdict: PASS

## Gate G1 Verification (WP-15)
- Date: 2026-11-06T14:56:20+05:30
- Command: `PATH=".venv/bin:$PATH" PYTHONPATH=. bash scripts/gate_g1.sh`
- Output:
```
=== Running Gate G1 Checks ===
tests/test_degrade.py .....                                              [ 23%]
tests/test_validate.py .......                                           [ 57%]
tests/test_metrics.py .....                                              [ 80%]
tests/test_baseline.py ....                                              [100%]
============================== 21 passed in 1.23s ==============================
=== Running Evaluation Harness Smoke Test ===

| Estimator | ATE (m) | RPE 1s (m) | Start-Anchored (m) | Checkpoint (m) | Blind-5s (m) | Rot Err (°) | Objective J |
| --------- | ------- | ---------- | ------------------ | -------------- | ------------ | ----------- | ----------- |
| naive     | 1.3452  | 2.6824     | 7.2197             | 1.3452         | 13.9211      | 2.64        | 1.0000      |
| mahony    | 2.0381  | 2.2855     | 6.0947             | 2.0381         | 10.5289      | 5.66        | 1.2639      |

Results table written to results/table_g1.md
GATE G1 PASS
```
- Verdict: PASS

## Work Packet WP-20 Verification
- Date: 2026-11-06T15:15:00+05:30
- Commands:
  - `bash scripts/check_scope.sh WP-20`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest tests/test_mathlib.py`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest -q`
- Output:
```
Scope OK for WP-20
============================== 9 passed in 0.14s ===============================
...........................................                              [100%]
43 passed in 1.97s
```
- Mutation check: Mutated `skew` anti-symmetry in `src/ref/mathlib.py`; pytest failed with 2 failures (exit code 1); restored and verified 9/9 passed.
- Verdict: PASS

## Work Packet WP-21 Verification
- Date: 2026-11-06T15:26:00+05:30
- Commands:
  - `bash scripts/check_scope.sh WP-21`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest tests/test_parser.py`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest -q`
- Output:
```
Scope OK for WP-21
============================== 13 passed in 0.13s ==============================
........................................................                 [100%]
56 passed in 3.02s
```
- Mutation check: Mutated `is_valid = False` in `parse_imu_line` (`src/ref/parser.py`); pytest failed with 4 failures (exit code 1); restored and verified 13/13 passed.
- Checklist:
  - Tests exist for all items under "Tests first": YES
  - No magic numbers; constants come from configs/params.json: YES
  - No forbidden constructs (no global state, no dicts inside filter loops, fixed-size math): YES
  - Output format unchanged: YES
  - decisions.md updated for every deviation: YES (no deviations)
- Verdict: PASS

## Work Packet WP-22 Verification
- Date: 2026-11-06T15:40:00+05:30
- Commands:
  - `bash scripts/check_scope.sh WP-22`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest tests/test_init.py tests/test_strapdown.py`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest -q`
- Output:
```
Scope OK for WP-22
============================== 15 passed in 0.62s ==============================
.......................................................................  [100%]
71 passed in 2.71s
```
- Mutation check: Mutated position integration (`p_new = state.p + state.v * dt_f`) in `src/ref/strapdown.py`; pytest failed with 2 failures (exit code 1); restored and verified 7/7 passed.
- Checklist:
  - Tests exist for all items under "Tests first": YES (8 in test_init.py, 7 in test_strapdown.py)
  - Static 60s IMU drift < 1e-6 m: YES (tested upright and arbitrarily tilted)
  - No magic numbers; constants come from configs/params.json: YES
  - No forbidden constructs (pure functions, no global state, no dicts in filter loops, fixed-size math): YES
  - Output format unchanged: YES
  - decisions.md updated for every deviation: YES (no deviations)
- Verdict: PASS

## Work Packet WP-23 Verification
- Date: 2026-11-06T15:54:00+05:30
- Commands:
  - `bash scripts/check_scope.sh WP-23`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest tests/test_eskf.py tests/test_jacobians.py`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest -q`
- Output:
```
Scope OK for WP-23
============================== 13 passed in 0.43s ==============================
........................................................................ [ 85%]
............                                                             [100%]
84 passed in 3.31s
```
- Mutation checks:
  1. Mutated F position transition (`F[6:9, 3:6] = np.eye(3) * 2.0 * dt`) in `src/ref/eskf.py`; failed 3 tests in `tests/test_jacobians.py` (`test_analytical_f_structure`, `test_f_against_finite_differences_small_dt`, `test_f_against_finite_differences_nominal_dt`); restored and verified clean.
  2. Mutated discrete Q calculation (`q_theta = (sigma_g * sigma_g) * 2.0 * dt`) in `src/ref/eskf.py`; failed `test_discrete_q_construction` in `tests/test_eskf.py`; restored and verified clean.
- Checklist:
  - Tests exist for all items under "Tests first": YES (9 in test_eskf.py, 4 in test_jacobians.py)
  - Numerical Jacobian verification matches finite differences to O(dt^2): YES (max diff < 1e-6 at dt=1e-4s; all blocks match at dt=0.005s)
  - P remains symmetric positive-definite after 10,000 steps: YES (asym < 1e-12, all eigvals > 0, Cholesky passes)
  - Discrete Q matches SPEC §12.4: YES (15x15 block diagonal with kappa factor)
  - Generic Joseph update helper implemented and preserves positive definiteness: YES (closed form 3x3 inversion with det floor, D P D congruence scaling on cap)
  - State injection into (q, v, p, b_g, b_a) and error state reset: YES (unit quaternion renormalization, sign continuity, G matrix reset)
  - No magic numbers; constants come from configs/params.json: YES
  - No forbidden constructs (no global state, no dicts inside filter loops, fixed-size math): YES
  - Output format unchanged: YES
  - decisions.md updated for every deviation: YES (no deviations)
- Verdict: PASS




## Work Packet WP-24 Verification
- Date: 2026-11-06T16:11:00+05:30
- Commands:
  - `bash scripts/check_scope.sh WP-24`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest tests/test_updates.py`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest -q`
- Output:
```
Scope OK for WP-24
============================== 20 passed in 0.15s ==============================
........................................................................ [ 69%]
................................                                         [100%]
104 passed in 2.01s
```
- Mutation check: Mutated tilt innovation `y = z + h_hat` in `src/ref/updates.py`; failed `test_tilt_innovation_and_h_matrix` (exit code 1); restored and verified 20/20 passed.
- Checklist:
  - Tilt update converges on synth with initial 5 deg error: YES (pitch, roll, and combined roll+pitch converge to < 0.5 deg within 1.0s - 1.5s)
  - ZUPT stationary detector correctly identifies stationary vs moving windows: YES (passes on static window, rejects on gyro motion > 0.03 rad/s, accel dev > 0.15 m/s^2, or high variance)
  - ZUPT zeroes velocity drift: YES (reduces drift by > 99% in 2 updates, final v_norm < 0.01 m/s)
  - Gating and quality threshold rules respected: YES (tilt gating expands with degraded quality, ZUPT disables when quality < 0.3)
  - No magic numbers, constants from params.json, no global state, pure functions, portable to C++: YES (uses pre-allocated circular ring buffer in StationaryWindowBuffer and fixed-size arrays)
  - Output format unchanged: YES
  - decisions.md updated for every deviation: YES (no deviations)
- Verdict: PASS

## Work Packet WP-25 Verification
- Date: 2026-11-06T16:25:00+05:30
- Commands:
  - `bash scripts/check_scope.sh WP-25`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest tests/test_monitor.py`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest -q`
- Output:
```
Scope OK for WP-25
============================== 16 passed in 0.12s ==============================
........................................................................ [ 60%]
................................................                         [100%]
120 passed in 2.45s
```
- Mutation check: Mutated quality score calculation (`val = 2.0 / n`) in `src/ref/monitor.py`; failed `test_quality_score_clamping` (exit code 1); restored and verified 16/16 passed.
- Checklist:
  - High-frequency residual tracking for accel and gyro: YES (Euclidean norm residuals r_k and rho_k tested on identical vectors, 3D displacements, and lists)
  - Exponential moving variance with alpha = 0.01: YES (formula and asymptotic convergence verified)
  - Reference variance initialization from first 1.0s window: YES (tracks during static window, freezes at T_init=1.0s, nominal quality=1.0 and kappa=1.0)
  - Quality score falls monotonically as noise grows: YES (verified across progressive noise multipliers 1x to 64x; bounds 1.0 to 0.05 respected)
  - Inflation factor kappa = clamp(n, 1.0, kappa_max) where kappa_max = 50.0: YES (verified bounds [1.0, 50.0] and monotonic increase)
  - Static stationary gyro bias observation helper: YES (H=[0,0,0,I,0], R=sigma_bgm^2*I, innovation, bias trend EMA, and ESKF measurement update verified)
  - No magic numbers, constants from params.json, no global state, pure functions / fixed-size ring buffer, portable to C++: YES
  - Output format unchanged: YES
  - decisions.md updated for every deviation: YES (no deviations)
- Verdict: PASS

## Work Packet WP-26 Verification
- Date: 2026-11-06T23:59:00+05:30
- Commands:
  - `bash scripts/check_scope.sh WP-26`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest tests/test_gap.py`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest -q`
- Output:
```
Scope OK for WP-26
============================== 14 passed in 0.22s ==============================
........................................................................ [ 53%]
..............................................................           [100%]
134 passed in 3.14s
```
- Mutation check: Mutated velocity decay factor (`decay = 1.0`) in `src/ref/gap.py`; pytest failed with 3 failures (`test_gap_substep_velocity_decay_and_covariance`, `test_propagate_gap_multiple_substeps`, `test_synthetic_circle_gap_continuous_and_bounded`, exit code 1); restored and verified 14/14 passed.
- Checklist:
  - Invalid sample detection (NaN, all-zero, repeated stuck >= 3, out-of-range): YES (covered by 4 classification tests)
  - dt <= 0 duplicate handling: YES (freezes state, skips updates)
  - Missing row gaps sub-stepping with velocity decay v * exp(-dt_sub / tau_gap): YES (sub-stepping loop with tau_gap=2.0s, Q inflated by kappa_gap=5.0)
  - Across a 2.0 s gap on synthetic data, no position jump > bound occurs (continuous smooth transition): YES (verified on synthetic circle and figure-eight trajectories)
  - Invalid rows handled with single dt_nom gap sub-step: YES (emits 1 step, sets recover = recover_steps)
  - Post-gap recovery counter decaying R_meas inflation over recover_steps: YES (multiplier (1 + 10 * recover / recover_steps) decaying to 1.0)
  - Master clock int64 nanosecond subtraction used before float scaling: YES (`dt_ns = int(t_ns) - int(self.prev_t_ns); dt = float(dt_ns) * 1e-9`)
  - No magic numbers, constants from params.json, no global state, pure functions / fixed-size ring buffer, portable to C++: YES (StateRingBuffer preallocated capacity 64, pure functions for step and classification)
  - Output format unchanged: YES
  - decisions.md updated for every deviation: YES (no deviations)
- Verdict: PASS

## Work Packet WP-27 Verification
- Date: 2026-11-07T00:13:00+05:30
- Commands:
  - `bash scripts/check_scope.sh WP-27`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest tests/test_blind.py`
  - `PATH=".venv/bin:$PATH" PYTHONPATH=. pytest -q`
- Output:
```
Scope OK for WP-27
============================== 17 passed in 0.28s ==============================
........................................................................ [ 47%]
........................................................................ [ 95%]
.......                                                                  [100%]
151 passed in 2.71s
```
- Mutation check: Mutated position decay equation `decay_factor = 1.0 - math.exp(-2.0 * tau_val / tau_v_val)` in `src/ref/blind.py`; pytest failed with 1 failure (`test_predict_blind_position_math`, exit code 1); restored and verified 17/17 passed.
- Checklist:
  - Tests exist for all items under "Tests first": YES (17 tests covering all requirements)
  - Failure declaration after T_fail (1.0 s) continuous invalid data: YES (`test_blind_predictor_continuous_invalid_trigger`, `test_blind_predictor_stream_end`)
  - State rollback to pre-failure reliable state: YES (`test_rollback_to_reliable_prior_to_onset`, `test_reliable_ring_buffer_fifo`)
  - Decaying velocity kinematic model: p(tau) = p_f + v_f * shrink_v * tau_v * (1 - exp(-tau/tau_v)): YES (`test_predict_blind_position_math`, `test_predict_blind_velocity`)
  - Decaying turn rate kinematic model: q(tau) = q_f (x) exp(omega_f * tau_omega * (1 - exp(-tau/tau_omega))): YES (`test_predict_blind_orientation_math`)
  - Stationary threshold (< 0.05 m/s) holds position stationary: YES (`test_predict_blind_position_stationary_threshold`)
  - Poses beyond 5.0 s clamped to tau = 5.0 s pose: YES (`test_predict_blind_position_clamping_beyond_5s`, `test_predict_blind_orientation_clamping_beyond_5s`, `test_blind_predictor_generate_blind_poses`)
  - Blind RMSE < naive open-loop integration on synthetic circle & figure-eight failures: YES (`test_acceptance_synth_circle_blind_rmse_vs_naive`, `test_acceptance_synth_figure_eight_blind_rmse_vs_naive`)
  - No magic numbers, constants from params.json, no global state, pure functions / fixed-size ring buffer, portable to C++: YES (ReliableRingBuffer preallocated capacity 64, pure functions for extrapolation)
  - Output format unchanged: YES
  - decisions.md updated for every deviation: YES (no deviations)
- Verdict: PASS

