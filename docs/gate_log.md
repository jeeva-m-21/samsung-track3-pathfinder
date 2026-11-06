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
