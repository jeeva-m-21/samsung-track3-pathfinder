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


