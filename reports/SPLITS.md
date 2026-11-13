# SPLITS.md — Data Split Definition

**Committed before any tuning. Do not modify.**
**Commit:** (to be filled after commit)
**Date:** 2026-11-13

---

## Purpose

This file defines the train/stress/final partition for the Track 03 "Path Finder" experiments.
Once committed, the Final Set is **sealed**: do not evaluate against it, do not look at its numbers,
until Phase 5 (`release-candidate` tag is applied).

---

## Dev Set (open — tuning, ablations, LOOCV)

| Sequence | Type | Dataset | Seeds |
|----------|------|---------|-------|
| MH_01_easy | MAV flight | EuRoC | 1–10 |
| V1_01_easy | MAV flight | EuRoC | 1–10 |
| V2_02_medium | MAV flight | EuRoC | 1–10 |
| room1 | Handheld | TUM-VI | 1–10 |
| room2 | Handheld | TUM-VI | 1–10 |

**LOOCV folds:** Hold out one sequence at a time (5 folds × 10 seeds = 50 runs for a full dev evaluation).

**Dev seeds:** `{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}` (deterministic via degrade.py RNG).

> [!IMPORTANT]
> Seeds 1–10 are **dev seeds only**. Final seeds 101–110 are for the Final Set only.
> There is no overlap.

---

## Stress Set (robustness diagnostics — not scored)

| Sequence | Type | Purpose |
|----------|------|---------|
| UZH-FPV fwd7 | Agile flight | Extreme dynamics stress test |
| synthetic circle | Synthetic | Regression: circular motion |
| synthetic figure_eight | Synthetic | Regression: figure-eight motion |

> [!NOTE]
> UZH-FPV is outside the scored datasets. Use it to check robustness only. Never let its
> numbers influence parameter selection.

---

## Final Set (SEALED — run ONCE in Phase 5)

| Sequence | Type | Dataset | Seeds |
|----------|------|---------|-------|
| MH_02_easy | MAV flight | EuRoC | 101–110 |
| MH_03_medium | MAV flight | EuRoC | 101–110 |
| MH_04_difficult | MAV flight | EuRoC | 101–110 |
| MH_05_difficult | MAV flight | EuRoC | 101–110 |
| V1_02_medium | MAV flight | EuRoC | 101–110 |
| V2_01_easy | MAV flight | EuRoC | 101–110 |
| V2_03_difficult | MAV flight | EuRoC | 101–110 |
| room3 | Handheld | TUM-VI | 101–110 |
| room4 | Handheld | TUM-VI | 101–110 |
| room5 | Handheld | TUM-VI | 101–110 |
| room6 | Handheld | TUM-VI | 101–110 |

> [!CAUTION]
> If any of the above sequences are unavailable, fall back to sealing **V1_01_easy** and
> **room2** from the dev set, reducing the dev set to 3 sequences. Record the fallback here
> and do NOT use sealed sequences for tuning.

**Final seeds:** `{101, 102, 103, 104, 105, 106, 107, 108, 109, 110}`

**Final Set results go in:** `reports/FINAL_SET.md`
**Code must be tagged** `release-candidate` **before the Final Set is run.**
**Record the git commit hash** in `reports/FINAL_SET.md`.

---

## Noise Floor Protocol

Run the **frozen baseline** (`baseline-honest-eval`, commit `35c3334`) on 10 dev seeds.
For each scorecard metric, compute the standard deviation across 10 seeds × 5 sequences.
An experiment improvement must exceed **2× this std** to be considered real.

Noise floor results: `reports/NOISE_FLOOR.md`

---

## Degradation Config

All evaluations use: `configs/degrade_default.json`

```json
{
  "noise_growth_rate": 0.05,
  "bias_drift_walk": 0.001,
  "dropouts": [
    {"start_s": 10.0, "duration_s": 1.5, "encoding": "missing"},
    {"start_s": 25.0, "duration_s": 2.0, "encoding": "nan"},
    {"start_s": 40.0, "duration_s": 1.0, "encoding": "zeros"},
    {"start_s": 55.0, "duration_s": 0.8, "encoding": "repeated"}
  ],
  "gravity_rotation": true,
  "spike_rate": 0.02,
  "failure_at_end_s": 5.0
}
```

The 2 s NaN gap at t=25 s is the primary dropout metric window.
The final 5 s failure is the blind dead-reckoning window.
