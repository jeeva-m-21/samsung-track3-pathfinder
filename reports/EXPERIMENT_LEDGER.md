# EXPERIMENT LEDGER — Track 03 "Path Finder" (VITC_SingleThread)

**Append-only. Never delete entries. Rejected experiments stay.**
**Format:** one `## EXP-NNN` section per experiment.

---

## Frozen Baseline

- **Tag:** `baseline-honest-eval`
- **Commit:** `35c3334`
- **Date:** 2026-11-13
- **Params:** `configs/params.json` (sha matches commit)
- **Dev set scorecard (seeds 42, 123, 456, 789, 999 — legacy seeds):**

| Metric | Held-Out Mean | Held-Out Median | Worst |
|--------|-------------|-----------------|-------|
| ATE RMSE (m) | 3.151 | 2.340 | 7.127 |
| Blind 5 s RMSE (m) | 8.340 | — | 23.49 (V2_02) |
| Gap 2 s RMSE (m) | 8.027 | — | 15.40 (V2_02) |
| RPE 1 s (m) | 0.887 (mean) | — | — |

> [!NOTE]
> V2_02 is pathological — blind 23.5 m, gap 15.4 m vs V1_01's 2.1 m / 2.0 m.
> This disparity is a signal, not noise. Investigate in Phase 0.

---

<!-- Add new experiments below this line, oldest first -->

## EXP-001: [Phase 0] Noise Floor Measurement

- **Date / commit:** TBD (parent: `35c3334`)
- **Hypothesis:** Measure baseline variance across 10 seeds to establish noise floor before any tuning.
- **Change:** None — baseline code only. Run with seeds 1–10.
- **Budget cost:** +0 KB RSS, +0 ms/s
- **Dev results:** → see `reports/NOISE_FLOOR.md`
- **Decision:** N/A (measurement only)
- **Notes:** Required before any experiment can be ACCEPTed.

---
