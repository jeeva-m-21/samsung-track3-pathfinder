# NOISE_FLOOR.md — Baseline Noise Floor Measurement

**Binary:** `./main` at commit `35c3334` (`baseline-honest-eval`)
**Run date:** 2026-11-13
**Seeds:** 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 (10 seeds, dev seeds per SPLITS.md)
**Sequences:** MH_01_easy, V1_01_easy, V2_02_medium, room1, room2 (5 dev sequences)
**Total runs:** 50

> This is the reference for the acceptance rule: an improvement must exceed 2× std.

---

## Per-Sequence Noise Floor (std over 10 seeds)

| Sequence | ATE std (m) | 2× ATE | Blind std (m) | 2× Blind | Gap std (m) | 2× Gap |
|----------|------------|--------|--------------|----------|------------|--------|
| MH_01_easy | 0.0027 | 0.005 | 0.058 | 0.115 | 0.090 | 0.180 |
| V1_01_easy | 0.0010 | 0.002 | 0.023 | 0.047 | 0.207 | 0.413 |
| V2_02_medium | 0.0071 | 0.014 | 0.106 | 0.212 | 0.199 | 0.399 |
| room1 | 0.138 | 0.275 | 1.056 | 2.112 | 0.648 | 1.297 |
| room2 | **0.973** | **1.947** | **3.752** | **7.504** | **3.589** | **7.178** |

> [!WARNING]
> **room2 seed=8 is an outlier:** ATE=6.13 m vs mean=2.80 m, Blind=22.6 m vs mean=10.4 m.
> This is a known failure mode (likely V at failure causes runaway blind propagation).
> Its std heavily skews the conservative thresholds. Use per-sequence thresholds.

---

## Conservative Thresholds (worst-sequence rule, used for acceptance)

For the acceptance rule, we apply **per-sequence thresholds**. An improvement must pass the
threshold on EACH sequence, not just on the mean.

| Metric | Threshold |
|--------|-----------|
| ATE RMSE | 0.275 m (room1 limited; MH_01/V-sequences are much tighter) |
| Blind 5s RMSE | 2.112 m (room1 limited; room2 is dominated by seed=8 outlier) |
| Gap 2s RMSE | 1.297 m (room1 limited) |

> For room2 specifically, the 2× std threshold is 1.95 m (ATE), 7.5 m (blind), 7.2 m (gap).
> This means improvements on room2 are hard to distinguish from noise due to the seed=8 outlier.
> **Recommendation:** when reporting room2 results, always flag the seed=8 run separately.

---

## Per-Sequence, Per-Seed Raw Numbers (10-seed baseline)

### MH_01_easy

| Seed | ATE (m) | Blind 5s (m) | Gap 2s (m) |
|------|---------|-------------|-----------|
| 1 | 3.4153 | 4.6138 | 12.8481 |
| 2 | 3.4174 | 4.5254 | 12.6952 |
| 3 | 3.4171 | 4.5626 | 12.8041 |
| 4 | 3.4133 | 4.6402 | 12.8245 |
| 5 | 3.4160 | 4.5745 | 12.6684 |
| 6 | 3.4178 | 4.5034 | 12.6310 |
| 7 | 3.4188 | 4.5333 | 12.7753 |
| 8 | 3.4203 | 4.4653 | 12.6321 |
| 9 | 3.4235 | 4.4460 | 12.5799 |
| 10 | 3.4171 | 4.5505 | 12.7898 |
| **mean** | **3.4177** | **4.5415** | **12.7248** |
| **std** | **0.0027** | **0.0577** | **0.0898** |

### V1_01_easy

| Seed | ATE (m) | Blind 5s (m) | Gap 2s (m) |
|------|---------|-------------|-----------|
| 1 | 1.7143 | 1.9321 | 1.9856 |
| 2 | 1.7170 | 1.9011 | 2.1499 |
| 3 | 1.7134 | 1.9410 | 1.8169 |
| 4 | 1.7143 | 1.9598 | 1.9356 |
| 5 | 1.7138 | 1.9844 | 1.3918 |
| 6 | 1.7146 | 1.9565 | 1.7406 |
| 7 | 1.7136 | 1.9584 | 1.8541 |
| 8 | 1.7148 | 1.9288 | 2.0698 |
| 9 | 1.7153 | 1.9696 | 2.0019 |
| 10 | 1.7146 | 1.9242 | 2.0538 |
| **mean** | **1.7146** | **1.9456** | **1.9000** |
| **std** | **0.0010** | **0.0234** | **0.2065** |

### V2_02_medium

| Seed | ATE (m) | Blind 5s (m) | Gap 2s (m) |
|------|---------|-------------|-----------|
| 1 | 2.2570 | 5.2476 | 4.8177 |
| 2 | 2.2635 | 5.5047 | 4.4028 |
| 3 | 2.2672 | 5.2822 | 4.6680 |
| 4 | 2.2645 | 5.4219 | 4.3943 |
| 5 | 2.2644 | 5.5756 | 4.2196 |
| 6 | 2.2583 | 5.4078 | 4.2895 |
| 7 | 2.2666 | 5.3730 | 4.6671 |
| 8 | 2.2682 | 5.3816 | 4.4910 |
| 9 | 2.2652 | 5.5143 | 4.4559 |
| 10 | 2.2847 | 5.5615 | 4.8162 |
| **mean** | **2.2660** | **5.4270** | **4.5222** |
| **std** | **0.0071** | **0.1060** | **0.1994** |

> [!NOTE]
> V2_02 ATE dropped from 7.13 m (legacy seeds) to 2.27 m (new seeds 1-10). This is a **major
> discrepancy** vs the loocv_baseline_scorecard.md baseline (which showed 7.13 m).
> Root cause: the binary at commit `35c3334` (`baseline-honest-eval`) is **different** from the
> commit in the legacy scorecard — the zupt calibration in `0d5e736` reduced V2_02 ATE from 7.1 m
> to 2.5 m. The legacy scorecard was run on an older build. The 10-seed numbers are correct.

### room1

| Seed | ATE (m) | Blind 5s (m) | Gap 2s (m) |
|------|---------|-------------|-----------|
| 1 | 1.2650 | 2.0691 | 2.4367 |
| 2 | 1.6627 | 5.2191 | 4.3838 |
| 3 | 1.2025 | 1.6761 | 2.4446 |
| 4 | 1.4685 | 3.8602 | 3.4733 |
| 5 | 1.3465 | 3.3746 | 2.5719 |
| 6 | 1.4069 | 3.3294 | 3.2514 |
| 7 | 1.2869 | 2.4900 | 2.5889 |
| 8 | 1.5784 | 4.6617 | 3.9983 |
| 9 | 1.4784 | 3.9342 | 3.5349 |
| 10 | 1.3331 | 2.9576 | 2.9373 |
| **mean** | **1.4029** | **3.3572** | **3.1621** |
| **std** | **0.138** | **1.056** | **0.648** |

### room2

| Seed | ATE (m) | Blind 5s (m) | Gap 2s (m) |
|------|---------|-------------|-----------|
| 1 | 3.2891 | 12.7271 | 11.2525 |
| 2 | 2.8082 | 9.9033 | 9.1184 |
| 3 | 2.9270 | 10.3278 | 9.5219 |
| 4 | 2.7972 | 9.8404 | 9.1248 |
| 5 | 3.0932 | 11.1951 | 10.5406 |
| 6 | 2.8599 | 10.1883 | 9.5439 |
| 7 | 2.9340 | 10.3813 | 9.5493 |
| 8 | **6.1280** | **22.6361** | **21.3855** |
| 9 | 2.7982 | 9.8447 | 9.1223 |
| 10 | 2.7994 | 9.7394 | 8.9982 |
| **mean** | **3.2434** | **11.6784** | **10.8157** |
| **std** | **0.973** | **3.752** | **3.589** |
| mean (excl s=8) | 2.914 | 10.455 | 9.645 |
| std (excl s=8) | 0.139 | 0.919 | 0.797 |

> Seed=8 triggers an outlier failure on room2. When this seed is excluded, the noise floor
> for room2 drops to std~0.14 m (ATE), ~0.92 m (blind), ~0.80 m (gap) — consistent with room1.
> Phase 1 investigation: determine whether seed=8/room2 is a systematic bug or a stochastic outlier.

---

## Summary: Effective Thresholds for EXP Acceptance

Use the **per-sequence 2× std** (excluding the seed=8 room2 outlier):

| Sequence | ATE threshold | Blind threshold | Gap threshold |
|----------|--------------|----------------|---------------|
| MH_01_easy | 0.005 m | 0.115 m | 0.180 m |
| V1_01_easy | 0.002 m | 0.047 m | 0.413 m |
| V2_02_medium | 0.014 m | 0.212 m | 0.399 m |
| room1 | 0.275 m | 2.112 m | 1.297 m |
| room2 (excl s=8) | 0.277 m | 1.838 m | 1.593 m |

> [!TIP]
> An experiment that improves the **mean** across sequences by < 0.3 m (ATE), < 2 m (blind), or
> < 1.6 m (gap) is in the noise floor. These are surprisingly tight — the filter is already
> deterministic given the same seed; variance comes only from the random degradation.
