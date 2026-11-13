# DEGRADE_MODEL.md — Generative Model of tools/degrade.py

**Written as part of Phase 0. Used to design Phase 2 experiments.**
**Source file:** `tools/degrade.py`

---

## Overview

The degradation pipeline applies five independent perturbation steps in sequence to a clean EuRoC IMU CSV.
Each step is deterministic given the RNG seed. The seed is the only experiment-varying parameter.

```
clean IMU → [1] gravity misalign → [2] noise ramp → [3] bias random walk
         → [3b] spikes → [4] dropouts → [5] failure at end
         → degraded IMU + sidecar JSON + ground truth (unchanged)
```

---

## Step 1: Gravity / Mounting Misalignment

**Config key:** `gravity_rotation: true`

```python
phi_mis = rng.normal(0.0, 0.1, size=3)   # ~5.7° std per axis
q_mis = quat_exp(phi_mis)
R_mis = quat_to_rot_matrix(q_mis)         # random SO(3) rotation
deg_omega = R_mis.T @ omega               # rotate both channels
deg_acc   = R_mis.T @ acc
```

**Effect:** Constant (time-invariant) rotation applied to both gyro and accel.
The magnitude is `||phi_mis|| ~ 0.1 * sqrt(3) ≈ 0.17 rad ≈ 10°` RMS.
This introduces a constant gravity component in the horizontal accelerometer channels
and a constant offset in the gyro channels.

**Implications for filter design:**
- Static initialisation must estimate this rotation (tilt observer handles it currently).
- Yaw component of misalignment is **unobservable** from IMU alone.
- After static init, the bias estimator will absorb residual gravity projection as `b_a`.

---

## Step 2: Progressive Noise Ramp

**Config key:** `noise_growth_rate: 0.05`

```python
sigma_g_base = 1.7e-4   # rad/s/sqrt(Hz), EuRoC typical
sigma_a_base = 2.0e-3   # m/s^2/sqrt(Hz)
g_ramp = sigma_g_base * (1 + 0.05 * t_s)   # linearly growing with time
a_ramp = sigma_a_base * (1 + 0.05 * t_s)
deg_omega += N(0, g_ramp^2)
deg_acc   += N(0, a_ramp^2)
```

**Law:** Noise std grows **linearly** with elapsed time (not exponential).
At t=100 s, noise is 6× baseline. At t=200 s, noise is 11× baseline.

**Key insight:** At the time of the blind window (last 5 s of data), noise is at its maximum.
The effective process noise Q should grow proportionally.

**Parameters for Phase 2 estimation:**
- 2 parameters per channel: `sigma_base`, `noise_growth_rate`
- Estimable from gyro/accel variance in short windows (2–5 s) across the sequence.

---

## Step 3: Progressive Bias Random Walk

**Config key:** `bias_drift_walk: 0.001`

```python
sigma_bg_walk = 1.9e-5 * sqrt(0.001)   # ≈ 6.0e-7 rad/s/sqrt(s)
sigma_ba_walk = 3.0e-3 * sqrt(0.001)   # ≈ 9.5e-5 m/s^2/sqrt(s)
bg_steps = N(0, sigma_bg_walk) * sqrt(dt)
ba_steps = N(0, sigma_ba_walk) * sqrt(dt)
deg_omega += cumsum(bg_steps)
deg_acc   += cumsum(ba_steps)
```

**Law:** Standard-deviation-√dt random walk (Wiener process / Brownian motion).
Bias drift is **not a ramp** — it is a random walk. A bias-rate state would not help.

**At t=100 s, expected bias magnitude:**
- Gyro: σ_bg * sqrt(100) ≈ 6e-7 * 10 ≈ 6e-6 rad/s (small)
- Accel: σ_ba * sqrt(100) ≈ 9.5e-5 * 10 ≈ 9.5e-4 m/s² (~0.1 mg, significant)

**Implications:** The existing bias random walk model in the ESKF is appropriate.
A bias-rate state (Phase 2, Task 2) is not warranted — skip it.

---

## Step 3b: Impulsive Spikes

**Config key:** `spike_rate: 0.02`

```python
spike_prob = clip(spike_rate * dt, 0, 0.1)   # ~0.02 * 0.005 = 0.0001 per sample
# When spike triggers:
deg_omega[i] += N(0, 5 * sigma_g_base)       # ~5× baseline noise magnitude
deg_acc[i]   += N(0, 5 * sigma_a_base)
```

**Rate:** ~0.02 spikes/s → ~1 spike per 50 s. For a 150 s sequence: ~3 spikes total.
**Magnitude:** 5× baseline noise std — detectable by Hampel filter (> 3σ outlier).

**Implications for Phase 2 front-end:**
- Hampel filter or simple >Nσ rejection will catch these.
- Must not inflate covariance permanently — only for the spiked sample.

---

## Step 4: Dropout Windows

**Config in degrade_default.json:**
```json
[
  {"start_s": 10.0, "duration_s": 1.5, "encoding": "missing"},
  {"start_s": 25.0, "duration_s": 2.0, "encoding": "nan"},
  {"start_s": 40.0, "duration_s": 1.0, "encoding": "zeros"},
  {"start_s": 55.0, "duration_s": 0.8, "encoding": "repeated"}
]
```

**Four fault types:**

| Type | How it appears in CSV | Detection method |
|------|----------------------|-----------------|
| `missing` | Rows simply absent (timestamp jump > 1.5× dt_nom) | dt > 2.5 × dt_nom → gap detected |
| `nan` | Rows present, values = NaN | NaN check on each sample |
| `zeros` | Rows present, values = 0.0 exactly | Near-zero accel magnitude (should be ~9.8) |
| `repeated` | Last pre-fault sample replicated N times | Bitwise identical to previous sample |

**The 2 s NaN window (t=25–27 s) is the primary gap metric.**

**Implications for GapHandler:**
- `missing` rows are already handled by dt-jump detection.
- `nan` rows need explicit NaN check before `is_valid_sample`.
- `zeros` rows: accel magnitude ≈ 0 → flag as invalid (gravity always ≈ 9.8 m/s²).
- `repeated` rows: bitwise equality check for gyro+accel against previous sample.
- All four → `is_valid = false` → use last valid smoothed gyro, inflate covariance.

---

## Step 5: Failure at End (Blind Window)

**Config key:** `failure_at_end_s: 5.0`

```python
failure_time_s = total_dur_s - 5.0
fail_mask = t_s >= failure_time_s
valid_mask[fail_mask] = False   # removes the last 5 s from the output CSV
```

**Effect:** The last 5 s of data is simply **not present** in the output CSV.
The filter sees no samples for the last 5 s. The binary must output one row per *input* row,
so for the original (pre-degradation) timestamps in the last 5 s, it must dead-reckon.

**Checkpoint times:** Every 10 s up to `failure_time_s`. For a 150 s sequence with 5 s failure,
checkpoints are at t=10, 20, 30, …, 140 s.

**Key insight:** The filter must know the original timestamp grid even though no IMU data arrives.
Currently `main.cpp` likely tracks the last seen timestamp and extrapolates — verify this.

---

## Sidecar JSON Structure

```json
{
  "seed": 42,
  "duration_s": 182.4,
  "t0_ns": 1403636579763555584,
  "tend_ns": 1403636761763555584,
  "failure_time_s": 177.4,
  "failure_t_ns": ...,
  "dropout_windows": [...],
  "gap_2s_window": {"start_s": 25.0, "duration_s": 2.0, "encoding": "nan", ...},
  "checkpoints_s": [10.0, 20.0, ..., 170.0]
}
```

The harness reads the sidecar to know where to score blind and gap windows.

---

## V2_02 Anomaly Analysis

V2_02 shows blind=23.5 m vs V1_01's 2.1 m. Hypothesis:

1. **High velocity at failure.** V2_02 is a "medium" difficulty sequence with faster dynamics.
   The filter's velocity estimate at the failure point may be large, causing runaway integration
   over 5 s even with decay.

2. **Noise at failure time.** Since noise grows linearly, by the failure point (last 5 s) the
   noise is at maximum. For longer sequences, this means more degraded state estimates at the
   worst possible time.

3. **Poor velocity estimate.** The filter's internal velocity may be corrupted by integrated
   noise/bias, while the actual platform velocity is different. This needs to be verified by
   comparing filter velocity vs ground-truth position difference at the failure point.

**Action (Phase 1):** Log filter velocity vs position-difference velocity at every failure
point to find which is more accurate as the blind-window initial condition.
