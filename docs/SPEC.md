# SPEC (Frozen Task Specification)

## 6.1 Input
- `imu0/data.csv`, EuRoC format, comma-separated, header line starts with `#`.
- Columns: `timestamp[ns], wx, wy, wz [rad/s], ax, ay, az [m/s^2]` (acceleration includes gravity).
- Nominal 200 Hz (verify per sequence; compute the median dt from the data).
- Input may be given as an unzipped folder containing `imu0/data.csv`.

## 6.2 Degradations to handle
Progressive noise growth, progressive bias drift, frame dropouts up to 2 s (encoding unknown: missing rows, NaN, zeros, or repeated values), unknown gravity alignment (sensor-to-world roll/pitch unknown), and complete IMU failure after which 5 s must be dead-reckoned.

## 6.3 Output
- File: `estimated_trajectory.txt`. One optional header line starting with `#`.
- Row: `timestamp tx ty tz qx qy qz qw`, space-separated, LF line endings, trailing newline.
- Timestamp: `<ns / 1_000_000_000>.<ns % 1_000_000_000 zero-padded to 9 digits>` from int64 integer math, never via double.
- Positions and quaternion: fixed notation, 9 decimals. Quaternion normalised, Hamilton, body to world.
- World frame: z up, gravity aligned, yaw 0 and position 0 at the first sample.
- Exactly one output row per input data row. Flag `--append-blind` (default off) appends 5.0 s of extrapolated poses after the last row at the median dt. `[ASSUMPTION]`

## 6.4 Budgets
Peak RSS gate 4 MB (hard 8 MB). CPU gate 2 ms per 1 s of IMU (hard 20 ms). One thread. Offline.

## 6.5 Scoring proxies (internal)
Official scoring uses the `evo` suite (ATE RMSE and RPE) against hidden ground truth, with weights: waypoints 30, blind 5 s 20, dropout 10 (closeness) and 40 for memory/runtime. The formula and segment definitions are unknown, so the harness computes all proxies in Section 11.

## 6.6 Assumption registry (each must be behind a flag or config key)

| ID | Assumption | Switch |
|---|---|---|
| A1 | One output row per input row | `--append-blind` |
| A2 | evo uses SE(3) alignment | harness runs aligned and unaligned |
| A3 | Gravity-aligned z-up, yaw 0, origin at start | `params.frame_mode` |
| A4 | Dropouts may appear as missing rows, NaN, zeros or repeats | gap handler handles all |
| A5 | Rate about 200 Hz | median dt computed from data |

## 6.7 Report template (every agent ends with this)

```markdown
## REPORT <WP-id> <agent>
Status: REVIEW | BLOCKED
Files changed: <list>
Acceptance command: <exact command>
Raw output (last 30 lines): <paste>
Decisions/assumptions added: <ids or none>
Open risks: <max 3 bullets>
Hypotheses tried (if any failures): <list>
```

## 6.8 Blocker template

```markdown
## BLOCKER <WP-id> <date>
Symptom: <exact error or metric>
Minimal reproduction: <command + input>
Hypotheses tried: 1) ... 2) ... 3) ...
Suspected cause: <one line>
Needs: <decision / file outside allowlist / human input>
```

---

# Algorithm Cookbook

All equations follow Solà, "Quaternion kinematics for the error-state Kalman filter" (local-angle error convention). Estimator agents implement these as written.

### 12.1 Math helpers
- Quaternion `q = [w, x, y, z]`, Hamilton product `q1 ⊗ q2`, body→world.
- `skew(v) = [[0,-vz,vy],[vz,0,-vx],[-vy,vx,0]]`.
- Exponential map of a rotation vector `φ` (angle `a = |φ|`): `exp(φ) = [cos(a/2), sin(a/2)·φ/a]`; for `a < 1e-8` use `[1, φ/2]` then normalise.
- Rotation matrix `R(q)` standard Hamilton formula. Tests: 90° about z gives `q=[0.70710678,0,0,0.70710678]`; `R(q) v` equals `q ⊗ [0,v] ⊗ q*`; `R^T R = I`.

### 12.2 Initialisation (first `T_init = 1.0 s` of valid samples; fallback `0.5 s`)
- `ā` = mean accelerometer, `ω̄` = mean gyro, `g_mag = |ā|` (clamp to [9.0, 10.5]).
- `b_g = ω̄` (assume static). Accel bias initial 0.
- Tilt: choose `q` so that `R(q) ā / |ā| = [0,0,1]` (body specific force maps to world +z); yaw 0.
- `v = 0, p = 0`. Initial `P = diag(σθ0², σv0², σp0², σbg0², σba0²)` with defaults: `σθ0 = 0.02 rad, σv0 = 0.05, σp0 = 0.01, σbg0 = 0.02, σba0 = 0.2`.
- If motion is detected in the window (gyro std > 0.2 rad/s or accel-norm std > 0.5), use the first sample's tilt, `b_g = 0`, and multiply `σθ0` by 5.
- World gravity vector: `g_w = [0, 0, -g_mag]`.

### 12.3 Nominal state propagation (per valid sample, `dt` in seconds)

```
ω̂ = ω − b_g          â = a − b_a
q ← normalize( q ⊗ exp(ω̂ dt) )              # use q_old for R below
a_w = R(q_old) â + g_w
p ← p + v dt + 0.5 a_w dt²
v ← v + a_w dt
b_g, b_a unchanged
```

### 12.4 Error-state covariance propagation
Error state `δx = [δθ, δv, δp, δb_g, δb_a]` (indices 0–2, 3–5, 6–8, 9–11, 12–14).

```
F = I15
F[0:3, 0:3]   = R(ω̂ dt)^T            # = exp(-skew(ω̂ dt))  (or I - skew(ω̂)dt)
F[0:3, 9:12]  = -I * dt
F[3:6, 0:3]   = -R(q_old) * skew(â) * dt
F[3:6, 12:15] = -R(q_old) * dt
F[6:9, 3:6]   = I * dt

Q = diag blocks:  θ: σ_g²·dt·I   v: σ_a²·dt·I   p: 0
                  b_g: σ_bg²·dt·I   b_a: σ_ba²·dt·I
P ← F P F^T + Q ;  symmetrise ; floor diag at 1e-12
```

`σ_g, σ_a, σ_bg, σ_ba` are continuous noise densities (units rad/s/√Hz etc.) times the **quality inflation** `κ` from the monitor (Section 12.6): `σ ← σ · κ`. Starting priors (EuRoC / TUM-VI published values; verify with Tavily once and log): gyro 1.7e-4 / 1.6e-4, gyro walk 1.9e-5 / 2.2e-5, accel 2.0e-3 / 2.8e-3, accel walk 3.0e-3 / 8.6e-4. Use **ten times** these as starting values for the degraded data (tunable).

### 12.5 Updates (generic Joseph form)

```
S = H P H^T + R ;  K = P H^T S^{-1}   (S is 3x3: invert with closed form + floor)
δx = K y
P  = (I - K H) P (I - K H)^T + K R K^T ; symmetrise
inject: q ← q ⊗ exp(δθ) ; v += δv ; p += δp ; b_g += δb_g ; b_a += δb_a
reset:  P ← G P G^T with G = I except G[0:3,0:3] = I - skew(δθ/2)   (may be skipped; log if skipped)
```

**Tilt update** (applied when `| |a| - g_mag | < τ_a · (1 + 5·(1-quality))` and `|ω| < ω_max`, with `τ_a = 0.3 m/s²`, `ω_max = 1.0 rad/s`):
- `z = (a - b_a)/|a - b_a|`, `ĥ = R(q)^T [0,0,1]`.
- `y = z - ĥ`, `H = [skew(ĥ), 0, 0, 0, 0]` (3×15; only the θ block is nonzero).
- `R_meas = σ_tilt² I3`, `σ_tilt = 0.02 / max(quality, 0.1)`.

**ZUPT** (stationary detector: over the last `0.25 s` window gyro-norm mean < 0.03 rad/s, accel-norm deviation from `g_mag` < 0.15, and accel variance < threshold):
- `y = -v`, `H = [0, I, 0, 0, 0]`, `R_meas = σ_z² I3`, `σ_z = 0.05`.
- Disable if the monitor reports quality < 0.3.

### 12.6 Degradation monitor
- High-frequency residual: `r_k = a_k - a_{k-1}` (accel) and `ρ_k = ω_k - ω_{k-1}` (gyro), each as norm.
- Exponential moving variance with `α = 0.01`; keep the first-second values as the reference `σ_ref`.
- Noise ratio `n = max(σ_now/σ_ref, 1)`.
- Quality `quality = clamp(1 / n, 0.05, 1)`; inflation `κ = clamp(n, 1, κ_max)` with `κ_max = 50`.
- Bias trend: exponential average of the static-window gyro mean when ZUPT-detected stationary (feeds `b_g` observation as an additional pseudo-measurement `y = (ω̄_static - b_g)`, `H = [0,0,0,I,0]`, with R of `σ_bgm²`, default 5e-3). Optional; implement last.

### 12.7 Gap, invalid-sample and failure handling

Definitions: `dt_nom` = median dt of the first 200 valid samples. A sample is **invalid** if any field is non-finite, all six values are exactly zero, or the sample equals the previous one exactly in all six fields for ≥ 3 consecutive rows (stuck), or |ω| > 35 rad/s or |a| > 160 m/s².

| Case | Behaviour |
|---|---|
| `dt ≤ 0` | Output the previous pose (copy), skip integration and updates |
| `dt ≤ 2.5·dt_nom` and valid | Normal propagation |
| `dt > 2.5·dt_nom` (rows missing) | Gap mode: sub-step with `n = ceil(dt/dt_nom)` steps (cap `n` at 800), using smoothed pre-gap `ω_s`, and velocity decay `v ← v·exp(-dt_sub/τ_gap)`; `P` inflated by `Q·κ_gap` each sub-step; no updates; set `recover = recover_steps` |
| Invalid row | Same as gap mode for one `dt_nom` step; still emit one row |
| After a gap | For `recover` steps (default 100) multiply tilt/ZUPT `R_meas` by `(1 + 10·recover/recover_steps)`; accept updates again |
| Failure | Declared when invalid rows persist ≥ `T_fail = 1.0 s` continuously, **or** valid data ends. Freeze: store `(q, v, p, ω_s)` as the last reliable state; from then on use the blind predictor |

Smoothed motion: `ω_s` = exponential average of valid gyro (corrected for bias) with a time constant 0.25 s; stored in a ring buffer of the last 64 states for rollback to the last reliable state when failure is declared (rollback by `T_fail`).

### 12.8 Parser rules
- Skip lines starting with `#`, blank lines; strip `\r` and spaces.
- A row needs a parsable timestamp (int64). If it does not have one, skip the row (it cannot be emitted without a timestamp) and count it.
- Any other field that fails to parse becomes NaN (row flagged invalid).
- A truncated final line is parsed with what exists; missing numbers become NaN.

### 12.9 Blind predictor (after failure at `t_f`)
For each row with `t ≥ t_f`, `τ = t - t_f`:

```
p(τ) = p_f + v_f · τ_v · (1 - exp(-τ/τ_v))          # decaying-velocity model
q(τ) = q_f ⊗ exp( ω_f · τ_ω · (1 - exp(-τ/τ_ω)) )   # decaying turn rate
```

Defaults `τ_v = 3.0 s`, `τ_ω = 1.5 s`; velocity `v_f` is shrunk by a factor `shrink_v = 0.9` (tunable). If `|v_f| < 0.05` the prediction is stationary. Rows beyond `t_f + 5 s` keep the last predicted pose. With `--append-blind`, additional rows are generated at `dt_nom`.

### 12.10 Synthetic oracle (`tools/synth.py`)
Generates ideal IMU from analytic trajectories: (1) static, (2) constant yaw rate, (3) circle at constant speed with banked roll, (4) figure-eight with varying altitude, (5) random smooth (sum of sines). For a position `p(t)` and orientation `q(t)`: `ω = vee(log(q^T q̇))`, `a = R^T (p̈ - g_w)`. Same CSV format and ground truth in TUM format. Unit tests use these with zero noise to confirm strapdown error < 1e-6 m over 10 s.

### 12.11 Differential tolerances (Python vs C++, same input, same params)
Position max abs diff ≤ 1e-6 m on synth, ≤ 1e-4 m on degraded 5-minute runs; quaternion components ≤ 1e-6; timestamps identical strings. If not met, first check parameter export (`params.h` equals `params.json`), then operation order, then the parser.
