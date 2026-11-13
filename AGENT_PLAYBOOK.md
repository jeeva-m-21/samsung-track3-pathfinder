# AGENT PLAYBOOK — Track 03 "Path Finder" (VITC_SingleThread)

Audience: Orchestrator agent, Worker subagents, and Reviewer subagent.
Purpose: Push accuracy as far as evidence supports, under strict budget and protocol constraints, then stop.

---

## 0. Ground Rules

1. **Evidence over belief.** A change exists only if a measured, held-out improvement says so. "Should help" is not a reason to keep code.
2. **One change at a time.** Never bundle. Each experiment gets its own branch/commit, worktree, and ablation row.
3. **Never touch the final set.** The Final Set is run once, at the end (Phase 5), and only to confirm.
4. **Budgets are hard limits.** Peak RSS <= 8 MB (target <= 4 MB), <= 20 ms compute per 1 s of IMU (target <= 5 ms/s), single thread, offline, zero heap allocation in the filter loop, no third-party libs, no neural networks.
5. **Output format is non-negotiable.** `estimated_trajectory.txt`, space separated, 8 columns (`t tx ty tz qx qy qz qw`), exactly **one row per input IMU row**, timestamps in seconds printed with >= 9 decimals, unit quaternions, no NaN/Inf.
6. **Do not optimize the wrong thing.** Report raw meters on waypoint ATE, blind 5s, and gap 2s.

---

## 1. What is Scored

| Component | Weight | Target Metric |
| :--- | :---: | :--- |
| Peak memory | 20% | Peak RSS <= 4 MB |
| Runtime | 20% | Compute <= 5 ms/s |
| Waypoint trajectory closeness | 30% | ATE / RPE (evo) across mission |
| Blind 5 s after IMU death | 20% | Dead-reckoned trajectory after last checkpoint |
| Dropout resilience (2 s gap) | 10% | Error through and after signal loss |

Efficiency (40%) is already near ceiling. All upside is in the 60% closeness, with Blind and Gap having the highest leverage.

---

## 2. Roles and Delegation

### Orchestrator
- Maintains the experiment queue, ledger, and `reports/CURRENT_BEST.md`.
- Spawns workers with precise briefs. Runs experiments in separate git worktrees/branches (`exp/<id>-<name>`).
- Forwards completed worker reports to the Reviewer. Merges only on Reviewer ACCEPT.
- Re-baselines after each merge (`best-<NNN>`), then rebases queued experiments.
- Never edits experiment source code or computes evaluation metrics directly except to merge approved branches.

### Worker
- Implements exactly one experiment, runs the full evaluation protocol (dev seeds 1-10, LOOCV), and reports raw numbers.
- Never touches sealed final set, harness, seeds, thresholds, or other branches.
- Reports negative results with complete fidelity.

### Reviewer
- Starts from a clean checkout of branch and parent tag.
- Re-runs evaluation protocol independently with dev seeds 1-10 AND confirmation seeds 201-210.
- Fills reviewer checklist and issues ACCEPT, REJECT, or PARK.

---

## 3. Data Discipline

| Set | Sequences | Seeds | Use |
| :--- | :--- | :--- | :--- |
| Dev | EuRoC MH_01, V1_01, V2_02; TUM-VI room1, room2 | 1–10 | Design, ablation, LOOCV |
| Confirmation | Same dev sequences | 201–210 | Reviewer-only winner's curse guard |
| Stress | UZH-FPV fwd7, synthetic circle/figure_eight | — | Robustness diagnostics only |
| Sealed Final | EuRoC MH_02..05, V1_02, V2_01, V2_03; TUM-VI room3..6 | 101–110 | Evaluated ONCE in Phase 5 |

---

## 4. Evaluation Protocol

Per sequence and per seed:
1. Waypoint ATE RMSE (SE(3) Umeyama aligned) and start-anchored ATE.
2. RPE (1s and 10s) translation and rotation.
3. Blind 5s RMSE at official cutoff and >= 30 simulated cutoffs per sequence.
4. 2s gap RMSE inside gap and 5s after.
5. Attitude error in degrees over trajectory and at end of blind window.
6. Peak RSS (KB) and ms/s with `taskset -c 0` on release binary.
7. Format audit: `tools/validate_trajectory.py`.

Paired comparison with parent tag by seed and sequence. 95% paired bootstrap CI (10,000 resamples). Comparison to noise floor (`reports/NOISE_FLOOR.md`).

---

## 5. Acceptance Rule

ACCEPT only if ALL hold:
1. **Target gain:** Target component improves >= 5% relative, 95% bootstrap CI excludes 0, gain >= 2x noise-floor std.
2. **No collateral loss:** No other scored component degrades > 3% relative; attitude error degrades <= 5%.
3. **Per-sequence guard:** No single sequence degrades > 15%; worst-case seed within 25%.
4. **Confirmation holds:** Seeds 201-210 preserve effect sign and >= 50% effect size.
5. **Budgets:** RSS <= 4 MB, <= 5 ms/s, 0 heap in loop, single thread.
6. **Integrity:** `make test`, pytest, ASan/UBSan, fuzz, format audit clean.
7. **Simplicity:** Diff <= 150 lines per 5% gain.
8. **No per-dataset constants:** Strictly generalizable physics/filters.

---

## 6. Experiment Queue

### Tier 0 — Measurement & Baselines (no merges)
- **E00 Ceiling matrix:** Inject GT velocity at cutoff, GT velocity throughout, GT speed magnitude, GT yaw, GT attitude, GT bias, GT gravity -> `reports/CEILINGS.md`.
  - Gate: If GT velocity improves blind or waypoint < 15%, skip velocity priors in Tier 3. If GT yaw improves < 15%, skip yaw items.
- **E01 Anchor reconciliation:** Verify baseline tags and cumulative metrics on identical seeds.
- **E02 Gap-handling check:** Inspect post-gap behavior and confirm latch vs resume logic.

### Tier 1 — Blind 5s & 2s Gap (30% score weight)
- **E10 Velocity at cutoff:** Compare filter velocity vs short-window position diff vs blend across >= 30 cutoffs.
- **E11 Bidirectional gap bridge:** Forward ESKF + backward ESKF with cubic Hermite / minimum-jerk trajectory bridge and slerp attitude.
- **E12 Re-anchored resume:** If E02 allows, resume with decayed pre-gap velocity and inflated covariance.
- **E13 Blind attitude:** Attitude hold vs decayed bias-corrected gyro extrapolation.
- **E14 Blind decay model:** Regime-conditioned decay (`shrink_v`, `tau_v` coarse grid).
- **E15 Gap angular-rate extrapolation (re-test):** Gated by rate-of-turn for steady orbits only.

### Tier 2 — Exploiting Degradation Model
- **E20 Sensor-noise schedule:** Dynamic measurement noise scaling without corrupting bias random walk.
- **E21 Bias model:** Higher-order bias drift if degradation drift is ramped.
- **E22 Spike front-end:** Gyro/accel 3-sample median / Hampel filter.
- **E23 Frozen/clipped sample detection and uniform resampling.**
- **E24 Late-data discounting.**

### Tier 3 — Velocity Information & Structure (Gated by E00)
- **E30 tau_ou sweep:** {1.8, 3, 5, 10, off} with tilt gating enabled.
- **E31 Prior ablation:** velocity envelope, tau_ou, rotor drag, pedestrian prior (one-by-one).
- **E32 Regime selector hardening:** Neutral fallback and decision logging.
- **E33 Gyro saturation handling:** Flag clipped samples and widen noise.
- **E34 Runtime self-calibration:** Estimate |g|, noise std from static window.
- **E35 Speed-magnitude regressor (yaw-invariant):** Ridge regressor on ~1s window features (blocked until approved).
- **E36 Blind-window ridge predictor:** Residual displacement predictor (blocked until approved).
- **E37 Regime classifier:** Logistic/tree classifier if hand thresholds fail.
- **E38 Online drag coefficient estimation:** Per-axis online/file drag estimation (gate: beats fixed coefficients).
- **E39 Angular-rate drag terms:** Rotor drag angular velocity coupling (flight regime only).
- **E39b Handheld Step-Phase Simulated ZUPT:** Step-phase ZUPT for pedestrian regime (gate: beats current pedestrian prior on TUM-VI).

### Tier 4 — Estimator & Smoothing
- **E40 Ensemble:** 4-8 filter instances fused with trimmed mean.
- **E41 RTS / Fixed-lag smoother:** On velocity and attitude only.
- **E42 Alternative filter:** InEKF / iterated EKF if NEES shows inconsistency.

### Tier 5 — Parameter Tuning
- **E50 Parameter search:** Frozen structure only; <= 12 parameters; CMA-ES / random search on held-out folds.
