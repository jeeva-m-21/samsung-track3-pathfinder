# GEMINI.md — Track 03 "Path Finder" (VITC_SingleThread)

> **This file is the AI assistant instruction sheet for this project.**
> Read it at the start of every session. It governs behaviour, roles, workflows, and
> hard rules for every agent that works in this repo.
> Orchestrator instructions and execution guidelines are also in `AGENT_PLAYBOOK.md`.

---

## 0. Project Quick-Reference

| Item | Value |
|------|-------|
| Binary entry point | `./main` (C++17, single-thread, zero dynamic heap in filter loop) |
| Python entry point | `src/main.py` (wraps `./main`) |
| Build | `cd src && make && cd ..` (or `make -C src release`) → `../main` |
| Params | `configs/params.json` → `src/params.h` via `tools/export_params.py` |
| Harness | `tools/eval_degraded_harness.py` |
| Dev seeds | `{1..10}` (10 seeds for development and tuning) |
| Confirmation seeds | `{201..210}` (Reviewer-only guard against winner's curse) |
| Final seeds | `{101..110}` (Sealed evaluation completed in Phase 5) |
| Freeze deadline | Tagged `release-candidate` (commit `b9eed87`) |
| Final submission | Tagged `submission-final` (commit `3a3339f`) |
| Baseline tag | `baseline-honest-eval` (commit `35c3334`) |
| Current best tag | `best-007` (commit `8b9b91f`) |

---

## 1. Agent Roles

### 1.1 Orchestrator (Main Agent)
- Maintains the experiment queue, ledger, and `reports/CURRENT_BEST.md`.
- Spawns worker subagents with a precise brief. Runs experiments in separate git worktrees/branches (`exp/<id>-<name>`).
- Submits completed worker reports to the Reviewer. Merges only on Reviewer ACCEPT.
- Re-baselines after each merge (`best-<NNN>`), then rebases queued experiments.
- Never edits experiment source code or runs evaluations directly except to merge approved branches.
- Never approves your own or a worker's results.

### 1.2 Worker (Subagent)
- Implements exactly one experiment, runs the full evaluation protocol on dev data (seeds 1–10, LOOCV), and reports raw numbers.
- Must not touch sealed final set, harness, seeds, thresholds, or other branches.
- Must report negative results with complete fidelity.

### 1.3 Reviewer (Independent Subagent)
- Starts from a clean checkout of the branch and parent tag.
- Re-runs the evaluation protocol independently with the same seeds (1–10) AND with a fresh confirmation seed set (`201–210`) to guard against winner's curse.
- Fills the reviewer checklist and marks ACCEPT / REJECT / PARK with the deciding rule.
- If reviewer and worker numbers differ by more than the noise floor, verdict is REJECT until explained.
- Reviewer never writes experiment code.

---

## 2. Hard Budget Constraints

| Constraint | Limit | Target |
|------------|-------|--------|
| Peak RSS | ≤ 8 MB | ≤ 4 MB |
| Compute | ≤ 20 ms per 1 s IMU | ≤ 5 ms/s |
| Threads | 1 | 1 |
| Heap alloc in filter loop | 0 | 0 |
| Third-party libs | none | none |
| Neural networks | none | none |

Measure RSS and ms/s with `taskset -c 0` on the **release** binary.

---

## 3. Output Format (non-negotiable)

```
t tx ty tz qx qy qz qw
```

- File: `estimated_trajectory.txt`
- Space-separated, **exactly one row per input IMU row**
- Timestamps in **seconds**, printed with ≥ 9 decimal places
- Unit quaternions (norm within 1e-6 of 1.0)
- No NaN, no Inf
- Timestamps monotonically non-decreasing

Run `tools/validate_trajectory.py` on every output before accepting any experiment.

---

## 4. Scoring Weights (what to optimise)

| Component | Weight | Target & Priority |
|-----------|--------|-------------------|
| Peak memory | 20% | Target ≤ 4.0 MB (Currently ~1.76 MB ✅) |
| Runtime | 20% | Target ≤ 5.0 ms/s (Currently ~0.66 ms/s ✅) |
| Waypoint ATE | 30% | 2.409 m (SE(3) Umeyama aligned) |
| Blind 5 s | 20% | 5.194 m ← **High Priority** |
| Dropout 2 s gap | 10% | 6.625 m ← **High Priority** |

Efficiency is near ceiling. All upside is in the 60% closeness, with Blind and Gap having the highest leverage.

---

## 5. Data Discipline

### 5.1 Sets

| Set | Sequences | Seeds | Status |
|-----|-----------|-------|--------|
| Dev | MH_01_easy, V1_01_easy, V2_02_medium, room1, room2 | 1–10 | Open (tuning, ablations, LOOCV) |
| Confirmation | Same 5 dev sequences | 201–210 | Reviewer-only (winner's curse guard) |
| Stress | UZH-FPV fwd7, circle, figure_eight | — | Diagnostic only (not scored) |
| **Final (sealed)** | MH_02–05, V1_02, V2_01, V2_03, room3–6 | 101–110 | **DO NOT USE until Phase 5** |

> [!CAUTION]
> Never look at Final Set ground-truth numbers during Phases 0–4. Looking makes it not final.

### 5.2 Noise Floor

The noise floor is recorded in `reports/NOISE_FLOOR.md`. An improvement must beat **2× the noise-floor std** to be accepted.

---

## 6. Evaluation Protocol (Every Experiment)

Per sequence and per seed:
1. Waypoint ATE RMSE (SE(3) Umeyama aligned) and start-anchored ATE.
2. RPE (1s and 10s) translation and rotation.
3. Blind 5s RMSE at official cutoff and >= 30 simulated cutoffs per sequence.
4. 2s gap RMSE inside gap and 5s after.
5. Attitude error in degrees over trajectory and at end of blind window.
6. Peak RSS (KB) and ms/s with `taskset -c 0` on release binary.
7. Format audit: `tools/validate_trajectory.py`.

Paired comparison with parent tag by seed and sequence. Report 95% paired bootstrap CI (10,000 resamples).

---

## 7. Acceptance Rule

ACCEPT only if **all** hold:

1. **Target gain:** Target component improves ≥ 5% relative, 95% bootstrap CI excludes 0, gain ≥ 2× noise-floor std.
2. **No collateral loss:** No other scored component worsens > 3% relative; attitude error does not worsen > 5%.
3. **Per-sequence guard:** No single sequence worsens > 15%; worst-case seed does not worsen > 25%.
4. **Confirmation holds:** Reviewer confirmation run on seeds 201–210 preserves effect sign and ≥ 50% effect size.
5. **Budgets:** RSS ≤ 4 MB, ≤ 5 ms/s, 0 dynamic heap in filter loop, 1 thread.
6. **Integrity:** `make test`, pytest, ASan/UBSan, fuzz, format audit clean on every output file.
7. **Simplicity:** Diff adds ≤ 150 lines per 5% gain.
8. **No per-dataset constants:** Strictly generalizable physics/filters.

---

## 8. Reviewer Checklist (Section 9)

```
- [ ] Clean checkout of parent tag and candidate branch; same seeds.
- [ ] Baseline and candidate numbers reproduced independently; agree with worker within noise.
- [ ] Paired deltas, mean/median/worst, bootstrap 95% CI, noise-floor comparison.
- [ ] Target gain >= 5%, CI excludes zero, >= 2x noise std.
- [ ] No other component worse than 3%; attitude not worse than 5%.
- [ ] No sequence worse than 15%; worst-case seed within 25%.
- [ ] Ablation on/off present; multi-part changes split.
- [ ] Confirmation run on seeds 201-210 holds sign and >= half the effect.
- [ ] Robustness checks pass (timestamp jitter, resample, sensor scale identity test, regime log).
- [ ] Budgets measured on release binary with taskset -c 0.
- [ ] Tests, sanitizers, fuzz, format audit all pass on every output file.
- [ ] No per-dataset constants introduced.
- [ ] Diff size versus gain acceptable (<= 150 lines per 5% gain).
- [ ] Verdict, deciding rule, and caveats written to ledger.
```

---

## 9. Experiment Queue

### Tier 0 — Measurement & Ceilings (do first; no merges)
- **E00 Ceiling matrix:** Inject GT velocity at cutoff, GT velocity throughout, GT speed magnitude, GT yaw, GT attitude, GT bias, GT gravity -> `reports/CEILINGS.md`.
  - Gate: If GT velocity improves blind or waypoint < 15%, skip velocity priors in Tier 3. If GT yaw improves < 15%, skip yaw items.
- **E01 Anchor reconciliation:** Verify baseline tags and cumulative metrics on identical seeds.
- **E02 Gap-handling check:** Inspect post-gap behavior and confirm latch vs resume logic.

### Tier 1 — Blind 5 s & 2 s Gap (30% score weight)
- **E10 Velocity at cutoff:** Compare filter velocity vs short-window position diff vs blend across >= 30 cutoffs.
- **E11 Bidirectional gap bridge:** Forward ESKF + backward ESKF with cubic Hermite / minimum-jerk bridge and slerp attitude.
- **E12 Re-anchored resume:** If E02 allows, resume with decayed pre-gap velocity and inflated covariance.
- **E13 Blind attitude:** Attitude hold vs decayed bias-corrected gyro extrapolation.
- **E14 Blind decay model:** Regime-conditioned decay (`shrink_v`, `tau_v` coarse grid).
- **E15 Gap angular-rate extrapolation (re-test):** Gated by rate-of-turn for steady orbits only.

### Tier 2 — Exploit Degradation Model
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

---

## 10. Stop Rules

Stop optimising and go to Phase 5 when **any** of:
1. Three consecutive experiments REJECT or noise-indistinguishable and no unrun Tier 1–3 items remain with open gates.
2. Freeze date reached (set at project start, ≥ 24 h before deadline).
3. Dev improves while confirmation (seeds 201–210) or LOOCV worsens for two experiments running (overfitting signal).
4. RSS > 4 MB or > 5 ms/s without matching accuracy gain.
5. A change adds > 150 lines per 5% improvement.

---

## 11. Quick Commands

```bash
# Build release binary
cd src && make release && cd ..

# Export params after editing configs/params.json
python tools/export_params.py configs/params.json src/params.h

# Run harness (dev seeds 1-10)
python tools/eval_degraded_harness.py --seeds 1,2,3,4,5,6,7,8,9,10 --loocv

# Reviewer confirmation run (seeds 201-210)
python tools/eval_degraded_harness.py --seeds 201,202,203,204,205,206,207,208,209,210 --loocv

# Format audit on output
python tools/validate_trajectory.py estimated_trajectory.txt <input_imu_rows>

# Measure RSS + ms/s
taskset -c 0 /usr/bin/time -v ./main <args> 2>&1 | grep "Maximum resident"

# Run tests
PYTHONPATH=. pytest tests/ -v
```
