# GEMINI.md — Track 03 "Path Finder" (VITC_SingleThread)

> **This file is the AI assistant instruction sheet for this project.**
> Read it at the start of every session. It governs behaviour, roles, workflows, and
> hard rules for every agent that works in this repo.

---

## 0. Project Quick-Reference

| Item | Value |
|------|-------|
| Binary entry point | `./main` (C++17, single-thread, zero dynamic heap in filter loop) |
| Python entry point | `src/main.py` (wraps `./main`) |
| Build | `cd src && make release` → `../main` |
| Params | `configs/params.json` → `src/params.h` via `tools/export_params.py` |
| Harness | `tools/eval_degraded_harness.py` |
| Dev seeds | `{1..10}` (10 seeds for noise floor runs) |
| Final seeds | `{101..110}` (sealed — do not use until Phase 5) |
| Freeze deadline | Tag `release-candidate` before freeze |
| Baseline tag | `baseline-honest-eval` (commit `35c3334`) |

---

## 1. Agent Roles

### 1.1 Implementer (you — the main agent)
- Owns Phases 0–5 execution.
- Writes all code, runs experiments, maintains ledger.
- Proposes every change via a single-commit branch; never bundles multiple changes.
- Calls for the **Reviewer** after every experiment that produced numbers.

### 1.2 Reviewer (separate agent — invoke explicitly)
When an experiment is complete, invoke a **Code Review subagent** via the swarm or `invoke_subagent`. Give it:
- The diff (or commit hash)
- The raw scorecard table (baseline vs new, per-sequence, per-seed)
- The noise floor std table

The Reviewer must independently reproduce the numbers from a **clean checkout**, then fill in the [Section 9 checklist](##9-reviewer-checklist) in the ledger entry.

**Do NOT auto-accept any experiment without reviewer sign-off.** Mark it `PENDING REVIEW` until the reviewer responds.

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

Measure RSS and ms/s with `taskset -c 0` on the **release** binary (not debug).

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

| Component | Weight | Current baseline |
|-----------|--------|-----------------|
| Peak memory | 20% | 1,876 KB ✅ |
| Runtime | 20% | 1.25 ms/s ✅ |
| Waypoint ATE | 30% | 3.15 m (held-out mean) |
| Blind 5 s | 20% | 8.34 m ← **priority** |
| Dropout 2 s gap | 10% | 8.03 m ← **priority** |

Efficiency is near ceiling. Focus all effort on the 60% accuracy components, especially blind (20%) and gap (10%).

---

## 5. Data Discipline

### 5.1 Sets

| Set | Sequences | Seeds | Status |
|-----|-----------|-------|--------|
| Dev | MH_01_easy, V1_01_easy, V2_02_medium, room1, room2 | 1–10 | Open |
| Stress | UZH-FPV fwd7, circle, figure_eight | — | Diagnostic only |
| **Final (sealed)** | MH_02–05, V1_02, V2_01, V2_03, room3–6 | 101–110 | **DO NOT USE until Phase 5** |

> [!CAUTION]
> Never look at Final Set ground-truth numbers during Phases 0–4. Looking makes it not final.

### 5.2 Noise Floor

The noise floor is recorded in `reports/NOISE_FLOOR.md`. An improvement must beat **2× the noise-floor std** to be accepted.

---

## 6. Workflow Per Experiment

```
1. Hypothesis → physical reason (one sentence)
2. Create branch: git checkout -b exp/EXP-NNN-short-name
3. Single minimal change
4. Build release binary
5. Run harness: 10 seeds, LOOCV, all 5 dev sequences
6. Format audit: validate_trajectory.py
7. RSS + ms/s with taskset -c 0
8. Append entry to reports/EXPERIMENT_LEDGER.md (PENDING REVIEW)
9. Invoke Reviewer subagent with diff + scorecard
10. Reviewer fills checklist, marks ACCEPT / REJECT / PARK
11. If ACCEPT: merge to master, git tag best-NNN, update CURRENT_BEST.md
12. If REJECT: stay on baseline, record reason in ledger
```

---

## 7. Acceptance Rule

ACCEPT only if **all** hold on dev set (LOOCV, 10 seeds, paired by seed):

1. Target component improves ≥ 5% relative **and** ≥ 2× noise-floor std.
2. No other scored component worsens > 3% relative.
3. No single sequence worsens > 15%.
4. RSS ≤ 4 MB, ≤ 5 ms/s.
5. All tests pass; ASan/UBSan clean; format audit clean.
6. **Reviewer independently reproduced the numbers.**

---

## 8. File Locations

```
configs/params.json          ← tuneable parameters (source of truth)
src/params.h                 ← generated from params.json (do not edit directly)
reports/SPLITS.md            ← data split definition (committed before any tuning)
reports/EXPERIMENT_LEDGER.md ← append-only experiment log
reports/CURRENT_BEST.md      ← current best config + scorecard
reports/NOISE_FLOOR.md       ← baseline noise floor run (10 seeds)
reports/DEGRADE_MODEL.md     ← written model of degrade.py generative process
reports/FINAL_SET.md         ← written ONCE in Phase 5 (sealed)
reports/runs/EXP-NNN/        ← raw outputs per experiment (never overwrite)
```

---

## 9. Reviewer Checklist (copy into ledger)

```
- [ ] Hypothesis and physical reason recorded.
- [ ] Single change; diff is minimal.
- [ ] Baseline and new numbers reproduced from a clean checkout, same seeds.
- [ ] Noise floor reported; improvement exceeds 2× std.
- [ ] Raw-meter scorecard: blind, gap, attitude, per-sequence deltas included.
- [ ] No sequence regressed > 15%.
- [ ] RSS and ms/s measured on release build with taskset -c 0.
- [ ] Tests, sanitizers, fuzz, format audit all pass.
- [ ] Ledger entry complete; decision follows the acceptance rule.
- [ ] No hard-coded per-dataset constants introduced.
```

---

## 10. Stop Rules

Stop optimising and go to Phase 5 when **any** of:

1. Three consecutive experiments REJECT or noise-indistinguishable.
2. Freeze date reached (set at project start, ≥ 24 h before deadline).
3. Dev improves while LOOCV held-out worsens for two experiments running.
4. RSS > 4 MB or > 5 ms/s without matching accuracy gain.
5. A change adds > 150 lines or new state group for < 5% improvement.

---

## 11. Phased Plan Summary

| Phase | Focus | Key exit condition |
|-------|-------|--------------------|
| 0 | Foundation audit, harness, noise floor, dry run | Scorecard trustworthy |
| 1 | Blind 5 s + 2 s gap | Beat best trivial baseline by acceptance rule |
| 2 | Degradation modelling | Each experiment ACCEPT/REJECT in ledger |
| 3 | Trajectory accuracy | Accept/reject decisions logged |
| 4 | Parameter search (frozen structure only) | params-final tag |
| 5 | Freeze, final set, submission bundle | done |

---

## 12. What NOT To Do

- Do not tune on the Final Set or look at its numbers early.
- Do not report results without baseline row, noise floor, and per-sequence deltas.
- Do not keep a change because it "looks right" in a plot.
- Do not add: learned model, third-party lib, heap alloc in loop, second thread.
- Do not chase UZH-FPV (stress only).
- Do not use null baseline, span ratio, or J3 as acceptance gate.
- Do not rewrite stable tested modules for style.
- Do not change output format without re-running format audit.
- Do not merge worker code without reviewer reproduction.

---

## 13. Quick Commands

```bash
# Build release binary
cd /home/jeeva/projects/samsung_hackathon/src && make release && cd ..

# Export params after editing configs/params.json
python tools/export_params.py configs/params.json src/params.h

# Run harness (all dev sequences, seeds 1-10)
python tools/eval_degraded_harness.py --seeds 1,2,3,4,5,6,7,8,9,10 --loocv

# Format audit on output
python tools/validate_trajectory.py estimated_trajectory.txt <input_imu_rows>

# Measure RSS + ms/s
taskset -c 0 /usr/bin/time -v ./main <args> 2>&1 | grep "Maximum resident"

# Run all tests
cd tests && python -m pytest -v && cd ..

# Build submission bundle
bash tools/make_bundle.sh
```
