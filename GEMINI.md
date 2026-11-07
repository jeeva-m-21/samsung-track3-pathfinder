# Antigravity Global Instructions (Track 3 "Path Finder")

These rules are unconditionally active for all agent sessions in this repository.

---

## 1. Session Start Protocol (Mandatory, in this order)
1. Read `docs/STATE.md`.
2. Read ONLY your designated packet file `docs/packets/<WP>.md` and the files listed under "Read".
3. Do not read other files unless explicitly instructed by the packet. Do not read `PLAN.md` in full.

---

## 2. Scope & File Governance
- Edit ONLY files listed in the packet's `Allowlist`. Create nothing else.
- If a change is needed outside the allowlist, STOP and log it in `docs/blockers.md`.
- Never delete or rewrite working code or files that already pass tests unless the packet instructs it.
- Work packet isolation: one packet per conversation.

---

## 3. Task & System Constraints
- **Sensors**: IMU data only (`imu0/data.csv`). No camera, GPS, or magnetometer. No learned/neural models.
- **Estimator Target**: C++17 single-threaded streaming binary, offline, no dynamic allocation after startup, no `<iostream>` (use `cstdio`/`stdio`), double precision.
- **Budgets**: Peak RSS ≤ 4 MB (hard limit 8 MB); CPU ≤ 2 ms per 1 s of IMU data (hard limit 20 ms).
- **Output Format**: Frozen 8 space-separated columns:
  `timestamp tx ty tz qx qy qz qw`
  (One row per input row, unit quaternion, timestamp formatted directly from int64 nanoseconds without converting to double, no NaN/Inf).

---

## 4. Numerics & Algorithmic Rules
- **Quaternion**: Hamilton convention, `[w, x, y, z]` internally, body-to-world, output as `qx qy qz qw`.
- **Continuity**: Renormalise `q` every step; maintain sign continuity.
- **Covariance**: Symmetrise `P` after every predict and update; floor diagonal at `1e-12`, cap at `1e6`. Use Joseph form for updates.
- **Time/Gaps**: Master clock is int64 ns (`dt = (t_k - t_{k-1}) * 1e-9`). When `dt <= 0` or across large gaps, never integrate raw IMU across it (use gap handler). Never divide by norm without floor (`1e-12`).
- **Math Source**: Strictly implement equations from `docs/SPEC.md` Algorithm Cookbook. Never invent or re-derive formulas.

---

## 5. Honesty, Verification & Loop Breaker
- **Evidence First**: Never claim "done", "passes", or "works" without pasting the raw command output.
- **No API/Flag Hallucinations**: Verify CLI flags with `<tool> --help`. Query facts via Tavily and record sources in `docs/decisions.md`.
- **Loop Breaker**: At most 3 fix attempts for any failing gate/test. Before each attempt, state:
  `Hypothesis N: <cause> -> change: <what>`
  After the 3rd failure, log a blocker in `docs/blockers.md`, set status to `BLOCKED` in `docs/STATE.md`, and stop.
- **Reviewer Only Sets DONE**: Worker agents may only set status to `REVIEW`. Only the Reviewer agent verifies and transitions to `DONE`.

---

## 6. Code Style & Parameter Governance
- **Python**: Type hints, no global state, NumPy only (no Pandas in `src/ref/`).
- **C++**: C++17, no exceptions, no RTTI, no dynamic heap allocations in main loop, no `<iostream>`.
- **Single Source of Truth**: Tunable constants reside in `configs/params.json` (Python) and generated `src/cpp/params.h` (C++). No hardcoded magic numbers.

---

## 7. Reporting Template
Every session concludes with the standard report:
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
