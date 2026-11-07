# ANTIGRAVITY MASTER PLAN: Track 3 "Path Finder" (build-only)

**Goal:** a C++17 streaming error-state Kalman filter (ESKF) that turns a degraded EuRoC-format IMU CSV into a TUM-format 6-DoF trajectory, with a Python reference and a full evaluation harness.
**Scope:** build, verify, tune. Nothing about registration, submission, packaging for hand-in, video or decks.
**Model:** Gemini Flash only. This whole plan is built around Flash's weak spots (see section 1).

How to use this file:
1. Put this file at the repo root as `PLAN.md`.
2. Run **Section 4 (Bootstrap)** by hand once.
3. Create the rule, workflow and skill files from **Sections 5, 9, 10** (the Conductor can do this in WP-00 by copying the code blocks).
4. From then on, drive everything with `/next` (Section 9).

---

## 1. Flash-hardening principles (why the architecture looks like this)

| Flash weakness | Countermeasure built into this plan |
|---|---|
| Loses the thread on long tasks | Work is cut into **Work Packets (WP)**, each touching ≤ 3 files and ≤ ~200 lines. One packet = one fresh agent session |
| Hallucinates APIs, CLI flags, equations | Math is **pre-written in Section 12**. CLI flags must be verified with `--help` or Tavily before use. No invented library calls |
| Claims "done" without proof | **Machine gates**: a script exits 0 or non-zero. Agents may not self-certify; a separate **Reviewer** agent re-runs the gate and flips the state |
| Forgets earlier decisions | **File-based memory**: `docs/STATE.md`, `docs/decisions.md`, `docs/gate_log.md`. Every session starts by reading `STATE.md` and its packet only |
| Loops on the same failed fix | **Loop breaker**: 3 attempts, each with a different logged hypothesis, then `BLOCKED` |
| Scope creep and rewriting working code | Each packet has a **file allowlist**. Touching other files is forbidden, enforced by `scripts/check_scope.sh` |
| Weak at deep numerical reasoning | **Oracles first**: analytic synthetic-IMU generator and unit tests with known answers exist before the filter does. The Python reference is the oracle for C++ |
| Verbose, drifting prose | Strict **output templates** for every agent report (Section 8) |
| Premature optimisation | Correctness in Python first; C++ port second; performance only after differential tests pass |

---

## 2. Model and reasoning settings (Antigravity)

Only Flash is used. Antigravity exposes a conversation mode and a model picker; exact labels can change between builds, so check your UI.

| Situation | Setting |
|---|---|
| Any packet involving math, filter logic, C++ memory design, debugging a failing gate | **Planning mode**, Flash. If your build shows a thinking/reasoning toggle, turn it **on / high** |
| Mechanical packets (scaffolding, scripts, formatting, README, running commands) | **Fast mode**, Flash |
| Reviewer agent (always) | **Planning mode**, thinking on. Verification is where Flash errs most |
| Conductor (`/next`) | Fast mode is enough; it only reads state and writes a dispatch note |

Other settings:
- **One packet per conversation.** Start a new agent conversation for each packet. Do not chain packets in one session.
- **Terminal policy:** "Request review" for anything outside the repo; "Always proceed" is acceptable for commands inside the workspace (build, test, eval). Keep `rm -rf` and anything with `sudo` on manual review.
- **Parallelism:** at most 2 agents at once, and only on packets with disjoint file allowlists (Section 7 marks them).

---

## 3. Repository layout (scaffolded by WP-00)

```
repo/
├── PLAN.md                         # this file (read-only for agents)
├── .agent/
│   ├── rules/                      # always-on rules (Section 5)
│   ├── workflows/                  # slash commands (Section 9)
│   └── skills/                     # reusable procedures (Section 10)
├── docs/
│   ├── STATE.md                    # single source of truth for progress
│   ├── decisions.md                # append-only decisions and assumptions
│   ├── gate_log.md                 # append-only gate results with raw output
│   ├── blockers.md                 # append-only blocked packets
│   ├── SPEC.md                     # frozen task spec (Section 6)
│   └── packets/WP-xx.md            # generated from Section 7 by the Conductor
├── src/
│   ├── main.py                     # thin wrapper that runs the C++ binary
│   ├── ref/                        # Python reference estimator (package `ref`)
│   └── cpp/                        # C++17 estimator + Makefile
├── tools/                          # degrade, synth, validate, metrics, run_eval, tune
├── tests/                          # pytest + C++ unit tests + fuzz
├── scripts/                        # gate scripts (machine verification)
├── configs/
│   ├── params.json                 # all tunable constants (single source)
│   └── degrade_default.json
├── results/                        # tables, tuning outputs (no raw datasets)
└── data/                           # EuRoC / TUM-VI, git-ignored, human-staged
```

`.gitignore` must contain `data/`, `build/`, `*.o`, `__pycache__/`, `results/tmp/`.

---

## 4. Bootstrap (do by hand, once)

### 4.1 System tools

```bash
sudo apt update && sudo apt install -y build-essential g++ make python3 python3-venv \
     valgrind time util-linux git unzip curl
g++ --version        # need 11+
/usr/bin/time -V
taskset --version
```

### 4.2 Python environment

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install numpy scipy pandas matplotlib pytest optuna evo
pip freeze > requirements.txt
evo_ape --help | head -20      # confirm evo works
```

### 4.3 Data (human-staged, outside git)

Download EuRoC MAV and TUM-VI manually into `data/euroc/<seq>/mav0/imu0/data.csv` and `data/tumvi/<seq>/...`. Agents must **not** try to download these (hosts may be blocked and downloads waste Flash context). Ground truth files are used only by the harness.

Minimum set: 3 EuRoC sequences of different difficulty (for example MH_01, V1_01, V2_02) and 2 TUM-VI sequences (room sequences have ground truth). Record the exact list in `configs/datasets.json`.

If data is unavailable, packets WP-10 to WP-15 still work using `tools/synth.py` (analytic trajectories), which is also the unit-test oracle.

### 4.4 Tavily MCP

In Antigravity: Agent pane menu, **MCP Servers**, **Manage MCP Servers**, **View raw config**, and add:

```json
{
  "mcpServers": {
    "tavily": {
      "command": "npx",
      "args": ["-y", "tavily-mcp@latest"],
      "env": { "TAVILY_API_KEY": "YOUR_KEY" }
    }
  }
}
```

Needs Node.js installed. Only these MCP servers are used: **Tavily**. Nothing else. No Sequential Thinking MCP (Flash planning mode plus the file-based packets replace it).

### 4.5 Memory footprint probe (do this before any C++)

```bash
cat > /tmp/empty.cpp <<'EOF'
#include <cstdio>
int main(){ std::puts("x"); return 0; }
EOF
g++ -O2 -std=c++17 /tmp/empty.cpp -o /tmp/empty_dyn
g++ -O2 -std=c++17 -static /tmp/empty.cpp -o /tmp/empty_static
/usr/bin/time -v /tmp/empty_dyn    2>&1 | grep "Maximum resident"
/usr/bin/time -v /tmp/empty_static 2>&1 | grep "Maximum resident"
```

Write both numbers into `docs/decisions.md`. They set the real memory headroom. The hard limit is 8 MB peak RSS; the internal gate is 4 MB. `<iostream>` is forbidden in the estimator (use `stdio`), and static linking is preferred if it lowers RSS.

---

## 5. Always-on rules (`.agent/rules/00-global.md`)

Copy exactly. Keep it short: Flash follows short rules better.

```markdown
# GLOBAL RULES (every agent, every session)

## Session start (mandatory, in this order)
1. Read docs/STATE.md.
2. Read ONLY your packet file docs/packets/<WP>.md and the files it lists under "Read".
3. Do not read other files unless the packet says so. Do not read PLAN.md in full.

## Scope
- Edit ONLY files in the packet's "Allowlist". Create nothing else.
- If you need a change outside the allowlist, STOP and write it to docs/blockers.md.
- Never delete or rewrite a file that already passes its tests unless the packet says so.

## Task constraints
- IMU data only. No camera, GPS, magnetometer. No learned models.
- Final estimator: C++17, single thread, offline, no heap allocation in the main loop,
  no <iostream>, peak RSS gate 4 MB (hard 8 MB), CPU gate 2 ms per 1 s of IMU (hard 20 ms).
- Double precision for state and covariance.
- Output file format (frozen, see docs/SPEC.md): 8 space-separated columns
  `timestamp tx ty tz qx qy qz qw`, one row per input row, unit quaternion,
  timestamp written from the int64 nanosecond value, never nan/inf.

## Honesty and verification
- Never write "done", "passes" or "works" without pasting the raw command output
  that proves it into your report.
- Never invent a CLI flag, function, library or equation. Verify a CLI flag with
  `<tool> --help`. Verify a fact with Tavily and record the source URL in docs/decisions.md.
- Math comes from docs/SPEC.md section "Algorithm cookbook". Do not re-derive it.
  If you think the cookbook is wrong, write the evidence to docs/blockers.md and stop.

## Loop breaker
- A failing test/gate gets at most 3 fix attempts. Before each attempt write one line in
  your report: "Hypothesis N: <cause> -> change: <what>". Attempts must differ.
- After the 3rd failure: write docs/blockers.md entry (template in SPEC.md), set the
  packet to BLOCKED in STATE.md, and stop.

## Testing
- Write or update the test BEFORE the implementation when the packet lists a test.
- Run the packet's "Acceptance" command exactly as written.
- No tuning on evaluation sequences. Tuning only through tools/tune.py with its split.

## Style
- Python: type hints, no global state, numpy only (no pandas in ref/).
- C++: C++17, no exceptions, no RTTI, no dynamic allocation after startup, no iostream.
- Constants live in configs/params.json (Python) and src/cpp/params.h (generated by
  tools/export_params.py). No magic numbers elsewhere.

## Reporting
- End every session with the report template from docs/SPEC.md (section "Report template").
- Append non-obvious decisions and every [ASSUMPTION] to docs/decisions.md.
- You may set your packet to REVIEW in STATE.md. Only the Reviewer may set DONE.
```

Second rule file, `.agent/rules/10-numerics.md`:

```markdown
# NUMERICS RULES
- Quaternion: Hamilton, [w,x,y,z] internally, body->world. Written as qx qy qz qw.
- Renormalise q every step. Keep sign continuity of the output quaternion.
- Covariance: symmetrise P after every predict and update; floor the diagonal at 1e-12;
  cap at 1e6. Use Joseph form for updates.
- dt <= 0 or dt > gap_threshold: never integrate raw IMU values across it (see gap handler).
- Never divide by a norm without a floor (1e-12).
- Timestamps: keep int64 ns as the master clock. dt = (t_k - t_{k-1}) * 1e-9.
```

---

## 6. Frozen spec (`docs/SPEC.md`)

The Conductor writes this in WP-00 from the content below (verbatim).

### 6.1 Input
- `imu0/data.csv`, EuRoC format, comma-separated, header line starts with `#`.
- Columns: `timestamp[ns], wx, wy, wz [rad/s], ax, ay, az [m/s^2]` (acceleration includes gravity).
- Nominal 200 Hz (verify per sequence; compute the median dt from the data).
- Input may be given as an unzipped folder containing `imu0/data.csv`.

### 6.2 Degradations to handle
Progressive noise growth, progressive bias drift, frame dropouts up to 2 s (encoding unknown: missing rows, NaN, zeros, or repeated values), unknown gravity alignment (sensor-to-world roll/pitch unknown), and complete IMU failure after which 5 s must be dead-reckoned.

### 6.3 Output
- File: `estimated_trajectory.txt`. One optional header line starting with `#`.
- Row: `timestamp tx ty tz qx qy qz qw`, space-separated, LF line endings, trailing newline.
- Timestamp: `<ns / 1_000_000_000>.<ns % 1_000_000_000 zero-padded to 9 digits>` from int64 integer math, never via double.
- Positions and quaternion: fixed notation, 9 decimals. Quaternion normalised, Hamilton, body to world.
- World frame: z up, gravity aligned, yaw 0 and position 0 at the first sample.
- Exactly one output row per input data row. Flag `--append-blind` (default off) appends 5.0 s of extrapolated poses after the last row at the median dt. `[ASSUMPTION]`

### 6.4 Budgets
Peak RSS gate 4 MB (hard 8 MB). CPU gate 2 ms per 1 s of IMU (hard 20 ms). One thread. Offline.

### 6.5 Scoring proxies (internal)
Official scoring uses the `evo` suite (ATE RMSE and RPE) against hidden ground truth, with weights: waypoints 30, blind 5 s 20, dropout 10 (closeness) and 40 for memory/runtime. The formula and segment definitions are unknown, so the harness computes all proxies in Section 11.

### 6.6 Assumption registry (each must be behind a flag or config key)

| ID | Assumption | Switch |
|---|---|---|
| A1 | One output row per input row | `--append-blind` |
| A2 | evo uses SE(3) alignment | harness runs aligned and unaligned |
| A3 | Gravity-aligned z-up, yaw 0, origin at start | `params.frame_mode` |
| A4 | Dropouts may appear as missing rows, NaN, zeros or repeats | gap handler handles all |
| A5 | Rate about 200 Hz | median dt computed from data |

### 6.7 Report template (every agent ends with this)

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

### 6.8 Blocker template

```markdown
## BLOCKER <WP-id> <date>
Symptom: <exact error or metric>
Minimal reproduction: <command + input>
Hypotheses tried: 1) ... 2) ... 3) ...
Suspected cause: <one line>
Needs: <decision / file outside allowlist / human input>
```

(The Algorithm cookbook, Section 12 below, is also appended to `docs/SPEC.md`.)

---

## 7. Work packets, dependency graph and gates

Status values: `TODO`, `IN_PROGRESS`, `REVIEW`, `DONE`, `BLOCKED`. Only the Reviewer sets `DONE`.

### 7.1 Phases and gates

```
P0 Bootstrap --G0--> P1 Harness --G1--> P2 Python ref --G2--> P3 C++ port --G3--> P4 QA --G4--> P5 Tune --G5--> FINAL
```

| Gate | Command | Passes when |
|---|---|---|
| G0 | `scripts/gate_g0.sh` | Repo layout exists, tools installed, footprint probe recorded |
| G1 | `scripts/gate_g1.sh` | pytest for degrade/validate/metrics green; naive baseline shows quadratic drift; `run_eval.py` prints a table |
| G2 | `scripts/gate_g2.sh` | Python ESKF beats naive baseline on ATE and blind-5 s on all held-out sequences; unit tests green |
| G3 | `scripts/gate_g3.sh` | C++ matches Python (diff tolerances), validator passes, RSS ≤ 4 MB, ≤ 2 ms per 1 s, 1 thread, deterministic |
| G4 | `scripts/gate_g4.sh` | Fuzz suite green under ASan/UBSan; budget regression green |
| G5 | `scripts/gate_g5.sh` | Tuned params improve held-out score over untuned; ablation table in `results/`; G3 and G4 re-pass |

### 7.2 Packets

`Par` marks packets that can run in parallel with the packet in the same row group (disjoint allowlists).

| WP | Agent | Title | Depends | Allowlist (files it may touch) | Acceptance |
|---|---|---|---|---|---|
| WP-00 | Conductor | Scaffold repo, rules, workflows, STATE, SPEC | none | everything under `.agent/`, `docs/`, `scripts/`, `configs/`, `.gitignore`, empty dirs | `scripts/gate_g0.sh` |
| WP-01 | Harness | Footprint probe and tool checks | WP-00 | `docs/decisions.md` | probe numbers present |
| WP-10 | Harness | `tools/synth.py`: analytic trajectories to ideal IMU + truth (Section 12.9) | WP-01 | `tools/synth.py`, `tests/test_synth.py` | `pytest tests/test_synth.py` |
| WP-11 | Harness | `tools/degrade.py` + config | WP-10 | `tools/degrade.py`, `configs/degrade_default.json`, `tests/test_degrade.py` | `pytest tests/test_degrade.py` (determinism by seed) |
| WP-12 | Harness | `tools/validate_trajectory.py` | WP-01 | `tools/validate_trajectory.py`, `tests/test_validate.py` | `pytest tests/test_validate.py` |
| WP-13 | Harness | `tools/metrics.py`: evo wrapper + segment metrics (Section 11) | WP-11 | `tools/metrics.py`, `tests/test_metrics.py` | `pytest tests/test_metrics.py` |
| WP-14 | Harness | Baselines: `src/ref/baseline_naive.py`, Mahony baseline | WP-10 | `src/ref/baseline_*.py`, `tests/test_baseline.py` | pytest; naive drift grows quadratically on synth |
| WP-15 | Harness | `tools/run_eval.py` + results table writer | WP-12, 13, 14 | `tools/run_eval.py`, `results/` | `scripts/gate_g1.sh` |
| WP-20 | Estimator | `ref/mathlib.py`: quaternion, exp map, skew, rotation (Section 12.1) | G0 | `src/ref/mathlib.py`, `tests/test_mathlib.py` | pytest |
| WP-21 | Estimator | `ref/parser.py`: tolerant streaming CSV reader + validity flags (Section 12.8) | G0 | `src/ref/parser.py`, `tests/test_parser.py` | pytest |
| WP-22 | Estimator | `ref/init.py` + strapdown only (no filter); static test | WP-20, 21 | `src/ref/init.py`, `src/ref/strapdown.py`, tests | pytest: static IMU 60 s drifts < 1e-6 m |
| WP-23 | Estimator | `ref/eskf.py` predict + Joseph update helper | WP-22 | `src/ref/eskf.py`, tests | pytest: P symmetric PD after 10k steps |
| WP-24 | Estimator | Tilt update and ZUPT | WP-23 | `src/ref/eskf.py`, `src/ref/updates.py`, tests | pytest: tilt error converges on synth with initial 5 deg error |
| WP-25 | Estimator | Degradation monitor (quality score) | WP-23 | `src/ref/monitor.py`, tests | pytest: score falls as injected noise grows |
| WP-26 | Estimator | Gap/dropout handler | WP-24, 25 | `src/ref/gap.py`, tests | pytest: no position jump > bound across a 2 s gap on synth |
| WP-27 | Estimator | Failure detector + blind-5 s predictor | WP-26 | `src/ref/blind.py`, tests | pytest: blind RMSE < naive on synth |
| WP-28 | Estimator | `ref/pipeline.py` + CLI; ablations | WP-27, G1 | `src/ref/pipeline.py`, `src/ref/__main__.py`, `results/ablation_py.md` | `scripts/gate_g2.sh` |
| WP-30 | CppPort | C++ skeleton: stdio parser, int64 timestamp writer, params.h export | G2 | `src/cpp/io.*`, `src/cpp/main.cpp`, `src/cpp/Makefile`, `tools/export_params.py` | passthrough output validates; RSS probe |
| WP-31 | CppPort | `mat.h`, `quat.h` fixed-size math + C++ unit tests | G2 | `src/cpp/mat.h`, `src/cpp/quat.h`, `tests/cpp/test_math.cpp` | `make -C src/cpp test` (compares to golden values from Python) |
| WP-32 | CppPort | Port init, strapdown, ESKF predict | WP-30, 31 | `src/cpp/eskf.*`, `src/cpp/init.*` | diff test on synth, tolerance in 12.10 |
| WP-33 | CppPort | Port updates, monitor, gap, blind | WP-32 | `src/cpp/updates.*`, `monitor.*`, `gap.*`, `blind.*` | diff test on degraded sets |
| WP-34 | CppPort | `scripts/diff_py_cpp.py` and full differential test | WP-33 | `scripts/diff_py_cpp.py` | zero tolerance violations |
| WP-35 | CppPort | Performance pass if budgets missed | WP-34 | `src/cpp/*` (no behaviour change) | `scripts/gate_g3.sh` |
| WP-40 | QA | Fuzz inputs generator + runner | WP-34 | `tests/fuzz/*` | all cases produce valid output |
| WP-41 | QA | ASan/UBSan build and run | WP-40 | `src/cpp/Makefile` (sanitize target only), `tests/fuzz/*` | zero sanitizer findings or blockers filed |
| WP-42 | QA | Budget regression + determinism script | WP-35 | `scripts/budget_check.sh` | `scripts/gate_g4.sh` |
| WP-50 | Tuner | `tools/tune.py` (Optuna, CV by sequence) + export | G3 | `tools/tune.py`, `configs/params.json`, `results/tune_*` | held-out score improves |
| WP-51 | Tuner | Final ablation table, plots | WP-50 | `results/*`, `tools/plot_*.py` | `scripts/gate_g5.sh` |
| WP-90 | Reviewer | Review of any packet in REVIEW | any | `docs/STATE.md`, `docs/gate_log.md`, `docs/blockers.md` | see Section 8.6 |

Parallel groups (disjoint allowlists): {WP-12, WP-14} after WP-10; {WP-20, WP-21}; {WP-30, WP-31}; {WP-40 with WP-42 prep}. Everything else is sequential.

### 7.3 `docs/STATE.md` format (Conductor owns structure; agents edit only their row)

```markdown
# STATE
Updated: <ISO time> by <agent>
Current phase: P0
Last passed gate: none

| WP | Status | Owner | Attempts | Last note |
|----|--------|-------|----------|-----------|
| WP-00 | TODO | Conductor | 0 | |
...
```

---

## 8. Agents (roles, tools, prompts)

All agents: Flash. Tools available to all: file read/write/edit, terminal, git (local). **Tavily only for Harness (WP-01, 13) and Estimator (WP-20 to 28) when they need to verify a fact.** Cpp, QA, Tuner agents do not use Tavily.

### 8.1 Conductor (A0)

Purpose: keep the DAG honest, dispatch the next packet, never writes algorithm code.

```markdown
ROLE: Conductor for the Track 3 IMU estimator build. You manage state; you never write
estimator or tool code.

EVERY SESSION
1. Read docs/STATE.md and PLAN.md section 7 (packet table) only.
2. If any packet is REVIEW -> output: "Dispatch Reviewer for <WP>" and stop.
3. Else if any packet is BLOCKED -> read docs/blockers.md, propose ONE resolution
   (clarify spec, widen allowlist, split packet, or ask the human), update the packet
   file, set it back to TODO, and stop.
4. Else pick the first TODO packet whose dependencies are all DONE and whose gate (if
   any) has passed. Generate docs/packets/<WP>.md from the packet table using the packet
   template below. Set status IN_PROGRESS. Output the exact agent name and the exact
   one-line instruction to paste into a new Antigravity conversation.
5. When a phase's last packet is DONE, run `scripts/gate_gN.sh`, paste raw output into
   docs/gate_log.md, and update "Last passed gate".
6. Never mark a packet DONE yourself.

PACKET TEMPLATE
# <WP-id>: <title>
Agent: <name>   Mode: Planning|Fast
Goal: <1-2 sentences>
Read: <list of files, max 5>
Allowlist: <files that may be created/edited>
Forbidden: touching anything else; changing the output format; adding dependencies
Steps: <numbered, <= 8 steps, each one verifiable>
Tests first: <test names to write before code>
Acceptance: <exact command and expected result>
Hints: <relevant cookbook sections>
```

### 8.2 Harness agent (A1): WP-01, 10 to 15

```markdown
ROLE: Evaluation harness engineer. You build the measuring stick; you never touch
src/ref or src/cpp.

KNOWLEDGE YOU MUST USE
- Metric definitions: PLAN.md/SPEC "Metrics" (checkpoint, blind-5s, dropout, ATE, RPE).
- Before using `evo`, run `evo_ape --help` and `evo_rpe --help` and use only flags you
  saw. Use Tavily only to confirm published IMU noise values or evo behaviour; log
  sources in docs/decisions.md.

RULES
- Every tool is a CLI with argparse and a pure-function core that tests can import.
- Deterministic given --seed. No hidden randomness.
- degrade.py must support four dropout encodings (missing rows, NaN, zeros, repeated
  values), progressive noise ramp, bias random walk, a random but known gravity-alignment
  rotation, and a final-failure point. It must write: degraded imu CSV, ground-truth TUM
  file, and a JSON sidecar listing the dropout windows, failure time and checkpoint times
  (the sidecar is what the segment metrics read).
- The estimator never sees the sidecar.

OUTPUT: the packet's files + the report template. Do not exceed the allowlist.
```

### 8.3 Estimator agent (A2): WP-20 to 28 (Python reference)

```markdown
ROLE: Estimation engineer for a 15-state ESKF in Python. Everything you write must be
portable to fixed-size C++: no dicts of arrays, no dynamic lists inside the filter loop,
history only in a fixed ring buffer.

KNOWLEDGE YOU MUST USE
- docs/SPEC.md "Algorithm cookbook" for all equations and default constants.
  Implement exactly those. Do not substitute other formulations.
- If a cookbook step is ambiguous, implement the simplest reading and log it in
  decisions.md.

RULES
- Tests first (packet lists them). Use tools/synth.py trajectories as oracles.
- One module per packet. Do not refactor earlier modules.
- Constants come from configs/params.json via a single loader; no inline constants.
- Pure functions where possible: step(state, sample, params) -> state.
- Print nothing in the filter loop.

VERIFY EQUATIONS
- If you doubt a Jacobian, check against the numerical-derivative test in
  tests/test_jacobians.py (create it in WP-23). Do not search the web for alternate
  forms unless the numerical test fails; then log the source.
```

### 8.4 CppPort agent (A3): WP-30 to 35

```markdown
ROLE: C++17 embedded-style engineer. Port the validated Python reference to a streaming
binary.

RULES
- Port module by module, in the packet order. After each module run the differential
  test on the golden vectors in tests/golden/ (produced by the Python reference).
- Matrix type: fixed-size template Mat<R,C> on stack/static storage, row-major, double.
  15x15 covariance is a static array. No std::vector, no std::string in the main loop,
  no iostream, no exceptions, no new/malloc after startup.
- Input: fopen/fread with a 4 KB static buffer; parse lines by hand; custom decimal
  parser (handle sign, digits, '.', 'e/E', 'nan', 'inf', empty).
- Output: fwrite from a static 64 KB buffer; format the int64 timestamp with integer
  arithmetic; format doubles with a hand-written fixed 9-decimal writer (or snprintf
  into a small static buffer if the speed gate holds).
- Compile flags: -O2 -std=c++17 -Wall -Wextra -Werror; add -static if it lowers RSS.
- Do NOT change filter behaviour to gain speed. Only restructure computation.
- Do NOT tune constants. They come from src/cpp/params.h (generated).

PERFORMANCE ORDER (only if gates fail)
1. Check baseline RSS of an empty static binary (docs/decisions.md). Subtract it.
2. Remove iostream/locale/stdlib bloat; avoid printf with floating formats if slow.
3. Exploit symmetry/sparsity of F (identity blocks) when computing F P F^T.
4. Only then micro-optimise.
```

### 8.5 QA agent (A4): WP-40 to 42

```markdown
ROLE: Adversarial tester. You find failures; you do not fix estimator logic.

RULES
- Each fuzz case is a generated file in tests/fuzz/cases/ plus an expectation list:
  exit code 0, validator pass, row count equals input data row count, no nan/inf.
- Cases: NaN fields, empty fields, all-zero rows, repeated rows, 2 s and 10 s gaps,
  gap through end of file, duplicate timestamps, reversed timestamps, truncated last
  line, CRLF, no header, header only, empty file, 10-minute file, huge values (1e30),
  tiny dt (1 ns), timestamps near 2^63.
- Run under ASan+UBSan via `make -C src/cpp sanitize`.
- Minimal parser/writer fixes are allowed ONLY if the packet allowlist includes them;
  otherwise file a blocker with a minimal reproducing input.
```

### 8.6 Reviewer agent (A5): WP-90 (the most important Flash safeguard)

```markdown
ROLE: Independent verifier. You assume the worker is wrong until the evidence proves
otherwise. Use Planning mode with thinking on.

PROCEDURE for the packet in REVIEW
1. Read docs/packets/<WP>.md and the worker's REPORT.
2. Run `git diff --stat` and `scripts/check_scope.sh <WP>`; fail if files outside the
   allowlist changed.
3. Re-run the packet's Acceptance command yourself. Compare with the pasted output.
4. Run the full regression: `pytest -q` (and `make -C src/cpp test` once C++ exists).
5. Checklist (answer yes/no with evidence for each):
   a. Tests exist for every item under "Tests first" and fail when the code is broken
      (mutate one line to confirm at least one test fails, then restore).
   b. No magic numbers; constants come from params.
   c. No forbidden constructs (iostream, dynamic allocation in main loop for C++; global
      state in Python ref).
   d. Output format unchanged (run tools/validate_trajectory.py on a sample).
   e. decisions.md updated for every deviation.
6. Append to docs/gate_log.md: WP, date, commands, raw output tails, verdict.
7. Verdict PASS -> set DONE. Verdict FAIL -> set TODO, increment Attempts, and write the
   exact failing evidence into the packet file under "Reviewer feedback".
8. If Attempts reaches 3 -> BLOCKED.

You may not edit code. You may only edit STATE.md, gate_log.md, blockers.md and the
packet file's "Reviewer feedback" section.
```

### 8.7 Tuner agent (A6): WP-50, 51

```markdown
ROLE: Parameter tuner and analyst.

RULES
- Search space and bounds are in Section 13 of PLAN.md (copied to configs/tune_space.json).
- Tune with the PYTHON reference for speed, then verify the best parameters on the C++
  binary (they must agree within the diff tolerances).
- Cross-validation by SEQUENCE (leave-one-sequence-out), multiple degradation seeds.
  Report mean and worst case on held-out folds only.
- Objective: weighted sum (Section 11.4). Reject parameter sets that worsen worst-case
  by more than 10 percent even if the mean improves.
- Do not touch estimator code. Only configs/params.json and results/.
```

---

## 9. Workflows (`.agent/workflows/*.md`, invoked as slash commands)

`/next` (`.agent/workflows/next.md`):

```markdown
---
description: Advance the build by exactly one step
---
1. Read docs/STATE.md.
2. Follow the Conductor procedure (PLAN.md section 8.1) and output the dispatch line.
3. Do not start the dispatched packet yourself.
```

`/review` (`.agent/workflows/review.md`):

```markdown
---
description: Independent verification of the packet currently in REVIEW
---
1. Read docs/STATE.md and find the packet in REVIEW.
2. Follow the Reviewer procedure (PLAN.md section 8.6) exactly.
3. Output the verdict and the new STATE.md row.
```

`/gate` (`.agent/workflows/gate.md`):

```markdown
---
description: Run a phase gate and log it
---
1. Ask which gate (G0..G5) if not given.
2. Run `scripts/gate_<n>.sh`. Paste the raw output into docs/gate_log.md.
3. Report pass or fail only. Do not fix anything.
```

`/blocked` (`.agent/workflows/blocked.md`):

```markdown
---
description: Triage a blocked packet
---
1. Read docs/blockers.md (last entry) and the packet file.
2. Propose ONE of: clarify spec, split the packet, widen the allowlist, or ask the human.
3. Edit only the packet file and STATE.md.
```

**Operating loop (you, the human):**

```
/next            -> Conductor tells you which agent and which packet
(open new conversation) paste the dispatch line -> worker does the packet, ends in REVIEW
(open new conversation) /review                 -> Reviewer sets DONE or TODO/BLOCKED
repeat
```

---

## 10. Skills (`.agent/skills/<name>/SKILL.md`)

Short, single-purpose procedures that Flash loads on demand.

**`run-diff-test`**

```markdown
---
name: run-diff-test
description: Compare Python reference and C++ binary outputs numerically. Use after any change in src/cpp or src/ref.
---
1. Build: `make -C src/cpp`.
2. For each file in tests/golden/inputs/: run `python -m ref <in> /tmp/py.txt` and
   `src/cpp/build/main <in> /tmp/cpp.txt`.
3. Run `python scripts/diff_py_cpp.py /tmp/py.txt /tmp/cpp.txt` and report max abs
   position diff, max quaternion component diff, first row exceeding tolerance.
```

**`measure-budget`**

```markdown
---
name: measure-budget
description: Measure peak RSS, CPU time per second of IMU data, thread count. Use for any budget claim.
---
1. `/usr/bin/time -v taskset -c 0 src/cpp/build/main <in> /tmp/o.txt 2> /tmp/time.txt`
2. Peak RSS: `grep "Maximum resident" /tmp/time.txt`
3. CPU: user+system seconds from the same file, divided by data duration
   (last timestamp minus first, in seconds); report in ms per 1 s.
4. Report raw lines plus the computed numbers. Never estimate.
```

**`verify-cli`**

```markdown
---
name: verify-cli
description: Confirm a command-line flag exists before using it.
---
1. Run `<tool> --help` (or `<tool> <subcommand> --help`).
2. Quote the line that defines the flag in your report.
3. If absent, use Tavily for the tool's docs and record the URL in docs/decisions.md.
```

**`log-blocker`**

```markdown
---
name: log-blocker
description: Write a blocker entry after 3 failed attempts.
---
Append the template in docs/SPEC.md section 6.8 to docs/blockers.md, set the packet to
BLOCKED in docs/STATE.md, then stop.
```

---

## 11. Metrics and harness definitions

### 11.1 Data flow

```
data/<seq>/imu0/data.csv ──degrade.py(seed, cfg)──► work/<seq>_<seed>/imu0/data.csv
                                                    work/<seq>_<seed>/gt.txt (TUM)
                                                    work/<seq>_<seed>/meta.json
estimator(cmd) ──► estimated_trajectory.txt ──validate──► metrics.py ──► results table
```

`meta.json` fields: `dropout_windows[{start_ns,end_ns,encoding}]`, `failure_ns`, `checkpoint_ns[]` (every 10 s by default), `gravity_rotation`, `seed`.

### 11.2 Alignment

The unobservable start pose and yaw make alignment matter. Always report both: `evo_ape ... -a` (SE(3) alignment) and unaligned. Also compute our own **start-anchored** error, which is the position error after anchoring both trajectories at the first pose (this matches how the estimator defines its world frame).

### 11.3 Segment metrics (all computed on the estimated trajectory against truth, after the same alignment option)

| Metric | Definition |
|---|---|
| `ate_rmse` | evo APE translation RMSE, full trajectory |
| `rpe_1s` | evo RPE translation and rotation over 1 s windows |
| `checkpoint_err` | Mean position error at `checkpoint_ns` |
| `blind5_rmse` | Position RMSE over `[failure_ns, failure_ns + 5 s]`, plus final-point error |
| `dropout_rmse` | Position RMSE inside each dropout window and in the 5 s after it |
| `rot_err_deg` | Mean orientation error at the same instants |

### 11.4 Scalar objective for tuning

```
J = 0.30 * norm(checkpoint_err) + 0.20 * norm(blind5_rmse) + 0.10 * norm(dropout_rmse)
  + 0.25 * norm(ate_rmse)        + 0.15 * norm(rpe_1s)
```

`norm(x)` = `x / x_naive` for the same sequence and seed (ratio to the naive baseline, so sequences are comparable). Lower is better. Report the mean and worst case over held-out folds.

### 11.5 Baselines

| Name | Description |
|---|---|
| `naive` | Initialise tilt from first 1 s mean, integrate raw IMU, no corrections |
| `mahony` | Mahony attitude filter plus gravity-removed integration |
| `eskf_basic` | ESKF with fixed noise, no monitor, no gap handler, no blind model |
| `eskf_full` | Full system |

---

## 12. Algorithm cookbook (append to `docs/SPEC.md`)

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

---

## 13. Tuning search space (`configs/tune_space.json`)

| Parameter | Range | Scale |
|---|---|---|
| `noise_mult_gyro` (multiplier on density) | 1 to 100 | log |
| `noise_mult_acc` | 1 to 100 | log |
| `walk_mult_bg` | 1 to 200 | log |
| `walk_mult_ba` | 1 to 200 | log |
| `kappa_max` | 5 to 200 | log |
| `tilt_tau_a` | 0.1 to 1.0 | linear |
| `tilt_sigma` | 0.005 to 0.1 | log |
| `zupt_sigma` | 0.01 to 0.2 | log |
| `tau_gap` | 0.5 to 10 s | log |
| `recover_steps` | 20 to 400 | int |
| `tau_v` (blind) | 0.5 to 10 s | log |
| `tau_omega` (blind) | 0.3 to 5 s | log |
| `shrink_v` | 0.5 to 1.0 | linear |

Protocol: 200 Optuna TPE trials on the Python reference; leave-one-sequence-out CV with 3 degradation seeds per sequence; accept only if held-out mean J improves and worst-case does not worsen by more than 10 percent. Then re-run G3 with the C++ binary and the exported `params.h`.

---

## 14. Gate scripts (`scripts/`)

Each gate script exits non-zero on the first failure and prints raw output. Conductor creates stubs in WP-00; owners fill them as packets land.

**`scripts/check_scope.sh <WP>`** reads the allowlist from `docs/packets/<WP>.md`, runs `git diff --name-only HEAD`, and exits 1 if any file is outside the allowlist.

**`scripts/budget_check.sh <input_dir>`** (WP-42; reference implementation):

```bash
#!/usr/bin/env bash
set -euo pipefail
IN="$1"; BIN=src/cpp/build/main; OUT=$(mktemp)
/usr/bin/time -v taskset -c 0 "$BIN" "$IN" "$OUT" 2> /tmp/time.txt
RSS_KB=$(grep "Maximum resident" /tmp/time.txt | awk '{print $NF}')
USER=$(grep "User time" /tmp/time.txt | awk '{print $NF}')
SYS=$(grep "System time" /tmp/time.txt | awk '{print $NF}')
DUR=$(python3 tools/data_duration.py "$IN")
MS=$(python3 -c "print(($USER+$SYS)/$DUR*1000)")
echo "peak_rss_kb=$RSS_KB cpu_ms_per_s=$MS duration_s=$DUR"
[ "$RSS_KB" -le 4096 ] || { echo "FAIL: RSS gate"; exit 1; }
python3 -c "import sys; sys.exit(0 if $MS <= 2.0 else 1)" || { echo "FAIL: CPU gate"; exit 1; }
"$BIN" "$IN" "$OUT"; sha1sum "$OUT" > /tmp/a.sha
"$BIN" "$IN" "$OUT"; sha1sum "$OUT" > /tmp/b.sha
cmp /tmp/a.sha /tmp/b.sha || { echo "FAIL: nondeterministic"; exit 1; }
THREADS=$(grep -c . /dev/null; true)
python3 tools/validate_trajectory.py "$IN" "$OUT"
echo "BUDGET OK"
```

Gates G0 to G5 run the commands listed in Section 7.1 and finish with an explicit `GATE Gn PASS` line. A gate script that cannot print that line has failed.

---

## 15. Failure playbook (what to do when Flash goes wrong)

| Symptom | Action |
|---|---|
| Agent edits files outside the allowlist | Revert with `git checkout -- <files>`, rerun with the allowlist quoted at the top of the prompt |
| Agent claims success without raw output | Reviewer fails the packet; no exceptions |
| Estimator diverges on synth with zero noise | Run `tests/test_jacobians.py` (numerical vs analytic F). Check quaternion convention (Hamilton, body→world) and the sign of gravity (`g_w = [0,0,-g]`) before touching anything else |
| C++ and Python disagree early | Compare after init only, then after 1 step, then 100 steps. First divergence tells the module |
| RSS above gate | Check the empty static baseline; remove iostream; check static arrays are not duplicated; check output buffer size |
| Packet too big for Flash (3 failed attempts) | Conductor splits it into two packets and updates the table |
| Context confusion mid-session | Stop; start a new conversation for the same packet; the packet file and STATE.md are the memory |
| Tuning improves mean but hurts worst case | Reject; keep previous params |
| Tavily returns conflicting constants | Prefer the dataset's own documentation; log both; use the more conservative (larger noise) value |

---

## 16. Definition of done (build)

1. G0 to G5 all PASS with raw output in `docs/gate_log.md`.
2. `src/cpp/build/main <folder> out.txt` produces a valid file; `tools/validate_trajectory.py` passes.
3. Peak RSS ≤ 4 MB, ≤ 2 ms per 1 s of IMU data, one thread, deterministic.
4. Fuzz suite green under ASan/UBSan.
5. `results/ablation.md` shows `naive`, `mahony`, `eskf_basic`, `eskf_full` on held-out sequences, with the full-system worst case better than `naive`.
6. `docs/decisions.md` lists every assumption (A1 to A5) with its switch.

---

## 17. First five actions

1. Do Section 4 by hand (tools, venv, data, Tavily, footprint probe).
2. New Antigravity conversation, Fast mode, Flash: *"Act as the Conductor in PLAN.md section 8.1. Execute WP-00."* (Conductor creates rules, workflows, skills, `STATE.md`, `SPEC.md`, gate stubs from this file.)
3. `/review` in a new conversation (Planning mode) to verify WP-00.
4. `/next`, then dispatch WP-01, WP-10 to the Harness agent.
5. Keep running the `/next` → worker → `/review` loop until G5 passes.