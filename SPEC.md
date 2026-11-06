# Technical PRD: Track 3, Path Finder (Trajectory Estimation Under Sensor Degradation)

| Field | Value |
|---|---|
| **Event** | XR Hackathon Challenge 2026, Samsung R&D Institute India, Bangalore (XR Perception Group + PRISM) |
| **Track** | Track 3: Path Finder |
| **Document version** | v1.0, 6 Nov 2026 |
| **Status** | Locked track; implementation not started |
| **Source of truth** | `XR_Hackathon_2026_Consolidated_Slides_for_sharing_v1.pdf` (Track 3 = slides 12 to 17; common rules = slides 24 to 27; overview = slide 2) |
| **Hard deadlines** | Registration closes **8 Nov 2026, 11:59 PM**. Final submission closes **18 Nov 2026, 11:59 PM, no extension** |
| **Organizer contact** | prism@samsung.com |

**Reading guide.** Every statement is tagged by origin so you can tell what the organizers actually require from what we decided ourselves.

- `[DECK s.N]` = specified by the organizers on slide N.
- `[DECISION]` = our engineering choice.
- `[ASSUMPTION]` = not specified; our working assumption, to be confirmed (see section 17).

---

## 1. One-page summary

**Task.** Take a **degraded raw IMU CSV** (EuRoC format) and produce a **6-DoF trajectory** (position + orientation), one pose per IMU row, in **TUM pose format**. After the IMU fails completely, **dead-reckon 5 seconds** forward. `[DECK s.13, 14, 16]`

**Hard budgets.** Peak RAM **≤ 8 MB**, compute **≤ 20 ms per 1 s of IMU data**, **single CPU thread**, **offline**. `[DECK s.13, 14]`

**Scoring (100 pts).** `[DECK s.14]`

| Block | Weight | Sub-component | Points |
|---|---|---|---|
| Execution & Efficiency | **40%** | Peak memory | **20** |
| | | Runtime | **20** |
| Trajectory Closeness | **60%** | Waypoints | **30** |
| | | Blind 5 s | **20** |
| | | Dropout | **10** |
| **Total** | | | **100** |

**Ranked with** the open-source `evo` suite (ATE RMSE and RPE) against a hidden ground truth. `[DECK s.17]`

**Our strategy.** `[DECISION]`
1. **Bank the 40 Execution points** with a streaming C++ error-state Kalman filter that uses a few MB of RAM and a small fraction of the time budget.
2. **Compete on the 60 Closeness points** with adaptive noise handling, a dedicated dropout handler and a conservative blind-5-s extrapolator, all tuned on our own harness over EuRoC and TUM-VI.
3. **Never lose points to packaging.** Exact output format, exact bundle layout, clean-room tested.

---

## 2. Background and problem statement

### 2.1 Scenario `[DECK s.13]`

An aeroplane navigates through waypoints in a GPS-denied, hostile environment, capturing reconnaissance images. Enemy jamming causes escalating IMU noise, bias drift and frame dropouts until the IMU fails entirely. The aircraft must keep estimating its pose through all mission checkpoints, then dead-reckon its 3D path for 5 seconds after the final checkpoint "to strike the target".

### 2.2 Why it is hard `[DECK s.13]`

| Problem | Effect |
|---|---|
| Double integration of acceleration | Position error grows **quadratically** with time under noise and bias |
| Progressive noise and bias drift | Fixed-parameter filters become over-confident and diverge |
| Frame dropouts, including up to **2 s** of signal loss | Naive integration breaks or jumps across the gap |
| Unknown gravity alignment `[DECK s.14]` | The sensor frame to world frame relation (roll and pitch) is not given and must be estimated |
| Complete IMU failure | Nothing to integrate; the trajectory must be predicted |

### 2.3 XR relevance (for the jury narrative) `[DECISION]`

Inertial-only pose continuity is what a headset or glasses needs when camera tracking fails (occlusion, darkness, motion blur). The "aircraft" is the competition's framing; the algorithm is the same family used in XR pose tracking.

---

## 3. Mark split-out (detailed)

### 3.1 Official split `[DECK s.14]`

| # | Block | Component | Points | What it measures (deck wording) | Scored by |
|---|---|---|---|---|---|
| 1 | Execution & Efficiency (40%) | **Peak memory** | **20** | Peak RAM against the 8 MB budget | Organizer measurement |
| 2 | | **Runtime** | **20** | Compute per 1 s of IMU against the 20 ms budget | Organizer measurement |
| 3 | Trajectory Closeness (60%) | **Waypoints** | **30** | Closeness of the trajectory at mission checkpoints | `evo` vs hidden GT |
| 4 | | **Blind 5 s** | **20** | Closeness of the 5 s dead-reckoned segment after IMU failure | `evo` vs hidden GT |
| 5 | | **Dropout** | **10** | Closeness during or after dropout periods | `evo` vs hidden GT |
| | | **Total** | **100** | | |

Underlying metrics `[DECK s.17]`:
- **ATE (RMSE):** absolute trajectory error, overall global consistency.
- **RPE:** relative pose error, local drift and rotational error per unit time.

### 3.2 Tier mapping `[DECK s.14]`

| Tier | Meaning | Maps to scored component |
|---|---|---|
| **T1** | Checkpoints | Waypoints (30) |
| **T2** | Blind 5 s, "(+20%)" | Blind 5 s (20) |
| **T3** | Dropout resilience, "(+20%)" | Dropout (10) |

> **Discrepancy.** The tier bonuses read "+20%" each, but the evaluation split gives Blind 5 s = 20 and Dropout = 10. Our plan treats the **evaluation split (20 / 10) as authoritative** and asks the organizers to confirm (Q8).

### 3.3 What is **not** specified (unknowns that affect points)

| Unknown | Why it matters |
|---|---|
| Formula that converts ATE/RPE (and memory/runtime) into points | Pass/fail vs graded vs relative rank decides how much margin we need |
| Where the memory budget is measured (whole process RSS vs algorithm only) | Decides whether Python is viable at all |
| Definition of "checkpoints" and "dropout" windows in scoring | Decides where to focus tuning |
| Alignment used by `evo` (none, SE(3), Sim(3)) | Decides how much the unobservable start pose and yaw matter |
| Whether blind-5-s poses are extra rows or rows already present | Decides the output row count |

### 3.4 Points plan `[DECISION]`

| Component | Max | Our target | How we secure it |
|---|---|---|---|
| Peak memory | 20 | **20** | Streaming C++ with fixed buffers; target ≤ **3 MB**, giving 5 MB headroom |
| Runtime | 20 | **20** | Target ≤ **2 ms per 1 s** of IMU, a 10x margin |
| Waypoints | 30 | Top-quartile among teams | Adaptive ESKF, tilt and gyro-bias correction, tuned offline |
| Blind 5 s | 20 | Top-quartile among teams | Conservative decaying-velocity and constant-turn-rate extrapolation, tuned for 5 s error |
| Dropout | 10 | Top-quartile among teams | Gap detector, covariance inflation, motion-model propagation |
| **Floor (execution only)** | **40** | **40** | |

Execution is where engineering discipline alone earns points, so it is our first priority. Closeness is relative to other teams and uses a hidden ground truth, so we measure ourselves on an internal harness against a naive baseline and tune for margin (section 12).

---

## 4. Goals, non-goals and success criteria

### 4.1 Goals

- **G1.** Valid `estimated_trajectory.txt` for every input, in the exact required format. `[DECK s.16]`
- **G2.** Meet every budget with large margin: RAM ≤ 8 MB, ≤ 20 ms per 1 s IMU, single thread, offline. `[DECK s.13]`
- **G3.** Minimize ATE and RPE across checkpoints, the blind 5 s and dropout windows.
- **G4.** Fully reproducible submission that runs from a clean environment using only the README.
- **G5.** Submit all common deliverables on time, in compliant form. `[DECK s.25]`

### 4.2 Non-goals

- Using camera, GPS, magnetometer or any non-IMU data. The task says **only degraded IMU streams**. `[DECK s.2, 13]`
- Deep-learning models that cannot fit the 8 MB budget.
- Real-time hardware deployment or a GUI product.
- Multithreading, GPU, network access or runtime downloads. `[DECK s.13]`

### 4.3 Success criteria (acceptance gates)

| ID | Criterion | Threshold |
|---|---|---|
| AC-1 | Output passes the format validator on all test inputs | 100% |
| AC-2 | Row count equals the IMU row count (or the confirmed rule) | exact |
| AC-3 | Peak RSS on the largest test input | ≤ **4 MB** internal gate (8 MB hard) |
| AC-4 | CPU time per 1 s of IMU data | ≤ **2 ms** internal gate (20 ms hard) |
| AC-5 | Runs with a single thread | verified under `taskset -c 0` |
| AC-6 | Deterministic: two runs give byte-identical output | yes |
| AC-7 | Better than the naive double-integration baseline on ATE and 5 s blind error for every test sequence | yes |
| AC-8 | Clean-room run from README succeeds in a fresh container | yes |

---

## 5. Input specification `[DECK s.15]`

The evaluation dataset is a **zipped folder of raw IMU telemetry** in the industry-standard **EuRoC MAV** format. "Non-conforming submissions fail the automated evaluation pipeline."

| Property | Specification |
|---|---|
| Container | Zipped folder |
| File path | `imu0/data.csv` |
| Delimiter | Comma |
| Header line | `#timestamp [ns], w_RS_S_x [rad s^-1], w_RS_S_y [rad s^-1], w_RS_S_z [rad s^-1], a_RS_S_x [m s^-2], a_RS_S_y [m s^-2], a_RS_S_z [m s^-2]` |
| Columns (7) | timestamp, then gyro x, y, z, then accelerometer x, y, z |
| `timestamp [ns]` | 64-bit integer, nanoseconds since Unix epoch |
| `w_RS_S_*` | "Compensated" angular velocity, rad/s |
| `a_RS_S_*` | "Compensated" linear acceleration, m/s², **includes gravity** |
| Rate | The sample rows are 5 ms apart, i.e. **200 Hz** `[ASSUMPTION: general; verify per sequence]` |

**Sample input** `[DECK s.15]`:

```
#timestamp [ns], w_RS_S_x [rad s^-1], w_RS_S_y [rad s^-1], w_RS_S_z [rad s^-1], a_RS_S_x [m s^-2], a_RS_S_y [m s^-2], a_RS_S_z [m s^-2]
1403636579758555392,0.0124,-0.0023,0.0051,0.1241,0.0312,9.8105
1403636579763555392,0.0119,-0.0021,0.0049,0.1235,0.0305,9.8099
```

(The deck wraps the header across several lines for display; the real file has one header line.)

### 5.1 Degradation (what the data will look like) `[DECK s.13, 14]`

- **Progressive noise** that grows over the mission.
- **Bias drift** that grows over the mission.
- **Frame dropouts**, including signal loss of **2 s**.
- **Unknown gravity alignment.**
- **Final complete IMU failure**, after which 5 s must be dead-reckoned.
- An **organizer degradation script is provided** for generating degraded data from EuRoC MAV and TUM-VI. `[DECK s.13, 14]`

### 5.2 Input robustness requirements `[DECISION]`

Because the exact encoding of dropouts and failure is not specified, the parser must tolerate:

| Case | Required handling |
|---|---|
| Header line starting with `#` | Skip |
| Blank lines, CRLF line endings, trailing spaces | Ignore or strip |
| Missing or empty fields, `NaN`, `inf` | Row kept; flagged invalid |
| Rows of all zeros, or exactly repeated rows (stuck sensor) | Flagged as suspect dropout |
| Missing rows (timestamp gap larger than the nominal interval) | Detected as a gap |
| Non-monotonic or duplicate timestamps | Row kept; no integration for `dt ≤ 0` |
| Truncated final line | Parse what is valid; do not crash |
| Empty file | Exit cleanly with a valid (possibly header-only) output |
| Input given as a zip or as an unzipped folder | Both accepted (see FR-1) |

---

## 6. Output specification `[DECK s.16]`

### 6.1 Required format

| Property | Specification |
|---|---|
| **File name** | `estimated_trajectory.txt` |
| Delimiter | **Space-separated** |
| Header | Optional; any comment line must start with `#` |
| Rows | **One estimated pose per row, exactly 8 columns** |
| Column order | `timestamp tx ty tz qx qy qz qw` |
| `timestamp` | Floating-point **seconds** (Unix epoch). Conversion: `s = ns × 10⁻⁹` |
| `tx ty tz` | 3D position vector in **meters** |
| `qx qy qz qw` | **Normalized unit quaternion**, world frame |
| Synchronization | **Exactly one pose per IMU frame.** "Dropped or misaligned rows incur heavy penalties or failure." |

**Sample output** `[DECK s.16]`:

```
# timestamp tx ty tz qx qy qz qw
1403636579.7585 0.0012 0.0045 0.0102 0.0001 0.0023 0.0011 0.9999
1403636579.7635 0.0015 0.0049 0.0105 0.0002 0.0024 0.0011 0.9999
```

(The deck wraps each pose over two lines for display; the real file has **one pose per line**.)

### 6.2 Our formatting rules `[DECISION]`

| Rule | Reason |
|---|---|
| Write the timestamp **directly from the int64 nanosecond value** as `<ns / 1e9>.<ns % 1e9 padded to 9 digits>` | A double near 1.4e9 only holds about 7 decimal places, so converting through floating point can shift timestamps and create misalignment. This yields the exact input timestamp |
| Write positions and quaternion components with **9 decimals** in fixed notation | Avoids truncation and scientific notation |
| Always normalize the quaternion before writing; keep `qw` in the 4th column | Spec requires unit norm and `qx qy qz qw` order |
| Use the **Hamilton** convention, quaternion = rotation from body (IMU) frame to world frame | Standard TUM pose meaning `[ASSUMPTION]` |
| Keep sign continuity of the quaternion between consecutive rows | Avoids spurious flips for tools that interpolate |
| Include one comment header line `# timestamp tx ty tz qx qy qz qw` | Allowed by spec; helps reading |
| LF line endings, trailing newline | Portable |
| **World frame:** z-up, gravity-aligned, yaw 0 and position 0 at the first sample | IMU-only data cannot observe the true start pose `[ASSUMPTION]`; confirm in Q2 |
| Position, orientation and timestamp **must be finite** in every row | Never emit `nan` or `inf`; fall back to the last valid value |

### 6.3 Row-count rule (critical, see Q1)

"Exactly one pose per IMU frame" is explicit. What is unclear is how the **5-second blind segment** is represented: as input rows after the failure (which our output covers naturally), or as 5 s of *extra* rows beyond the end of the input. `[ASSUMPTION]` Default: **one output row per input row**. A command-line flag `--append-blind` (default **off** until confirmed) appends 5.0 s of extrapolated poses at the median sample interval after the last row.

### 6.4 Optional metadata `[DECK s.17]`

`metadata.json` is optional ("runtime perf, compute specs"). `[DECISION]` We include it, because measured proof of the budgets supports the Execution points and the jury story:

```json
{
  "team": "CollegeName_TeamName",
  "implementation": "C++17, single thread",
  "peak_rss_kb": 0,
  "cpu_ms_per_second_of_imu": 0.0,
  "samples": 0,
  "data_duration_s": 0.0,
  "host": {"cpu": "...", "os": "...", "compiler": "..."},
  "commit": "..."
}
```

---

## 7. Constraints and budgets `[DECK s.13, 14]`

| Constraint | Limit | Internal gate `[DECISION]` | How we verify |
|---|---|---|---|
| Peak RAM | **≤ 8 MB** | ≤ 4 MB (aim ≈ 3 MB) | `/usr/bin/time -v` (Maximum resident set size) and `valgrind --tool=massif` as a cross-check |
| Compute | **≤ 20 ms per 1 s of IMU** | ≤ 2 ms | CPU time ÷ data duration; at 200 Hz this is ≈ 100 µs per sample allowed, ≈ 10 µs targeted |
| Threads | **single CPU thread** | 1 | `taskset -c 0`; no threading libraries linked |
| Network | **offline** | none | Run with networking disabled in the clean-room test |
| Input | Degraded IMU only | | No other data used anywhere |
| Python version (Track 2 only) | Not specified for Track 3 | | Track 3 deck gives `main.py / main.cpp` as entry `[DECK s.17]` |

**Python feasibility warning.** A bare CPython process already occupies roughly 8 to 10 MB of resident memory, and importing NumPy adds far more. If memory is measured as whole-process RSS, a Python-only submission cannot meet 8 MB. This is why the primary implementation is C++ (see 11.1 and Q3).

---

## 8. Deliverables and submission package

### 8.1 Track-specific deliverables `[DECK s.14, 17]`

| # | Deliverable | Notes |
|---|---|---|
| D1 | **Standalone program**: degraded IMU CSV → 6-DoF trajectory | Slide 14 says "Standalone Python script"; slide 17 says `main.py / main.cpp` as "entrypoint / build system" |
| D2 | **`estimated_trajectory.txt`** | Format in section 6 |
| D3 | `metadata.json` | Optional: runtime performance, compute specs |
| D4 | `src/` with complete source, **`README.md`** "how to compile & run your code" | For reproducibility |
| D5 | **Pipeline walkthrough video, 2 to 3 min** | Slide 14 |

**Bundle layout** `[DECK s.17]`:

```
team_name.zip
├── estimated_trajectory.txt     # final generated trajectory file
├── metadata.json                # optional: runtime perf, compute specs
└── src/                         # complete source for reproducibility
    ├── main.py / main.cpp       # entrypoint / build system
    └── README.md                # how to compile & run your code
```

> **Discrepancy.** Slide 14 lists "Trajectory CSV: timestamp, x, y, z, qx, qy, qz, qw", but slides 15, 16 and 17 define a **space-separated `.txt`** in TUM format. `[DECISION]` Follow the data-format slides (`estimated_trajectory.txt`). Confirm in Q9.

### 8.2 Common deliverables for every track `[DECK s.25]`

| Item | Requirement |
|---|---|
| Working prototype code | **Public or shared GitHub repo** |
| README | Reproducible setup instructions, **Docker files** and other requirements |
| Demo video | **Max 3 minutes** (YouTube or Drive link) |
| Presentation | PPT or PDF named `CollegeName_TeamName` (follow the nomenclature) |
| Submission | **One submission per team via the Google Form** |
| Presentation content | Theme ID, project title and team details; problem statement in your own words; solution and architecture diagram; tools and tech stack; innovation highlights, results and limitations |
| Penalty | "Teams NOT following the submission guideline would lead to direct disqualification" |

### 8.3 Team rules `[DECK s.25]`

Max 4 members from a single college. Team name `CollegeName_TeamName`. One theme per team. Per-member details: name, email, phone, year and branch. Registration via Google Form (`https://forms.gle/oGBXsydqbhE3HS2o9`, slide 27).

### 8.4 Calendar `[DECK s.24, 27]`

| Date | Milestone |
|---|---|
| 5 Nov | Launch |
| **8 Nov** | Registration closes (11:59 PM); final submission link opens (sent only to registered teams) |
| **18 Nov** | Final submission closes, 11:59 PM, no extension. Prototype code, demo video and deck due |
| 23 Nov | Top 15 announced |
| 27 Nov | Final demo: top 15 present to the Samsung R&D jury |
| 2 Dec | Final results |

The dates are marked "tentative, subject to minor changes" on slide 27.

---

## 9. Functional requirements

Priority: **P0** = must have for submission, **P1** = should have, **P2** = nice to have.

### 9.1 I/O

| ID | Requirement | Priority |
|---|---|---|
| FR-1 | CLI: `main <input> [output]`. `<input>` is a path to an unzipped folder containing `imu0/data.csv`, or to the `.zip`. Default output is `estimated_trajectory.txt` in the working directory | P0 |
| FR-2 | Stream the CSV one row at a time; never load the file or keep full history in memory | P0 |
| FR-3 | Parse robustly per 5.2 (headers, CRLF, blanks, NaN, truncation) | P0 |
| FR-4 | Write `estimated_trajectory.txt` per section 6, exactly one pose per input row | P0 |
| FR-5 | Timestamp written exactly from the int64 ns value | P0 |
| FR-6 | Exit code 0 on success. On malformed input, still write the best valid output and print a warning to stderr | P0 |
| FR-7 | Optional `--append-blind` flag (section 6.3) | P1 |
| FR-8 | Optional `--metrics` flag that writes `metadata.json` with runtime and peak RSS | P1 |
| FR-9 | Direct reading of `.zip` input (streaming inflate, e.g. a single-file library) without extracting to disk | P1 |

### 9.2 Estimation

| ID | Requirement | Priority |
|---|---|---|
| FR-10 | **Initialization:** estimate gravity direction (roll and pitch) and gyro bias from an initial quasi-static window; yaw set to 0; gravity magnitude estimated from data | P0 |
| FR-11 | **Strapdown propagation** of attitude (quaternion), velocity and position at every valid sample | P0 |
| FR-12 | **Error-state Kalman filter** (15 states: attitude, velocity, position, gyro bias, accelerometer bias) with fixed-size matrices | P0 |
| FR-13 | **Gravity-direction (tilt) update** when specific-force magnitude is close to gravity | P0 |
| FR-14 | **Zero-velocity update** when the vehicle is detected as stationary | P1 |
| FR-15 | **Degradation monitor:** rolling noise and bias statistics; adaptive inflation of process and measurement noise as quality falls | P0 |
| FR-16 | **Dropout handler:** detect gaps, NaNs, zeros and stuck values; propagate with a motion model through the gap, inflate covariance, resume without a jump; still emit one row per input row | P0 |
| FR-17 | **Failure detector:** declare IMU failure from sustained invalid or stuck data, or from missing data to the end of the file | P0 |
| FR-18 | **Blind-5-s predictor:** after failure, extrapolate from the last reliable state with a decaying-velocity / constant-turn-rate model; parameters tuned offline | P0 |
| FR-19 | Numerical safeguards: quaternion renormalization, covariance symmetrization and floor, clamp on implausible velocity | P0 |
| FR-20 | Fixed-lag smoothing over a small window if memory headroom allows | P2 |

### 9.3 Tooling (not part of the scored binary, but part of the repo)

| ID | Requirement | Priority |
|---|---|---|
| FR-21 | `tools/validate_trajectory.py`: format and row-count validator (section 13.2) | P0 |
| FR-22 | `tools/run_eval.py`: degrade, run, score with `evo`, produce per-sequence and per-segment tables | P0 |
| FR-23 | `tools/degrade.py`: surrogate degradation generator matching section 5.1, used until or alongside the organizer script | P1 |
| FR-24 | `tools/tune.py`: offline parameter search; constants then copied into the C++ source | P1 |
| FR-25 | `tools/plot_*.py`: figures for the video and deck | P1 |

---

## 10. Non-functional requirements

| ID | Requirement |
|---|---|
| NFR-1 | **Memory:** ≤ 8 MB peak RSS hard, ≤ 4 MB internal gate. No dynamic allocation in the main loop; fixed ring buffers (for example, 2 s of state history ≈ 512 entries) |
| NFR-2 | **Speed:** ≤ 20 ms per 1 s hard, ≤ 2 ms internal gate |
| NFR-3 | **Single thread, offline** |
| NFR-4 | **Determinism:** no randomness in the shipped binary; identical output across runs and machines of the same architecture |
| NFR-5 | **Portability:** builds with `g++` 11+ (or newer) with `-O2`, standard C++17, no exotic dependencies; static linking where possible |
| NFR-6 | **Reproducibility:** pinned tool versions, Dockerfile, build and run in at most three commands |
| NFR-7 | **Robustness:** no crash on any input in the fuzz suite (section 13.4) |
| NFR-8 | **Numerical:** use double precision for state and covariance (single precision is not safe for 1.4e9-scale timestamps and long integrations) |
| NFR-9 | **Licensing/data:** do not commit EuRoC or TUM-VI data to the repo; document download steps and respect dataset licenses |

---

## 11. System design

### 11.1 Implementation strategy `[DECISION]`

| Implementation | Role | Notes |
|---|---|---|
| **Impl-A: C++17 streaming ESKF** | **Primary scored implementation** | Meets memory and runtime budgets with large margin |
| **Impl-B: Python reference** | Development and tuning reference, plus conformance fallback | Written with NumPy for fast development; used to cross-check Impl-A. A **stdlib-only "lite" variant** (no covariance matrix, a Mahony-style attitude filter with scalar bias terms and velocity damping) exists only as a contingency if the organizers require a Python entry |

`src/main.py` is the documented entry point. It builds (if needed) and runs the compiled binary, and falls back to the lite Python path if no compiler or binary is available. Whether this wrapper changes how peak memory is measured is unknown (Q3). If the organizers measure the Python process, the contingency path is used.

### 11.2 Pipeline

```
imu0/data.csv ──► Streaming parser ──► Sample validator ──► Degradation monitor
                                              │                    │
                                              ▼                    ▼
                                      Gap / failure detector   Adaptive noise (Q, R)
                                              │                    │
                                              ▼                    ▼
 Initializer ──► Strapdown propagation ──► ESKF predict ──► Updates (tilt, ZUPT)
 (gravity, bias)        (q, v, p)                                  │
                                                                   ▼
                          Blind-5-s predictor ◄── failure ── Pose writer ──► estimated_trajectory.txt
```

### 11.3 State and kinematics (reference equations)

Nominal state: orientation quaternion `q` (body to world), velocity `v`, position `p`, gyro bias `b_g`, accelerometer bias `b_a`.

```
ω̂ = ω_meas − b_g                       â = a_meas − b_a
q_{k+1} = q_k ⊗ exp(½ ω̂ Δt)            (use exact exponential map or RK4)
a_w     = R(q_k) â + g,   g = [0, 0, −|g|]
v_{k+1} = v_k + a_w Δt
p_{k+1} = p_k + v_k Δt + ½ a_w Δt²
```

Error state (15): `δx = [δθ, δv, δp, δb_g, δb_a]`, with covariance `P` (15×15, fixed size).

Initial noise priors, to be tuned `[DECISION]`:

| Parameter | EuRoC MAV imu0 | TUM-VI imu |
|---|---|---|
| Gyro noise density (rad/s/√Hz) | 1.7e-4 | 1.6e-4 |
| Gyro bias random walk (rad/s²/√Hz) | 1.9e-5 | 2.2e-5 |
| Accel noise density (m/s²/√Hz) | 2.0e-3 | 2.8e-3 |
| Accel bias random walk (m/s³/√Hz) | 3.0e-3 | 8.6e-4 |

(These are commonly published calibration values; verify against the dataset documentation. The degraded data will have larger and growing noise, which is what the adaptive monitor handles.)

### 11.4 Module responsibilities

| Module | Responsibility | Key design points |
|---|---|---|
| Parser | Streaming CSV parse with a small fixed buffer | Custom number parser; tolerant of malformed rows; no per-row heap allocation |
| Validator | Flags NaN, empty, zero-row, repeated-row, out-of-range values | Plausibility limits on |ω| and |a| |
| Degradation monitor | Rolling variance of high-frequency residuals; bias-trend estimate | Output is a quality score in [0, 1] that scales Q and R |
| Initializer | Static-window mean for gravity direction and gyro bias | Fallback if no static window exists: use the first second with a high initial covariance |
| Strapdown | Quaternion, velocity, position integration | Double precision, renormalization each step |
| ESKF | Covariance propagation and correction | Fixed-size arrays, Joseph-form or symmetrized updates, covariance floors |
| Tilt update | Corrects roll and pitch from the accelerometer when |a| ≈ g | Measurement noise inflated with the degradation score |
| ZUPT | Zero-velocity pseudo-measurement when stationary | Off when the motion detector is uncertain |
| Gap handler | Handles `dt` jumps, NaN rows and stuck sensors | Propagate with the motion model, not with garbage data; inflate `P`; do not apply updates from invalid samples |
| Failure detector | Declares complete failure | Triggered by sustained invalid data or end of valid data; freezes the filter state |
| Blind predictor | Extrapolates 5 s | Decaying-velocity and constant-turn-rate hypotheses; selection and decay constants tuned offline |
| Writer | Formats rows | int64-derived timestamp, 9 decimals, unit quaternion |

### 11.5 Blind-5-s predictor design `[DECISION]`

After failure, integrating the last acceleration would amplify bias. Instead:
1. Take the last reliable state `(p, v, q, ω_smooth)` from a short smoothing window.
2. Predict position with a **decaying-velocity model**, `v(t) = v₀ · exp(−t/τ)`, and orientation with a **constant turn rate** that also decays.
3. Choose `τ`, and the weighting between the two models, to **minimize 5-s RMSE** across the tuning set.
4. Prefer conservative extrapolation: under an RMSE-type metric a smaller expected error beats a bold wrong prediction.

### 11.6 Dropout handler design `[DECISION]`

| Situation | Behaviour |
|---|---|
| Rows missing (gap in timestamps) | Detect `dt` far above the nominal interval; propagate across the gap using smoothed pre-gap velocity and turn rate with decay; inflate `P`; resume corrections after the gap |
| Rows present but invalid (NaN, zeros, stuck) | Same treatment per row; skip updates; keep emitting rows |
| Recovery | After valid data resumes, re-open the gates gradually (higher `R` initially) to avoid a position jump |
| Gap longer than a threshold | Treat as failure if it continues to the end of the data (see Q4) |

---

## 12. Evaluation harness (our internal copy of the organizers' scoring)

### 12.1 Data

- **Datasets:** EuRoC MAV and TUM-VI. `[DECK s.13]` Download and keep them outside the repo.
- **Degradation:** use the **organizer degradation script** as soon as it is available. Until then use our surrogate `tools/degrade.py` (progressive noise ramp, bias random-walk growth, 2-s dropout windows, final failure) `[DECISION]`.
- **Splits:** tune on some sequences, hold out others (cross-validate by sequence), and randomize degradation seeds. Never evaluate on the data used to tune.

### 12.2 Scoring with `evo`

```
# Absolute trajectory error (translation RMSE), after SE(3) alignment
evo_ape tum gt.txt estimated_trajectory.txt -a --pose_relation trans_part

# Same with scale correction (Sim(3)) in case the organizers use it
evo_ape tum gt.txt estimated_trajectory.txt -as

# Relative pose error over a fixed 1 s window at 200 Hz (200 frames)
evo_rpe tum gt.txt estimated_trajectory.txt --delta 200 --delta_unit f -a
```

Verify flags with `--help` on the installed `evo` version, and run **both aligned and unaligned** variants until Q2 is answered.

### 12.3 Segment-wise metrics (proxy for the three Closeness components)

Because the deck does not define the scored segments, we compute all of the following and optimize the sum:

| Our metric | Proxy for | Definition |
|---|---|---|
| Checkpoint error | Waypoints (30) | Position and orientation error at timestamps of the simulated mission checkpoints (for example every N seconds) |
| Blind-5-s error | Blind 5 s (20) | RMSE over the 5 s after the simulated failure |
| Dropout error | Dropout (10) | RMSE during and in the 5 s after each inserted dropout window |
| Global ATE and RPE | The `evo` metrics named in the deck | Full-trajectory `evo_ape` and `evo_rpe` |

### 12.4 Baselines for comparison

| Baseline | Purpose |
|---|---|
| Naive double integration | Shows the "explodes quadratically" problem; the floor we must clearly beat |
| Complementary / Mahony attitude plus integration | Mid-level reference |
| Our ESKF without adaptivity | Ablation |
| Full system | Final result |

Report every variant in an ablation table (ATE, RPE, blind-5-s error, peak RAM, ms per 1 s).

---

## 13. Test plan

### 13.1 Unit tests
Quaternion algebra and normalization, exponential map, rotation matrix conversion, number parser (edge formats), timestamp formatter (int64 → string), covariance symmetry and positive-definiteness checks.

### 13.2 Format validator (`tools/validate_trajectory.py`)

Checks, each failing loudly:

1. Every non-comment line has **exactly 8** space-separated fields.
2. All fields are finite numbers; no `nan`, `inf` or stray text.
3. Row count equals the number of IMU data rows (or the confirmed rule).
4. Timestamps match the input timestamps exactly (`ns × 1e-9`) and are in input order.
5. Quaternion norm within `1 ± 1e-6`.
6. Comment lines start with `#`.
7. Single trailing newline; LF endings.

### 13.3 Budget tests

| Test | Command / method | Pass |
|---|---|---|
| Peak RSS | `/usr/bin/time -v ./main in.zip out.txt` | ≤ 4 MB gate |
| Runtime per 1 s | CPU time ÷ data duration, on the longest sequence | ≤ 2 ms gate |
| Single thread | `taskset -c 0 ...`; check thread count in `/proc` | 1 thread |
| Offline | Run in a container with networking disabled | Success |
| Determinism | Run twice, compare checksums | Identical |

### 13.4 Robustness / fuzz suite
Inject: NaN and empty fields, all-zero rows, repeated rows, 2-s and longer gaps, duplicate and reversed timestamps, truncated last line, CRLF, no header, empty file, very long file (10+ minutes). Pass = no crash, valid output.

### 13.5 Regression
Every change reruns the full evaluation table; commit results to `results/` so degradations are visible in version control.

### 13.6 Clean-room test
In a fresh container with only the README commands, build, run on a sample input, validate output, and check budgets. Repeat on the final tagged commit before submission.

---

## 14. Repository layout `[DECISION]`

```
repo/
├── README.md                     # top-level overview and quick start
├── Dockerfile                    # reproducible environment (required by common rules)
├── src/                          # becomes src/ in the submission bundle
│   ├── main.py                   # entry point wrapper / Python reference
│   ├── cpp/                      # C++ estimator (parser, eskf, writer, ...)
│   ├── Makefile
│   └── README.md                 # how to compile and run
├── tools/                        # validator, eval runner, degrader, tuner, plots
├── results/                      # ablation tables, metrics (no raw datasets)
├── docs/                         # architecture diagram, deck, video storyboard
└── tests/                        # unit, robustness and regression tests
```

Submission bundle `team_name.zip` is assembled by a script (`tools/make_bundle.sh`) containing only `estimated_trajectory.txt`, `metadata.json` and `src/` with its README, matching section 8.1.

---

## 15. Demo video (2 to 3 min, max 3) and deck

### 15.1 Video storyboard `[DECISION]`

| Time | Content |
|---|---|
| 0:00 to 0:20 | The XR problem: camera tracking lost, inertial pose must continue |
| 0:20 to 1:30 | 3D plot, ground truth vs estimate, as degradation ramps up; naive integration diverging beside our filter |
| 1:30 to 2:15 | The blind 5 s with an uncertainty cone; dropout handling |
| 2:15 to 3:00 | Budgets proven live (peak RSS, time per second), one-command reproducibility, limitations |

Keep it at **3 minutes or less**, which satisfies both the "max 3 min" (slide 25) and "5-min" (slides 24, 27) statements.

### 15.2 Deck (CollegeName_TeamName) `[DECK s.25]`
Theme ID (Track 3), title and team, problem in our own words, architecture diagram (section 11.2), tools and stack, innovation highlights, results with the ablation table, honest limitations.

---

## 16. Milestones and plan `[DECISION]`

Roles (4 members): **A** estimation lead, **B** evaluation and tuning, **C** C++ and packaging, **D** video, deck and QA.

| Date | Milestone | Exit criterion | Owner |
|---|---|---|---|
| 6 Nov | M0: Setup | Repo, data downloaded, `evo` working, questions emailed | All |
| 7 Nov | Registration data ready | Names, emails, phones, year and branch collected | D |
| **8 Nov** | **Registration submitted before 11:59 PM**; M1: harness | Degrade, run, score pipeline works on a trivial estimator | B |
| 9 to 10 Nov | M2: Python reference | ESKF beats naive integration on all held-out sequences | A |
| 11 to 12 Nov | M3: C++ port | C++ output matches Python within tolerance; RSS and time gates met | C |
| 13 to 14 Nov | M4: Robustness | Gap handler, adaptive noise, blind predictor tuned with cross-validation | A, B |
| 15 Nov | **Feature freeze**; M5: results | Ablation table and plots final | B |
| 16 Nov | M6: Packaging | Video, deck, README, Dockerfile; clean-room pass | C, D |
| **17 Nov** | **Submit** (one day early) | All links verified while logged out | All |
| 18 Nov | Hard deadline buffer, 11:59 PM | | |
| 23 to 27 Nov | Shortlist, final demo rehearsal | Backup recording ready | D |

---

## 17. Risks, open questions and discrepancy log

### 17.1 Risks

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R1 | Memory measured on a Python process makes ≤ 8 MB impossible | Medium | High (up to 20 pts) | Ask Q3; keep the stdlib-only lite Python contingency; keep the C++ path |
| R2 | Output row-count rule mismatch (blind segment) | Medium | High (heavy penalties or failure) | Ask Q1; flag-controlled behaviour; validator tests both |
| R3 | Unknown frame or alignment makes GT comparison inconsistent | Medium | High | Ask Q2; evaluate with and without alignment |
| R4 | Degradation script differs from our surrogate; overfitting to our simulation | Medium | Medium | Switch to the official script on arrival; cross-validate by sequence |
| R5 | Accuracy limited by IMU physics (drift is unbounded) | High | Medium | Focus on robustness and conservative blind prediction; rank is relative |
| R6 | Timestamp rounding creates misalignment | Low | High | Write timestamps from int64 (FR-5) |
| R7 | C++ build fails on the organizers' machine | Low-Medium | High | Plain C++17, Makefile, static binary option, Docker; Python fallback |
| R8 | Schedule slip | Medium | Medium | Feature freeze on 15 Nov; submit on 17 Nov |
| R9 | Guideline violation, leading to disqualification | Low | Severe | Section 18 checklist; validator; clean-room test |

### 17.2 Open questions for the organizers (prism@samsung.com)

| ID | Priority | Question | Default assumption used until answered |
|---|---|---|---|
| Q1 | **Critical** | How is the blind 5 s represented? Are there IMU rows after failure (with invalid data), or must we append 5 s of extra poses? | One output row per input row; `--append-blind` off |
| Q2 | **Critical** | What frame is the output expressed in, and how does `evo` align it (none, SE(3), Sim(3))? | Gravity-aligned z-up, yaw 0, origin at start; assume SE(3) alignment |
| Q3 | **Critical** | How is peak RAM measured (whole-process RSS vs algorithm)? Is a C++ binary acceptable as the entry point? | C++ binary acceptable; whole-process RSS |
| Q4 | High | How do dropouts and the final failure appear in the CSV (missing rows, NaN, zeros, repeated values)? | Handle all |
| Q5 | High | What exactly are the "checkpoints" and "dropout" scoring windows? | Our proxy metrics (12.3) |
| Q6 | Medium | Which sequences are in the hidden test set, and what is the sample rate? | EuRoC and TUM-VI style, ~200 Hz |
| Q7 | Medium | How do ATE, RPE, memory and runtime convert into points (pass/fail, graded, or rank)? | Graded; maximize margin everywhere |
| Q8 | Medium | Tier bonuses say "+20%", but the split is Blind 5 s 20 and Dropout 10. Which is correct? | 20 / 10 split |
| Q9 | Medium | Slide 14 mentions a "Trajectory CSV" while slide 16 specifies a space-separated `.txt`. Which is correct? | `estimated_trajectory.txt` |
| Q10 | Low | Is the demo video limit 3 min (slide 25) or 5 min (slides 24, 27)? | ≤ 3 min |
| Q11 | Low | Slide 14 says "Standalone Python script" while slide 17 allows `main.cpp`. Is a compiled language fine? | Yes |

### 17.3 Deck discrepancies relevant to Track 3

| # | Where | Issue | Our resolution |
|---|---|---|---|
| 1 | s.14 vs s.16 | "Trajectory CSV" vs space-separated `.txt` | `.txt` |
| 2 | s.14 vs s.17 | "Standalone Python script" vs `main.py / main.cpp` | Ship both paths |
| 3 | s.14 | Tier bonuses "+20%" vs evaluation split 20 / 10 | 20 / 10 |
| 4 | s.13 vs s.15/16 | Narrative says "waypoints" and "checkpoints"; the data format mentions neither | Compute our own proxies |
| 5 | s.24 vs s.25 vs s.27 | Demo video 5 min vs max 3 min | ≤ 3 min |
| 6 | s.16 | Sample timestamps show 4 decimals (`1403636579.7585`) while the input has ns resolution | Write exact 9-decimal timestamps |

---

## 18. Pre-submission checklist

**Track deliverables**
- [ ] `estimated_trajectory.txt`: 8 columns, space-separated, seconds timestamps, unit quaternions, one pose per IMU row
- [ ] `metadata.json` (optional but included) with measured RSS and runtime
- [ ] `src/` with `main.py` / `main.cpp`, Makefile and `README.md`
- [ ] Validator passes on every test input
- [ ] Budgets measured and within gates (≤ 4 MB, ≤ 2 ms per 1 s, 1 thread, offline)
- [ ] Pipeline walkthrough video, 2 to 3 min

**Common rules**
- [ ] Registered by 8 Nov, 11:59 PM; team name `CollegeName_TeamName`; ≤ 4 members; single college
- [ ] Public or shared GitHub repo with README, Docker files and requirements
- [ ] Demo video ≤ 3 min, link viewable without login
- [ ] Deck `CollegeName_TeamName` covering theme ID, title and team, problem, architecture diagram, stack, innovation, results, limitations
- [ ] One submission via the Google Form before 18 Nov, 11:59 PM (target 17 Nov)
- [ ] Clean-room test passed on the final tagged commit
- [ ] Organizer clarifications recorded in the README

---

## 19. Glossary

| Term | Meaning |
|---|---|
| IMU | Inertial measurement unit (gyroscope + accelerometer) |
| 6-DoF | Six degrees of freedom: 3 for position, 3 for orientation |
| Dead reckoning | Estimating the current pose by integrating motion from a known earlier pose |
| ESKF | Error-state Kalman filter |
| ZUPT | Zero-velocity update |
| ATE | Absolute trajectory error (RMSE of position after alignment) |
| RPE | Relative pose error over a fixed interval |
| `evo` | Open-source trajectory evaluation package used by the organizers |
| TUM format | `timestamp tx ty tz qx qy qz qw`, space-separated |
| EuRoC MAV / TUM-VI | The public datasets named for this track |
| RSS | Resident set size, the memory a process holds in RAM |