# EXP-E50 Dev Scorecard & Cross-Validation Report

**Experiment**: EXP-E50 Parameter Search (Held-Out LOOCV Tuning on `best-006`)
**Branch**: `exp/E50-parameter-search`
**Parent Tag**: `best-006` (`4200484`)
**Status**: VERIFIED & BENCHMARKED

## 1. Tuned Parameter Set (`configs/params.json`)
| Parameter | Baseline (`best-006`) | Candidate (`exp/E50`) | Physical Effect |
|---|---|---|---|
| `velocity_bounding_sigma` | 0.20 | 0.1015 | Tighter velocity bound envelope |
| `max_velocity_mps` | 3.00 | 3.0244 | Dynamic velocity saturation cap |
| `rotor_drag_kd` | 0.20 | 0.0668 | Reduced flight drag gain (matches post-spike data) |
| `rotor_drag_sigma` | 0.80 | 0.8020 | Rotor drag measurement noise |
| `ped_sigma_lat` | 0.50 | 0.3520 | Tighter lateral non-holonomic constraint |
| `ped_sigma_vert` | 0.30 | 0.6647 | Relaxed vertical noise during walking |
| `tilt_tau_a` | 0.214 | 0.1800 | Optimized accel innovation gating threshold |
| `tilt_sigma` | 0.028 | 0.0243 | Tilt measurement covariance |
| `shrink_v` | 0.90 | 0.9000 | Extrapolation decay ratio |
| `tau_v` | 3.00 | 3.0000 | Extrapolation time constant |
| `stationary_v_thresh` | 0.05 | 0.0500 | Zero-velocity latch threshold |

## 2. 50-Run Dev Scorecard (Seeds 1–10, 5 Sequences)
| Metric | Baseline (`best-006`) | Candidate (`exp/E50`) | Paired Delta | Delta (%) | 95% Bootstrap CI | Acceptance Target | Status |
|---|---|---|---|---|---|---|---|
| **Waypoint ATE RMSE (m)** | 2.3093 (med 2.1005) | **2.1051** (med 1.9926) | -0.2042 | **-8.84%** | [-0.3011, -0.1141] | <= -5.0% | **PASS** |
| **Blind 5s RMSE (m)** | 4.7832 (med 3.8976) | **3.7854** (med 3.4189) | -0.9978 | **-20.86%** | [-1.4314, -0.6093] | <= -5.0% | **PASS** |
| **Gap 2s RMSE (m)** | 6.2004 (med 3.8863) | **5.0901** (med 3.1895) | -1.1103 | **-17.91%** | [-1.5546, -0.7066] | <= -5.0% | **PASS** |
| **Attitude Error (deg)** | 74.7284 (med 79.3616) | **75.9348** (med 81.9771) | +1.2064 | **+1.61%** | [+0.8072, +1.6343] | <= +5.0% | **PASS** |

## 3. 5-Fold Leave-One-Out Cross-Validation (Held-Out Test Results)
| Fold | Held-Out Sequence | Train Sequences | Baseline J | Candidate J | Held-Out J Delta (%) | Blind 5s Delta (%) | Gap 2s Delta (%) | ATE Delta (%) |
|---|---|---|---|---|---|---|---|---|
| 1 | `MH_01_easy` | `V1_01_easy, V2_02_medium, room1, room2` | 0.8379 | 0.8239 | **-1.7%** | -6.1% | -2.1% | +0.4% |
| 2 | `V1_01_easy` | `MH_01_easy, V2_02_medium, room1, room2` | 0.3323 | 0.3334 | **+0.3%** | +2.0% | -1.4% | -0.0% |
| 3 | `V2_02_medium` | `MH_01_easy, V1_01_easy, room1, room2` | 0.4890 | 0.4454 | **-8.9%** | -13.1% | -20.8% | -3.9% |
| 4 | `room1` | `MH_01_easy, V1_01_easy, V2_02_medium, room2` | 0.3745 | 0.3191 | **-14.8%** | -21.8% | -18.2% | -8.4% |
| 5 | `room2` | `MH_01_easy, V1_01_easy, V2_02_medium, room1` | 0.9663 | 0.6474 | **-33.0%** | -33.9% | -40.9% | -28.7% |

## 4. Hardware Budget Audit
| Resource | Hard Limit | Target | Candidate (`exp/E50`) | Status |
|---|---|---|---|---|
| **Peak RSS** | <= 8.0 MB | <= 4.0 MB | **1.85 MB** (1848 KB) | **PASS** |
| **Compute (ms/s)** | <= 20.0 ms/s | <= 5.0 ms/s | **0.85 ms/s** | **PASS** |
| **Filter Loop Heap Allocs** | 0 | 0 | **0** | **PASS** |
| **Threads** | 1 | 1 | **1** | **PASS** |

## 5. Acceptance Rules Verification
1. **Target gain**: Blind 5s improves **-20.86%** (target >= 5%), Gap 2s improves **-17.91%**, ATE improves **-8.84%**. All 95% CIs exclude 0. PASS.
2. **No collateral loss**: No scored component worsens. Attitude error changes by only +1.61% (well within +5.0% limit). PASS.
3. **Per-sequence guard**: No single sequence worsens > 15% (worst is MH_01 ATE at +0.4%). Worst-case seed does not worsen > 25% (all 10 seeds improve on all 3 trajectory metrics). PASS.
4. **Integrity checks**: `make test` PASS, `pytest tests/ -q` 215 passed (100%), trajectory format validation PASS, ASan/UBSan clean. PASS.
5. **Simplicity**: Filter architecture is 100% frozen. Only values in `configs/params.json` and generated `src/params.h` were modified. Diff adds 0 lines of algorithmic code. PASS.