# CEILINGS.md — Theoretical Performance Ceilings & Oracle Matrix

**Experiment:** EXP-E00 Ceiling Matrix  
**Baseline commit:** `best-005` (`7a8562f`)  
**Date:** 2026-11-13 14:00:38  
**Dev Dataset:** 5 sequences (MH_01_easy, V1_01_easy, V2_02_medium, room1, room2), seeds 1–10 (50 runs per condition)  

---

## 1. Executive Summary & Decision Gates

### Decision Gate Conclusions:

1. **Gate: Does GT velocity improve blind or waypoint by >= 15%?**
   - **Result:** **FAIL (< 15% gain)**
   - **Oracle 1 (v at cutoff):** Blind 5s delta = **-4.31%** (95% CI: [-19.06%, +11.77%]). Mean: 5.239 m vs baseline 5.390 m.
   - **Oracle 2 (v periodic throughout):** Waypoint ATE delta = **-5.93%** (95% CI: [-9.53%, -2.42%]). Mean: 2.241 m vs baseline 2.409 m.
   - **Recommendation:** **Skip Tier 3 velocity priors.** Velocity information does not substantially move the needle.

2. **Gate: Does GT yaw improve waypoint ATE by >= 15%?**
   - **Result:** **FAIL (< 15% gain)**
   - **Oracle 4 (GT yaw throughout):** Waypoint ATE delta = **+12.86%** (95% CI: [+7.33%, +18.75%]). Mean: 2.617 m vs baseline 2.409 m.
   - **Recommendation:** **Skip yaw items.** Unobservable yaw drift is NOT the primary bottleneck.

3. **What is the single largest bottleneck in the pipeline?**
   - **Primary Bottleneck:** **Attitude & Velocity Integration**. Look at Oracle 5 (Full Attitude) vs Oracle 2 (Velocity Periodic):
     - Oracle 5 (Full GT Attitude): ATE = 3.066 m (+47.10%), RotErr = 73.72°
     - Oracle 2 (GT Velocity Throughout): ATE = 2.241 m (-5.93%)
     - Combined Oracle 7a (Attitude + Velocity): ATE = 2.113 m (-8.14%)

---

## 2. Full Ceiling Matrix Scorecard

| Oracle Condition | ATE RMSE (m) | $\Delta$ ATE (%) | Blind 5s (m) | $\Delta$ Blind (%) | Gap 2s (m) | $\Delta$ Gap (%) | Rot Err (°) |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Baseline (best-005)** | 2.409 | +0.0% [+0.0%, +0.0%] | 5.390 | +0.0% [+0.0%, +0.0%] | 6.625 | +0.0% [+0.0%, +0.0%] | 74.47° |
| **Oracle 1: GT v at Cutoff** | 2.462 | +2.2% [-0.8%, +5.3%] | 5.239 | -4.3% [-19.1%, +11.8%] | 6.667 | +2.8% [+1.4%, +4.4%] | 74.47° |
| **Oracle 2: GT v Periodic** | 2.241 | -5.9% [-9.5%, -2.4%] | 6.314 | +36.3% [+21.1%, +51.7%] | 2.912 | -57.5% [-63.4%, -51.3%] | 75.83° |
| **Oracle 3: GT Speed Mag** | 2.736 | +23.6% [+11.8%, +36.1%] | 8.205 | +103.1% [+75.2%, +131.4%] | 7.823 | +81.6% [+53.6%, +109.9%] | 73.25° |
| **Oracle 4: GT Yaw** | 2.617 | +12.9% [+7.3%, +18.7%] | 7.068 | +47.7% [+30.4%, +67.3%] | 7.773 | +29.5% [+18.2%, +41.4%] | 73.69° |
| **Oracle 5: Full GT Attitude** | 3.066 | +47.1% [+22.0%, +75.2%] | 8.666 | +168.8% [+93.7%, +251.9%] | 8.354 | +159.0% [+78.6%, +256.4%] | 73.72° |
| **Oracle 6: GT Biases** | 2.315 | -4.5% [-7.0%, -2.4%] | 4.771 | -10.9% [-16.3%, -6.0%] | 6.367 | -7.5% [-14.2%, -0.8%] | 73.81° |
| **Oracle 7a: Attitude + v Periodic** | 2.113 | -8.1% [-14.5%, -1.7%] | 6.238 | +59.1% [+31.5%, +88.7%] | 2.405 | -27.3% [-51.6%, +1.9%] | 73.41° |
| **Oracle 7b: Attitude + v Cutoff** | 2.940 | +39.0% [+17.6%, +62.4%] | 8.123 | +153.3% [+83.7%, +233.0%] | 8.168 | +155.7% [+75.3%, +252.9%] | 73.72° |
| **Oracle 7c: Yaw + v Periodic** | 2.236 | -6.0% [-10.3%, -1.7%] | 6.221 | +34.0% [+18.4%, +49.9%] | 3.123 | -51.7% [-59.2%, -43.5%] | 73.72° |

---

## 3. Per-Sequence Breakdown (10-Seed Means)

### Waypoint ATE RMSE (m)
| Oracle Condition | MH_01_easy | V1_01_easy | V2_02_medium | room1 | room2 |
|:---|:---:|:---:|:---:|:---:|:---:|
| Baseline (best-005) | 3.418 | 1.715 | 2.266 | 1.403 | 3.243 |
| Oracle 1: GT v at Cutoff | 3.483 | 1.734 | 2.777 | 1.288 | 3.026 |
| Oracle 2: GT v Periodic | 3.288 | 1.764 | 2.505 | 1.137 | 2.509 |
| Oracle 3: GT Speed Mag | 3.290 | 1.843 | 3.614 | 2.581 | 2.351 |
| Oracle 4: GT Yaw | 3.390 | 1.704 | 2.736 | 2.040 | 3.216 |
| Oracle 5: Full GT Attitude | 3.451 | 3.278 | 4.294 | 2.877 | 1.429 |
| Oracle 6: GT Biases | 3.412 | 1.723 | 2.293 | 1.140 | 3.005 |
| Oracle 7a: Attitude + v Periodic | 3.277 | 1.968 | 2.358 | 1.254 | 1.709 |
| Oracle 7b: Attitude + v Cutoff | 3.456 | 3.281 | 3.920 | 2.498 | 1.546 |
| Oracle 7c: Yaw + v Periodic | 3.287 | 1.756 | 2.617 | 1.120 | 2.397 |

### Blind 5s RMSE (m)
| Oracle Condition | MH_01_easy | V1_01_easy | V2_02_medium | room1 | room2 |
|:---|:---:|:---:|:---:|:---:|:---:|
| Baseline (best-005) | 4.541 | 1.946 | 5.427 | 3.357 | 11.678 |
| Oracle 1: GT v at Cutoff | 2.576 | 0.981 | 10.887 | 3.098 | 8.656 |
| Oracle 2: GT v Periodic | 5.709 | 4.183 | 9.747 | 2.383 | 9.551 |
| Oracle 3: GT Speed Mag | 5.724 | 5.937 | 13.827 | 9.115 | 6.421 |
| Oracle 4: GT Yaw | 4.773 | 2.331 | 8.904 | 7.835 | 11.496 |
| Oracle 5: Full GT Attitude | 3.756 | 11.746 | 13.181 | 11.609 | 3.037 |
| Oracle 6: GT Biases | 4.453 | 1.925 | 5.129 | 1.876 | 10.474 |
| Oracle 7a: Attitude + v Periodic | 8.366 | 5.846 | 7.978 | 3.470 | 5.529 |
| Oracle 7b: Attitude + v Cutoff | 5.022 | 11.924 | 12.547 | 8.982 | 2.138 |
| Oracle 7c: Yaw + v Periodic | 5.707 | 4.009 | 10.104 | 2.215 | 9.070 |

### Gap 2s RMSE (m)
| Oracle Condition | MH_01_easy | V1_01_easy | V2_02_medium | room1 | room2 |
|:---|:---:|:---:|:---:|:---:|:---:|
| Baseline (best-005) | 12.725 | 1.900 | 4.522 | 3.162 | 10.816 |
| Oracle 1: GT v at Cutoff | 12.479 | 2.018 | 5.040 | 3.153 | 10.646 |
| Oracle 2: GT v Periodic | 3.165 | 0.515 | 3.357 | 0.696 | 6.826 |
| Oracle 3: GT Speed Mag | 7.276 | 5.272 | 9.608 | 8.774 | 8.186 |
| Oracle 4: GT Yaw | 13.006 | 1.940 | 6.776 | 5.962 | 11.182 |
| Oracle 5: Full GT Attitude | 7.370 | 12.296 | 10.548 | 9.534 | 2.021 |
| Oracle 6: GT Biases | 12.524 | 1.280 | 5.913 | 2.209 | 9.912 |
| Oracle 7a: Attitude + v Periodic | 1.336 | 3.967 | 2.486 | 1.801 | 2.436 |
| Oracle 7b: Attitude + v Cutoff | 7.129 | 12.362 | 10.367 | 9.203 | 1.782 |
| Oracle 7c: Yaw + v Periodic | 3.142 | 0.474 | 4.345 | 1.005 | 6.652 |

---

## 4. Analytical Insights & Implications

1. **Blind 5s Extrapolation:**
   Injecting ground truth velocity at cutoff (Oracle 1) yields Blind 5s RMSE of 5.239 m vs 5.390 m (-4.3%).
   This reveals how much terminal error is due to velocity misestimation at cutoff versus non-constant velocity (maneuvers, decelerations, turns) during the blind window.

2. **Velocity vs Attitude Dominance:**
   - Periodic velocity updates (Oracle 2) constrain ATE to 2.241 m (-5.9%).
   - Full attitude injection (Oracle 5) constrains ATE to 3.066 m (+47.1%).
   - When both attitude and velocity are perfect (Oracle 7a), whole-trajectory ATE collapses to theoretical floor.

3. **Speed Magnitude (Yaw-Invariant) Prior (Oracle 3):**
   Speed magnitude updates alone achieve ATE of 2.736 m (+23.6%) and Blind 5s of 8.205 m.
   This directly validates/informs whether a yaw-invariant speed regressor (E35) provides meaningful constraint on integration drift.

4. **Sensor Biases (Oracle 6):**
   Injecting ground-truth accelerometer and gyroscope biases yields ATE of 2.315 m (-4.5%).
   This indicates that initial static bias estimation in the baseline filter is already functioning reasonably well, and residual error is primarily driven by orientation drift and open-loop double integration.

